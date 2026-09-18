#!/usr/bin/env python3
import errno
import glob
import os
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FMT = '<IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)


class GamepadTakeoverNode(Node):
    """8BitDo Ultimate 2C gamepad takeover using Linux /dev/input/js*.

    Mapping:
      LB(button 4): AUTO <-> PAD
      D-pad up/down(axis 7): drive stage
      D-pad left/right(axis 6): steering +/-5 command-deg
      A(button 0): steering center
      X(button 2): immediate stop while staying in PAD mode

    Safety:
      PAD disconnect never returns control to AUTO automatically.
      PAD ownership remains selected and STOP is held.
    """

    def __init__(self):
        super().__init__('t870_gamepad_takeover')

        defaults = {
            'device': '/dev/input/js0',
            'auto_find_device': True,
            'lb_button': 4,
            'a_button': 0,
            'x_button': 2,
            'dpad_x_axis': 6,
            'dpad_y_axis': 7,
            'axis_threshold': 16000,
            'steer_step_deg': 5,
            'max_steer_deg': 22,
            'heartbeat_hz': 10.0,
            'reconnect_s': 1.0,
            'mirror_manual_topics': True,
            'manual_drive_topic': '/manual_drive',
            'manual_wheel_topic': '/manual_wheel',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        gp = lambda name: self.get_parameter(name).value

        self.device_param = str(gp('device'))
        self.auto_find = bool(gp('auto_find_device'))
        self.lb_button = int(gp('lb_button'))
        self.a_button = int(gp('a_button'))
        self.x_button = int(gp('x_button'))
        self.dpad_x_axis = int(gp('dpad_x_axis'))
        self.dpad_y_axis = int(gp('dpad_y_axis'))
        self.axis_threshold = int(gp('axis_threshold'))
        self.steer_step = int(gp('steer_step_deg'))
        self.max_steer = abs(int(gp('max_steer_deg')))
        self.heartbeat_hz = max(1.0, float(gp('heartbeat_hz')))
        self.reconnect_s = max(0.1, float(gp('reconnect_s')))
        self.mirror_manual = bool(gp('mirror_manual_topics'))

        self.fd = None
        self.device_path = None
        self.last_connect_try = 0.0
        self.takeover = False
        self.stop_active = True
        self.drive_stage = 0
        self.steer_deg = 0
        self.mcu_wheel_deg = 0
        self.last_axis = {}

        self.pub_takeover = self.create_publisher(Bool, '/pad/takeover', 10)
        self.pub_drive = self.create_publisher(Float32, '/pad/cmd_drive', 10)
        self.pub_wheel = self.create_publisher(Int32, '/pad/cmd_wheel', 10)
        self.pub_stop = self.create_publisher(Bool, '/pad/cmd_stop', 10)
        self.pub_heartbeat = self.create_publisher(Bool, '/pad/heartbeat', 10)

        self.manual_drive_pub = None
        self.manual_wheel_pub = None
        if self.mirror_manual:
            self.manual_drive_pub = self.create_publisher(
                Float32, str(gp('manual_drive_topic')), 10)
            self.manual_wheel_pub = self.create_publisher(
                Int32, str(gp('manual_wheel_topic')), 10)

        self.create_subscription(
            Int32, '/mcu/applied_wheel', self.on_mcu_wheel, 10)

        self.create_timer(0.01, self.poll_js)
        self.create_timer(1.0 / self.heartbeat_hz, self.heartbeat_tick)

        self.publish_state()
        self.get_logger().info(
            f'8BitDo takeover ready: LB={self.lb_button}, '
            f'D-pad axes=({self.dpad_x_axis},{self.dpad_y_axis}), '
            f'steer step={self.steer_step}deg, manual mirror={self.mirror_manual}'
        )

    @staticmethod
    def now():
        return time.monotonic()

    def resolve_device(self):
        if os.path.exists(self.device_param):
            return self.device_param
        if self.auto_find:
            found = sorted(glob.glob('/dev/input/js*'))
            if found:
                return found[0]
        return None

    def connect_if_needed(self):
        if self.fd is not None:
            return
        now = self.now()
        if now - self.last_connect_try < self.reconnect_s:
            return
        self.last_connect_try = now
        path = self.resolve_device()
        if not path:
            return
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            self.device_path = path
            self.last_axis.clear()
            self.get_logger().info(f'gamepad connected: {path}')
        except OSError as exc:
            self.fd = None
            if exc.errno == errno.EACCES:
                self.get_logger().error(f'permission denied: {path}')
            elif exc.errno != errno.ENOENT:
                self.get_logger().warn(f'cannot open gamepad {path}: {exc}')

    def disconnect(self, reason='disconnected'):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.device_path = None
        if self.takeover:
            self.stop_active = True
            self.drive_stage = 0
            self.publish_state()
            self.get_logger().error(
                f'gamepad {reason} during PAD takeover -> STOP held; mode remains PAD')

    def on_mcu_wheel(self, msg: Int32):
        self.mcu_wheel_deg = max(
            -self.max_steer, min(self.max_steer, int(msg.data)))

    def publish_state(self):
        self.pub_takeover.publish(Bool(data=self.takeover))
        self.pub_drive.publish(Float32(data=float(self.drive_stage)))
        self.pub_wheel.publish(Int32(data=int(self.steer_deg)))
        self.pub_stop.publish(Bool(data=self.stop_active))

        if self.manual_drive_pub is not None:
            manual_drive = float(self.drive_stage) if self.takeover else 0.0
            self.manual_drive_pub.publish(Float32(data=manual_drive))
            self.manual_wheel_pub.publish(Int32(data=int(self.steer_deg)))

    def heartbeat_tick(self):
        connected = self.fd is not None
        self.pub_heartbeat.publish(Bool(data=connected))
        self.publish_state()

    def toggle_takeover(self):
        self.takeover = not self.takeover
        self.drive_stage = 0
        if self.takeover:
            self.stop_active = False
            self.steer_deg = self.mcu_wheel_deg
            self.get_logger().warn(
                f'LB -> PAD TAKEOVER: start wheel={self.steer_deg}deg, drive=0')
        else:
            self.stop_active = True
            self.get_logger().warn(
                'LB -> AUTO RETURN: STOP requested during handover')
        self.publish_state()

    def step_drive(self, direction):
        order = [-1, 0, 1, 2, 3]
        idx = order.index(self.drive_stage) if self.drive_stage in order else 1
        idx = max(0, min(len(order) - 1, idx + direction))
        self.drive_stage = order[idx]
        self.stop_active = False
        self.get_logger().info(f'PAD drive stage -> {self.drive_stage}')
        self.publish_state()

    def step_steer(self, direction):
        self.steer_deg = max(
            -self.max_steer,
            min(self.max_steer, self.steer_deg + direction * self.steer_step),
        )
        self.stop_active = False
        self.get_logger().info(f'PAD steer -> {self.steer_deg:+d} deg')
        self.publish_state()

    def center_steer(self):
        self.steer_deg = 0
        self.stop_active = False
        self.get_logger().info('PAD steer -> CENTER 0 deg')
        self.publish_state()

    def immediate_stop(self):
        self.drive_stage = 0
        self.stop_active = True
        self.get_logger().warn(
            'PAD X -> IMMEDIATE STOP (PAD mode retained)')
        self.publish_state()

    def handle_button(self, number, value):
        if value != 1:
            return
        if number == self.lb_button:
            self.toggle_takeover()
            return
        if not self.takeover:
            return
        if number == self.a_button:
            self.center_steer()
        elif number == self.x_button:
            self.immediate_stop()

    def axis_edge(self, number, value):
        prev = self.last_axis.get(number, 0)
        self.last_axis[number] = value
        th = self.axis_threshold
        prev_dir = -1 if prev < -th else (1 if prev > th else 0)
        new_dir = -1 if value < -th else (1 if value > th else 0)
        if new_dir == 0 or new_dir == prev_dir:
            return 0
        return new_dir

    def handle_axis(self, number, value):
        direction = self.axis_edge(number, value)
        if direction == 0 or not self.takeover:
            return
        if number == self.dpad_x_axis:
            self.step_steer(+1 if direction < 0 else -1)
        elif number == self.dpad_y_axis:
            self.step_drive(+1 if direction < 0 else -1)

    def poll_js(self):
        self.connect_if_needed()
        if self.fd is None:
            return

        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
                if not data:
                    self.disconnect('closed')
                    return
                if len(data) != JS_EVENT_SIZE:
                    return
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self.disconnect(str(exc))
                return

            _time_ms, value, event_type, number = struct.unpack(
                JS_EVENT_FMT, data)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_BUTTON:
                self.handle_button(number, value)
            elif event_type == JS_EVENT_AXIS:
                self.handle_axis(number, value)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTakeoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
