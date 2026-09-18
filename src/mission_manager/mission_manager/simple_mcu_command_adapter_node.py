#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class SimpleMcuCommandAdapter(Node):
    """Final command arbiter for AUTO (GPS/LiDAR) and PAD control.

    AUTO:
      mode 7/10 -> LiDAR source
      other modes -> GPS/DR source

    PAD:
      /pad/takeover=True gives the gamepad final command ownership.
      Switching source always applies a STOP hold.
      Missing/stale PAD heartbeat keeps PAD ownership and holds STOP;
      it never silently falls back to AUTO.
    """

    def __init__(self):
        super().__init__('simple_mcu_command_adapter')
        for name, value in {
            'gps_drive_topic': '/cmd_drive',
            'gps_wheel_topic': '/cmd_wheel',
            'lidar_drive_topic': '/lidar_drive',
            'lidar_wheel_topic': '/lidar_wheel',
            'mode_topic': '/drive_mode',
            'pad_takeover_topic': '/pad/takeover',
            'pad_drive_topic': '/pad/cmd_drive',
            'pad_wheel_topic': '/pad/cmd_wheel',
            'pad_stop_topic': '/pad/cmd_stop',
            'pad_heartbeat_topic': '/pad/heartbeat',
            'control_mode_topic': '/control/mode',
            'mcu_drive_topic': '/mcu/cmd_drive',
            'mcu_wheel_topic': '/mcu/cmd_wheel',
            'mcu_stop_topic': '/mcu/cmd_stop',
            'command_timeout_s': 0.35,
            'pad_heartbeat_timeout_s': 0.75,
            'switch_hold_s': 0.30,
            'publish_rate_hz': 20.0,
            'max_steer_deg': 22,
        }.items():
            self.declare_parameter(name, value)

        gp = lambda name: self.get_parameter(name).value
        self.timeout = max(0.10, float(gp('command_timeout_s')))
        self.pad_timeout = max(0.10, float(gp('pad_heartbeat_timeout_s')))
        self.switch_hold = max(0.0, float(gp('switch_hold_s')))
        self.rate = max(5.0, float(gp('publish_rate_hz')))
        self.max_steer = abs(int(gp('max_steer_deg')))

        self.sources = {
            'gps': {'drive': 0.0, 'wheel': 0, 'td': None, 'tw': None},
            'lidar': {'drive': 0.0, 'wheel': 0, 'td': None, 'tw': None},
        }
        self.mode = None

        self.pad_mode = False
        self.pad_drive = 0.0
        self.pad_wheel = 0
        self.pad_stop = True
        self.pad_heartbeat = False
        self.pad_heartbeat_rx = None
        self.switch_until = 0.0
        self.last_control_mode = None
        self.last_warn = 0.0

        self.pub_drive = self.create_publisher(
            Float32, str(gp('mcu_drive_topic')), 10)
        self.pub_wheel = self.create_publisher(
            Int32, str(gp('mcu_wheel_topic')), 10)
        self.pub_stop = self.create_publisher(
            Bool, str(gp('mcu_stop_topic')), 10)
        self.pub_control_mode = self.create_publisher(
            String, str(gp('control_mode_topic')), 10)

        self.create_subscription(
            Float32, str(gp('gps_drive_topic')),
            lambda msg: self.on_drive('gps', msg), 20)
        self.create_subscription(
            Int32, str(gp('gps_wheel_topic')),
            lambda msg: self.on_wheel('gps', msg), 20)
        self.create_subscription(
            Float32, str(gp('lidar_drive_topic')),
            lambda msg: self.on_drive('lidar', msg), 20)
        self.create_subscription(
            Int32, str(gp('lidar_wheel_topic')),
            lambda msg: self.on_wheel('lidar', msg), 20)
        self.create_subscription(
            String, str(gp('mode_topic')), self.on_mode, 20)

        self.create_subscription(
            Bool, str(gp('pad_takeover_topic')), self.on_pad_takeover, 20)
        self.create_subscription(
            Float32, str(gp('pad_drive_topic')), self.on_pad_drive, 20)
        self.create_subscription(
            Int32, str(gp('pad_wheel_topic')), self.on_pad_wheel, 20)
        self.create_subscription(
            Bool, str(gp('pad_stop_topic')), self.on_pad_stop, 20)
        self.create_subscription(
            Bool, str(gp('pad_heartbeat_topic')), self.on_pad_heartbeat, 20)

        self.create_timer(1.0 / self.rate, self.tick)
        self.publish_control_mode(force=True)
        self.get_logger().info(
            'command adapter ready: AUTO modes 7/10=lidar, others=gps; '
            'LB PAD takeover enabled; one /mcu/cmd_* publisher')

    @staticmethod
    def now():
        return time.monotonic()

    def on_mode(self, msg: String):
        try:
            self.mode = int(str(msg.data).strip())
        except ValueError:
            self.mode = None

    def on_drive(self, source, msg: Float32):
        self.sources[source]['drive'] = float(msg.data)
        self.sources[source]['td'] = self.now()

    def on_wheel(self, source, msg: Int32):
        self.sources[source]['wheel'] = max(
            -self.max_steer, min(self.max_steer, int(msg.data)))
        self.sources[source]['tw'] = self.now()

    def on_pad_takeover(self, msg: Bool):
        requested = bool(msg.data)
        if requested == self.pad_mode:
            return
        self.pad_mode = requested
        self.switch_until = self.now() + self.switch_hold
        target = 'PAD' if requested else 'AUTO'
        self.get_logger().warn(
            f'CONTROL TAKEOVER -> {target}; STOP hold {self.switch_hold:.2f}s')
        self.publish_final(
            0.0,
            self.pad_wheel if requested else self.current_auto_wheel(),
            True,
        )
        self.publish_control_mode(force=True)

    def on_pad_drive(self, msg: Float32):
        raw = float(msg.data)
        stage = int(round(raw))
        if abs(raw - stage) <= 1e-3 and stage in (-1, 0, 1, 2, 3):
            self.pad_drive = float(stage)

    def on_pad_wheel(self, msg: Int32):
        self.pad_wheel = max(
            -self.max_steer, min(self.max_steer, int(msg.data)))

    def on_pad_stop(self, msg: Bool):
        self.pad_stop = bool(msg.data)

    def on_pad_heartbeat(self, msg: Bool):
        self.pad_heartbeat = bool(msg.data)
        self.pad_heartbeat_rx = self.now()

    def current_auto_owner(self):
        if self.mode in (7, 10):
            return 'lidar'
        return 'gps'

    def current_auto_wheel(self):
        return int(self.sources[self.current_auto_owner()]['wheel'])

    def publish_control_mode(self, force=False):
        text = 'PAD' if self.pad_mode else 'AUTO'
        if force or text != self.last_control_mode:
            self.pub_control_mode.publish(String(data=text))
            self.last_control_mode = text

    def publish_final(self, drive, wheel, stop):
        wheel = max(-self.max_steer, min(self.max_steer, int(wheel)))
        self.pub_stop.publish(Bool(data=bool(stop)))
        self.pub_wheel.publish(Int32(data=wheel))
        self.pub_drive.publish(Float32(data=float(drive)))

    def publish_stop(self, wheel=0):
        self.publish_final(0.0, wheel, True)

    def warn_throttled(self, message):
        now = self.now()
        if now - self.last_warn >= 1.0:
            self.get_logger().warn(message)
            self.last_warn = now

    def tick(self):
        now = self.now()
        self.publish_control_mode()

        if now < self.switch_until:
            wheel = self.pad_wheel if self.pad_mode else self.current_auto_wheel()
            self.publish_stop(wheel)
            return

        if self.pad_mode:
            heartbeat_fresh = (
                self.pad_heartbeat
                and self.pad_heartbeat_rx is not None
                and now - self.pad_heartbeat_rx <= self.pad_timeout
            )
            if not heartbeat_fresh:
                self.publish_stop(self.pad_wheel)
                self.warn_throttled(
                    'PAD ownership active but heartbeat missing/stale -> '
                    'STOP held; no AUTO fallback')
                return

            drive = float(self.pad_drive)
            if drive not in (-1.0, 0.0, 1.0, 2.0, 3.0):
                self.publish_stop(self.pad_wheel)
                return

            self.publish_final(drive, self.pad_wheel, self.pad_stop)
            return

        if self.mode is None:
            self.publish_stop(self.current_auto_wheel())
            return

        owner = self.current_auto_owner()
        source = self.sources[owner]
        if (
            source['td'] is None
            or source['tw'] is None
            or now - source['td'] > self.timeout
            or now - source['tw'] > self.timeout
        ):
            self.publish_stop(source['wheel'])
            self.warn_throttled(
                f'AUTO {owner} command stale/missing -> STOP held')
            return

        drive = float(source['drive'])
        wheel = int(source['wheel'])
        if drive not in (-1.0, 0.0, 1.0, 2.0, 3.0):
            self.publish_stop(wheel)
            return

        if abs(drive) < 0.1:
            self.publish_stop(wheel)
            return

        self.publish_final(drive, wheel, False)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMcuCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
