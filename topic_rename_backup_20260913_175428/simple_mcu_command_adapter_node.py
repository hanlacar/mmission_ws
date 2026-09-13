#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class SimpleMcuCommandAdapter(Node):
    """Single command owner: DR follower gps/lidar outputs -> SIMPLE MCU inputs."""

    def __init__(self):
        super().__init__('simple_mcu_command_adapter')
        for n, v in {
            'gps_drive_topic': '/gps_drive',
            'gps_wheel_topic': '/gps_wheel',
            'lidar_drive_topic': '/lidar_drive',
            'lidar_wheel_topic': '/lidar_wheel',
            'mode_topic': '/drive_mode',
            'mcu_drive_topic': '/mcu/cmd_drive',
            'mcu_wheel_topic': '/mcu/cmd_wheel',
            'mcu_stop_topic': '/mcu/cmd_stop',
            'command_timeout_s': 0.35,
            'publish_rate_hz': 20.0,
            'max_steer_deg': 22,
        }.items():
            self.declare_parameter(n, v)
        gp = lambda n: self.get_parameter(n).value
        self.timeout = max(0.10, float(gp('command_timeout_s')))
        self.rate = max(5.0, float(gp('publish_rate_hz')))
        self.max_steer = abs(int(gp('max_steer_deg')))

        self.sources = {
            'gps': {'drive': 0.0, 'wheel': 0, 'td': None, 'tw': None},
            'lidar': {'drive': 0.0, 'wheel': 0, 'td': None, 'tw': None},
        }
        self.mode = None

        self.pub_drive = self.create_publisher(Float32, str(gp('mcu_drive_topic')), 10)
        self.pub_wheel = self.create_publisher(Int32, str(gp('mcu_wheel_topic')), 10)
        self.pub_stop = self.create_publisher(Bool, str(gp('mcu_stop_topic')), 10)

        self.create_subscription(Float32, str(gp('gps_drive_topic')), lambda m: self.on_drive('gps', m), 20)
        self.create_subscription(Int32, str(gp('gps_wheel_topic')), lambda m: self.on_wheel('gps', m), 20)
        self.create_subscription(Float32, str(gp('lidar_drive_topic')), lambda m: self.on_drive('lidar', m), 20)
        self.create_subscription(Int32, str(gp('lidar_wheel_topic')), lambda m: self.on_wheel('lidar', m), 20)
        self.create_subscription(String, str(gp('mode_topic')), self.on_mode, 20)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info('command adapter ready: modes 7/10=lidar, others=gps; one /mcu/cmd_* publisher')

    def on_mode(self, msg: String):
        try:
            self.mode = int(str(msg.data).strip())
        except ValueError:
            self.mode = None

    def on_drive(self, source, msg: Float32):
        self.sources[source]['drive'] = float(msg.data)
        self.sources[source]['td'] = time.monotonic()

    def on_wheel(self, source, msg: Int32):
        self.sources[source]['wheel'] = max(-self.max_steer, min(self.max_steer, int(msg.data)))
        self.sources[source]['tw'] = time.monotonic()

    def publish_stop(self):
        self.pub_stop.publish(Bool(data=True))
        self.pub_drive.publish(Float32(data=0.0))

    def tick(self):
        if self.mode is None:
            self.publish_stop()
            return
        owner = 'lidar' if self.mode in (7, 10) else 'gps'
        s = self.sources[owner]
        now = time.monotonic()
        if s['td'] is None or s['tw'] is None or now - s['td'] > self.timeout or now - s['tw'] > self.timeout:
            self.publish_stop()
            return

        drive = float(s['drive'])
        wheel = int(s['wheel'])
        if drive not in (-1.0, 0.0, 1.0, 2.0, 3.0):
            self.publish_stop()
            return

        if abs(drive) < 0.1:
            self.pub_stop.publish(Bool(data=True))
            self.pub_drive.publish(Float32(data=0.0))
            self.pub_wheel.publish(Int32(data=wheel))
            return

        # Release stop before forwarding active command; repeated at 20 Hz.
        self.pub_stop.publish(Bool(data=False))
        self.pub_wheel.publish(Int32(data=wheel))
        self.pub_drive.publish(Float32(data=drive))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMcuCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Last fail-safe stop pulse.
        node.pub_stop.publish(Bool(data=True))
        node.pub_drive.publish(Float32(data=0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
