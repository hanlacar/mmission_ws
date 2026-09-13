#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class IntersectionTimeoutBridge(Node):
    """
    Compatibility bridge for SIMPLE MCU stack.

    dr_real_segmented_follower already owns the 3 s timeout and publishes:
      /dr/intersection_timeout_release = True

    The old MCU manager consumed that signal.  SIMPLE MCU has no manager,
    so relay the timeout back into the follower's normal release input:
      /mission/intersection_go = True

    This does NOT command the motors directly.
    """

    def __init__(self):
        super().__init__('intersection_timeout_bridge')

        self.declare_parameter(
            'timeout_topic', '/dr/intersection_timeout_release')
        self.declare_parameter(
            'intersection_go_topic', '/mission/intersection_go')
        self.declare_parameter('mode_topic', '/drive_mode')
        self.declare_parameter('pulse_sec', 0.10)

        timeout_topic = str(self.get_parameter('timeout_topic').value)
        go_topic = str(self.get_parameter('intersection_go_topic').value)
        mode_topic = str(self.get_parameter('mode_topic').value)
        self.pulse_sec = max(
            0.05, float(self.get_parameter('pulse_sec').value))

        self.mode = None
        self.relay_armed = True
        self.reset_timer = None

        self.pub_go = self.create_publisher(Bool, go_topic, 10)
        self.create_subscription(
            Bool, timeout_topic, self.on_timeout, 10)
        self.create_subscription(
            String, mode_topic, self.on_mode, 10)

        self.get_logger().info(
            f'intersection timeout bridge ready: '
            f'{timeout_topic} -> {go_topic}; modes 4/6 only')

    def on_mode(self, msg: String):
        new_mode = str(msg.data).strip()
        if new_mode != self.mode:
            self.relay_armed = True
        self.mode = new_mode

    def on_timeout(self, msg: Bool):
        if not bool(msg.data):
            self.relay_armed = True
            return

        if self.mode not in ('4', '6'):
            self.get_logger().warn(
                f'ignored timeout release outside intersection mode: '
                f'mode={self.mode!r}')
            return

        if not self.relay_armed:
            return

        self.relay_armed = False
        self.pub_go.publish(Bool(data=True))
        self.get_logger().warn(
            f'3 s intersection timeout: releasing follower in mode {self.mode}')

        if self.reset_timer is not None:
            self.reset_timer.cancel()
        self.reset_timer = self.create_timer(
            self.pulse_sec, self.publish_false_once)

    def publish_false_once(self):
        self.pub_go.publish(Bool(data=False))
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
            self.reset_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = IntersectionTimeoutBridge()
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
