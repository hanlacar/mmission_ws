"""Test-only publisher that mimics the mission-facing imu_ws topic contract."""
import math
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from .imu_heading_estimator import normalize_angle


class TestImuPublisher(Node):
    __test__ = False
    def __init__(self) -> None:
        super().__init__('gps_test_imu_publisher')
        self.heading = None
        self.reference = None
        self.previous = None
        self.previous_time = None
        self.drive = 1.0
        self.yaw_pub = self.create_publisher(Float32, '/imu_yaw', 10)
        self.rate_pub = self.create_publisher(Float32, '/imu_yaw_rate', 10)
        self.valid_pub = self.create_publisher(Bool, '/imu_valid', 10)
        self.create_subscription(TwistWithCovarianceStamped, '/vel', self._velocity, 10)
        self.create_subscription(Float32, '/gps_drive', self._drive, 10)
        self.create_timer(0.02, self._publish)

    def _velocity(self, msg: TwistWithCovarianceStamped) -> None:
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        if math.hypot(vx, vy) > 0.02:
            self.heading = normalize_angle(math.atan2(vy, vx) + (math.pi if self.drive < 0 else 0.0))
            if self.reference is None:
                self.reference = self.heading

    def _drive(self, msg: Float32) -> None:
        if abs(msg.data) > 0.01:
            self.drive = float(msg.data)

    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        relative = 0.0 if self.heading is None or self.reference is None else normalize_angle(
            self.heading - self.reference)
        rate = 0.0
        if self.previous is not None and self.previous_time is not None and now > self.previous_time:
            rate = math.degrees(normalize_angle(relative - self.previous)) / (now - self.previous_time)
        self.previous, self.previous_time = relative, now
        self.yaw_pub.publish(Float32(data=float(math.degrees(relative))))
        self.rate_pub.publish(Float32(data=float(rate)))
        self.valid_pub.publish(Bool(data=True))


def main() -> None:
    rclpy.init(); node = TestImuPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
