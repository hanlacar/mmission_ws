#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class FakeRearLidar(Node):
    """
    Synthetic rear LaserScan for T-parking branch tests.

    /test/parking/block accepts:
      A     -> obstacle in A ROI
      B     -> obstacle in B ROI
      BOTH  -> obstacle in both
      NONE  -> both empty

    Default is BOTH so the selector does not choose a branch until the tester
    explicitly chooses a scenario.
    """

    def __init__(self):
        super().__init__("fake_rear_lidar")

        self.declare_parameter("scan_topic", "/scan_rear")
        self.declare_parameter("scenario_topic", "/test/parking/block")
        self.declare_parameter("status_topic", "/test/parking/status")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("frame_id", "rear_laser")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("default_scenario", "BOTH")

        self.declare_parameter("a_center_x", -0.85)
        self.declare_parameter("a_center_y", -0.45)
        self.declare_parameter("b_center_x", -0.85)
        self.declare_parameter("b_center_y", 0.45)

        gp = lambda n: self.get_parameter(n).value
        self.scan_topic = str(gp("scan_topic"))
        self.scenario_topic = str(gp("scenario_topic"))
        self.status_topic = str(gp("status_topic"))
        self.odom_topic = str(gp("odom_topic"))
        self.frame_id = str(gp("frame_id"))
        hz = max(1.0, float(gp("publish_hz")))

        self.a_center = (
            float(gp("a_center_x")),
            float(gp("a_center_y")),
        )
        self.b_center = (
            float(gp("b_center_x")),
            float(gp("b_center_y")),
        )

        self.scenario = str(gp("default_scenario")).upper()
        self.last_odom = None

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(String, self.scenario_topic, self._scenario_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"Fake rear LiDAR ready. Default={self.scenario}; "
            f"control topic={self.scenario_topic}"
        )

    def _scenario_cb(self, msg):
        value = str(msg.data).strip().upper()
        if value not in {"A", "B", "BOTH", "NONE"}:
            self.get_logger().error(
                f"invalid scenario={value}; use A|B|BOTH|NONE"
            )
            return
        self.scenario = value
        self.get_logger().info(f"fake parking obstacle={self.scenario}")

    def _odom_cb(self, msg):
        self.last_odom = msg

    @staticmethod
    def _quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _publish_tf(self):
        if self.last_odom is None:
            return

        msg = self.last_odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = self.frame_id

        # For bench simulation, rear_laser origin is coincident with base_link.
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = 0.20
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.sendTransform(t)

    @staticmethod
    def _cluster(cx, cy):
        # Multiple physical points ensure occupied_min_points is exceeded.
        pts = []
        for dx in (-0.08, -0.04, 0.0, 0.04, 0.08):
            for dy in (-0.05, 0.0, 0.05):
                pts.append((cx + dx, cy + dy))
        return pts

    def _tick(self):
        self._publish_tf()

        n = 720
        amin = -math.pi
        amax = math.pi
        inc = (amax - amin) / n
        ranges = [float("inf")] * n

        obstacles = []
        if self.scenario in ("A", "BOTH"):
            obstacles.extend(self._cluster(*self.a_center))
        if self.scenario in ("B", "BOTH"):
            obstacles.extend(self._cluster(*self.b_center))

        for x, y in obstacles:
            r = math.hypot(x, y)
            a = math.atan2(y, x)
            idx = int(round((a - amin) / inc))
            if 0 <= idx < n:
                ranges[idx] = min(ranges[idx], r)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = amin
        scan.angle_max = amax
        scan.angle_increment = inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 8.0
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        self.status_pub.publish(
            String(data=f"blocked={self.scenario}")
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeRearLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
