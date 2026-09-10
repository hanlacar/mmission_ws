#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TParkingLidarSelector(Node):
    """
    Bench/simulation T-parking slot selector.

    Reads /scan_rear and checks two configurable rectangular ROIs in the
    LaserScan frame.  It only decides while current_segment == T_foword.

    Decision:
      A occupied, B free  -> T_B
      B occupied, A free  -> T_A
      both free           -> default_free_branch (default T_A)
      both occupied       -> no branch selected

    A selection must be stable for confirmation_frames consecutive scans and
    is locally latched after publishing /dr_branch/select once.
    """

    def __init__(self):
        super().__init__("t_parking_lidar_selector")

        self.declare_parameter("scan_topic", "/scan_rear")
        self.declare_parameter("segment_topic", "/dr_navigation/current_segment")
        self.declare_parameter("branch_select_topic", "/dr_branch/select")
        self.declare_parameter("status_topic", "/t_parking/lidar_selector/status")

        self.declare_parameter("active_segment", "T_foword")
        self.declare_parameter("branch_a", "T_A")
        self.declare_parameter("branch_b", "T_B")
        self.declare_parameter("default_free_branch", "T_A")

        # Simulation defaults.  rear_laser frame follows ROS convention:
        # x forward, y left.  Therefore A ROI is configured on y<0 and B on y>0.
        # Actual vehicle values must be calibrated from real /scan_rear.
        self.declare_parameter("a_x_min", -1.40)
        self.declare_parameter("a_x_max", -0.35)
        self.declare_parameter("a_y_min", -0.80)
        self.declare_parameter("a_y_max", -0.15)

        self.declare_parameter("b_x_min", -1.40)
        self.declare_parameter("b_x_max", -0.35)
        self.declare_parameter("b_y_min", 0.15)
        self.declare_parameter("b_y_max", 0.80)

        self.declare_parameter("min_valid_range_m", 0.08)
        self.declare_parameter("max_valid_range_m", 2.0)
        self.declare_parameter("occupied_min_points", 3)
        self.declare_parameter("confirmation_frames", 3)

        gp = lambda n: self.get_parameter(n).value

        self.scan_topic = str(gp("scan_topic"))
        self.segment_topic = str(gp("segment_topic"))
        self.select_topic = str(gp("branch_select_topic"))
        self.status_topic = str(gp("status_topic"))

        self.active_segment = str(gp("active_segment"))
        self.branch_a = str(gp("branch_a"))
        self.branch_b = str(gp("branch_b"))
        self.default_free = str(gp("default_free_branch"))

        self.a_roi = (
            float(gp("a_x_min")), float(gp("a_x_max")),
            float(gp("a_y_min")), float(gp("a_y_max")),
        )
        self.b_roi = (
            float(gp("b_x_min")), float(gp("b_x_max")),
            float(gp("b_y_min")), float(gp("b_y_max")),
        )

        self.rmin = float(gp("min_valid_range_m"))
        self.rmax = float(gp("max_valid_range_m"))
        self.min_points = max(1, int(gp("occupied_min_points")))
        self.confirm_frames = max(1, int(gp("confirmation_frames")))

        self.current_segment = ""
        self.last_candidate = None
        self.candidate_count = 0
        self.latched_branch = None

        self.pub_select = self.create_publisher(String, self.select_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(String, self.segment_topic, self._segment_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_service(
            Trigger,
            "/t_parking/lidar_selector/reset",
            self._reset_cb,
        )

        self.get_logger().info(
            f"T parking LiDAR selector ready: scan={self.scan_topic}, "
            f"active={self.active_segment}, confirm={self.confirm_frames}"
        )

    @staticmethod
    def _inside(x, y, roi):
        xmin, xmax, ymin, ymax = roi
        return xmin <= x <= xmax and ymin <= y <= ymax

    def _clear_decision(self):
        self.last_candidate = None
        self.candidate_count = 0
        self.latched_branch = None

    def _segment_cb(self, msg):
        new_segment = str(msg.data).strip()
        old_segment = self.current_segment

        if new_segment != old_segment:
            self.current_segment = new_segment
            self.last_candidate = None
            self.candidate_count = 0

            # follower reset/새 T 주차 시작:
            # T_A 또는 T_B에서 T_foword로 돌아오면 이전 LiDAR 선택 latch를 해제.
            if (
                new_segment == self.active_segment
                and old_segment
                and old_segment != self.active_segment
            ):
                self.latched_branch = None
                self.get_logger().info(
                    f"new T parking run: {old_segment} -> {new_segment}; "
                    "LiDAR branch latch cleared"
                )

    def _reset_cb(self, request, response):
        del request
        self._clear_decision()
        response.success = True
        response.message = "T parking LiDAR selector reset"
        self.get_logger().info("LiDAR selector latch/counter RESET")
        return response

    def _scan_cb(self, scan):
        if self.current_segment != self.active_segment:
            return

        if self.latched_branch is not None:
            self._publish_status(0, 0, False, False, self.latched_branch, "LATCHED")
            return

        a_count = 0
        b_count = 0

        angle = float(scan.angle_min)
        for rr in scan.ranges:
            r = float(rr)
            if math.isfinite(r) and self.rmin <= r <= self.rmax:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                if self._inside(x, y, self.a_roi):
                    a_count += 1
                if self._inside(x, y, self.b_roi):
                    b_count += 1

            angle += float(scan.angle_increment)

        a_occ = a_count >= self.min_points
        b_occ = b_count >= self.min_points

        if a_occ and not b_occ:
            candidate = self.branch_b
            reason = "A_OCCUPIED"
        elif b_occ and not a_occ:
            candidate = self.branch_a
            reason = "B_OCCUPIED"
        elif not a_occ and not b_occ:
            candidate = self.default_free
            reason = "BOTH_FREE"
        else:
            candidate = None
            reason = "BOTH_OCCUPIED"

        if candidate is None:
            self.last_candidate = None
            self.candidate_count = 0
            self._publish_status(
                a_count, b_count, a_occ, b_occ, "NONE", reason
            )
            return

        if candidate == self.last_candidate:
            self.candidate_count += 1
        else:
            self.last_candidate = candidate
            self.candidate_count = 1

        self._publish_status(
            a_count, b_count, a_occ, b_occ,
            candidate,
            f"{reason} confirm={self.candidate_count}/{self.confirm_frames}"
        )

        if self.candidate_count >= self.confirm_frames:
            self.latched_branch = candidate
            self.pub_select.publish(String(data=candidate))
            self.get_logger().info(
                f"AUTO SELECT -> {candidate} "
                f"(A_points={a_count}, B_points={b_count}, reason={reason})"
            )

    def _publish_status(self, ac, bc, ao, bo, candidate, reason):
        text = (
            f"segment={self.current_segment or 'NONE'} "
            f"A_points={ac} A_occ={int(ao)} "
            f"B_points={bc} B_occ={int(bo)} "
            f"candidate={candidate} "
            f"latched={self.latched_branch or 'NONE'} "
            f"{reason}"
        )
        self.pub_status.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = TParkingLidarSelector()
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
