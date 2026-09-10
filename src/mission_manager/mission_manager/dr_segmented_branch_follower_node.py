#!/usr/bin/env python3
import csv
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class DrSegmentedBranchFollower(Node):
    """
    T parking branch test:
      T_foword -> (T_A or T_B)

    - One segmented network CSV contains all three segments.
    - /dr_branch/select accepts "T_A" or "T_B".
    - Selection is latched until /dr_route/reset.
    - If T_foword finishes before a branch is selected, the vehicle stops and waits.
    - Same rigid transform is applied to every segment at start, preserving branch geometry.
    """

    def __init__(self):
        super().__init__("dr_segmented_branch_follower")

        # Input / output
        self.declare_parameter("network_path", "")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("drive_topic", "/gps_drive")
        self.declare_parameter("wheel_topic", "/gps_wheel")
        self.declare_parameter("status_topic", "/dr_navigation/status")
        self.declare_parameter("branch_select_topic", "/dr_branch/select")
        self.declare_parameter("branch_selected_topic", "/dr_branch/selected")
        self.declare_parameter("current_segment_topic", "/dr_navigation/current_segment")

        # Segments
        self.declare_parameter("forward_segment", "T_foword")
        self.declare_parameter("branch_a_segment", "T_A")
        self.declare_parameter("branch_b_segment", "T_B")

        # T870
        self.declare_parameter("wheelbase_m", 0.73)
        self.declare_parameter("max_steer_deg", 22.0)
        self.declare_parameter("steering_sign", 1)

        # Pure Pursuit
        self.declare_parameter("lookahead_forward_m", 0.80)
        self.declare_parameter("lookahead_reverse_m", 0.60)
        self.declare_parameter("goal_tolerance_m", 0.30)
        self.declare_parameter("segment_switch_tolerance_m", 0.40)
        self.declare_parameter("off_route_warn_m", 1.0)
        self.declare_parameter("off_route_stop_m", 2.0)
        self.declare_parameter("odom_timeout_s", 0.5)
        self.declare_parameter("align_route_to_start", True)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("search_window_points", 100)

        gp = lambda n: self.get_parameter(n).value
        self.network_path = os.path.expanduser(str(gp("network_path")))
        if not self.network_path:
            raise RuntimeError("network_path가 비어 있습니다.")

        self.odom_topic = str(gp("odom_topic"))
        self.drive_topic = str(gp("drive_topic"))
        self.wheel_topic = str(gp("wheel_topic"))
        self.status_topic = str(gp("status_topic"))
        self.branch_select_topic = str(gp("branch_select_topic"))
        self.branch_selected_topic = str(gp("branch_selected_topic"))
        self.current_segment_topic = str(gp("current_segment_topic"))

        self.forward_id = str(gp("forward_segment"))
        self.branch_a_id = str(gp("branch_a_segment"))
        self.branch_b_id = str(gp("branch_b_segment"))
        self.valid_branches = {self.branch_a_id, self.branch_b_id}

        self.wheelbase = float(gp("wheelbase_m"))
        self.max_steer_deg = abs(float(gp("max_steer_deg")))
        self.steering_sign = int(gp("steering_sign"))
        self.lookahead_fwd = float(gp("lookahead_forward_m"))
        self.lookahead_rev = float(gp("lookahead_reverse_m"))
        self.goal_tol = float(gp("goal_tolerance_m"))
        self.switch_tol = float(gp("segment_switch_tolerance_m"))
        self.off_warn = float(gp("off_route_warn_m"))
        self.off_stop = float(gp("off_route_stop_m"))
        self.odom_timeout = float(gp("odom_timeout_s"))
        self.align_to_start = bool(gp("align_route_to_start"))
        self.started = bool(gp("auto_start"))
        self.search_window = max(20, int(gp("search_window_points")))

        self.raw_segments = self._load_network(Path(self.network_path))
        for sid in [self.forward_id, self.branch_a_id, self.branch_b_id]:
            if sid not in self.raw_segments:
                raise RuntimeError(f"segment 없음: {sid}")

        self.segments = self._copy_segments(self.raw_segments)
        self.route_aligned = False

        self.current_segment_id = self.forward_id
        self.cursor = 0
        self.selected_branch = None
        self.finished = False
        self.waiting_for_branch = False

        self.last_odom_time = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.actual_trace = []

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.drive_pub = self.create_publisher(Float32, self.drive_topic, 10)
        self.wheel_pub = self.create_publisher(Int32, self.wheel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.branch_pub = self.create_publisher(
            String, self.branch_selected_topic, latched
        )
        self.segment_pub = self.create_publisher(
            String, self.current_segment_topic, latched
        )

        # RViz
        self.fwd_path_pub = self.create_publisher(
            NavPath, "/dr_branch_viz/t_foword", latched
        )
        self.a_path_pub = self.create_publisher(
            NavPath, "/dr_branch_viz/t_a", latched
        )
        self.b_path_pub = self.create_publisher(
            NavPath, "/dr_branch_viz/t_b", latched
        )
        self.active_path_pub = self.create_publisher(
            NavPath, "/dr_branch_viz/active_path", latched
        )
        self.actual_path_pub = self.create_publisher(
            NavPath, "/dr_branch_viz/actual_path", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/dr_branch_viz/markers", 10
        )

        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 30)
        self.create_subscription(
            String, self.branch_select_topic, self._on_branch_select, 10
        )

        self.create_service(Trigger, "/dr_route/start", self._on_start)
        self.create_service(Trigger, "/dr_route/stop", self._on_stop)
        self.create_service(Trigger, "/dr_route/reset", self._on_reset)
        self.create_service(Trigger, "/dr_branch/clear", self._on_clear_branch)

        self.create_timer(0.1, self._watchdog)
        self.create_timer(0.1, self._publish_viz)

        self._publish_stop()
        self._publish_branch_state()
        self._publish_segment_state()

        self.get_logger().info(
            f"segmented DR ready: {self.forward_id} -> "
            f"({self.branch_a_id}|{self.branch_b_id})"
        )
        self.get_logger().info(
            f"branch select: ros2 topic pub --once {self.branch_select_topic} "
            "std_msgs/msg/String \"{data: 'T_A'}\""
        )

    # -----------------------------------------------------------------
    # CSV / geometry
    # -----------------------------------------------------------------
    def _load_network(self, path):
        if not path.is_file():
            raise RuntimeError(f"network CSV 없음: {path}")

        segs = {}
        with path.open(newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            required = {
                "segment_id", "point_index", "x_m", "y_m",
                "direction", "mode", "drive_level"
            }
            names = set(reader.fieldnames or [])
            missing = required - names
            if missing:
                raise RuntimeError(f"segmented CSV 필수 컬럼 없음: {sorted(missing)}")

            for row in reader:
                sid = str(row["segment_id"]).strip()
                p = {
                    "point_index": int(float(row["point_index"])),
                    "x": float(row["x_m"]),
                    "y": float(row["y_m"]),
                    "direction": 1 if int(float(row["direction"])) >= 0 else -1,
                    "mode": int(float(row.get("mode", 1) or 1)),
                    "drive_level": float(row.get("drive_level", 2.0) or 2.0),
                    "yaw": 0.0,
                }
                segs.setdefault(sid, []).append(p)

        for sid, pts in segs.items():
            pts.sort(key=lambda p: p["point_index"])
            self._derive_yaw(pts)

        return segs

    @staticmethod
    def _derive_yaw(pts):
        if not pts:
            return
        if len(pts) == 1:
            pts[0]["yaw"] = 0.0
            return
        for i in range(len(pts) - 1):
            dx = pts[i + 1]["x"] - pts[i]["x"]
            dy = pts[i + 1]["y"] - pts[i]["y"]
            if abs(dx) + abs(dy) > 1e-9:
                pts[i]["yaw"] = math.atan2(dy, dx)
            elif i > 0:
                pts[i]["yaw"] = pts[i - 1]["yaw"]
        pts[-1]["yaw"] = pts[-2]["yaw"]

    @staticmethod
    def _copy_segments(src):
        return {
            sid: [dict(p) for p in pts]
            for sid, pts in src.items()
        }

    def _align_all_segments(self):
        if self.route_aligned:
            return

        self.segments = self._copy_segments(self.raw_segments)

        if not self.align_to_start:
            self.route_aligned = True
            self._publish_candidate_paths()
            return

        first = self.segments[self.forward_id][0]
        yaw_offset = norm_angle(self.current_yaw - first["yaw"])
        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        x0 = first["x"]
        y0 = first["y"]

        for pts in self.segments.values():
            for p in pts:
                dx = p["x"] - x0
                dy = p["y"] - y0
                p["x"] = self.current_x + c * dx - s * dy
                p["y"] = self.current_y + s * dx + c * dy
                p["yaw"] = norm_angle(p["yaw"] + yaw_offset)

        self.route_aligned = True
        self._publish_candidate_paths()
        self.get_logger().info(
            "T_foword 시작점을 현재 /odom 위치/방향에 맞춰 모든 분기를 동일 변환했습니다."
        )

    # -----------------------------------------------------------------
    # Services / branch selection
    # -----------------------------------------------------------------
    def _on_start(self, request, response):
        del request
        if self.finished:
            response.success = False
            response.message = "완료 상태입니다. /dr_route/reset 후 다시 시작하세요."
            return response

        if self.last_odom_time is None:
            response.success = False
            response.message = "/odom 수신 없음"
            return response

        self._align_all_segments()
        self.started = True
        self.waiting_for_branch = False
        response.success = True
        response.message = (
            f"DR 시작: {self.forward_id}"
            + (f" -> {self.selected_branch}" if self.selected_branch else " -> branch 대기")
        )
        return response

    def _on_stop(self, request, response):
        del request
        self.started = False
        self._publish_stop()
        response.success = True
        response.message = "DR 정지"
        return response

    def _on_reset(self, request, response):
        del request
        self.started = False
        self.finished = False
        self.waiting_for_branch = False
        self.current_segment_id = self.forward_id
        self.cursor = 0
        self.selected_branch = None
        self.route_aligned = False
        self.segments = self._copy_segments(self.raw_segments)
        self.actual_trace.clear()
        self._publish_stop()
        self._publish_branch_state()
        self._publish_segment_state()
        response.success = True
        response.message = "DR segmented follower reset"
        return response

    def _on_clear_branch(self, request, response):
        del request
        if self.current_segment_id != self.forward_id:
            response.success = False
            response.message = "이미 branch에 진입했습니다. /dr_route/reset을 사용하세요."
            return response
        self.selected_branch = None
        self._publish_branch_state()
        response.success = True
        response.message = "branch 선택 해제"
        return response

    def _on_branch_select(self, msg):
        requested = str(msg.data).strip()

        if requested not in self.valid_branches:
            self.get_logger().error(
                f"잘못된 branch '{requested}'. 허용: "
                f"{self.branch_a_id}, {self.branch_b_id}"
            )
            return

        if self.selected_branch is not None:
            if requested != self.selected_branch:
                self.get_logger().warn(
                    f"branch 이미 latch됨: {self.selected_branch}; "
                    f"{requested} 요청 무시"
                )
            return

        if self.current_segment_id != self.forward_id:
            self.get_logger().warn(
                f"이미 {self.current_segment_id} 진입 후 branch 선택 요청 무시"
            )
            return

        self.selected_branch = requested
        self._publish_branch_state()
        self.get_logger().info(f"branch latch: {requested}")

        # T_foword 끝에서 기다리고 있었다면 바로 branch로 전환
        if self.waiting_for_branch and self.started:
            self._switch_to_selected_branch()

    # -----------------------------------------------------------------
    # Odom / follow
    # -----------------------------------------------------------------
    def _on_odom(self, msg):
        self.last_odom_time = time.monotonic()
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

        self.actual_trace.append((self.current_x, self.current_y))
        if len(self.actual_trace) > 15000:
            del self.actual_trace[:5000]

        if not self.started or self.finished:
            return

        if not self.route_aligned:
            self._align_all_segments()

        self._follow_current_segment()

    def _follow_current_segment(self):
        pts = self.segments[self.current_segment_id]
        if not pts:
            self._fail_stop("EMPTY_SEGMENT")
            return

        x, y, yaw = self.current_x, self.current_y, self.current_yaw

        search_end = min(len(pts), self.cursor + self.search_window)
        nearest_idx = self.cursor
        nearest_dist = float("inf")

        for i in range(self.cursor, search_end):
            d = math.hypot(pts[i]["x"] - x, pts[i]["y"] - y)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        if nearest_idx > self.cursor:
            self.cursor = nearest_idx

        if nearest_dist > self.off_stop:
            self._publish_stop()
            self._status(
                f"OFF_ROUTE_STOP segment={self.current_segment_id} "
                f"idx={self.cursor} dist={nearest_dist:.2f}"
            )
            return

        # Segment end
        end = pts[-1]
        end_dist = math.hypot(end["x"] - x, end["y"] - y)
        if self.cursor >= len(pts) - 2 and end_dist <= (
            self.switch_tol if self.current_segment_id == self.forward_id else self.goal_tol
        ):
            if self.current_segment_id == self.forward_id:
                if self.selected_branch is None:
                    self.waiting_for_branch = True
                    self._publish_stop()
                    self._status(
                        f"WAIT_BRANCH segment={self.forward_id} "
                        f"choose={self.branch_a_id}|{self.branch_b_id}"
                    )
                    return
                self._switch_to_selected_branch()
                return

            self.finished = True
            self.started = False
            self._publish_stop()
            self._status(
                f"GOAL_REACHED segment={self.current_segment_id}"
            )
            return

        current = pts[self.cursor]
        direction = current["direction"]
        lookahead = self.lookahead_fwd if direction >= 0 else self.lookahead_rev

        target_idx = self.cursor
        accumulated = 0.0
        prev = pts[self.cursor]

        for i in range(self.cursor + 1, len(pts)):
            if pts[i]["direction"] != direction:
                break
            accumulated += math.hypot(
                pts[i]["x"] - prev["x"],
                pts[i]["y"] - prev["y"],
            )
            target_idx = i
            prev = pts[i]
            if accumulated >= lookahead:
                break

        target = pts[target_idx]

        # Reverse Pure Pursuit: use virtual heading +pi and signed velocity.
        control_yaw = yaw if direction >= 0 else norm_angle(yaw + math.pi)
        dx = target["x"] - x
        dy = target["y"] - y
        alpha = norm_angle(math.atan2(dy, dx) - control_yaw)
        ld = max(0.05, math.hypot(dx, dy))

        curvature = 2.0 * math.sin(alpha) / ld
        steer = math.atan(self.wheelbase * curvature)
        steer_deg = math.degrees(steer)

        # For physical steering while reversing, reverse the sign relative to
        # virtual forward heading.
        if direction < 0:
            steer_deg = -steer_deg

        steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))
        steer_deg *= self.steering_sign

        if direction < 0:
            drive = -1.0
        else:
            drive = max(1.0, min(3.0, float(current["drive_level"])))
            if abs(steer_deg) >= 18.0:
                drive = 1.0

        self.drive_pub.publish(Float32(data=float(drive)))
        self.wheel_pub.publish(Int32(data=int(round(steer_deg))))

        state = "OFF_ROUTE_WARN" if nearest_dist > self.off_warn else "TRACKING"
        self._status(
            f"{state} segment={self.current_segment_id} "
            f"idx={self.cursor}/{len(pts)-1} dist={nearest_dist:.2f} "
            f"drive={drive:.1f} wheel={int(round(steer_deg))} "
            f"branch={self.selected_branch or 'NONE'}"
        )

    def _switch_to_selected_branch(self):
        if self.selected_branch not in self.valid_branches:
            self.waiting_for_branch = True
            self._publish_stop()
            return

        self.current_segment_id = self.selected_branch
        self.cursor = 0
        self.waiting_for_branch = False
        self._publish_segment_state()
        self._publish_active_path()

        self.get_logger().info(
            f"segment switch: {self.forward_id} -> {self.current_segment_id}"
        )

    def _watchdog(self):
        if not self.started or self.finished:
            return
        if self.last_odom_time is None:
            self._publish_stop()
            return
        age = time.monotonic() - self.last_odom_time
        if age > self.odom_timeout:
            self._publish_stop()
            self._status(f"ODOM_TIMEOUT age={age:.2f}")

    # -----------------------------------------------------------------
    # Output / status
    # -----------------------------------------------------------------
    def _publish_stop(self):
        self.drive_pub.publish(Float32(data=0.0))
        self.wheel_pub.publish(Int32(data=0))

    def _status(self, text):
        self.status_pub.publish(String(data=str(text)))

    def _publish_branch_state(self):
        self.branch_pub.publish(String(data=self.selected_branch or "NONE"))

    def _publish_segment_state(self):
        self.segment_pub.publish(String(data=self.current_segment_id))

    # -----------------------------------------------------------------
    # RViz
    # -----------------------------------------------------------------
    def _path_msg(self, pts):
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        for p in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p["x"]
            ps.pose.position.y = p["y"]
            ps.pose.orientation = quat_from_yaw(p["yaw"])
            msg.poses.append(ps)
        return msg

    def _publish_candidate_paths(self):
        if not self.route_aligned:
            return
        self.fwd_path_pub.publish(self._path_msg(self.segments[self.forward_id]))
        self.a_path_pub.publish(self._path_msg(self.segments[self.branch_a_id]))
        self.b_path_pub.publish(self._path_msg(self.segments[self.branch_b_id]))
        self._publish_active_path()

    def _publish_active_path(self):
        if not self.route_aligned:
            return

        pts = []
        if self.current_segment_id == self.forward_id:
            pts.extend(self.segments[self.forward_id])
            if self.selected_branch:
                pts.extend(self.segments[self.selected_branch])
        else:
            pts.extend(self.segments[self.current_segment_id])

        self.active_path_pub.publish(self._path_msg(pts))

    def _publish_actual_path(self):
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        for x, y in self.actual_trace:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.actual_path_pub.publish(msg)

    def _publish_viz(self):
        if self.last_odom_time is not None and not self.route_aligned:
            # preview before /dr_route/start
            self._align_all_segments()

        if self.route_aligned:
            self._publish_candidate_paths()
        self._publish_actual_path()

        if self.last_odom_time is None:
            return

        arr = MarkerArray()

        vehicle = Marker()
        vehicle.header.frame_id = "odom"
        vehicle.header.stamp = self.get_clock().now().to_msg()
        vehicle.ns = "vehicle"
        vehicle.id = 0
        vehicle.type = Marker.ARROW
        vehicle.action = Marker.ADD
        vehicle.pose.position.x = self.current_x
        vehicle.pose.position.y = self.current_y
        vehicle.pose.position.z = 0.08
        vehicle.pose.orientation = quat_from_yaw(self.current_yaw)
        vehicle.scale.x = 0.75
        vehicle.scale.y = 0.22
        vehicle.scale.z = 0.22
        vehicle.color.r = 1.0
        vehicle.color.g = 1.0
        vehicle.color.b = 1.0
        vehicle.color.a = 1.0
        arr.markers.append(vehicle)

        text = Marker()
        text.header = vehicle.header
        text.ns = "status"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.current_x
        text.pose.position.y = self.current_y
        text.pose.position.z = 0.9
        text.pose.orientation.w = 1.0
        text.scale.z = 0.28
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f"segment={self.current_segment_id}\n"
            f"branch={self.selected_branch or 'NONE'}"
        )
        arr.markers.append(text)

        self.marker_pub.publish(arr)

    def _fail_stop(self, reason):
        self.started = False
        self._publish_stop()
        self._status(reason)


def main(args=None):
    rclpy.init(args=args)
    node = DrSegmentedBranchFollower()
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
