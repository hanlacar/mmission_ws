"""GPS + DR route recorder.

기능
- GPS latitude / longitude 저장
- GPS 기준 상대 x/y 저장
- /odom 기준 상대 DR x/y/yaw 저장
- direction / mode / drive_level / wheel 저장
- mode 변경 시 SEG01, SEG02 ... 자동 구간 분리
- STOP_LINE 이벤트는 단 1개 waypoint에만 저장 후 자동 NONE 복귀
"""

import csv
import math
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Int32

from .geo_utils import latlon_to_xy


VALID_EVENTS = (
    "NONE",
    "STOP_LINE",
)

VALID_DRIVE_LEVELS = (
    1.0,
    2.0,
    3.0,
)


class RouteRecorder(Node):

    def __init__(self) -> None:
        super().__init__("gps_route_recorder")

        # ======================================================
        # Parameters
        # ======================================================

        for name, value in (
            ("out_csv", "reference_course.csv"),
            ("fix_topic", "/fix"),
            ("odom_topic", "/odom"),
            ("manual_drive_topic", "/manual_drive"),
            ("manual_wheel_topic", "/manual_wheel"),
            ("min_spacing_m", 0.15),
            ("record_direction", "forward"),
            ("record_mode", 1),
            ("record_drive_level", 2.0),
            ("record_event", "NONE"),
        ):
            self.declare_parameter(name, value)

        # ======================================================
        # Output
        # ======================================================

        self.out = Path(
            str(
                self.get_parameter(
                    "out_csv"
                ).value
            )
        ).expanduser()

        self.out.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.metadata_path = (
            self.out.with_suffix(".yaml")
        )

        self.segment_metadata_path = (
            self.out.with_name(
                self.out.stem
                + "_segments.yaml"
            )
        )

        # ======================================================
        # GPS
        # ======================================================

        self.spacing = float(
            self.get_parameter(
                "min_spacing_m"
            ).value
        )

        self.origin = None
        self.last = None
        self.count = 0

        self.force_record = True

        # ======================================================
        # ODOM / DR
        # ======================================================

        self.latest_odom = None

        self.dr_origin_x = None
        self.dr_origin_y = None
        self.dr_origin_yaw = None

        # ======================================================
        # Drive / Wheel
        # ======================================================

        direction = str(
            self.get_parameter(
                "record_direction"
            ).value
        )

        if direction == "reverse":
            self.current_direction = -1
        else:
            self.current_direction = 1

        self.current_drive_level = float(
            self.get_parameter(
                "record_drive_level"
            ).value
        )

        self.current_wheel = 0

        self.manual_drive_received = False
        self.manual_wheel_received = False

        # ======================================================
        # One-shot Event
        # ======================================================

        # STOP_LINE 입력 후
        # 다음 실제 기록 waypoint 1개에만 사용
        self.pending_event = "NONE"

        # 내부에서 STOP_LINE -> NONE으로 자동 초기화할 때
        # 다시 이벤트 입력으로 판단하지 않기 위한 플래그
        self.auto_resetting_event = False

        # ======================================================
        # Segments
        # ======================================================

        self.segments = []

        self.current_segment_mode = None
        self.current_segment_start = None

        # ======================================================
        # CSV
        # ======================================================

        self.stream = self.out.open(
            "w",
            newline="",
            encoding="utf-8",
        )

        self.writer = csv.writer(
            self.stream
        )

        self.writer.writerow(
            (
                "index",
                "latitude",
                "longitude",
                "x_m",
                "y_m",
                "dr_x_m",
                "dr_y_m",
                "dr_yaw_deg",
                "direction",
                "mode",
                "drive_level",
                "wheel",
                "event",
            )
        )

        self.stream.flush()

        # ======================================================
        # Parameter callback
        # ======================================================

        self.add_on_set_parameters_callback(
            self._parameters_changed
        )

        # ======================================================
        # Subscribers
        # ======================================================

        self.create_subscription(
            NavSatFix,
            str(
                self.get_parameter(
                    "fix_topic"
                ).value
            ),
            self._on_fix,
            10,
        )

        self.create_subscription(
            Odometry,
            str(
                self.get_parameter(
                    "odom_topic"
                ).value
            ),
            self._on_odom,
            20,
        )

        self.create_subscription(
            Float32,
            str(
                self.get_parameter(
                    "manual_drive_topic"
                ).value
            ),
            self._on_manual_drive,
            20,
        )

        self.create_subscription(
            Int32,
            str(
                self.get_parameter(
                    "manual_wheel_topic"
                ).value
            ),
            self._on_manual_wheel,
            20,
        )

        self.get_logger().info(
            "\n"
            "GPS + DR route recorder started\n"
            f"CSV: {self.out.resolve()}\n"
            f"Metadata: {self.metadata_path.resolve()}\n"
            f"Segments: {self.segment_metadata_path.resolve()}\n"
            f"GPS: {self.get_parameter('fix_topic').value}\n"
            f"ODOM: {self.get_parameter('odom_topic').value}\n"
            f"Drive: {self.get_parameter('manual_drive_topic').value}\n"
            f"Wheel: {self.get_parameter('manual_wheel_topic').value}\n"
            f"Mode: {self.get_parameter('record_mode').value}\n"
            "Event mode: ONE-SHOT"
        )

    # ==========================================================
    # Quaternion -> yaw
    # ==========================================================

    @staticmethod
    def _yaw_from_quaternion(q):

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = (
            1.0
            - 2.0
            * (
                q.y * q.y
                + q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    # ==========================================================
    # ODOM
    # ==========================================================

    def _on_odom(
        self,
        msg: Odometry,
    ):

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )

        yaw = self._yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        if not all(
            math.isfinite(v)
            for v in (
                x,
                y,
                yaw,
            )
        ):
            return

        if self.dr_origin_x is None:

            self.dr_origin_x = x
            self.dr_origin_y = y
            self.dr_origin_yaw = yaw

            self.get_logger().info(
                "DR origin initialized: "
                f"x={x:.3f}, "
                f"y={y:.3f}, "
                f"yaw={math.degrees(yaw):.2f} deg"
            )

        dx = (
            x
            - self.dr_origin_x
        )

        dy = (
            y
            - self.dr_origin_y
        )

        c = math.cos(
            self.dr_origin_yaw
        )

        s = math.sin(
            self.dr_origin_yaw
        )

        dr_x = (
            c * dx
            + s * dy
        )

        dr_y = (
            -s * dx
            + c * dy
        )

        dr_yaw = (
            yaw
            - self.dr_origin_yaw
        )

        dr_yaw = math.atan2(
            math.sin(dr_yaw),
            math.cos(dr_yaw),
        )

        self.latest_odom = (
            dr_x,
            dr_y,
            math.degrees(
                dr_yaw
            ),
        )

    # ==========================================================
    # Manual Drive
    # ==========================================================

    def _on_manual_drive(
        self,
        msg: Float32,
    ):

        value = float(
            msg.data
        )

        if not math.isfinite(
            value
        ):
            return

        self.manual_drive_received = True

        if abs(value) < 1.0e-6:
            return

        if value > 0.0:
            direction = 1
        else:
            direction = -1

        drive_level = abs(
            value
        )

        if (
            drive_level
            not in VALID_DRIVE_LEVELS
        ):

            self.get_logger().warning(
                "Unsupported /manual_drive: "
                f"{value:.3f}. "
                "Expected 0, ±1, ±2, ±3."
            )

            return

        changed = (
            direction
            != self.current_direction
            or abs(
                drive_level
                - self.current_drive_level
            )
            > 1.0e-6
        )

        self.current_direction = (
            direction
        )

        self.current_drive_level = (
            drive_level
        )

        if changed:

            self.force_record = True

            self.get_logger().info(
                "manual_drive changed: "
                f"raw={value:.2f}, "
                f"direction={direction}, "
                f"drive_level={drive_level:.2f}"
            )

    # ==========================================================
    # Manual Wheel
    # ==========================================================

    def _on_manual_wheel(
        self,
        msg: Int32,
    ):

        wheel = int(
            msg.data
        )

        self.manual_wheel_received = True

        if (
            wheel
            == self.current_wheel
        ):
            return

        self.current_wheel = (
            wheel
        )

        self.force_record = True

        self.get_logger().info(
            "manual_wheel changed: "
            f"{wheel}"
        )

    # ==========================================================
    # Parameter Change
    # ==========================================================

    def _parameters_changed(
        self,
        params,
    ):

        for param in params:

            # ------------------------------------------
            # Direction
            # ------------------------------------------

            if (
                param.name
                == "record_direction"
            ):

                if (
                    param.value
                    not in (
                        "forward",
                        "reverse",
                    )
                ):

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_direction must be "
                            "forward or reverse"
                        ),
                    )

            # ------------------------------------------
            # Mode
            # ------------------------------------------

            if (
                param.name
                == "record_mode"
            ):

                try:
                    mode_value = int(
                        param.value
                    )

                except (
                    ValueError,
                    TypeError,
                ):

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_mode must be "
                            "integer 1~11"
                        ),
                    )

                if not (
                    1
                    <= mode_value
                    <= 11
                ):

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_mode must be "
                            "integer 1~11"
                        ),
                    )

            # ------------------------------------------
            # Drive Level
            # ------------------------------------------

            if (
                param.name
                == "record_drive_level"
            ):

                try:
                    drive_value = float(
                        param.value
                    )

                except (
                    ValueError,
                    TypeError,
                ):

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_drive_level "
                            "must be numeric"
                        ),
                    )

                if (
                    drive_value
                    not in VALID_DRIVE_LEVELS
                ):

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_drive_level "
                            "must be 1/2/3"
                        ),
                    )

            # ==========================================
            # ONE-SHOT EVENT
            # ==========================================

            if (
                param.name
                == "record_event"
            ):

                event = str(
                    param.value
                ).strip().upper()

                if (
                    event
                    not in VALID_EVENTS
                ):

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_event must be "
                            "NONE or STOP_LINE"
                        ),
                    )

                # 자동 NONE 복귀가 아닌,
                # 사용자가 직접 이벤트를 입력한 경우
                if not self.auto_resetting_event:

                    self.pending_event = (
                        event
                    )

                    if (
                        event
                        != "NONE"
                    ):

                        # 다음 fix를 반드시 기록
                        self.force_record = True

                        self.get_logger().info(
                            "One-shot event armed: "
                            f"{event}"
                        )

            # ------------------------------------------
            # Manual drive fallback
            # ------------------------------------------

            if (
                not self.manual_drive_received
            ):

                if (
                    param.name
                    == "record_direction"
                ):

                    if (
                        param.value
                        == "reverse"
                    ):
                        self.current_direction = -1
                    else:
                        self.current_direction = 1

                if (
                    param.name
                    == "record_drive_level"
                ):

                    self.current_drive_level = float(
                        param.value
                    )

            # ------------------------------------------
            # 구간 / 방향 / 속도 변경점 강제 저장
            # ------------------------------------------

            if (
                param.name
                in (
                    "record_direction",
                    "record_mode",
                    "record_drive_level",
                )
            ):

                self.force_record = True

                self.get_logger().info(
                    "Recording boundary requested: "
                    f"{param.name}="
                    f"{param.value}"
                )

        return SetParametersResult(
            successful=True
        )

    # ==========================================================
    # GPS
    # ==========================================================

    def _on_fix(
        self,
        msg: NavSatFix,
    ):

        # ------------------------------------------
        # Fix validity
        # ------------------------------------------

        if (
            msg.status.status
            == NavSatStatus.STATUS_NO_FIX
        ):
            return

        if not (
            math.isfinite(
                msg.latitude
            )
            and math.isfinite(
                msg.longitude
            )
        ):
            return

        if (
            abs(
                msg.latitude
            )
            < 1.0e-12
            or abs(
                msg.longitude
            )
            < 1.0e-12
        ):
            return

        # ------------------------------------------
        # GPS Origin
        # ------------------------------------------

        if (
            self.origin
            is None
        ):

            self.origin = (
                msg.latitude,
                msg.longitude,
            )

            self._write_metadata()

            self.get_logger().info(
                "GPS origin initialized: "
                f"lat={msg.latitude:.10f}, "
                f"lon={msg.longitude:.10f}"
            )

        # ------------------------------------------
        # GPS -> local XY
        # ------------------------------------------

        x, y = latlon_to_xy(
            msg.latitude,
            msg.longitude,
            *self.origin,
        )

        # ------------------------------------------
        # Minimum spacing
        # ------------------------------------------

        distance_ok = (
            self.last is None
            or math.hypot(
                x
                - self.last[0],
                y
                - self.last[1],
            )
            >= self.spacing
        )

        if not (
            self.force_record
            or distance_ok
        ):
            return

        # ------------------------------------------
        # ODOM required
        # ------------------------------------------

        if (
            self.latest_odom
            is None
        ):

            self.get_logger().warning(
                "Valid GPS received but "
                "/odom has not been received yet. "
                "Waypoint not recorded."
            )

            return

        (
            dr_x,
            dr_y,
            dr_yaw_deg,
        ) = self.latest_odom

        # ------------------------------------------
        # Current state
        # ------------------------------------------

        direction = int(
            self.current_direction
        )

        mode = str(
            int(
                self.get_parameter(
                    "record_mode"
                ).value
            )
        )

        drive_level = float(
            self.current_drive_level
        )

        wheel = int(
            self.current_wheel
        )

        # 중요:
        # parameter를 매번 읽지 않고
        # 대기중인 이벤트 1개만 사용
        event = str(
            self.pending_event
        ).strip().upper()

        if (
            drive_level
            not in VALID_DRIVE_LEVELS
        ):

            self.get_logger().warning(
                "Invalid current drive_level: "
                f"{drive_level}. "
                "Waypoint not recorded."
            )

            return

        if (
            event
            not in VALID_EVENTS
        ):

            self.get_logger().error(
                "Invalid event: "
                f"{event}"
            )

            return

        waypoint_index = (
            self.count
        )

        # ------------------------------------------
        # CSV
        # ------------------------------------------

        self.writer.writerow(
            (
                waypoint_index,
                f"{msg.latitude:.10f}",
                f"{msg.longitude:.10f}",
                f"{x:.3f}",
                f"{y:.3f}",
                f"{dr_x:.3f}",
                f"{dr_y:.3f}",
                f"{dr_yaw_deg:.3f}",
                direction,
                mode,
                f"{drive_level:.2f}",
                wheel,
                event,
            )
        )

        self.stream.flush()

        self.get_logger().info(
            "Waypoint: "
            f"index={waypoint_index} "
            f"GPS=({x:.2f},{y:.2f}) "
            f"DR=({dr_x:.2f},{dr_y:.2f},"
            f"{dr_yaw_deg:.1f}deg) "
            f"dir={direction} "
            f"drive={drive_level:.2f} "
            f"wheel={wheel} "
            f"mode={mode} "
            f"event={event}"
        )

        # ==========================================
        # EVENT 1회 소비
        # ==========================================

        if (
            event
            != "NONE"
        ):

            self.get_logger().info(
                "Route event recorded ONCE: "
                f"index={waypoint_index} "
                f"event={event}"
            )

            # 내부 이벤트 제거
            self.pending_event = (
                "NONE"
            )

            # ROS parameter도 NONE으로 복귀
            self.auto_resetting_event = (
                True
            )

            try:

                self.set_parameters(
                    [
                        Parameter(
                            "record_event",
                            Parameter.Type.STRING,
                            "NONE",
                        )
                    ]
                )

            finally:

                self.auto_resetting_event = (
                    False
                )

            self.get_logger().info(
                "Route event consumed: "
                f"{event} -> NONE"
            )

        # ------------------------------------------
        # Segment
        # ------------------------------------------

        self._update_segment(
            waypoint_index,
            mode,
        )

        self._write_segment_metadata(
            current_end_index=(
                waypoint_index
            )
        )

        self.last = (
            x,
            y,
        )

        self.count += 1

        self.force_record = False

    # ==========================================================
    # Segment
    # ==========================================================

    def _update_segment(
        self,
        waypoint_index,
        mode,
    ):

        if (
            self.current_segment_mode
            is None
        ):

            self.current_segment_mode = (
                mode
            )

            self.current_segment_start = (
                waypoint_index
            )

            self.get_logger().info(
                "Segment start: "
                "SEG01 "
                f"mode={mode} "
                f"index={waypoint_index}"
            )

            return

        if (
            mode
            == self.current_segment_mode
        ):
            return

        previous_end = (
            waypoint_index
            - 1
        )

        segment_id = (
            f"SEG"
            f"{len(self.segments) + 1:02d}"
        )

        self.segments.append(
            {
                "id":
                    segment_id,

                "mode":
                    self.current_segment_mode,

                "start_index":
                    int(
                        self.current_segment_start
                    ),

                "end_index":
                    int(
                        previous_end
                    ),
            }
        )

        self.get_logger().info(
            "Segment complete: "
            f"{segment_id} "
            f"mode={self.current_segment_mode} "
            f"index="
            f"{self.current_segment_start}"
            f"~{previous_end}"
        )

        self.current_segment_mode = (
            mode
        )

        self.current_segment_start = (
            waypoint_index
        )

        next_segment_id = (
            f"SEG"
            f"{len(self.segments) + 1:02d}"
        )

        self.get_logger().info(
            "Segment start: "
            f"{next_segment_id} "
            f"mode={mode} "
            f"index={waypoint_index}"
        )

    # ==========================================================
    # Metadata
    # ==========================================================

    def _write_metadata(
        self,
    ):

        if (
            self.origin
            is None
        ):
            return

        metadata = {

            "format_version":
                1,

            "origin_lat":
                self.origin[0],

            "origin_lon":
                self.origin[1],

            "loop":
                False,

            "created_at":
                datetime.now(
                    timezone.utc
                ).isoformat(),

            "contains_dr":
                True,

            "dr_frame":
                "record_start_relative",

            "odom_source_topic":
                str(
                    self.get_parameter(
                        "odom_topic"
                    ).value
                ),
        }

        with self.metadata_path.open(
            "w",
            encoding="utf-8",
        ) as stream:

            yaml.safe_dump(
                metadata,
                stream,
                sort_keys=False,
            )

    # ==========================================================
    # Segment YAML
    # ==========================================================

    def _write_segment_metadata(
        self,
        current_end_index=None,
    ):

        if (
            self.count == 0
            and current_end_index
            is None
        ):
            return

        segments = [
            dict(segment)
            for segment
            in self.segments
        ]

        if (
            self.current_segment_mode
            is not None
            and self.current_segment_start
            is not None
        ):

            if (
                current_end_index
                is None
            ):

                end_index = max(
                    0,
                    self.count
                    - 1,
                )

            else:

                end_index = int(
                    current_end_index
                )

            current_id = (
                f"SEG"
                f"{len(segments) + 1:02d}"
            )

            segments.append(
                {
                    "id":
                        current_id,

                    "mode":
                        self.current_segment_mode,

                    "start_index":
                        int(
                            self.current_segment_start
                        ),

                    "end_index":
                        end_index,
                }
            )

        if not segments:
            return

        goal_index = max(
            int(
                segment[
                    "end_index"
                ]
            )
            for segment
            in segments
        )

        data = {

            "goal_index":
                goal_index,

            "segments":
                segments,
        }

        with self.segment_metadata_path.open(
            "w",
            encoding="utf-8",
        ) as stream:

            yaml.safe_dump(
                data,
                stream,
                allow_unicode=True,
                sort_keys=False,
            )

    # ==========================================================
    # Shutdown
    # ==========================================================

    def destroy_node(
        self,
    ):

        if (
            self.count > 0
            and self.current_segment_mode
            is not None
            and self.current_segment_start
            is not None
        ):

            final_end = (
                self.count
                - 1
            )

            final_id = (
                f"SEG"
                f"{len(self.segments) + 1:02d}"
            )

            self.segments.append(
                {
                    "id":
                        final_id,

                    "mode":
                        self.current_segment_mode,

                    "start_index":
                        int(
                            self.current_segment_start
                        ),

                    "end_index":
                        int(
                            final_end
                        ),
                }
            )

            self.current_segment_mode = (
                None
            )

            self.current_segment_start = (
                None
            )

            data = {

                "goal_index":
                    final_end,

                "segments":
                    self.segments,
            }

            with self.segment_metadata_path.open(
                "w",
                encoding="utf-8",
            ) as stream:

                yaml.safe_dump(
                    data,
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                )

            self.get_logger().info(
                "Segment metadata finalized: "
                f"{self.segment_metadata_path.resolve()} "
                f"segments={len(self.segments)} "
                f"goal_index={final_end}"
            )

        if not self.stream.closed:

            self.stream.flush()
            self.stream.close()

        super().destroy_node()


def main():

    rclpy.init()

    node = RouteRecorder()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()