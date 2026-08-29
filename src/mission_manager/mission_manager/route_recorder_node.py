"""Record versioned GPS routes with explicit direction, mode,
drive level and route event.

추가 기능:
- CSV 저장 중 mode 변화 지점을 자동 감지
- SEG01, SEG02 ... 자동 생성
- <route_name>_segments.yaml 자동 생성/갱신
- 마지막 waypoint를 goal_index로 자동 지정
- event 저장 지원
- STOP_LINE waypoint 저장 지원

예:

NORMAL → SLOPE → NORMAL → T_PARK

자동 생성:

SEG01 NORMAL
SEG02 SLOPE
SEG03 NORMAL
SEG04 T_PARK

event 예:

NONE
STOP_LINE
"""

import csv
import math
from datetime import (
    datetime,
    timezone,
)
from pathlib import Path

import rclpy
import yaml

from rcl_interfaces.msg import (
    SetParametersResult,
)
from rclpy.node import Node
from sensor_msgs.msg import (
    NavSatFix,
    NavSatStatus,
)

from .geo_utils import latlon_to_xy


VALID_EVENTS = (
    "NONE",
    "STOP_LINE",
)


class RouteRecorder(Node):

    def __init__(self) -> None:
        super().__init__(
            "gps_route_recorder"
        )

        # ------------------------------------------------------
        # Parameters
        # ------------------------------------------------------

        for name, value in (
            (
                "out_csv",
                "reference_course.csv",
            ),
            (
                "fix_topic",
                "/fix",
            ),
            (
                "min_spacing_m",
                0.15,
            ),
            (
                "record_direction",
                "forward",
            ),
            (
                "record_mode",
                1,
            ),
            (
                "record_drive_level",
                2.0,
            ),
            (
                "record_event",
                "NONE",
            ),
        ):
            self.declare_parameter(
                name,
                value,
            )

        # ------------------------------------------------------
        # Output paths
        # ------------------------------------------------------

        self.out = Path(
            str(
                self.get_parameter(
                    "out_csv"
                ).value
            )
        )

        self.out.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        # 일반 경로 metadata
        #
        # route01.csv
        # route01.yaml

        self.metadata_path = (
            self.out.with_suffix(
                ".yaml"
            )
        )

        # segment metadata
        #
        # route01.csv
        # route01_segments.yaml

        self.segment_metadata_path = (
            self.out.with_name(
                self.out.stem
                + "_segments.yaml"
            )
        )

        # ------------------------------------------------------
        # Recorder state
        # ------------------------------------------------------

        self.spacing = float(
            self.get_parameter(
                "min_spacing_m"
            ).value
        )

        self.origin = None
        self.last = None

        self.count = 0

        # mode / direction / drive_level /
        # event 변경 시 다음 GPS fix를
        # 무조건 기록한다.
        self.force_record = True

        # ------------------------------------------------------
        # Segment state
        # ------------------------------------------------------

        self.segments = []

        self.current_segment_mode = None
        self.current_segment_start = None

        # ------------------------------------------------------
        # CSV
        # ------------------------------------------------------

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
                "direction",
                "mode",
                "drive_level",
                "event",
            )
        )

        self.stream.flush()

        # ------------------------------------------------------
        # ROS
        # ------------------------------------------------------

        self.add_on_set_parameters_callback(
            self._parameters_changed
        )

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

        self.get_logger().info(
            "Recording GPS reference route\n"
            f"CSV: "
            f"{self.out.resolve()}\n"
            f"Metadata: "
            f"{self.metadata_path.resolve()}\n"
            f"Segments: "
            f"{self.segment_metadata_path.resolve()}\n"
            f"Direction: "
            f"{str(self.get_parameter('record_direction').value).upper()}\n"
            f"Mode: "
            f"{self.get_parameter('record_mode').value}\n"
            f"Drive level: "
            f"{float(self.get_parameter('record_drive_level').value):.2f}\n"
            f"Event: "
            f"{self.get_parameter('record_event').value}"
        )

    # ==========================================================
    # Runtime parameter change
    # ==========================================================

    def _parameters_changed(
        self,
        params,
    ):

        for param in params:

            # --------------------------------------------------
            # Direction validation
            # --------------------------------------------------

            if (
                param.name
                == "record_direction"
                and param.value
                not in (
                    "forward",
                    "reverse",
                )
            ):
                return SetParametersResult(
                    successful=False,
                    reason=(
                        "direction must be "
                        "forward/reverse"
                    ),
                )

            # --------------------------------------------------
            # Mode validation
            # --------------------------------------------------

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
                            "mode must be "
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
                            "mode must be "
                            "integer 1~11"
                        ),
                    )

            # --------------------------------------------------
            # Drive level validation
            # --------------------------------------------------

            if (
                param.name
                == "record_drive_level"
                and float(
                    param.value
                )
                not in (
                    1.0,
                    2.0,
                    3.0,
                )
            ):
                return SetParametersResult(
                    successful=False,
                    reason=(
                        "drive level must be "
                        "1/2/3"
                    ),
                )

            # --------------------------------------------------
            # Event validation
            # --------------------------------------------------

            if (
                param.name
                == "record_event"
            ):

                event = str(
                    param.value
                ).strip().upper()

                if event not in VALID_EVENTS:

                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "event must be "
                            "NONE or STOP_LINE"
                        ),
                    )

            # --------------------------------------------------
            # Boundary recording
            # --------------------------------------------------

            # 값이 바뀌는 지점은 waypoint를
            # 무조건 한 개 남긴다.

            if param.name in (
                "record_direction",
                "record_mode",
                "record_drive_level",
                "record_event",
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
    # GPS callback
    # ==========================================================

    def _on_fix(
        self,
        msg: NavSatFix,
    ) -> None:

        # ------------------------------------------------------
        # GPS validity
        # ------------------------------------------------------

        if (
            msg.status.status
            == NavSatStatus.STATUS_NO_FIX
            or not math.isfinite(
                msg.latitude
            )
            or not math.isfinite(
                msg.longitude
            )
        ):
            return

        # ------------------------------------------------------
        # First point -> origin
        # ------------------------------------------------------

        if self.origin is None:

            self.origin = (
                msg.latitude,
                msg.longitude,
            )

            self._write_metadata()

        # ------------------------------------------------------
        # GPS -> local XY
        # ------------------------------------------------------

        x, y = latlon_to_xy(
            msg.latitude,
            msg.longitude,
            *self.origin,
        )

        distance_ok = (
            self.last is None
            or math.hypot(
                x - self.last[0],
                y - self.last[1],
            )
            >= self.spacing
        )

        if not (
            self.force_record
            or distance_ok
        ):
            return

        # ------------------------------------------------------
        # Current record state
        # ------------------------------------------------------

        direction = (
            1
            if (
                self.get_parameter(
                    "record_direction"
                ).value
                == "forward"
            )
            else -1
        )

        mode = str(
            int(
                self.get_parameter(
                    "record_mode"
                ).value
            )
        )

        level = float(
            self.get_parameter(
                "record_drive_level"
            ).value
        )

        event = str(
            self.get_parameter(
                "record_event"
            ).value
        ).strip().upper()

        if event not in VALID_EVENTS:

            self.get_logger().error(
                "Invalid record_event: "
                f"{event}"
            )

            return

        waypoint_index = (
            self.count
        )

        # ------------------------------------------------------
        # CSV write
        # ------------------------------------------------------

        self.writer.writerow(
            (
                waypoint_index,
                f"{msg.latitude:.10f}",
                f"{msg.longitude:.10f}",
                f"{x:.3f}",
                f"{y:.3f}",
                direction,
                mode,
                f"{level:.2f}",
                event,
            )
        )

        self.stream.flush()

        # ------------------------------------------------------
        # Logging
        # ------------------------------------------------------

        if event != "NONE":

            self.get_logger().info(
                "Route event recorded: "
                f"index={waypoint_index} "
                f"event={event} "
                f"x={x:.2f} "
                f"y={y:.2f}"
            )

        # ------------------------------------------------------
        # Segment update
        # ------------------------------------------------------

        self._update_segment(
            waypoint_index,
            mode,
        )

        # segment YAML도 waypoint마다
        # 갱신한다.
        #
        # 갑작스러운 종료가 발생하더라도
        # 가능한 최신 상태를 남긴다.

        self._write_segment_metadata(
            current_end_index=(
                waypoint_index
            )
        )

        # ------------------------------------------------------
        # Recorder state
        # ------------------------------------------------------

        self.last = (
            x,
            y,
        )

        self.count += 1

        self.force_record = False

    # ==========================================================
    # Segment handling
    # ==========================================================

    def _update_segment(
        self,
        waypoint_index: int,
        mode: str,
    ) -> None:

        # ------------------------------------------------------
        # First waypoint
        # ------------------------------------------------------

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

        # ------------------------------------------------------
        # Same mode
        # ------------------------------------------------------

        if (
            mode
            == self.current_segment_mode
        ):
            return

        # ------------------------------------------------------
        # Mode changed
        # ------------------------------------------------------

        previous_end = (
            waypoint_index - 1
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
            f"mode="
            f"{self.current_segment_mode} "
            f"index="
            f"{self.current_segment_start}"
            f"~{previous_end}"
        )

        # ------------------------------------------------------
        # Start new segment
        # ------------------------------------------------------

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
    # Route metadata
    # ==========================================================

    def _write_metadata(
        self,
    ) -> None:

        if self.origin is None:
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
    # Segment metadata
    # ==========================================================

    def _write_segment_metadata(
        self,
        current_end_index=None,
    ) -> None:

        # 아직 waypoint가 없음

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

        # 현재 열려 있는 마지막 segment도
        # YAML에 임시 포함한다.

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
                    self.count - 1,
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
                segment["end_index"]
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

        with (
            self.segment_metadata_path.open(
                "w",
                encoding="utf-8",
            )
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
    ) -> None:

        # ------------------------------------------------------
        # Finalize last segment
        # ------------------------------------------------------

        if (
            self.count > 0
            and self.current_segment_mode
            is not None
            and self.current_segment_start
            is not None
        ):

            final_end = (
                self.count - 1
            )

            final_id = (
                f"SEG"
                f"{len(self.segments) + 1:02d}"
            )

            # 아직 finalized list에
            # 들어가지 않은 마지막 segment

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

            # 최종 segments.yaml

            if self.segments:

                data = {
                    "goal_index":
                        final_end,

                    "segments":
                        self.segments,
                }

                with (
                    self.segment_metadata_path
                    .open(
                        "w",
                        encoding="utf-8",
                    )
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

        # ------------------------------------------------------
        # CSV close
        # ------------------------------------------------------

        if not self.stream.closed:

            self.stream.flush()

            self.stream.close()

        super().destroy_node()


def main() -> None:

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