"""ROS adapter and sole authority for production GPS drive/wheel commands."""

import json
import math
import os

import rclpy
import yaml

from geometry_msgs.msg import TwistWithCovarianceStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Int32, String, UInt8

from .geo_utils import latlon_to_xy
from .gps_offset import AntennaOffset
from .gps_stability import GpsStability
from .intersection_reference import (
    IntersectionConfig,
    IntersectionRouteController,
    validate_active_dir,
)
from .navigation_controller import ControllerConfig, NavigationController
from .process_singleton import acquire_process_lock
from .route_loader import RouteValidationError, load_route


class GpsRouteFollowerNode(Node):

    def __init__(self) -> None:
        super().__init__("gps_route_follower")

        self._authority_lock = acquire_process_lock(
            "gps_route_follower"
        )

        defaults = ControllerConfig()

        parameters = {
            # --------------------------------------------------
            # Main route
            # --------------------------------------------------
            "route_path": "",

            # --------------------------------------------------
            # GPS / IMU input
            # --------------------------------------------------
            "gps_fix_topic": "/fix",
            "gps_velocity_topic": "/vel",

            "imu_yaw_topic": "/imu/relative_yaw_deg",
            "imu_yaw_rate_topic": "",
            "imu_valid_topic": "/imu/valid",

            # --------------------------------------------------
            # GPS control output
            # --------------------------------------------------
            "gps_drive_topic": "/gps_drive",
            "gps_wheel_topic": "/gps_wheel",

            "drive_mode_topic": "/drive_mode",
            "vehicle_mode_topic": "/vehicle_mode",

            "status_topic": "/gps_navigation/status",

            # STOP_LINE 통과 허가
            "stop_line_release_topic": "/stop_line/release",

            "control_rate_hz": 20.0,

            # --------------------------------------------------
            # GPS safety
            # --------------------------------------------------
            "max_gps_covariance_m2": 9.0,
            "gps_jump_m": 5.0,

            # GPS antenna offset
            "gps_antenna_forward_cm": 0.0,
            "gps_antenna_lateral_cm": 0.0,

            # GPS stability
            "gps_stability_topic": "/gps_stability",
            "gps_stability_good_cov_m2": 0.05,

            # --------------------------------------------------
            # Intersection
            # --------------------------------------------------
            "intersection.enabled": True,
            "intersection.control_mode": "reference_route",

            "intersection.dir_select_mode": "manual",
            "intersection.active_dir": "N",

            "intersection.route_dir": "",

            "intersection.drive_level": 2.0,
            "intersection.route_alignment_mode": "absolute",

            "intersection.heading_warn_deg": 10.0,
            "intersection.heading_stop_deg": 20.0,

            "intersection.heading_stop_count": 3,
            "intersection.heading_recover_count": 5,

            "intersection.goal_heading_tolerance_deg": 10.0,
            "intersection.complete_confirm_count": 5,

            "intersection.start_accept_radius_m": 1.0,
            "intersection.start_heading_tolerance_deg": 30.0,

            "intersection.off_route_warn_m": 0.20,
            "intersection.off_route_stop_m": 0.35,

            "intersection.off_route_stop_count": 3,
            "intersection.off_route_recover_count": 5,

            "intersection.steering_slowdown_deg": 18.0,
            "intersection.goal_tolerance_m": 0.35,
        }

        # ControllerConfig의 모든 값을 ROS parameter로 자동 등록
        parameters.update(defaults.__dict__)

        for name, value in parameters.items():
            self.declare_parameter(
                name,
                value,
            )

        # ======================================================
        # Main route load
        # ======================================================

        route_path = self._p("route_path")

        try:
            route = load_route(route_path)

        except RouteValidationError as exc:
            self.get_logger().fatal(
                str(exc)
            )
            raise RuntimeError(
                str(exc)
            ) from exc

        self.controller = NavigationController(
            route,
            self._controller_config(""),
        )

        # ======================================================
        # Segment metadata
        #
        # 예:
        #
        # rejoin_test_straight.csv
        # rejoin_test_straight_segments.yaml
        #
        # segments.yaml은 제어 경로를 나누는 것이 아니라
        # 현재 전체 CSV에서 어떤 segment를 주행 중인지
        # 표시/관리하기 위한 메타데이터다.
        # ======================================================

        self.segment_ranges = []
        self.segment_goal_index = None
        self.segment_metadata_path = None

        self._load_segment_metadata(
            route_path
        )

        # ======================================================
        # Intersection controller
        # ======================================================

        ix_cfg = IntersectionConfig(
            route_dir=self._p(
                "intersection.route_dir"
            ),
            dir_select_mode=self._p(
                "intersection.dir_select_mode"
            ),
            active_dir=self._p(
                "intersection.active_dir"
            ),
            drive_level=float(
                self.get_parameter(
                    "intersection.drive_level"
                ).value
            ),
            heading_warn_deg=float(
                self.get_parameter(
                    "intersection.heading_warn_deg"
                ).value
            ),
            heading_stop_deg=float(
                self.get_parameter(
                    "intersection.heading_stop_deg"
                ).value
            ),
            heading_stop_count=int(
                self.get_parameter(
                    "intersection.heading_stop_count"
                ).value
            ),
            heading_recover_count=int(
                self.get_parameter(
                    "intersection.heading_recover_count"
                ).value
            ),
            goal_heading_tolerance_deg=float(
                self.get_parameter(
                    "intersection.goal_heading_tolerance_deg"
                ).value
            ),
            complete_confirm_count=int(
                self.get_parameter(
                    "intersection.complete_confirm_count"
                ).value
            ),
            route_alignment_mode=self._p(
                "intersection.route_alignment_mode"
            ),
        )

        self.intersection_enabled = bool(
            self.get_parameter(
                "intersection.enabled"
            ).value
        )

        if (
            self._p("intersection.control_mode")
            != "reference_route"
        ):
            raise RuntimeError(
                "only intersection.control_mode="
                "reference_route is supported"
            )

        self.intersection = IntersectionRouteController(
            ix_cfg,
            self._controller_config(
                "intersection."
            ),
        )

        # ======================================================
        # Runtime state
        # ======================================================

        self.last_fix = None
        self.last_fix_time = None
        self.last_fix_good = False

        self.last_accepted_xy = None

        self.last_imu_yaw = 0.0
        self.last_imu_rate = 0.0

        self.last_imu_valid = False
        self.last_imu_time = None
        self.last_imu_yaw_time = None

        self.entry_heading = None
        self.entry_heading_time = None

        # ======================================================
        # GPS safety
        # ======================================================

        self.max_cov = float(
            self.get_parameter(
                "max_gps_covariance_m2"
            ).value
        )

        self.gps_jump = float(
            self.get_parameter(
                "gps_jump_m"
            ).value
        )

        # ======================================================
        # GPS antenna offset
        # ======================================================

        self.antenna_offset = AntennaOffset(
            forward_cm=float(
                self.get_parameter(
                    "gps_antenna_forward_cm"
                ).value
            ),
            lateral_cm=float(
                self.get_parameter(
                    "gps_antenna_lateral_cm"
                ).value
            ),
        )

        if not self.antenna_offset.is_zero():
            self.get_logger().info(
                "[GPS offset] "
                f"forward="
                f"{self.antenna_offset.forward_cm}cm "
                f"lateral="
                f"{self.antenna_offset.lateral_cm}cm "
                "(평면 보정 활성)"
            )

        # ======================================================
        # GPS stability
        # ======================================================

        gps_timeout = (
            float(
                self.get_parameter(
                    "gps_timeout_sec"
                ).value
            )
            if self.has_parameter(
                "gps_timeout_sec"
            )
            else 1.0
        )

        self.stability = GpsStability(
            good_cov_m2=float(
                self.get_parameter(
                    "gps_stability_good_cov_m2"
                ).value
            ),
            max_cov_m2=self.max_cov,
            jump_hard_m=self.gps_jump,
            timeout_s=gps_timeout,
        )

        self.stability_score = 0.0

        # ======================================================
        # Publishers
        # ======================================================

        self.drive_pub = self.create_publisher(
            Float32,
            self._p("gps_drive_topic"),
            10,
        )

        self.wheel_pub = self.create_publisher(
            Int32,
            self._p("gps_wheel_topic"),
            10,
        )

        self.mode_pub = self.create_publisher(
            String,
            self._p("drive_mode_topic"),
            10,
        )

        self.vehicle_mode_pub = self.create_publisher(
            String,
            self._p("vehicle_mode_topic"),
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self._p("status_topic"),
            10,
        )

        self.stability_pub = self.create_publisher(
            Float32,
            self._p("gps_stability_topic"),
            10,
        )

        # Intersection state는 latched
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.intersection_complete_pub = (
            self.create_publisher(
                Bool,
                "/intersection/complete",
                latched,
            )
        )

        self.intersection_state_pub = (
            self.create_publisher(
                String,
                "/intersection/state",
                latched,
            )
        )

        self.intersection_status_pub = (
            self.create_publisher(
                String,
                "/intersection/status",
                10,
            )
        )

        # ======================================================
        # Subscribers
        # ======================================================

        self.create_subscription(
            NavSatFix,
            self._p("gps_fix_topic"),
            self._on_fix,
            10,
        )

        self.create_subscription(
            TwistWithCovarianceStamped,
            self._p("gps_velocity_topic"),
            self._on_velocity,
            10,
        )

        self.create_subscription(
            Float32,
            self._p("imu_yaw_topic"),
            self._on_yaw,
            20,
        )

        rate_topic = self._p(
            "imu_yaw_rate_topic"
        )

        if rate_topic:
            self.create_subscription(
                Float32,
                rate_topic,
                self._on_rate,
                20,
            )

        self.create_subscription(
            Bool,
            self._p("imu_valid_topic"),
            self._on_valid,
            20,
        )

        self.create_subscription(
            UInt8,
            "/intersection/command",
            self._on_command,
            10,
        )

        self.create_subscription(
            Bool,
            self._p("stop_line_release_topic"),
            self._on_stop_line_release,
            10,
        )

        # ======================================================
        # Parameters / timer
        # ======================================================

        self.add_on_set_parameters_callback(
            self._parameters_changed
        )

        self.create_timer(
            1.0
            / float(
                self.get_parameter(
                    "control_rate_hz"
                ).value
            ),
            self._tick,
        )

    # ==========================================================
    # Segment metadata
    # ==========================================================

    def _load_segment_metadata(
        self,
        route_path: str,
    ) -> None:

        if not route_path:
            return

        root, _ext = os.path.splitext(
            route_path
        )

        metadata_path = (
            root
            + "_segments.yaml"
        )

        self.segment_metadata_path = (
            metadata_path
        )

        if not os.path.isfile(
            metadata_path
        ):
            self.get_logger().info(
                "segment metadata not found; "
                "segment_id reporting disabled: "
                f"{metadata_path}"
            )
            return

        try:
            with open(
                metadata_path,
                "r",
                encoding="utf-8",
            ) as f:
                data = (
                    yaml.safe_load(f)
                    or {}
                )

            goal_index = data.get(
                "goal_index"
            )

            if goal_index is not None:
                self.segment_goal_index = int(
                    goal_index
                )

            raw_segments = data.get(
                "segments",
                []
            )

            ranges = []

            for item in raw_segments:
                segment_id = str(
                    item["id"]
                )

                start_index = int(
                    item["start_index"]
                )

                end_index = int(
                    item["end_index"]
                )

                if start_index < 0:
                    raise ValueError(
                        f"{segment_id}: "
                        "start_index must be >= 0"
                    )

                if end_index < start_index:
                    raise ValueError(
                        f"{segment_id}: "
                        "end_index < start_index"
                    )

                ranges.append({
                    "id": segment_id,
                    "start_index": start_index,
                    "end_index": end_index,
                })

            # index 순서대로 정렬
            ranges.sort(
                key=lambda item:
                item["start_index"]
            )

            # segment overlap 검사
            previous_end = -1

            for item in ranges:

                if (
                    item["start_index"]
                    <= previous_end
                ):
                    raise ValueError(
                        "segment ranges overlap: "
                        f"{item['id']}"
                    )

                previous_end = (
                    item["end_index"]
                )

            self.segment_ranges = ranges

            if self.segment_ranges:
                names = ", ".join(
                    segment["id"]
                    for segment
                    in self.segment_ranges
                )

                self.get_logger().info(
                    "segment metadata loaded: "
                    f"{len(self.segment_ranges)} "
                    f"segments [{names}], "
                    f"goal_index="
                    f"{self.segment_goal_index}"
                )

        except Exception as exc:
            self.segment_ranges = []
            self.segment_goal_index = None

            self.get_logger().warning(
                "segment metadata load failed: "
                f"{metadata_path}: {exc}"
            )

    def _segment_id_for_index(
        self,
        route_index: int,
    ):

        for segment in self.segment_ranges:

            if (
                segment["start_index"]
                <= route_index
                <= segment["end_index"]
            ):
                return segment["id"]

        return None

    # ==========================================================
    # Controller config
    # ==========================================================

    def _controller_config(
        self,
        prefix: str,
    ) -> ControllerConfig:

        values = {}

        for name in ControllerConfig().__dict__:

            parameter = prefix + name

            selected_parameter = (
                parameter
                if self.has_parameter(
                    parameter
                )
                else name
            )

            values[name] = (
                self.get_parameter(
                    selected_parameter
                ).value
            )

        return ControllerConfig(
            **values
        )

    # ==========================================================
    # Helpers
    # ==========================================================

    def _p(
        self,
        name: str,
    ) -> str:

        return str(
            self.get_parameter(
                name
            ).value
        )

    def _now(
        self,
    ) -> float:

        return (
            self.get_clock()
            .now()
            .nanoseconds
            * 1.0e-9
        )

    # ==========================================================
    # Runtime parameter update
    # ==========================================================

    def _parameters_changed(
        self,
        params,
    ):

        for param in params:

            if (
                param.name
                == "intersection.active_dir"
            ):

                try:
                    validate_active_dir(
                        param.value
                    )

                except ValueError as exc:
                    return SetParametersResult(
                        successful=False,
                        reason=str(exc),
                    )

                if self.intersection.active:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "cannot change active_dir "
                            "while active"
                        ),
                    )

        for param in params:

            if (
                param.name
                == "intersection.active_dir"
            ):

                self.intersection.set_active_dir(
                    param.value
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
    ) -> None:

        now = self._now()

        finite = (
            math.isfinite(
                msg.latitude
            )
            and math.isfinite(
                msg.longitude
            )
        )

        covariance = max(
            msg.position_covariance[0],
            msg.position_covariance[4],
        )

        good = bool(
            finite
            and (
                msg.status.status
                != NavSatStatus.STATUS_NO_FIX
            )
            and math.isfinite(
                covariance
            )
            and covariance
            <= self.max_cov
        )

        jump_m = 0.0

        if finite:

            meta = (
                self.controller
                .route
                .metadata
            )

            nx, ny = latlon_to_xy(
                msg.latitude,
                msg.longitude,
                meta.origin_lat,
                meta.origin_lon,
            )

            # GPS 안테나 → 차량 기준점 보정
            if not self.antenna_offset.is_zero():

                heading = getattr(
                    self.controller.imu,
                    "heading",
                    None,
                )

                if (
                    heading is not None
                    and math.isfinite(
                        heading
                    )
                ):
                    nx, ny = (
                        self.antenna_offset
                        .antenna_to_reference(
                            nx,
                            ny,
                            heading,
                        )
                    )

            # GPS jump 검사
            if self.last_accepted_xy is not None:

                jump_m = math.hypot(
                    nx
                    - self.last_accepted_xy[0],
                    ny
                    - self.last_accepted_xy[1],
                )

                if jump_m > self.gps_jump:
                    good = False

            if good:
                self.last_accepted_xy = (
                    nx,
                    ny,
                )

            self.controller.set_position(
                nx,
                ny,
                now,
                good=good,
            )

            self.last_fix = (
                float(
                    msg.latitude
                ),
                float(
                    msg.longitude
                ),
            )

        self.last_fix_time = now
        self.last_fix_good = good

        # GPS stability
        has_fix = bool(
            finite
            and (
                msg.status.status
                != NavSatStatus.STATUS_NO_FIX
            )
        )

        self.stability_score = (
            self.stability.score(
                has_fix=has_fix,
                covariance=covariance,
                jump_m=jump_m,
                age_s=0.0,
            )
        )

        self.stability_pub.publish(
            Float32(
                data=float(
                    self.stability_score
                )
            )
        )

    # ==========================================================
    # GPS velocity
    # ==========================================================

    def _on_velocity(
        self,
        msg: TwistWithCovarianceStamped,
    ) -> None:

        vx = float(
            msg.twist.twist.linear.x
        )

        vy = float(
            msg.twist.twist.linear.y
        )

        if (
            math.isfinite(vx)
            and math.isfinite(vy)
            and math.hypot(
                vx,
                vy,
            )
            > 0.2
        ):

            self.entry_heading = math.atan2(
                vy,
                vx,
            )

            self.entry_heading_time = (
                self._now()
            )

    # ==========================================================
    # IMU
    # ==========================================================

    def _push_imu(
        self,
    ) -> None:

        # valid/rate 수신만으로 yaw freshness를 갱신하면 안 됨.
        # 실제 마지막 yaw sample timestamp 기준으로 timeout 판단.

        now = self._now()

        sample_time = (
            self.last_imu_yaw_time
            if (
                self.last_imu_yaw_time
                is not None
            )
            else now
        )

        sample_valid = bool(
            self.last_imu_valid
            and (
                self.last_imu_yaw_time
                is not None
            )
        )

        self.last_imu_time = (
            sample_time
        )

        self.controller.set_imu(
            self.last_imu_yaw,
            self.last_imu_rate,
            sample_valid,
            sample_time,
        )

    def _on_yaw(
        self,
        msg: Float32,
    ) -> None:

        self.last_imu_yaw = float(
            msg.data
        )

        self.last_imu_yaw_time = (
            self._now()
        )

        self._push_imu()

    def _on_rate(
        self,
        msg: Float32,
    ) -> None:

        self.last_imu_rate = float(
            msg.data
        )

        self._push_imu()

    def _on_valid(
        self,
        msg: Bool,
    ) -> None:

        self.last_imu_valid = bool(
            msg.data
        )

        self._push_imu()

    # ==========================================================
    # STOP_LINE release
    # ==========================================================

    def _on_stop_line_release(
        self,
        msg: Bool,
    ) -> None:

        # False는 아무 동작도 하지 않는다.
        if not msg.data:
            return

        released = (
            self.controller.release_stop_line()
        )

        if released:

            self.get_logger().info(
                "STOP_LINE released; "
                "route tracking resumed"
            )

        else:

            self.get_logger().warning(
                "STOP_LINE release ignored; "
                "vehicle is not stopped at STOP_LINE"
            )

    # ==========================================================
    # Intersection command
    # ==========================================================

    def _on_command(
        self,
        msg: UInt8,
    ) -> None:

        if (
            self.intersection_enabled
            and not self.intersection.on_command(
                msg.data
            )
        ):

            self.get_logger().warning(
                "ignored intersection "
                f"command={msg.data}; "
                f"state="
                f"{self.intersection.state.value}"
            )

    # ==========================================================
    # Intersection control
    # ==========================================================

    def _intersection_step(
        self,
        now: float,
    ):

        previous_state = (
            self.intersection.state
        )

        controller = (
            self.intersection.controller
        )

        x = None
        y = None

        if (
            controller is not None
            and self.last_fix is not None
        ):

            meta = (
                controller.route.metadata
            )

            x, y = latlon_to_xy(
                self.last_fix[0],
                self.last_fix[1],
                meta.origin_lat,
                meta.origin_lon,
            )

        cfg = (
            self.intersection
            .navigation_config
        )

        gps_ok = bool(
            self.last_fix_good
            and (
                self.last_fix_time
                is not None
            )
            and (
                now
                - self.last_fix_time
                <= cfg.gps_timeout_sec
            )
        )

        imu_ok = bool(
            self.last_imu_valid
            and (
                self.last_imu_yaw_time
                is not None
            )
            and math.isfinite(
                self.last_imu_yaw
            )
            and (
                now
                - self.last_imu_yaw_time
                <= cfg.imu_timeout_sec
            )
        )

        entry_heading = (
            self.entry_heading
            if (
                self.entry_heading_time
                is not None
                and (
                    now
                    - self.entry_heading_time
                    <= cfg.gps_timeout_sec
                )
            )
            else None
        )

        output = self.intersection.step(
            now,
            x=x,
            y=y,
            gps_healthy=gps_ok,
            imu_yaw_deg=self.last_imu_yaw,
            imu_rate_deg_s=self.last_imu_rate,
            imu_healthy=imu_ok,
            entry_heading_rad=entry_heading,
        )

        if (
            previous_state.value
            == "PREPARE"
            and (
                self.intersection
                .infeasible_curvature_segments
            )
        ):

            self.get_logger().warning(
                "recorded route has "
                f"{self.intersection.infeasible_curvature_segments} "
                "curvature samples below the "
                "configured turning radius"
            )

        return output

    # ==========================================================
    # Main control timer
    # ==========================================================

    def _tick(
        self,
    ) -> None:

        now = self._now()

        if self.intersection.active:
            output = (
                self._intersection_step(
                    now
                )
            )
        else:
            output = (
                self.controller.step(
                    now
                )
            )

        # ------------------------------------------------------
        # Control outputs
        # ------------------------------------------------------

        self.drive_pub.publish(
            Float32(
                data=float(
                    output.drive
                )
            )
        )

        self.wheel_pub.publish(
            Int32(
                data=int(
                    output.wheel
                )
            )
        )

        self.mode_pub.publish(
            String(
                data=output.mode
            )
        )

        # GPS route mode → MCU mode
        self.vehicle_mode_pub.publish(
            String(
                data=output.mode
            )
        )

        # ------------------------------------------------------
        # Intersection state
        # ------------------------------------------------------

        state = (
            self.intersection
            .state
            .value
        )

        self.intersection_state_pub.publish(
            String(
                data=state
            )
        )

        self.intersection_complete_pub.publish(
            Bool(
                data=self.intersection.complete
            )
        )

        ix_controller = (
            self.intersection.controller
        )

        route_size = (
            len(
                ix_controller
                .route
                .waypoints
            )
            if ix_controller
            else 0
        )

        ix_route_index = (
            output.route_index
            if self.intersection.active
            else 0
        )

        # ------------------------------------------------------
        # Sensor health
        # ------------------------------------------------------

        gps_healthy = bool(
            self.last_fix_good
            and (
                self.last_fix_time
                is not None
            )
            and (
                now
                - self.last_fix_time
                <= (
                    self.controller
                    .config
                    .gps_timeout_sec
                )
            )
        )

        imu_healthy = bool(
            self.last_imu_valid
            and (
                self.last_imu_yaw_time
                is not None
            )
            and (
                now
                - self.last_imu_yaw_time
                <= (
                    self.controller
                    .config
                    .imu_timeout_sec
                )
            )
        )

        # ------------------------------------------------------
        # Intersection status
        # ------------------------------------------------------

        ix_status = {
            "active_dir":
                self.intersection.active_dir,

            "selected_dir":
                self.intersection.selected_dir,

            "dir_select_mode":
                self.intersection.config.dir_select_mode,

            "active_command":
                self.intersection.active_command,

            "active_route":
                self.intersection.active_route,

            "state":
                state,

            "route_index":
                ix_route_index,

            "route_progress":
                (
                    ix_route_index
                    / max(
                        1,
                        route_size - 2,
                    )
                    if route_size
                    else 0.0
                ),

            "cross_track_error_m":
                output.cross_track_error,

            "heading_error_deg":
                self.intersection.heading_error_deg,

            "current_x_m":
                (
                    ix_controller.x
                    if ix_controller
                    else None
                ),

            "current_y_m":
                (
                    ix_controller.y
                    if ix_controller
                    else None
                ),

            "target_x_m":
                output.target_x,

            "target_y_m":
                output.target_y,

            "gps_healthy":
                gps_healthy,

            "imu_healthy":
                imu_healthy,

            "drive_command":
                output.drive,

            "wheel_command":
                output.wheel,

            "fault_reason":
                self.intersection.fault_reason,

            "complete_confirm_count":
                self.intersection.complete_count,

            "infeasible_curvature_segments":
                (
                    self.intersection
                    .infeasible_curvature_segments
                ),
        }

        self.intersection_status_pub.publish(
            String(
                data=json.dumps(
                    ix_status,
                    separators=(",", ":"),
                )
            )
        )

        # ------------------------------------------------------
        # Main route segment
        #
        # Intersection 전용 route가 활성화됐을 때는
        # main route용 segment metadata를 적용하지 않는다.
        # ------------------------------------------------------

        if self.intersection.active:
            segment_id = None
        else:
            segment_id = (
                self._segment_id_for_index(
                    output.route_index
                )
            )

        # ------------------------------------------------------
        # Main GPS status
        # ------------------------------------------------------

        status = {
            "state":
                output.state.value,

            "route_index":
                output.route_index,

            "segment_id":
                segment_id,

            "segment_goal_index":
                self.segment_goal_index,

            "mode":
                output.mode,

            "cross_track_error_m":
                round(
                    output.cross_track_error,
                    3,
                ),

            "direction":
                output.direction,

            "drive_level":
                output.drive_level,

            "gps_drive":
                output.drive,

            "gps_wheel":
                output.wheel,

            "target_x_m":
                output.target_x,

            "target_y_m":
                output.target_y,

            "imu_yaw_deg":
                round(
                    self.last_imu_yaw,
                    3,
                ),

            "world_heading_deg":
                (
                    None
                    if output.heading is None
                    else round(
                        math.degrees(
                            output.heading
                        ),
                        3,
                    )
                ),

            "gps_healthy":
                gps_healthy,

            "imu_healthy":
                imu_healthy,

            "reason":
                output.reason,

            "intersection_state":
                state,
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status,
                    separators=(",", ":"),
                )
            )
        )


def main() -> None:

    rclpy.init()

    node = None

    try:
        node = GpsRouteFollowerNode()

        rclpy.spin(
            node
        )

    except (
        KeyboardInterrupt,
        RuntimeError,
    ):
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()