"""ROS adapter and sole authority for production GPS drive/wheel commands."""
import json
import math
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Int32, String, UInt8
from .geo_utils import latlon_to_xy
from .gps_offset import AntennaOffset
from .gps_stability import GpsStability
from .intersection_reference import (IntersectionConfig, IntersectionRouteController,
                                     validate_active_dir)
from .navigation_controller import ControllerConfig, NavigationController
from .process_singleton import acquire_process_lock
from .route_loader import RouteValidationError, load_route


class GpsRouteFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_route_follower")
        self._authority_lock = acquire_process_lock("gps_route_follower")
        defaults = ControllerConfig()
        parameters = {
            "route_path": "", "gps_fix_topic": "/fix", "gps_velocity_topic": "/vel",
            "imu_yaw_topic": "/imu/relative_yaw_deg", "imu_yaw_rate_topic": "",
            "imu_valid_topic": "/imu/valid", "gps_drive_topic": "/gps_drive",
            "gps_wheel_topic": "/gps_wheel", "drive_mode_topic": "/drive_mode",
            "status_topic": "/gps_navigation/status", "control_rate_hz": 20.0,
            "max_gps_covariance_m2": 9.0, "gps_jump_m": 5.0,
            # GPS 안테나 평면 오프셋 (cm). 차량 기준점(뒷축)에서 본 안테나 위치.
            # forward: 앞=+, 뒤=-.  lateral: 왼쪽=+, 오른쪽=-.  0이면 보정 안 함.
            "gps_antenna_forward_cm": 0.0, "gps_antenna_lateral_cm": 0.0,
            # GPS 안정성(0~100%) 발행 토픽. 재밍/멀티패스로 값이 튀면 낮아진다.
            "gps_stability_topic": "/gps_stability",
            # 안정성 점수 임계(공분산 만점 기준). max는 max_gps_covariance_m2와 동일하게 씀.
            "gps_stability_good_cov_m2": 0.05,
            "intersection.enabled": True, "intersection.control_mode": "reference_route",
            "intersection.dir_select_mode": "manual", "intersection.active_dir": "N",
            "intersection.route_dir": "",
            "intersection.drive_level": 2.0, "intersection.route_alignment_mode": "absolute",
            "intersection.heading_warn_deg": 10.0, "intersection.heading_stop_deg": 20.0,
            "intersection.heading_stop_count": 3, "intersection.heading_recover_count": 5,
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
        parameters.update(defaults.__dict__)
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        try:
            route = load_route(self._p("route_path"))
        except RouteValidationError as exc:
            self.get_logger().fatal(str(exc))
            raise RuntimeError(str(exc)) from exc
        self.controller = NavigationController(route, self._controller_config(""))
        ix_cfg = IntersectionConfig(
            route_dir=self._p("intersection.route_dir"),
            dir_select_mode=self._p("intersection.dir_select_mode"),
            active_dir=self._p("intersection.active_dir"),
            drive_level=float(self.get_parameter("intersection.drive_level").value),
            heading_warn_deg=float(self.get_parameter("intersection.heading_warn_deg").value),
            heading_stop_deg=float(self.get_parameter("intersection.heading_stop_deg").value),
            heading_stop_count=int(self.get_parameter("intersection.heading_stop_count").value),
            heading_recover_count=int(self.get_parameter("intersection.heading_recover_count").value),
            goal_heading_tolerance_deg=float(self.get_parameter(
                "intersection.goal_heading_tolerance_deg").value),
            complete_confirm_count=int(self.get_parameter(
                "intersection.complete_confirm_count").value),
            route_alignment_mode=self._p("intersection.route_alignment_mode"))
        self.intersection_enabled = bool(self.get_parameter("intersection.enabled").value)
        if self._p("intersection.control_mode") != "reference_route":
            raise RuntimeError("only intersection.control_mode=reference_route is supported")
        self.intersection = IntersectionRouteController(
            ix_cfg, self._controller_config("intersection."))

        self.last_fix = None
        self.last_fix_time = None
        self.last_fix_good = False
        self.last_accepted_xy = None
        self.last_imu_yaw, self.last_imu_rate = 0.0, 0.0
        self.last_imu_valid, self.last_imu_time = False, None
        self.last_imu_yaw_time = None
        self.entry_heading, self.entry_heading_time = None, None
        self.max_cov = float(self.get_parameter("max_gps_covariance_m2").value)
        self.gps_jump = float(self.get_parameter("gps_jump_m").value)
        # 안테나 오프셋 (cm). 0,0이면 보정 없이 그대로 통과.
        self.antenna_offset = AntennaOffset(
            forward_cm=float(self.get_parameter("gps_antenna_forward_cm").value),
            lateral_cm=float(self.get_parameter("gps_antenna_lateral_cm").value))
        if not self.antenna_offset.is_zero():
            self.get_logger().info(
                f"[GPS offset] forward={self.antenna_offset.forward_cm}cm "
                f"lateral={self.antenna_offset.lateral_cm}cm (평면 보정 활성)")
        # GPS 안정성 점수기 (재밍/멀티패스 감지 → 0~100%)
        gps_timeout = float(self.get_parameter("gps_timeout_sec").value) \
            if self.has_parameter("gps_timeout_sec") else 1.0
        self.stability = GpsStability(
            good_cov_m2=float(self.get_parameter("gps_stability_good_cov_m2").value),
            max_cov_m2=self.max_cov,
            jump_hard_m=self.gps_jump,
            timeout_s=gps_timeout)
        self.stability_score = 0.0
        self.drive_pub = self.create_publisher(Float32, self._p("gps_drive_topic"), 10)
        self.wheel_pub = self.create_publisher(Int32, self._p("gps_wheel_topic"), 10)
        self.mode_pub = self.create_publisher(String, self._p("drive_mode_topic"), 10)
        self.status_pub = self.create_publisher(String, self._p("status_topic"), 10)
        self.stability_pub = self.create_publisher(
            Float32, self._p("gps_stability_topic"), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.intersection_complete_pub = self.create_publisher(Bool, "/intersection/complete", latched)
        self.intersection_state_pub = self.create_publisher(String, "/intersection/state", latched)
        self.intersection_status_pub = self.create_publisher(String, "/intersection/status", 10)
        self.create_subscription(NavSatFix, self._p("gps_fix_topic"), self._on_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, self._p("gps_velocity_topic"),
                                 self._on_velocity, 10)
        self.create_subscription(Float32, self._p("imu_yaw_topic"), self._on_yaw, 20)
        rate_topic = self._p("imu_yaw_rate_topic")
        if rate_topic:
            self.create_subscription(Float32, rate_topic, self._on_rate, 20)
        self.create_subscription(Bool, self._p("imu_valid_topic"), self._on_valid, 20)
        self.create_subscription(UInt8, "/intersection/command", self._on_command, 10)
        self.add_on_set_parameters_callback(self._parameters_changed)
        self.create_timer(1.0 / float(self.get_parameter("control_rate_hz").value), self._tick)

    def _controller_config(self, prefix: str) -> ControllerConfig:
        values = {}
        for name in ControllerConfig().__dict__:
            parameter = prefix + name
            values[name] = self.get_parameter(parameter if self.has_parameter(parameter) else name).value
        return ControllerConfig(**values)

    def _p(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _parameters_changed(self, params):
        for param in params:
            if param.name == "intersection.active_dir":
                try:
                    validate_active_dir(param.value)
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                if self.intersection.active:
                    return SetParametersResult(successful=False,
                                               reason="cannot change active_dir while active")
        for param in params:
            if param.name == "intersection.active_dir":
                self.intersection.set_active_dir(param.value)
        return SetParametersResult(successful=True)

    def _on_fix(self, msg: NavSatFix) -> None:
        now = self._now()
        finite = math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        covariance = max(msg.position_covariance[0], msg.position_covariance[4])
        good = bool(finite and msg.status.status != NavSatStatus.STATUS_NO_FIX and
                    math.isfinite(covariance) and covariance <= self.max_cov)
        jump_m = 0.0
        if finite:
            meta = self.controller.route.metadata
            nx, ny = latlon_to_xy(msg.latitude, msg.longitude, meta.origin_lat, meta.origin_lon)
            # 안테나 → 차량 기준점 평면 보정 (heading 있을 때만).
            if not self.antenna_offset.is_zero():
                heading = getattr(self.controller.imu, "heading", None)
                if heading is not None and math.isfinite(heading):
                    nx, ny = self.antenna_offset.antenna_to_reference(nx, ny, heading)
            if self.last_accepted_xy is not None:
                jump_m = math.hypot(nx-self.last_accepted_xy[0], ny-self.last_accepted_xy[1])
                if jump_m > self.gps_jump:
                    good = False
            if good:
                self.last_accepted_xy = (nx, ny)
            self.controller.set_position(nx, ny, now, good=good)
            self.last_fix = (float(msg.latitude), float(msg.longitude))
        self.last_fix_time, self.last_fix_good = now, good
        # GPS 안정성(0~100%) 계산·발행. 재밍/멀티패스로 값이 튀면 낮아진다.
        has_fix = bool(finite and msg.status.status != NavSatStatus.STATUS_NO_FIX)
        self.stability_score = self.stability.score(
            has_fix=has_fix, covariance=covariance, jump_m=jump_m, age_s=0.0)
        self.stability_pub.publish(Float32(data=float(self.stability_score)))

    def _on_velocity(self, msg: TwistWithCovarianceStamped) -> None:
        vx, vy = float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)
        if math.isfinite(vx) and math.isfinite(vy) and math.hypot(vx, vy) > 0.2:
            self.entry_heading = math.atan2(vy, vx)
            self.entry_heading_time = self._now()

    def _push_imu(self) -> None:
        # A valid/rate message must never refresh heading freshness.  The
        # controller timeout is anchored exclusively to the last yaw sample.
        now = self._now()
        sample_time = self.last_imu_yaw_time if self.last_imu_yaw_time is not None else now
        sample_valid = bool(self.last_imu_valid and self.last_imu_yaw_time is not None)
        self.last_imu_time = sample_time
        self.controller.set_imu(self.last_imu_yaw, self.last_imu_rate,
                                sample_valid, sample_time)

    def _on_yaw(self, msg: Float32) -> None:
        self.last_imu_yaw = float(msg.data)
        self.last_imu_yaw_time = self._now()
        self._push_imu()

    def _on_rate(self, msg: Float32) -> None:
        self.last_imu_rate = float(msg.data)
        self._push_imu()

    def _on_valid(self, msg: Bool) -> None:
        self.last_imu_valid = bool(msg.data)
        self._push_imu()

    def _on_command(self, msg: UInt8) -> None:
        if self.intersection_enabled and not self.intersection.on_command(msg.data):
            self.get_logger().warning(
                f"ignored intersection command={msg.data}; state={self.intersection.state.value}")

    def _intersection_step(self, now: float):
        previous_state = self.intersection.state
        controller = self.intersection.controller
        x = y = None
        if controller is not None and self.last_fix is not None:
            meta = controller.route.metadata
            x, y = latlon_to_xy(self.last_fix[0], self.last_fix[1],
                                meta.origin_lat, meta.origin_lon)
        cfg = self.intersection.navigation_config
        gps_ok = bool(self.last_fix_good and self.last_fix_time is not None and
                      now - self.last_fix_time <= cfg.gps_timeout_sec)
        imu_ok = bool(self.last_imu_valid and self.last_imu_yaw_time is not None and
                      math.isfinite(self.last_imu_yaw) and
                      now-self.last_imu_yaw_time <= cfg.imu_timeout_sec)
        entry_heading = (self.entry_heading if self.entry_heading_time is not None and
                         now-self.entry_heading_time <= cfg.gps_timeout_sec else None)
        output = self.intersection.step(
            now, x=x, y=y, gps_healthy=gps_ok, imu_yaw_deg=self.last_imu_yaw,
            imu_rate_deg_s=self.last_imu_rate, imu_healthy=imu_ok,
            entry_heading_rad=entry_heading)
        if (previous_state.value == "PREPARE" and
                self.intersection.infeasible_curvature_segments):
            self.get_logger().warning(
                f"recorded route has {self.intersection.infeasible_curvature_segments} "
                "curvature samples below the configured turning radius")
        return output

    def _tick(self) -> None:
        now = self._now()
        output = self._intersection_step(now) if self.intersection.active else self.controller.step(now)
        self.drive_pub.publish(Float32(data=float(output.drive)))
        self.wheel_pub.publish(Int32(data=int(output.wheel)))
        self.mode_pub.publish(String(data=output.mode))
        state = self.intersection.state.value
        self.intersection_state_pub.publish(String(data=state))
        self.intersection_complete_pub.publish(Bool(data=self.intersection.complete))
        ix_controller = self.intersection.controller
        route_size = len(ix_controller.route.waypoints) if ix_controller else 0
        route_index = output.route_index if self.intersection.active else 0
        gps_healthy = bool(self.last_fix_good and self.last_fix_time is not None and
                           now-self.last_fix_time <= self.controller.config.gps_timeout_sec)
        imu_healthy = bool(self.last_imu_valid and self.last_imu_yaw_time is not None and
                           now-self.last_imu_yaw_time <= self.controller.config.imu_timeout_sec)
        ix_status = {
            "active_dir": self.intersection.active_dir,
            "selected_dir": self.intersection.selected_dir,
            "dir_select_mode": self.intersection.config.dir_select_mode,
            "active_command": self.intersection.active_command,
            "active_route": self.intersection.active_route, "state": state,
            "route_index": route_index,
            "route_progress": route_index/max(1, route_size-2) if route_size else 0.0,
            "cross_track_error_m": output.cross_track_error,
            "heading_error_deg": self.intersection.heading_error_deg,
            "current_x_m": ix_controller.x if ix_controller else None,
            "current_y_m": ix_controller.y if ix_controller else None,
            "target_x_m": output.target_x, "target_y_m": output.target_y,
            "gps_healthy": gps_healthy, "imu_healthy": imu_healthy,
            "drive_command": output.drive, "wheel_command": output.wheel,
            "fault_reason": self.intersection.fault_reason,
            "complete_confirm_count": self.intersection.complete_count,
            "infeasible_curvature_segments": self.intersection.infeasible_curvature_segments,
        }
        self.intersection_status_pub.publish(String(data=json.dumps(ix_status, separators=(",", ":"))))
        status = {
            "state": output.state.value, "route_index": output.route_index,
            "mode": output.mode, "cross_track_error_m": round(output.cross_track_error, 3),
            "direction": output.direction, "drive_level": output.drive_level,
            "gps_drive": output.drive, "gps_wheel": output.wheel,
            "target_x_m": output.target_x, "target_y_m": output.target_y,
            "imu_yaw_deg": round(self.last_imu_yaw, 3),
            "world_heading_deg": None if output.heading is None else round(math.degrees(output.heading), 3),
            "gps_healthy": gps_healthy, "imu_healthy": imu_healthy,
            "reason": output.reason, "intersection_state": state,
        }
        self.status_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = GpsRouteFollowerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
