"""ROS-independent reference-route intersection orchestration.

This module selects a recorded route and owns a fresh NavigationController for
each command.  It deliberately does not implement steering: all tracking and
GPS/IMU health checks remain in the production navigation core.
"""

import math
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Callable, Optional

from . import intersection_command as command
from .imu_heading_estimator import normalize_angle
from .intersection import compass_deg_to_dir, math_heading_to_compass_deg
from .navigation_controller import (
    ControllerConfig, ControlOutput, FollowerState, NavigationController,
)
from .route_loader import load_route
from .route_model import Route


VALID_DIRS = ("N", "E", "S", "W")


class IntersectionState(str, Enum):
    IDLE = "IDLE"
    PREPARE = "PREPARE"
    ALIGN_ROUTE = "ALIGN_ROUTE"
    TRACKING_STRAIGHT = "TRACKING_STRAIGHT"
    TRACKING_LEFT = "TRACKING_LEFT"
    TRACKING_RIGHT = "TRACKING_RIGHT"
    EXIT_CONFIRM = "EXIT_CONFIRM"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class IntersectionConfig:
    route_dir: str
    dir_select_mode: str = "manual"
    active_dir: str = "N"
    drive_level: float = 2.0
    heading_warn_deg: float = 10.0
    heading_stop_deg: float = 20.0
    heading_stop_count: int = 3
    heading_recover_count: int = 5
    goal_heading_tolerance_deg: float = 10.0
    complete_confirm_count: int = 5
    route_alignment_mode: str = "absolute"


def validate_active_dir(value: str) -> str:
    result = str(value).strip().upper()
    if result not in VALID_DIRS:
        raise ValueError("active_dir must be one of N/E/S/W")
    return result


def direction_from_math_heading(heading_rad: float) -> str:
    if not math.isfinite(heading_rad):
        raise ValueError("entry heading must be finite")
    return compass_deg_to_dir(math_heading_to_compass_deg(heading_rad))


def select_intersection_route(active_dir: str, raw_command: int) -> str:
    direction = validate_active_dir(active_dir)
    if raw_command not in (command.STRAIGHT, command.LEFT, command.RIGHT):
        raise ValueError("command must be STRAIGHT, LEFT, or RIGHT")
    return f"{direction}_{command.name(raw_command)}.csv"


def count_infeasible_curvature_segments(route: Route, wheelbase_m: float,
                                         max_steering_deg: float) -> int:
    """Count interior samples whose circumradius is below the bicycle limit."""
    minimum_radius = wheelbase_m / math.tan(math.radians(max_steering_deg))
    bad = 0
    points = route.waypoints
    for a, b, c in zip(points, points[1:], points[2:]):
        ab = math.hypot(b.x_m-a.x_m, b.y_m-a.y_m)
        bc = math.hypot(c.x_m-b.x_m, c.y_m-b.y_m)
        ca = math.hypot(a.x_m-c.x_m, a.y_m-c.y_m)
        twice_area = abs((b.x_m-a.x_m)*(c.y_m-a.y_m) -
                         (b.y_m-a.y_m)*(c.x_m-a.x_m))
        radius = math.inf if twice_area < 1.0e-9 else ab*bc*ca/(2.0*twice_area)
        bad += radius < minimum_radius
    return bad


def _zero(state: IntersectionState, reason: str = "") -> ControlOutput:
    return ControlOutput(0.0, 0, "INTERSECTION", FollowerState.FAULT, 0, 0.0,
                         None, reason=reason, drive_level=0.0)


class IntersectionRouteController:
    """Command latch, route lifecycle, safety arbitration and completion latch."""

    def __init__(self, config: IntersectionConfig, navigation_config: ControllerConfig,
                 route_loader: Callable[[str], Route] = load_route) -> None:
        if config.dir_select_mode not in ("manual", "auto"):
            raise ValueError("dir_select_mode must be manual or auto")
        if config.route_alignment_mode != "absolute":
            raise ValueError("only absolute route alignment is currently supported")
        validate_active_dir(config.active_dir)
        self.config = config
        self.navigation_config = navigation_config
        self.route_loader = route_loader
        self.active_dir = validate_active_dir(config.active_dir)
        self.reset()

    @property
    def active(self) -> bool:
        return self.state != IntersectionState.IDLE

    @property
    def complete(self) -> bool:
        return self.state == IntersectionState.COMPLETE

    def set_active_dir(self, value: str) -> None:
        self.active_dir = validate_active_dir(value)

    def on_command(self, raw_value: int) -> bool:
        try:
            value = int(raw_value)
        except (TypeError, ValueError):
            return False
        if value not in command.VALID:
            return False
        if value == command.NONE:
            # Active commands are latched. NONE releases only terminal states.
            if self.state in (IntersectionState.COMPLETE, IntersectionState.FAULT):
                self.reset()
            return True
        if self.state != IntersectionState.IDLE:
            return False
        self.active_command = value
        self.state = IntersectionState.PREPARE
        return True

    def reset(self) -> None:
        self.state = IntersectionState.IDLE
        self.active_command = command.NONE
        self.selected_dir = ""
        self.active_route = ""
        self.controller: Optional[NavigationController] = None
        self.fault_reason = ""
        self.heading_error_deg = 0.0
        self.heading_bad = 0
        self.heading_good = 0
        self.complete_count = 0
        self.infeasible_curvature_segments = 0
        self.last_output = _zero(IntersectionState.IDLE)

    def _fault(self, reason: str) -> ControlOutput:
        self.state = IntersectionState.FAULT
        self.fault_reason = reason
        self.last_output = _zero(self.state, reason)
        return self.last_output

    def _load(self, entry_heading_rad: Optional[float]) -> Optional[ControlOutput]:
        try:
            if self.config.dir_select_mode == "manual":
                self.selected_dir = validate_active_dir(self.active_dir)
            else:
                if entry_heading_rad is None:
                    return self._fault("auto direction requires a valid entry heading")
                self.selected_dir = direction_from_math_heading(entry_heading_rad)
            filename = select_intersection_route(self.selected_dir, self.active_command)
            path = str(Path(self.config.route_dir) / filename)
            route = self.route_loader(path)
            if any(p.direction.value != 1 or p.mode != "INTERSECTION" for p in route.waypoints):
                return self._fault("intersection route must be forward and mode=INTERSECTION")
            # The recorded value is normalized to the configured practice speed.
            route = replace(route, waypoints=tuple(
                replace(p, drive_level=self.config.drive_level) for p in route.waypoints))
            self.infeasible_curvature_segments = count_infeasible_curvature_segments(
                route, self.navigation_config.wheelbase_m,
                self.navigation_config.max_steering_deg)
            self.active_route = path
            self.controller = NavigationController(route, self.navigation_config)
            self.state = IntersectionState.ALIGN_ROUTE
            return None
        except (OSError, ValueError) as exc:
            return self._fault(str(exc))

    def step(self, now: float, *, x: Optional[float], y: Optional[float],
             gps_healthy: bool, imu_yaw_deg: float, imu_rate_deg_s: float,
             imu_healthy: bool, entry_heading_rad: Optional[float] = None) -> ControlOutput:
        if self.state == IntersectionState.IDLE:
            self.last_output = _zero(self.state)
            return self.last_output
        if self.state in (IntersectionState.COMPLETE, IntersectionState.FAULT):
            return self.last_output
        if self.state == IntersectionState.PREPARE:
            result = self._load(entry_heading_rad)
            self.last_output = result or _zero(self.state, "route loaded")
            return self.last_output
        if not gps_healthy or x is None or y is None or not all(map(math.isfinite, (x, y))):
            return self._fault("GPS invalid, stale, or jump detected")
        if not imu_healthy or not all(map(math.isfinite, (imu_yaw_deg, imu_rate_deg_s))):
            return self._fault("IMU invalid or stale")
        assert self.controller is not None
        self.controller.set_position(x, y, now, good=True)
        self.controller.set_imu(imu_yaw_deg, imu_rate_deg_s, True, now)
        output = self.controller.step(now)

        if output.state in (FollowerState.WAITING_FOR_POSITION, FollowerState.WAITING_FOR_IMU,
                            FollowerState.ALIGNING, FollowerState.FAULT):
            return self._fault(output.reason or output.state.value)

        tangent = self.controller.tracker.tangent(self.controller.tracker.segment)
        heading = output.heading
        if heading is None:
            return self._fault("heading unavailable")
        self.heading_error_deg = math.degrees(normalize_angle(tangent - heading))
        abs_error = abs(self.heading_error_deg)
        if abs_error >= self.config.heading_stop_deg:
            self.heading_bad += 1
            self.heading_good = 0
        else:
            self.heading_good += 1
            self.heading_bad = 0
        if self.heading_bad >= self.config.heading_stop_count:
            return self._fault("heading error stop threshold exceeded")
        if self.heading_good >= self.config.heading_recover_count:
            self.heading_bad = 0
        if abs_error >= self.config.heading_warn_deg and output.drive != 0.0:
            output = replace(output, drive=math.copysign(min(abs(output.drive), 1.0), output.drive),
                             drive_level=min(output.drive_level, 1.0))

        maneuver_states = {
            command.STRAIGHT: IntersectionState.TRACKING_STRAIGHT,
            command.LEFT: IntersectionState.TRACKING_LEFT,
            command.RIGHT: IntersectionState.TRACKING_RIGHT,
        }
        if output.state == FollowerState.GOAL_REACHED:
            self.state = IntersectionState.EXIT_CONFIRM
            final_segment = len(self.controller.route.waypoints) - 2
            position_ok = self.controller.tracker.segment >= final_segment
            heading_ok = abs_error <= self.config.goal_heading_tolerance_deg
            if position_ok and heading_ok and gps_healthy and imu_healthy:
                self.complete_count += 1
            else:
                self.complete_count = 0
            if self.complete_count >= self.config.complete_confirm_count:
                self.state = IntersectionState.COMPLETE
                output = _zero(self.state, "intersection complete")
        else:
            self.state = maneuver_states[self.active_command]

        self.last_output = output
        return output
