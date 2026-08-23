"""ROS-independent GPS/IMU route follower and safety state machine."""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .imu_heading_estimator import ImuHeadingEstimator, normalize_angle
from .pure_pursuit import steering_angle
from .route_model import Route
from .route_tracker import RouteTracker


class FollowerState(str, Enum):
    WAITING_FOR_POSITION = "WAITING_FOR_POSITION"
    WAITING_FOR_IMU = "WAITING_FOR_IMU"
    ALIGNING = "ALIGNING"
    TRACKING = "TRACKING"
    APPROACH_CUSP = "APPROACH_CUSP"
    STOPPED_AT_CUSP = "STOPPED_AT_CUSP"
    GOAL_REACHED = "GOAL_REACHED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ControllerConfig:
    wheelbase_m: float = 0.30
    max_steering_deg: float = 27.0
    steering_sign: int = 1
    lookahead_slow_m: float = 0.7
    lookahead_normal_m: float = 1.0
    lookahead_fast_m: float = 1.4
    lookahead_reverse_m: float = 0.65
    gps_timeout_sec: float = 1.0
    imu_timeout_sec: float = 0.2
    imu_reset_jump_threshold_deg: float = 35.0
    imu_reset_yaw_rate_margin_deg: float = 8.0
    start_policy: str = "nearest"
    start_accept_radius_m: float = 2.0
    start_heading_tolerance_deg: float = 70.0
    off_route_warn_m: float = 1.0
    off_route_stop_m: float = 2.0
    off_route_stop_count: int = 3
    off_route_recover_count: int = 5
    cusp_tolerance_m: float = 0.35
    cusp_approach_m: float = 1.2
    direction_change_stop_s: float = 1.0
    goal_tolerance_m: float = 0.3
    goal_slowdown_m: float = 1.0
    steering_slowdown_deg: float = 18.0


@dataclass(frozen=True)
class ControlOutput:
    drive: float
    wheel: int
    mode: str
    state: FollowerState
    route_index: int
    cross_track_error: float
    heading: Optional[float]
    reason: str = ""
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    direction: int = 1
    drive_level: float = 0.0


class NavigationController:
    def __init__(self, route: Route, config: ControllerConfig) -> None:
        self.route = route
        self.config = config
        self.tracker = RouteTracker(route)
        self.imu = ImuHeadingEstimator(
            config.imu_timeout_sec, config.imu_reset_jump_threshold_deg,
            config.imu_reset_yaw_rate_margin_deg)
        self.state = FollowerState.WAITING_FOR_POSITION
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.last_fix_time: Optional[float] = None
        self.gps_good = False
        self.imu_yaw = 0.0
        self.imu_rate = 0.0
        self.imu_valid = False
        self.last_imu_time: Optional[float] = None
        self.stop_started: Optional[float] = None
        self.offroute_bad = 0
        self.offroute_good = 0
        self.offroute_latched = False

    def set_position(self, x: float, y: float, now: float, *, good: bool = True) -> None:
        self.x, self.y, self.last_fix_time, self.gps_good = x, y, now, good

    def set_imu(self, yaw_deg: float, yaw_rate_deg_s: float,
                valid: bool, now: float) -> None:
        self.imu_yaw, self.imu_rate, self.imu_valid = yaw_deg, yaw_rate_deg_s, valid
        self.last_imu_time = now
        if self.imu.anchor is not None:
            self.imu.update(yaw_deg, yaw_rate_deg_s, valid, now)

    def _stop(self, reason: str, state: Optional[FollowerState] = None) -> ControlOutput:
        if state is not None:
            self.state = state
        mode = self.route.waypoints[self.tracker.segment].mode
        return ControlOutput(0.0, 0, mode, self.state, self.tracker.segment,
                             0.0, self.imu.heading, reason)

    def step(self, now: float) -> ControlOutput:
        if (self.x is None or self.y is None or self.last_fix_time is None or
                not self.gps_good or now - self.last_fix_time > self.config.gps_timeout_sec):
            return self._stop("GPS invalid or stale", FollowerState.WAITING_FOR_POSITION)
        if (not self.imu_valid or self.last_imu_time is None or
                now - self.last_imu_time > self.config.imu_timeout_sec):
            return self._stop("IMU invalid or stale", FollowerState.WAITING_FOR_IMU)

        if self.imu.anchor is None:
            start = self.tracker.select_start(self.x, self.y, None, self.config.start_policy)
            if start.distance > self.config.start_accept_radius_m:
                return self._stop("route start too far", FollowerState.ALIGNING)
            route_heading = self.tracker.tangent(start.segment)
            self.imu.initialize(route_heading, self.imu_yaw, now)

        if not self.imu.healthy(now) or self.imu.heading is None:
            return self._stop("heading unavailable", FollowerState.WAITING_FOR_IMU)
        heading = self.imu.heading
        if not self.tracker.initialized:
            start = self.tracker.select_start(self.x, self.y, heading, self.config.start_policy)
            if start.distance > self.config.start_accept_radius_m:
                return self._stop("route start too far", FollowerState.ALIGNING)
        tangent = self.tracker.tangent(self.tracker.segment)
        if self.state in (FollowerState.WAITING_FOR_POSITION, FollowerState.WAITING_FOR_IMU,
                          FollowerState.ALIGNING):
            error = abs(math.degrees(normalize_angle(tangent - heading)))
            if error > self.config.start_heading_tolerance_deg:
                return self._stop("start heading mismatch", FollowerState.ALIGNING)

        projection = self.tracker.update(self.x, self.y, heading)
        if projection.distance >= self.config.off_route_stop_m:
            self.offroute_bad += 1
            self.offroute_good = 0
        else:
            self.offroute_good += 1
            self.offroute_bad = 0
        if self.offroute_bad >= self.config.off_route_stop_count:
            self.offroute_latched = True
        if self.offroute_good >= self.config.off_route_recover_count:
            self.offroute_latched = False
        if self.offroute_latched:
            return self._stop("off route", FollowerState.FAULT)

        last = self.route.waypoints[-1]
        goal_distance = math.hypot(last.x_m - self.x, last.y_m - self.y)
        if not self.route.metadata.loop and goal_distance <= self.config.goal_tolerance_m:
            return self._stop("goal reached", FollowerState.GOAL_REACHED)
        if self.state == FollowerState.GOAL_REACHED:
            return self._stop("goal latched")

        cusp = self.tracker.next_cusp()
        if cusp is not None:
            # Stop at the last waypoint of the current direction, then advance
            # to the first waypoint of the new direction after the dwell.
            cp = self.route.waypoints[cusp - 1]
            cusp_distance = math.hypot(cp.x_m - self.x, cp.y_m - self.y)
            if cusp_distance <= self.config.cusp_tolerance_m:
                if self.state != FollowerState.STOPPED_AT_CUSP:
                    self.state = FollowerState.STOPPED_AT_CUSP
                    self.stop_started = now
                if now - (self.stop_started or now) < self.config.direction_change_stop_s:
                    return self._stop("direction change dwell")
                self.tracker.segment = cusp
                self.stop_started = None
                self.state = FollowerState.TRACKING
            elif cusp_distance <= self.config.cusp_approach_m:
                self.state = FollowerState.APPROACH_CUSP
            else:
                self.state = FollowerState.TRACKING
        else:
            self.state = FollowerState.TRACKING

        point = self.route.waypoints[self.tracker.segment]
        direction = point.direction.value
        level = point.drive_level
        if (self.state == FollowerState.APPROACH_CUSP or
                (not self.route.metadata.loop and goal_distance < self.config.goal_slowdown_m)):
            level = min(level, 1.0)
        if projection.distance >= self.config.off_route_warn_m:
            level = min(level, 1.0)
        lookahead = self.config.lookahead_reverse_m if direction < 0 else {
            1.0: self.config.lookahead_slow_m,
            2.0: self.config.lookahead_normal_m,
            3.0: self.config.lookahead_fast_m,
        }[level]
        tx, ty = self.tracker.target(self.x, self.y, lookahead)
        steer = steering_angle(self.x, self.y, heading, tx, ty, direction,
                               self.config.wheelbase_m, self.config.max_steering_deg)
        if abs(steer) >= self.config.steering_slowdown_deg:
            level = min(level, 1.0)
        drive = round(direction * level + 1.0e-9, 2)
        wheel = int(round(steer * self.config.steering_sign))
        wheel = max(-int(self.config.max_steering_deg),
                    min(int(self.config.max_steering_deg), wheel))
        return ControlOutput(drive, wheel, point.mode, self.state,
                             self.tracker.segment, projection.distance, heading,
                             target_x=tx, target_y=ty, direction=direction,
                             drive_level=level)
