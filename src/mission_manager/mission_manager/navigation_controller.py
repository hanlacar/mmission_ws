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

    # 저장 경로 밖에서 시작했을 때 자동 합류
    REJOINING = "REJOINING"

    TRACKING = "TRACKING"

    APPROACH_CUSP = "APPROACH_CUSP"
    STOPPED_AT_CUSP = "STOPPED_AT_CUSP"

    GOAL_REACHED = "GOAL_REACHED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ControllerConfig:
    # ----------------------------------------------------------
    # Vehicle
    # ----------------------------------------------------------

    wheelbase_m: float = 0.30
    max_steering_deg: float = 27.0
    steering_sign: int = 1

    # ----------------------------------------------------------
    # Pure pursuit
    # ----------------------------------------------------------

    lookahead_slow_m: float = 0.7
    lookahead_normal_m: float = 1.0
    lookahead_fast_m: float = 1.4
    lookahead_reverse_m: float = 0.65

    # ----------------------------------------------------------
    # Sensor timeout
    # ----------------------------------------------------------

    gps_timeout_sec: float = 1.0
    imu_timeout_sec: float = 0.2

    imu_reset_jump_threshold_deg: float = 35.0
    imu_reset_yaw_rate_margin_deg: float = 8.0

    # ----------------------------------------------------------
    # Route start
    # ----------------------------------------------------------

    # nearest:
    # 전체 저장 경로 중 현재 차량과 가장 적합한 segment 선택
    start_policy: str = "nearest"

    # 경로에서 1m 이내면 바로 TRACKING
    start_accept_radius_m: float = 1.0

    # 1~5m이면 REJOINING
    # 5m보다 멀면 주행하지 않음
    rejoin_max_distance_m: float = 5.0

    # 재합류 시 저속
    rejoin_speed_level: float = 1.0

    # 재합류 시 선택된 segment 앞쪽 target
    rejoin_lookahead_m: float = 1.0

    start_heading_tolerance_deg: float = 70.0

    # ----------------------------------------------------------
    # Off route
    # ----------------------------------------------------------

    off_route_warn_m: float = 1.0
    off_route_stop_m: float = 2.0

    off_route_stop_count: int = 3
    off_route_recover_count: int = 5

    # ----------------------------------------------------------
    # Forward / reverse transition
    # ----------------------------------------------------------

    cusp_tolerance_m: float = 0.35
    cusp_approach_m: float = 1.2
    direction_change_stop_s: float = 1.0

    # ----------------------------------------------------------
    # Goal
    # ----------------------------------------------------------

    goal_tolerance_m: float = 0.3
    goal_slowdown_m: float = 1.0

    # ----------------------------------------------------------
    # Steering slowdown
    # ----------------------------------------------------------

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
    def __init__(
        self,
        route: Route,
        config: ControllerConfig,
    ) -> None:

        self.route = route
        self.config = config

        self.tracker = RouteTracker(route)

        self.imu = ImuHeadingEstimator(
            config.imu_timeout_sec,
            config.imu_reset_jump_threshold_deg,
            config.imu_reset_yaw_rate_margin_deg,
        )

        self.state = FollowerState.WAITING_FOR_POSITION

        # ------------------------------------------------------
        # Position
        # ------------------------------------------------------

        self.x: Optional[float] = None
        self.y: Optional[float] = None

        self.last_fix_time: Optional[float] = None
        self.gps_good = False

        # ------------------------------------------------------
        # IMU
        # ------------------------------------------------------

        self.imu_yaw = 0.0
        self.imu_rate = 0.0
        self.imu_valid = False

        self.last_imu_time: Optional[float] = None

        # ------------------------------------------------------
        # Direction change
        # ------------------------------------------------------

        self.stop_started: Optional[float] = None

        # ------------------------------------------------------
        # Off-route
        # ------------------------------------------------------

        self.offroute_bad = 0
        self.offroute_good = 0

        self.offroute_latched = False

    # ==========================================================
    # Sensor input
    # ==========================================================

    def set_position(
        self,
        x: float,
        y: float,
        now: float,
        *,
        good: bool = True,
    ) -> None:

        self.x = x
        self.y = y

        self.last_fix_time = now
        self.gps_good = good

    def set_imu(
        self,
        yaw_deg: float,
        yaw_rate_deg_s: float,
        valid: bool,
        now: float,
    ) -> None:

        self.imu_yaw = yaw_deg
        self.imu_rate = yaw_rate_deg_s
        self.imu_valid = valid

        self.last_imu_time = now

        if self.imu.anchor is not None:
            self.imu.update(
                yaw_deg,
                yaw_rate_deg_s,
                valid,
                now,
            )

    # ==========================================================
    # Stop helper
    # ==========================================================

    def _stop(
        self,
        reason: str,
        state: Optional[FollowerState] = None,
    ) -> ControlOutput:

        if state is not None:
            self.state = state

        segment = max(
            0,
            min(
                self.tracker.segment,
                len(self.route.waypoints) - 1,
            ),
        )

        point = self.route.waypoints[segment]

        return ControlOutput(
            drive=0.0,
            wheel=0,
            mode=point.mode,
            state=self.state,
            route_index=segment,
            cross_track_error=0.0,
            heading=self.imu.heading,
            reason=reason,
            target_x=None,
            target_y=None,
            direction=point.direction.value,
            drive_level=0.0,
        )

    # ==========================================================
    # REJOIN
    # ==========================================================

    def _rejoin_step(
        self,
        heading: float,
    ) -> Optional[ControlOutput]:
        """
        선택된 저장 경로 segment로 자동 접근한다.

        경로와 거리가 start_accept_radius_m 이하가 되면
        TRACKING으로 전환하고 None을 반환한다.

        None을 반환하면 현재 step()에서 그대로
        정상 Pure Pursuit 추종을 이어간다.
        """

        projection = self.tracker._project(
            self.x,
            self.y,
            self.tracker.segment,
        )

        # ------------------------------------------------------
        # 경로 진입 완료
        # ------------------------------------------------------

        if (
            projection.distance
            <= self.config.start_accept_radius_m
        ):
            self.state = FollowerState.TRACKING

            # REJOIN 중 경로이탈 카운터 초기화
            self.offroute_bad = 0
            self.offroute_good = 0
            self.offroute_latched = False

            return None

        # ------------------------------------------------------
        # 합류 가능한 범위 밖
        # ------------------------------------------------------

        if (
            projection.distance
            > self.config.rejoin_max_distance_m
        ):
            return self._stop(
                "rejoin target too far",
                FollowerState.ALIGNING,
            )

        point = self.route.waypoints[
            self.tracker.segment
        ]

        direction = point.direction.value

        # ------------------------------------------------------
        # 선택 segment 앞쪽을 목표점으로 사용
        # ------------------------------------------------------

        tx, ty = self.tracker.target(
            self.x,
            self.y,
            self.config.rejoin_lookahead_m,
        )

        steer = steering_angle(
            self.x,
            self.y,
            heading,
            tx,
            ty,
            direction,
            self.config.wheelbase_m,
            self.config.max_steering_deg,
        )

        wheel = int(
            round(
                steer
                * self.config.steering_sign
            )
        )

        wheel = max(
            -int(self.config.max_steering_deg),
            min(
                int(self.config.max_steering_deg),
                wheel,
            ),
        )

        # 재합류는 level 1 이하 저속
        level = min(
            abs(
                float(
                    self.config.rejoin_speed_level
                )
            ),
            1.0,
        )

        drive = round(
            direction * level + 1.0e-9,
            2,
        )

        return ControlOutput(
            drive=drive,
            wheel=wheel,
            mode=point.mode,
            state=FollowerState.REJOINING,
            route_index=self.tracker.segment,
            cross_track_error=projection.distance,
            heading=heading,
            reason="rejoining route",
            target_x=tx,
            target_y=ty,
            direction=direction,
            drive_level=level,
        )

    # ==========================================================
    # Main control
    # ==========================================================

    def step(
        self,
        now: float,
    ) -> ControlOutput:

        # ======================================================
        # GPS health
        # ======================================================

        if (
            self.x is None
            or self.y is None
            or self.last_fix_time is None
            or not self.gps_good
            or (
                now
                - self.last_fix_time
                > self.config.gps_timeout_sec
            )
        ):
            return self._stop(
                "GPS invalid or stale",
                FollowerState.WAITING_FOR_POSITION,
            )

        # ======================================================
        # IMU health
        # ======================================================

        if (
            not self.imu_valid
            or self.last_imu_time is None
            or (
                now
                - self.last_imu_time
                > self.config.imu_timeout_sec
            )
        ):
            return self._stop(
                "IMU invalid or stale",
                FollowerState.WAITING_FOR_IMU,
            )

        # ======================================================
        # 최초 경로 segment 검색
        # ======================================================

        if self.imu.anchor is None:

            start = self.tracker.select_start(
                self.x,
                self.y,
                None,
                self.config.start_policy,
            )

            # 5m 밖이면 주행 금지
            if (
                start.distance
                > self.config.rejoin_max_distance_m
            ):
                return self._stop(
                    "route start too far",
                    FollowerState.ALIGNING,
                )

            route_heading = self.tracker.tangent(
                start.segment
            )

            self.imu.initialize(
                route_heading,
                self.imu_yaw,
                now,
            )

            # 1m 밖이면 자동 합류
            if (
                start.distance
                > self.config.start_accept_radius_m
            ):
                self.state = FollowerState.REJOINING

        # ======================================================
        # Heading health
        # ======================================================

        if (
            not self.imu.healthy(now)
            or self.imu.heading is None
        ):
            return self._stop(
                "heading unavailable",
                FollowerState.WAITING_FOR_IMU,
            )

        heading = self.imu.heading

        # ======================================================
        # Tracker initialization fallback
        # ======================================================

        if not self.tracker.initialized:

            start = self.tracker.select_start(
                self.x,
                self.y,
                heading,
                self.config.start_policy,
            )

            if (
                start.distance
                > self.config.rejoin_max_distance_m
            ):
                return self._stop(
                    "route start too far",
                    FollowerState.ALIGNING,
                )

            if (
                start.distance
                > self.config.start_accept_radius_m
            ):
                self.state = FollowerState.REJOINING

        # ======================================================
        # 시작 heading 검사
        # ======================================================

        if self.state in (
            FollowerState.WAITING_FOR_POSITION,
            FollowerState.WAITING_FOR_IMU,
            FollowerState.ALIGNING,
        ):

            tangent = self.tracker.tangent(
                self.tracker.segment
            )

            heading_error = abs(
                math.degrees(
                    normalize_angle(
                        tangent - heading
                    )
                )
            )

            if (
                heading_error
                > self.config.start_heading_tolerance_deg
            ):
                return self._stop(
                    "start heading mismatch",
                    FollowerState.ALIGNING,
                )

        # ======================================================
        # REJOINING
        # ======================================================

        if (
            self.state
            == FollowerState.REJOINING
        ):

            output = self._rejoin_step(
                heading
            )

            # 아직 합류 중
            if output is not None:
                return output

            # output=None이면 방금 1m 이내 진입
            # 아래 정상 TRACKING으로 이어짐

        # ======================================================
        # Normal route tracking
        # ======================================================

        projection = self.tracker.update(
            self.x,
            self.y,
            heading,
        )

        # ======================================================
        # Off-route safety
        # ======================================================

        if (
            projection.distance
            >= self.config.off_route_stop_m
        ):

            self.offroute_bad += 1
            self.offroute_good = 0

        else:

            self.offroute_good += 1
            self.offroute_bad = 0

        if (
            self.offroute_bad
            >= self.config.off_route_stop_count
        ):
            self.offroute_latched = True

        if (
            self.offroute_good
            >= self.config.off_route_recover_count
        ):
            self.offroute_latched = False

        if self.offroute_latched:
            return self._stop(
                "off route",
                FollowerState.FAULT,
            )

        # ======================================================
        # Goal
        # ======================================================

        last = self.route.waypoints[-1]

        goal_distance = math.hypot(
            last.x_m - self.x,
            last.y_m - self.y,
        )

        # ------------------------------------------------------
        # 중요:
        #
        # 단순히 마지막 좌표와 가까운지만 보면
        # 시작점과 마지막점이 가까운 경로에서
        # 시작하자마자 GOAL_REACHED가 될 수 있음.
        #
        # 따라서 tracker가 실제 마지막 segment까지
        # 진행했을 때만 goal 판정을 허용한다.
        # ------------------------------------------------------

        last_segment = max(
            0,
            len(self.route.waypoints) - 2,
        )

        if (
            not self.route.metadata.loop
            and self.tracker.segment
            >= last_segment
            and goal_distance
            <= self.config.goal_tolerance_m
        ):
            return self._stop(
                "goal reached",
                FollowerState.GOAL_REACHED,
            )

        if (
            self.state
            == FollowerState.GOAL_REACHED
        ):
            return self._stop(
                "goal latched"
            )

        # ======================================================
        # Direction change / cusp
        # ======================================================

        cusp = self.tracker.next_cusp()

        if cusp is not None:

            # 현재 방향의 마지막 waypoint
            cp = self.route.waypoints[
                cusp - 1
            ]

            cusp_distance = math.hypot(
                cp.x_m - self.x,
                cp.y_m - self.y,
            )

            # --------------------------------------------------
            # cusp 도착
            # --------------------------------------------------

            if (
                cusp_distance
                <= self.config.cusp_tolerance_m
            ):

                if (
                    self.state
                    != FollowerState.STOPPED_AT_CUSP
                ):
                    self.state = (
                        FollowerState.STOPPED_AT_CUSP
                    )

                    self.stop_started = now

                if (
                    now
                    - (self.stop_started or now)
                    < self.config.direction_change_stop_s
                ):
                    return self._stop(
                        "direction change dwell"
                    )

                # 다음 방향 segment로 이동
                self.tracker.segment = cusp

                self.stop_started = None
                self.state = (
                    FollowerState.TRACKING
                )

            # --------------------------------------------------
            # cusp 접근 중
            # --------------------------------------------------

            elif (
                cusp_distance
                <= self.config.cusp_approach_m
            ):

                self.state = (
                    FollowerState.APPROACH_CUSP
                )

            else:

                self.state = (
                    FollowerState.TRACKING
                )

        else:

            self.state = (
                FollowerState.TRACKING
            )

        # ======================================================
        # Current route point
        # ======================================================

        point = self.route.waypoints[
            self.tracker.segment
        ]

        direction = point.direction.value

        level = float(
            point.drive_level
        )

        # ======================================================
        # Slow down near cusp / goal
        # ======================================================

        if (
            self.state
            == FollowerState.APPROACH_CUSP
            or (
                not self.route.metadata.loop
                and goal_distance
                < self.config.goal_slowdown_m
            )
        ):

            level = min(
                level,
                1.0,
            )

        # ======================================================
        # Off-route warning slowdown
        # ======================================================

        if (
            projection.distance
            >= self.config.off_route_warn_m
        ):

            level = min(
                level,
                1.0,
            )

        # ======================================================
        # Lookahead
        # ======================================================

        if direction < 0:

            lookahead = (
                self.config.lookahead_reverse_m
            )

        else:

            if level <= 1.0:
                lookahead = (
                    self.config.lookahead_slow_m
                )

            elif level <= 2.0:
                lookahead = (
                    self.config.lookahead_normal_m
                )

            else:
                lookahead = (
                    self.config.lookahead_fast_m
                )

        # ======================================================
        # Pure Pursuit target
        # ======================================================

        tx, ty = self.tracker.target(
            self.x,
            self.y,
            lookahead,
        )

        steer = steering_angle(
            self.x,
            self.y,
            heading,
            tx,
            ty,
            direction,
            self.config.wheelbase_m,
            self.config.max_steering_deg,
        )

        # ======================================================
        # Large steering slowdown
        # ======================================================

        if (
            abs(steer)
            >= self.config.steering_slowdown_deg
        ):

            level = min(
                level,
                1.0,
            )

        # ======================================================
        # Final command
        # ======================================================

        drive = round(
            direction * level + 1.0e-9,
            2,
        )

        wheel = int(
            round(
                steer
                * self.config.steering_sign
            )
        )

        wheel = max(
            -int(self.config.max_steering_deg),
            min(
                int(self.config.max_steering_deg),
                wheel,
            ),
        )

        return ControlOutput(
            drive=drive,
            wheel=wheel,
            mode=point.mode,
            state=self.state,
            route_index=self.tracker.segment,
            cross_track_error=projection.distance,
            heading=heading,
            target_x=tx,
            target_y=ty,
            direction=direction,
            drive_level=level,
        )