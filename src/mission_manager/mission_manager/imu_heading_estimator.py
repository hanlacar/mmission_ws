"""Anchor relative IMU yaw to a world-frame body heading."""

import math
from dataclasses import dataclass
from typing import Optional


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class HeadingUpdate:
    heading: Optional[float]
    reset_detected: bool = False


class ImuHeadingEstimator:
    def __init__(self, timeout_sec: float, reset_jump_deg: float,
                 yaw_rate_margin_deg: float) -> None:
        self.timeout_sec = timeout_sec
        self.reset_jump = math.radians(reset_jump_deg)
        self.rate_margin = math.radians(yaw_rate_margin_deg)
        self.anchor: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.last_time: Optional[float] = None
        self.last_valid_time: Optional[float] = None
        self.valid = False

    def initialize(self, route_body_heading: float, imu_yaw_deg: float,
                   now: float) -> float:
        yaw = math.radians(imu_yaw_deg)
        self.anchor = normalize_angle(route_body_heading - yaw)
        self.last_yaw = yaw
        self.last_time = now
        self.last_valid_time = now
        self.valid = True
        return self.heading  # type: ignore[return-value]

    def update(self, imu_yaw_deg: float, yaw_rate_deg_s: float,
               valid: bool, now: float) -> HeadingUpdate:
        if not valid or not all(math.isfinite(v) for v in (imu_yaw_deg, yaw_rate_deg_s, now)):
            self.valid = False
            return HeadingUpdate(None)
        yaw = math.radians(imu_yaw_deg)
        reset = False
        if self.anchor is not None and self.last_yaw is not None and self.last_time is not None:
            dt = max(0.0, now - self.last_time)
            jump = abs(normalize_angle(yaw - self.last_yaw))
            explainable = abs(math.radians(yaw_rate_deg_s)) * dt + self.rate_margin
            if jump >= self.reset_jump and jump > explainable:
                world_before = normalize_angle(self.anchor + self.last_yaw)
                self.anchor = normalize_angle(world_before - yaw)
                reset = True
        self.last_yaw = yaw
        self.last_time = now
        self.last_valid_time = now
        self.valid = True
        return HeadingUpdate(self.heading, reset)

    def healthy(self, now: float) -> bool:
        return (self.valid and self.anchor is not None and
                self.last_valid_time is not None and
                now - self.last_valid_time <= self.timeout_sec)

    @property
    def heading(self) -> Optional[float]:
        if self.anchor is None or self.last_yaw is None:
            return None
        return normalize_angle(self.anchor + self.last_yaw)

