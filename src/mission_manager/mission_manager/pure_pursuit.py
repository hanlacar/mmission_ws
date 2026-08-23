"""Direction-aware Pure Pursuit math with physical front-wheel convention."""

import math

from .imu_heading_estimator import normalize_angle


def steering_angle(x: float, y: float, body_heading: float,
                   target_x: float, target_y: float, direction: int,
                   wheelbase_m: float, max_steering_deg: float) -> float:
    """Return front-wheel angle in degrees; positive means wheel left."""
    dx, dy = target_x - x, target_y - y
    motion_heading = normalize_angle(body_heading if direction > 0 else body_heading + math.pi)
    alpha = normalize_angle(math.atan2(dy, dx) - motion_heading)
    distance = math.hypot(dx, dy)
    if distance < 1.0e-6:
        return 0.0
    motion_curvature = 2.0 * math.sin(alpha) / distance
    # Reverse motion reverses yaw response for the same physical wheel angle.
    body_curvature = motion_curvature if direction > 0 else -motion_curvature
    angle = math.degrees(math.atan(wheelbase_m * body_curvature))
    return max(-max_steering_deg, min(max_steering_deg, angle))

