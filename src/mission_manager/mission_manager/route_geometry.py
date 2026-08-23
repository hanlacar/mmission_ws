"""Shared route-segment projection and progress geometry."""

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class SegmentProjection:
    segment: int
    ratio: float
    x: float
    y: float
    distance: float


def project_to_segment(points: Sequence, x: float, y: float,
                       segment: int) -> SegmentProjection:
    """Project a point onto one route segment, clamping to its endpoints."""
    p = points[segment]
    q = points[(segment + 1) % len(points)]
    vx, vy = q.x_m - p.x_m, q.y_m - p.y_m
    denominator = vx * vx + vy * vy
    ratio = (0.0 if denominator == 0.0 else
             max(0.0, min(1.0, ((x-p.x_m)*vx + (y-p.y_m)*vy) / denominator)))
    px, py = p.x_m + ratio * vx, p.y_m + ratio * vy
    return SegmentProjection(segment, ratio, px, py, math.hypot(x-px, y-py))


class RouteGeometry:
    """Global nearest projection and along-route distance for a loaded route."""

    def __init__(self, route) -> None:
        self.route = route
        self.segment_count = (len(route.waypoints) if route.metadata.loop
                              else len(route.waypoints) - 1)
        self.segment_lengths = []
        self.cumulative_lengths = [0.0]
        for segment in range(self.segment_count):
            p = route.waypoints[segment]
            q = route.waypoints[(segment + 1) % len(route.waypoints)]
            length = math.hypot(q.x_m - p.x_m, q.y_m - p.y_m)
            self.segment_lengths.append(length)
            self.cumulative_lengths.append(self.cumulative_lengths[-1] + length)
        self.total_length_m = self.cumulative_lengths[-1]

    def nearest(self, x: float, y: float) -> SegmentProjection:
        return min(
            (project_to_segment(self.route.waypoints, x, y, segment)
             for segment in range(self.segment_count)),
            key=lambda projection: projection.distance,
        )

    def along_route_m(self, projection: SegmentProjection) -> float:
        return (self.cumulative_lengths[projection.segment] +
                projection.ratio * self.segment_lengths[projection.segment])

    def progress_pct(self, projection: SegmentProjection) -> float:
        if self.total_length_m <= 0.0:
            return 0.0
        return 100.0 * self.along_route_m(projection) / self.total_length_m
