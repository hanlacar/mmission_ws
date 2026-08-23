"""Segment projection, branch-safe progress, lookahead and cusp discovery."""

import math
from dataclasses import dataclass
from typing import Optional

from .imu_heading_estimator import normalize_angle
from .route_geometry import project_to_segment
from .route_model import Route


@dataclass(frozen=True)
class Projection:
    segment: int
    x: float
    y: float
    distance: float


class RouteTracker:
    def __init__(self, route: Route, local_window: int = 30) -> None:
        self.route = route
        self.local_window = local_window
        self.segment = 0
        self.initialized = False

    def tangent(self, segment: int, *, body: bool = True) -> float:
        points = self.route.waypoints
        p = points[segment]
        q = points[(segment + 1) % len(points)]
        angle = math.atan2(q.y_m - p.y_m, q.x_m - p.x_m)
        if body and p.direction.value < 0:
            angle = normalize_angle(angle + math.pi)
        return angle

    def _project(self, x: float, y: float, i: int) -> Projection:
        projection = project_to_segment(self.route.waypoints, x, y, i)
        return Projection(i, projection.x, projection.y, projection.distance)

    def select_start(self, x: float, y: float, body_heading: Optional[float],
                     policy: str, heading_weight_m: float = 2.0) -> Projection:
        segment_count = (len(self.route.waypoints) if self.route.metadata.loop
                         else len(self.route.waypoints) - 1)
        candidates = range(segment_count)
        if policy == "route_start":
            candidates = range(1)
        best = None
        best_score = float("inf")
        for i in candidates:
            projection = self._project(x, y, i)
            heading_error = 0.0 if body_heading is None else abs(normalize_angle(self.tangent(i) - body_heading))
            score = projection.distance + heading_weight_m * heading_error
            if score < best_score:
                best, best_score = projection, score
        assert best is not None
        self.segment = best.segment
        self.initialized = True
        return best

    def update(self, x: float, y: float, body_heading: float) -> Projection:
        best = self._project(x, y, self.segment)
        best_score = best.distance
        segment_count = (len(self.route.waypoints) if self.route.metadata.loop
                         else len(self.route.waypoints) - 1)
        count = min(self.local_window, segment_count)
        candidates = ((self.segment + offset) % segment_count for offset in range(count))
        if not self.route.metadata.loop:
            candidates = range(self.segment, min(segment_count, self.segment + count))
        for i in candidates:
            p = self._project(x, y, i)
            heading_error = abs(normalize_angle(self.tangent(i) - body_heading))
            score = p.distance + heading_error
            if score < best_score:
                best, best_score = p, score
        self.segment = best.segment if self.route.metadata.loop else max(self.segment, best.segment)
        return best

    def target(self, x: float, y: float, lookahead_m: float) -> tuple[float, float]:
        points = self.route.waypoints
        if self.route.metadata.loop:
            candidates = (points[(self.segment + offset) % len(points)]
                          for offset in range(1, len(points) + 1))
        else:
            candidates = iter(points[self.segment + 1:])
        for point in candidates:
            if math.hypot(point.x_m - x, point.y_m - y) >= lookahead_m:
                return point.x_m, point.y_m
        last = points[-1]
        return last.x_m, last.y_m

    def next_cusp(self) -> Optional[int]:
        points = self.route.waypoints
        if self.route.metadata.loop:
            for offset in range(1, len(points) + 1):
                i = (self.segment + offset) % len(points)
                if points[i].direction != points[i - 1].direction:
                    return i
            return None
        for i in range(self.segment + 1, len(points)):
            if points[i].direction != points[i - 1].direction:
                return i
        return None
