"""Production GPS reference-route data model."""

from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class Direction(int, Enum):
    FORWARD = 1
    REVERSE = -1


@dataclass(frozen=True)
class Waypoint:
    index: int
    latitude: float
    longitude: float
    x_m: float
    y_m: float
    direction: Direction
    mode: str
    drive_level: float


@dataclass(frozen=True)
class RouteMetadata:
    format_version: int
    origin_lat: float
    origin_lon: float
    loop: bool = False
    created_at: str = ""


@dataclass(frozen=True)
class Route:
    metadata: RouteMetadata
    waypoints: Sequence[Waypoint]

