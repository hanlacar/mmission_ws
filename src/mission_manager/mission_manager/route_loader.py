"""Strict loader and validator for versioned GPS routes."""

import csv
import math
from pathlib import Path
from typing import Any, Dict

import yaml

from .route_model import (
    Direction,
    Route,
    RouteMetadata,
    Waypoint,
)


class RouteValidationError(ValueError):
    pass


REQUIRED_COLUMNS = (
    "index",
    "latitude",
    "longitude",
    "x_m",
    "y_m",
    "direction",
    "mode",
    "drive_level",
)


VALID_EVENTS = (
    "NONE",
    "STOP_LINE",
)


def _metadata_path(csv_path: Path) -> Path:
    return csv_path.with_suffix(".yaml")


def load_route(
    path: str,
    *,
    max_segment_m: float = 20.0,
) -> Route:

    if not path or not str(path).strip():
        raise RouteValidationError(
            "route_path is empty; "
            "launch must provide an absolute "
            "or share-relative path"
        )

    csv_path = Path(path)
    meta_path = _metadata_path(csv_path)

    if not csv_path.is_file():
        raise RouteValidationError(
            f"route file not found: {csv_path}"
        )

    if not meta_path.is_file():
        raise RouteValidationError(
            f"route metadata not found: {meta_path}"
        )

    # ----------------------------------------------------------
    # Metadata
    # ----------------------------------------------------------

    with meta_path.open(
        encoding="utf-8"
    ) as stream:

        raw: Dict[str, Any] = (
            yaml.safe_load(stream)
            or {}
        )

    try:
        metadata = RouteMetadata(
            format_version=int(
                raw["format_version"]
            ),
            origin_lat=float(
                raw["origin_lat"]
            ),
            origin_lon=float(
                raw["origin_lon"]
            ),
            loop=bool(
                raw.get(
                    "loop",
                    False,
                )
            ),
            created_at=str(
                raw.get(
                    "created_at",
                    "",
                )
            ),
        )

    except (
        KeyError,
        TypeError,
        ValueError,
    ) as exc:

        raise RouteValidationError(
            f"invalid metadata: {exc}"
        ) from exc

    if metadata.format_version != 1:
        raise RouteValidationError(
            "only format_version 1 is supported"
        )

    if not all(
        math.isfinite(v)
        for v in (
            metadata.origin_lat,
            metadata.origin_lon,
        )
    ):
        raise RouteValidationError(
            "route origin must be finite"
        )

    # ----------------------------------------------------------
    # Waypoints
    # ----------------------------------------------------------

    points = []

    with csv_path.open(
        newline="",
        encoding="utf-8",
    ) as stream:

        reader = csv.DictReader(stream)

        if (
            reader.fieldnames is None
            or any(
                column not in reader.fieldnames
                for column
                in REQUIRED_COLUMNS
            )
        ):
            raise RouteValidationError(
                "route CSV has missing columns"
            )

        for expected, row in enumerate(reader):

            try:
                index = int(
                    row["index"]
                )

                numeric = [
                    float(row[key])
                    for key in (
                        "latitude",
                        "longitude",
                        "x_m",
                        "y_m",
                        "drive_level",
                    )
                ]

                (
                    lat,
                    lon,
                    x_m,
                    y_m,
                    drive,
                ) = numeric

                direction = Direction(
                    int(
                        row["direction"]
                    )
                )

                mode = str(
                    int(
                        row["mode"]
                    )
                )

                # 기존 CSV에는 event 컬럼이 없을 수 있으므로
                # 없으면 자동으로 NONE 처리한다.
                event = str(
                    row.get(
                        "event",
                        "NONE",
                    )
                    or "NONE"
                ).strip().upper()

            except (
                ValueError,
                TypeError,
                KeyError,
            ) as exc:

                raise RouteValidationError(
                    f"invalid waypoint row "
                    f"{expected}: {exc}"
                ) from exc

            # --------------------------------------------------
            # Validation
            # --------------------------------------------------

            if index != expected:
                raise RouteValidationError(
                    "waypoint indices must be "
                    "contiguous from zero"
                )

            if not all(
                math.isfinite(v)
                for v in numeric
            ):
                raise RouteValidationError(
                    f"non-finite waypoint "
                    f"at index {index}"
                )

            if drive not in (
                1.0,
                2.0,
                3.0,
            ):
                raise RouteValidationError(
                    f"invalid drive_level "
                    f"at index {index}"
                )

            mode_value = int(
                mode
            )

            if not (
                1
                <= mode_value
                <= 11
            ):
                raise RouteValidationError(
                    f"invalid mode "
                    f"at index {index}: "
                    f"{mode_value}; expected 1~11"
                )

            if event not in VALID_EVENTS:
                raise RouteValidationError(
                    f"invalid event "
                    f"at index {index}: "
                    f"{event}"
                )

            # --------------------------------------------------
            # Waypoint
            # --------------------------------------------------

            point = Waypoint(
                index=index,
                latitude=lat,
                longitude=lon,
                x_m=x_m,
                y_m=y_m,
                direction=direction,
                mode=mode,
                drive_level=drive,
                event=event,
            )

            # --------------------------------------------------
            # Segment continuity
            # --------------------------------------------------

            if points:

                distance = math.hypot(
                    x_m
                    - points[-1].x_m,
                    y_m
                    - points[-1].y_m,
                )

                if distance < 1.0e-6:
                    raise RouteValidationError(
                        f"duplicate point "
                        f"at index {index}"
                    )

                if distance > max_segment_m:
                    raise RouteValidationError(
                        f"route jump "
                        f"at index {index}: "
                        f"{distance:.2f} m"
                    )

            points.append(point)

    # ----------------------------------------------------------
    # Final validation
    # ----------------------------------------------------------

    if len(points) < 2:
        raise RouteValidationError(
            "route requires at least "
            "two waypoints"
        )

    return Route(
        metadata,
        tuple(points),
    )