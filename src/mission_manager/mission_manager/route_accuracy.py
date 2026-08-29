"""ROS-independent route-accuracy statistics."""

import math
from dataclasses import dataclass


def valid_gps_fix(
    status: int,
    latitude: float,
    longitude: float,
) -> bool:
    return (
        status != -1
        and math.isfinite(latitude)
        and math.isfinite(longitude)
    )


def encoder_calibrated(
    counts_per_meter: float,
) -> bool:
    return (
        math.isfinite(counts_per_meter)
        and counts_per_meter > 0.0
    )


@dataclass(frozen=True)
class AccuracySnapshot:
    current_error_m: float
    mean_error_m: float
    rmse_m: float
    p95_error_m: float
    max_error_m: float
    accuracy_pct: float
    valid_samples: int
    success_samples: int


class AccuracyAccumulator:

    def __init__(
        self,
        tolerance_m: float,
    ) -> None:

        if (
            not math.isfinite(tolerance_m)
            or tolerance_m < 0.0
        ):
            raise ValueError(
                "accuracy_tolerance_m must be finite and >= 0"
            )

        self.tolerance_m = tolerance_m
        self.reset()

    def reset(self) -> None:

        self.valid_samples = 0
        self.success_samples = 0

        self.sum_error = 0.0
        self.sum_squared_error = 0.0

        self.max_error = 0.0
        self.current_error = 0.0

        # P95 계산용
        self.errors = []

    def add(
        self,
        error_m: float,
    ) -> AccuracySnapshot:

        if (
            not math.isfinite(error_m)
            or error_m < 0.0
        ):
            raise ValueError(
                "cross-track error must be finite and >= 0"
            )

        self.current_error = error_m

        self.valid_samples += 1

        self.success_samples += int(
            error_m <= self.tolerance_m
        )

        self.sum_error += error_m
        self.sum_squared_error += (
            error_m * error_m
        )

        self.max_error = max(
            self.max_error,
            error_m,
        )

        self.errors.append(
            float(error_m)
        )

        return self.snapshot()

    def _percentile(
        self,
        values,
        percentile,
    ) -> float:

        if not values:
            return 0.0

        ordered = sorted(values)

        if len(ordered) == 1:
            return ordered[0]

        position = (
            percentile
            / 100.0
            * (len(ordered) - 1)
        )

        lower = int(
            math.floor(position)
        )

        upper = int(
            math.ceil(position)
        )

        if lower == upper:
            return ordered[lower]

        fraction = (
            position - lower
        )

        return (
            ordered[lower]
            + fraction
            * (
                ordered[upper]
                - ordered[lower]
            )
        )

    def snapshot(
        self,
    ) -> AccuracySnapshot:

        count = self.valid_samples

        return AccuracySnapshot(
            current_error_m=(
                self.current_error
            ),
            mean_error_m=(
                self.sum_error / count
                if count
                else 0.0
            ),
            rmse_m=(
                math.sqrt(
                    self.sum_squared_error
                    / count
                )
                if count
                else 0.0
            ),
            p95_error_m=(
                self._percentile(
                    self.errors,
                    95.0,
                )
            ),
            max_error_m=(
                self.max_error
            ),
            accuracy_pct=(
                100.0
                * self.success_samples
                / count
                if count
                else 0.0
            ),
            valid_samples=count,
            success_samples=(
                self.success_samples
            ),
        )
