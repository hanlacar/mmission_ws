import math

import pytest

from mission_manager.route_accuracy import (
    AccuracyAccumulator,
    encoder_calibrated,
    valid_gps_fix,
)
from mission_manager.route_geometry import RouteGeometry, project_to_segment
from mission_manager.route_model import Direction, Route, RouteMetadata, Waypoint


def _straight_route():
    metadata = RouteMetadata(1, 37.5, 127.0)
    points = (
        Waypoint(0, 37.5, 127.0, 0.0, 0.0, Direction.FORWARD, "NORMAL", 2.0),
        Waypoint(1, 37.5, 127.0, 10.0, 0.0, Direction.FORWARD, "NORMAL", 2.0),
    )
    return Route(metadata, points)


def test_shared_projection_exact_route_is_100_percent():
    geometry = RouteGeometry(_straight_route())
    projection = geometry.nearest(4.0, 0.0)
    statistics = AccuracyAccumulator(0.30).add(projection.distance)
    assert projection.distance == pytest.approx(0.0)
    assert geometry.progress_pct(projection) == pytest.approx(40.0)
    assert statistics.accuracy_pct == pytest.approx(100.0)
    assert statistics.mean_error_m == pytest.approx(0.0)
    assert statistics.rmse_m == pytest.approx(0.0)


def test_tolerance_mix_eight_of_ten_is_80_percent():
    accumulator = AccuracyAccumulator(0.30)
    for error in [0.10] * 8 + [0.31, 0.50]:
        result = accumulator.add(error)
    assert result.valid_samples == 10
    assert result.success_samples == 8
    assert result.accuracy_pct == pytest.approx(80.0)


def test_rmse_matches_python_math():
    errors = [0.0, 0.1, 0.2, 0.4]
    accumulator = AccuracyAccumulator(0.30)
    for error in errors:
        result = accumulator.add(error)
    expected = math.sqrt(sum(error * error for error in errors) / len(errors))
    assert result.rmse_m == pytest.approx(expected)
    assert result.mean_error_m == pytest.approx(sum(errors) / len(errors))
    assert result.max_error_m == pytest.approx(max(errors))


def test_invalid_gps_is_not_added_to_statistics():
    accumulator = AccuracyAccumulator(0.30)
    invalid = [(-1, 37.5, 127.0), (0, math.nan, 127.0),
               (0, 37.5, math.inf)]
    for status, latitude, longitude in invalid:
        if valid_gps_fix(status, latitude, longitude):
            accumulator.add(0.0)
    assert accumulator.snapshot().valid_samples == 0


def test_off_route_sample_is_excluded():
    geometry = RouteGeometry(_straight_route())
    accumulator = AccuracyAccumulator(0.30)
    projection = geometry.nearest(5.0, 3.01)
    max_route_match_distance_m = 3.0
    if projection.distance <= max_route_match_distance_m:
        accumulator.add(projection.distance)
    assert projection.distance > max_route_match_distance_m
    assert accumulator.snapshot().valid_samples == 0


def test_uncalibrated_encoder_does_not_affect_gps_accuracy():
    assert not encoder_calibrated(0.0)
    accumulator = AccuracyAccumulator(0.30)
    assert accumulator.add(0.1).accuracy_pct == pytest.approx(100.0)


def test_accuracy_statistics_reset_between_runs():
    accumulator = AccuracyAccumulator(0.30)
    accumulator.add(0.50)
    accumulator.reset()
    result = accumulator.add(0.10)
    assert result.valid_samples == 1
    assert result.success_samples == 1
    assert result.accuracy_pct == pytest.approx(100.0)


def test_route_tracker_and_monitor_use_same_segment_projection():
    route = _straight_route()
    common = project_to_segment(route.waypoints, 2.0, 0.25, 0)
    monitor = RouteGeometry(route).nearest(2.0, 0.25)
    assert monitor == common
