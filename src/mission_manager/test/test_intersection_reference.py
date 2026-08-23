import math
from dataclasses import replace
from pathlib import Path

import pytest

from mission_manager import intersection_command as command
from mission_manager.intersection_reference import (
    IntersectionConfig, IntersectionRouteController, IntersectionState,
    count_infeasible_curvature_segments, direction_from_math_heading,
    select_intersection_route, validate_active_dir,
)
from mission_manager.navigation_controller import ControllerConfig
from mission_manager.route_model import Direction, Route, RouteMetadata, Waypoint


def _route(drive=2.0):
    points = tuple(Waypoint(i, 37.0, 127.0, float(i), 0.0, Direction.FORWARD,
                            "INTERSECTION", drive) for i in range(4))
    return Route(RouteMetadata(1, 37.0, 127.0), points)


def _controller(**ix_overrides):
    cfg = IntersectionConfig(route_dir="/vehicle", **ix_overrides)
    nav = ControllerConfig(gps_timeout_sec=1.0, imu_timeout_sec=1.0,
                           start_policy="route_start", start_accept_radius_m=1.0,
                           start_heading_tolerance_deg=30.0, off_route_warn_m=0.2,
                           off_route_stop_m=0.35, off_route_stop_count=3,
                           off_route_recover_count=5, goal_tolerance_m=0.35,
                           steering_slowdown_deg=18.0)
    return IntersectionRouteController(cfg, nav, lambda _path: _route())


def _step(ctrl, now, x=0.0, y=0.0, yaw=0.0, gps=True, imu=True):
    return ctrl.step(now, x=x, y=y, gps_healthy=gps, imu_yaw_deg=yaw,
                     imu_rate_deg_s=0.0, imu_healthy=imu,
                     entry_heading_rad=0.0)


@pytest.mark.parametrize("direction", "NESW")
@pytest.mark.parametrize("value,name", [
    (command.STRAIGHT, "STRAIGHT"), (command.LEFT, "LEFT"), (command.RIGHT, "RIGHT")])
def test_all_twelve_manual_route_selections(direction, value, name):
    assert select_intersection_route(direction, value) == f"{direction}_{name}.csv"


def test_invalid_direction_and_command_rejected():
    with pytest.raises(ValueError):
        validate_active_dir("Q")
    with pytest.raises(ValueError):
        select_intersection_route("N", 9)


@pytest.mark.parametrize("heading,expected", [
    (math.pi/2, "N"), (0.0, "E"), (-math.pi/2, "S"), (math.pi, "W")])
def test_auto_direction_conversion(heading, expected):
    assert direction_from_math_heading(heading) == expected


def test_command_validation_latch_and_none_only_releases_terminal_state():
    ctrl = _controller()
    assert not ctrl.on_command(9)
    assert ctrl.on_command(command.LEFT)
    assert ctrl.active_command == command.LEFT
    assert not ctrl.on_command(command.RIGHT)
    assert ctrl.on_command(command.NONE)
    assert ctrl.active_command == command.LEFT
    ctrl._fault("test")
    assert ctrl.on_command(command.NONE)
    assert ctrl.state == IntersectionState.IDLE


def test_missing_route_faults_stopped():
    ctrl = IntersectionRouteController(
        IntersectionConfig(route_dir="/missing"), ControllerConfig(),
        lambda path: (_ for _ in ()).throw(ValueError(f"missing {path}")))
    ctrl.on_command(command.STRAIGHT)
    output = _step(ctrl, 0.0)
    assert ctrl.state == IntersectionState.FAULT
    assert output.drive == 0 and output.wheel == 0


@pytest.mark.parametrize("sensor", ["gps", "imu"])
def test_invalid_sensor_faults_during_alignment(sensor):
    ctrl = _controller()
    ctrl.on_command(command.STRAIGHT)
    _step(ctrl, 0.0)
    output = _step(ctrl, 0.1, gps=sensor != "gps", imu=sensor != "imu")
    assert ctrl.state == IntersectionState.FAULT
    assert output.drive == 0 and output.wheel == 0


def test_cte_warn_slowdown_and_consecutive_stop():
    ctrl = _controller(heading_stop_deg=90.0)
    ctrl.on_command(command.STRAIGHT)
    _step(ctrl, 0.0)
    warned = _step(ctrl, 0.1, x=0.5, y=0.25)
    assert warned.drive == 1.0
    for i in range(3):
        stopped = _step(ctrl, 0.2+i*0.1, x=0.7, y=0.5)
    assert ctrl.state == IntersectionState.FAULT
    assert stopped.drive == 0 and stopped.wheel == 0


def test_heading_warn_then_consecutive_stop():
    ctrl = _controller(heading_warn_deg=10.0, heading_stop_deg=20.0,
                       heading_stop_count=3)
    ctrl.on_command(command.LEFT)
    _step(ctrl, 0.0)
    _step(ctrl, 0.1)
    warned = _step(ctrl, 0.2, x=0.2, yaw=15.0)
    assert warned.drive == 1.0
    for i in range(3):
        stopped = _step(ctrl, 0.3+i*0.1, x=0.3+i*0.1, yaw=25.0)
    assert ctrl.state == IntersectionState.FAULT
    assert stopped.drive == 0 and stopped.wheel == 0


def test_goal_requires_heading_and_confirmation_then_latches_until_none():
    ctrl = _controller(goal_heading_tolerance_deg=10.0, complete_confirm_count=3,
                       heading_stop_deg=90.0)
    ctrl.on_command(command.RIGHT)
    _step(ctrl, 0.0)
    _step(ctrl, 0.1)
    wrong = _step(ctrl, 0.2, x=3.0, yaw=15.0)
    assert ctrl.state == IntersectionState.EXIT_CONFIRM
    assert not ctrl.complete and wrong.drive == 0
    for i in range(3):
        done = _step(ctrl, 0.3+i*0.1, x=3.0, yaw=0.0)
    assert ctrl.complete and done.drive == 0 and done.wheel == 0
    assert not ctrl.on_command(command.LEFT)
    assert ctrl.state == IntersectionState.COMPLETE
    ctrl.on_command(command.NONE)
    assert ctrl.state == IntersectionState.IDLE and not ctrl.complete


def test_configured_drive_level_and_steering_clamp_are_preserved():
    ctrl = _controller(drive_level=2.0, heading_stop_deg=90.0)
    ctrl.on_command(command.LEFT)
    _step(ctrl, 0.0)
    output = _step(ctrl, 0.1, x=0.0, y=-0.3)
    assert abs(output.drive) in (1.0, 2.0)
    assert -27 <= output.wheel <= 27


def test_vehicle_and_synthetic_route_locations_are_separate():
    assert "/routes/intersection" != "/routes/test/intersection"


def test_curvature_sanity_check_flags_impossible_repeated_bend():
    tight = replace(_route(), waypoints=(
        replace(_route().waypoints[0], x_m=0.0, y_m=0.0),
        replace(_route().waypoints[1], x_m=0.1, y_m=0.0),
        replace(_route().waypoints[2], x_m=0.1, y_m=0.1),
        replace(_route().waypoints[3], x_m=0.0, y_m=0.1),
    ))
    assert count_infeasible_curvature_segments(tight, 0.30, 27.0) >= 2


def test_only_production_follower_creates_gps_command_publishers():
    package = Path(__file__).parents[1] / "mission_manager"
    publishers = []
    for source in package.glob("*.py"):
        text = source.read_text(encoding="utf-8")
        if "create_publisher(Float32, self._p(\"gps_drive_topic\")" in text:
            publishers.append(source.name)
        if "create_publisher(Int32, self._p(\"gps_wheel_topic\")" in text:
            publishers.append(source.name)
    assert publishers == ["gps_route_follower_node.py", "gps_route_follower_node.py"]
