import math
from pathlib import Path
import pytest
from mission_manager.imu_heading_estimator import ImuHeadingEstimator
from mission_manager.navigation_controller import ControllerConfig, FollowerState, NavigationController
from mission_manager.pure_pursuit import steering_angle
from mission_manager.process_singleton import DuplicateProcessError, acquire_process_lock
from mission_manager.route_loader import RouteValidationError, load_route
from mission_manager.route_model import Direction, Route, RouteMetadata, Waypoint
from mission_manager.route_tracker import RouteTracker


ROUTES = Path(__file__).parents[1] / "routes" / "test"


def route(name="01_straight_forward"):
    return load_route(str(ROUTES / f"{name}.csv"))


def test_route_load_metadata_direction_drive_origin():
    result = route("04_forward_reverse_cusp")
    assert result.metadata.format_version == 1
    assert result.metadata.origin_lat == 37.5
    assert result.waypoints[0].direction == Direction.FORWARD
    assert result.waypoints[3].direction == Direction.REVERSE
    assert result.waypoints[-1].drive_level == 3.0


def test_route_rejects_missing_metadata(tmp_path):
    csv_path = tmp_path / "bad.csv"
    csv_path.write_text("index,latitude,longitude,x_m,y_m,direction,mode,drive_level\n", encoding="utf-8")
    with pytest.raises(RouteValidationError):
        load_route(str(csv_path))


def test_imu_anchor_turn_invalid_timeout_and_reset_continuity():
    imu = ImuHeadingEstimator(0.2, 35.0, 8.0)
    assert math.degrees(imu.initialize(math.radians(32), 0.0, 0.0)) == pytest.approx(32)
    assert math.degrees(imu.update(10.0, 10.0, True, 1.0).heading) == pytest.approx(42)
    assert math.degrees(imu.update(-5.0, -15.0, True, 2.0).heading) == pytest.approx(27)
    before = imu.update(65.0, 0.0, True, 3.0).heading
    update = imu.update(0.2, 0.0, True, 3.05)
    assert update.reset_detected
    assert update.heading == pytest.approx(before)
    assert not imu.healthy(3.30)
    assert imu.update(0.2, 0.0, False, 3.31).heading is None


@pytest.mark.parametrize("direction,target,sign", [
    (1, (2, 0), 0), (1, (2, 1), 1), (1, (2, -1), -1),
    (-1, (-2, 0), 0), (-1, (-2, 1), 1), (-1, (-2, -1), -1),
])
def test_pure_pursuit_forward_reverse(direction, target, sign):
    angle = steering_angle(0, 0, 0, *target, direction, 0.3, 27)
    assert (angle > 0) - (angle < 0) == sign


def test_wheel_clamp():
    assert steering_angle(0, 0, 0, 0.01, 0.01, 1, 0.3, 27) == 27


def test_self_intersection_progress_never_goes_back():
    tracker = RouteTracker(route("06_self_intersection"))
    tracker.select_start(-1.8, -1.8, math.pi / 4, "nearest")
    first = tracker.update(0, 0, math.pi / 4).segment
    second = tracker.update(0.1, 0.1, math.pi / 4).segment
    assert second >= first
    assert second < 3


def _ready_controller(route_name="01_straight_forward", **kwargs):
    cfg = ControllerConfig(**kwargs)
    controller = NavigationController(route(route_name), cfg)
    controller.set_position(0, 0, 0.0)
    controller.set_imu(0, 0, True, 0.0)
    return controller


def test_output_quantization_types_and_safety():
    controller = _ready_controller()
    output = controller.step(0.0)
    assert output.drive == 2.0 and isinstance(output.wheel, int)
    controller.set_imu(0, 0, False, 0.1)
    assert controller.step(0.1).drive == 0.0
    controller.set_imu(0, 0, True, 0.2)
    assert controller.step(0.5).state == FollowerState.WAITING_FOR_IMU
    assert controller.step(1.2).state == FollowerState.WAITING_FOR_POSITION


def test_goal_latches():
    controller = _ready_controller()
    controller.step(0.0)
    controller.set_position(2, 0, 0.1)
    controller.set_imu(0, 0, True, 0.1)
    assert controller.step(0.1).state == FollowerState.GOAL_REACHED
    controller.set_position(1.5, 0, 0.2)
    controller.set_imu(0, 0, True, 0.2)
    assert controller.step(0.2).drive == 0.0


def test_cusp_stops_then_changes_direction():
    controller = _ready_controller("04_forward_reverse_cusp", direction_change_stop_s=0.5)
    controller.step(0.0)
    controller.set_position(2, 0, 0.1); controller.set_imu(0, 0, True, 0.1)
    stopped = controller.step(0.1)
    assert stopped.state == FollowerState.STOPPED_AT_CUSP and stopped.drive == 0
    controller.set_position(1.95, 0.05, 0.7); controller.set_imu(0, 0, True, 0.7)
    assert controller.step(0.7).drive < 0


def test_mode_reset_keeps_heading_and_steering_continuous():
    controller = _ready_controller("05_mode_transition")
    before = controller.step(0.0)
    controller.set_position(1.8, 0, 0.1); controller.set_imu(65, 0, True, 0.1)
    controller.step(0.1)
    controller.set_imu(0, 0, True, 0.15)
    after = controller.step(0.15)
    assert abs(after.wheel - before.wheel) < 27


def test_off_route_requires_consecutive_samples_and_recovers():
    controller = _ready_controller(off_route_stop_count=3, off_route_recover_count=2)
    controller.step(0.0)
    for i in range(2):
        now = 0.1 + i * 0.1
        controller.set_position(0.5, 3, now); controller.set_imu(0, 0, True, now)
        assert controller.step(now).state != FollowerState.FAULT
    controller.set_position(0.5, 3, 0.3); controller.set_imu(0, 0, True, 0.3)
    assert controller.step(0.3).state == FollowerState.FAULT
    for now in (0.4, 0.5):
        controller.set_position(0.5, 0, now); controller.set_imu(0, 0, True, now)
        recovered = controller.step(now)
    assert recovered.state == FollowerState.TRACKING


def test_drive_is_always_cent_quantized():
    controller = _ready_controller()
    output = controller.step(0.0)
    assert output.drive * 100 == round(output.drive * 100)
    assert -27 <= output.wheel <= 27


def test_circle_loop_wraps_without_goal_and_keeps_forward_level_two():
    circular = route("07_circle_loop")
    assert circular.metadata.loop
    assert len(circular.waypoints) == 72
    assert all(p.direction == Direction.FORWARD and p.mode == "NORMAL" and
               p.drive_level == 2.0 for p in circular.waypoints)
    controller = NavigationController(circular, ControllerConfig())
    indices = []
    for lap_fraction in range(73):
        i = lap_fraction % 72
        point = circular.waypoints[i]
        now = lap_fraction * 0.01
        controller.set_position(point.x_m, point.y_m, now)
        controller.set_imu(i * 5.0, 500.0, True, now)
        output = controller.step(now)
        indices.append(output.route_index)
        assert output.state == FollowerState.TRACKING
        assert output.drive == 2.0
        assert -27 <= output.wheel <= 27
    assert max(indices) >= 70
    assert indices[-1] <= 1


@pytest.mark.parametrize("name,direction,drive,wheel", [
    ("08_circle_forward_max27", Direction.FORWARD, 2.0, 27),
    ("09_circle_reverse_max27", Direction.REVERSE, -1.0, -27),
])
def test_max27_visual_routes_use_natural_pure_pursuit(name, direction, drive, wheel):
    circular = route(name)
    expected_radius = 0.30 / math.tan(math.radians(27.0))
    assert circular.metadata.loop and len(circular.waypoints) == 144
    assert all(p.direction == direction and p.mode == "NORMAL" for p in circular.waypoints)
    assert math.hypot(circular.waypoints[0].x_m,
                      circular.waypoints[0].y_m) == pytest.approx(expected_radius, abs=1e-6)
    controller = NavigationController(circular, ControllerConfig(steering_slowdown_deg=28.0))
    first = circular.waypoints[0]
    controller.set_position(first.x_m, first.y_m, 0.0)
    controller.set_imu(0.0, 0.0, True, 0.0)
    output = controller.step(0.0)
    assert output.state == FollowerState.TRACKING
    assert output.drive == drive
    assert abs(output.wheel - wheel) <= 1


def test_authoritative_ros_nodes_are_process_singletons():
    role = f'pytest_singleton_{id(object())}'
    first = acquire_process_lock(role)
    try:
        with pytest.raises(DuplicateProcessError):
            acquire_process_lock(role)
    finally:
        first.close()
    replacement = acquire_process_lock(role)
    replacement.close()
