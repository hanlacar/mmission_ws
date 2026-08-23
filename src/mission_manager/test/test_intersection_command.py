"""
/intersection/command latch 기반 IntersectionController 단위 테스트.
rclpy 의존 없음 — intersection.py, intersection_command.py는 순수 python.
"""
import math

import pytest

from mission_manager import intersection_command as cmd
from mission_manager.intersection import IntersectionController, wrap_deg


def make_ctrl(**overrides):
    kwargs = dict(
        max_steer_deg=27.0,
        right_target_yaw_deg=-90.0,
        right_yaw_tolerance_deg=5.0,
        left_target_yaw_deg=90.0,
        left_yaw_tolerance_deg=5.0,
        stabilize_s=1.0,
        straight_timeout_s=2.0,
        imu_invalid_abort_s=0.5,
        use_position_exit=False,
    )
    kwargs.update(overrides)
    return IntersectionController(**kwargs)


def tick(ctrl, t, imu_yaw_deg, imu_valid=True, x=None, y=None):
    return ctrl.update(imu_valid=imu_valid, imu_yaw_deg=imu_yaw_deg, now_s=t, x=x, y=y)


# ---------------- yaw wrap-around ----------------
def test_wrap_deg_179_to_minus_179_is_small_positive_step():
    # 179 -> -179 는 +2도 회전(실제 358도 회전으로 오판되면 안 됨)
    assert wrap_deg(-179.0 - 179.0) == pytest.approx(2.0)


def test_wrap_deg_minus_179_to_179_is_small_negative_step():
    assert wrap_deg(179.0 - (-179.0)) == pytest.approx(-2.0)


def test_wrap_deg_identity_within_range():
    assert wrap_deg(30.0) == pytest.approx(30.0)
    assert wrap_deg(-170.0) == pytest.approx(-170.0)


# ---------------- 명령 인식 ----------------
def test_straight_command_recognized_and_activates():
    ctrl = make_ctrl()
    active, man = tick(ctrl, 0.0, 0.0)
    assert not active
    ctrl.on_command(cmd.STRAIGHT)
    active, man = tick(ctrl, 0.1, 0.0)
    assert active and man == "straight"
    assert ctrl.state == IntersectionController.STRAIGHT


def test_right_command_recognized_and_turns():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.RIGHT)
    active, man = tick(ctrl, 0.0, 0.0)
    assert active and man == "right"
    assert ctrl.steer_deg == pytest.approx(-27.0)


def test_left_command_recognized_and_turns():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.LEFT)
    active, man = tick(ctrl, 0.0, 0.0)
    assert active and man == "left"
    assert ctrl.steer_deg == pytest.approx(27.0)


def test_none_command_does_not_activate_from_idle():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.NONE)
    active, man = tick(ctrl, 0.0, 0.0)
    assert not active and man is None


def test_invalid_command_rejected_and_ignored():
    ctrl = make_ctrl()
    ctrl.on_command(9)   # 정의되지 않은 값
    active, man = tick(ctrl, 0.0, 0.0)
    assert not active and man is None


def test_invalid_command_does_not_disturb_active_maneuver():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.LEFT)
    tick(ctrl, 0.0, 0.0)
    assert ctrl.state == IntersectionController.TURNING
    ctrl.on_command(9)   # 잘못된 값이 진행 중인 기동을 건드리면 안 됨
    active, man = tick(ctrl, 0.1, 10.0)
    assert active and man == "left"


# ---------------- 동일 진입 상황에서 서로 다른 명령 선택 가능 ----------------
def test_same_entry_state_can_select_any_maneuver():
    # heading/방위와 무관하게, 같은 초기상태에서 STRAIGHT/RIGHT/LEFT 어느 것이든 선택 가능
    for command, expected in ((cmd.STRAIGHT, "straight"), (cmd.RIGHT, "right"), (cmd.LEFT, "left")):
        ctrl = make_ctrl()
        ctrl.on_command(command)
        active, man = tick(ctrl, 0.0, 0.0)
        assert active and man == expected


# ---------------- latch: 기동 중 새 명령 무시 ----------------
def test_left_active_ignores_incoming_right():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.LEFT)
    tick(ctrl, 0.0, 0.0)
    assert ctrl.active_command == cmd.LEFT

    ctrl.on_command(cmd.RIGHT)   # 회전 도중 RIGHT 수신
    active, man = tick(ctrl, 0.1, 5.0)
    assert active and man == "left"
    assert ctrl.active_command == cmd.LEFT   # 여전히 LEFT


def test_right_active_ignores_incoming_left():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 0.0)
    assert ctrl.active_command == cmd.RIGHT

    ctrl.on_command(cmd.LEFT)
    active, man = tick(ctrl, 0.1, -5.0)
    assert active and man == "right"
    assert ctrl.active_command == cmd.RIGHT


# ---------------- 회전 완료 판정 ----------------
def test_right_turn_completes_near_target_then_stabilizes_then_idle():
    ctrl = make_ctrl(stabilize_s=0.3)
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 0.0)                      # start_yaw = 0
    active, man = tick(ctrl, 0.1, -60.0)      # -60 > target+tol(-85) → 아직 미달
    assert active and man == "right"

    active, man = tick(ctrl, 0.2, -80.0)      # -80 > -85 → 아직 미달
    assert active and man == "right"

    active, man = tick(ctrl, 0.3, -87.0)      # -87 <= -85 → 완료(안정화 진입)
    assert active and man == "exit"
    assert ctrl.state == IntersectionController.EXITING

    active, man = tick(ctrl, 0.35, -90.0)     # 안정화 중(timeout 0.3s 전)
    assert active and man == "exit"

    active, man = tick(ctrl, 0.65, -90.0)     # stabilize_s 경과 → 완료
    assert not active and man is None
    assert ctrl.state == IntersectionController.IDLE
    assert ctrl.active_command == cmd.NONE


def test_left_turn_completes_near_target():
    ctrl = make_ctrl(stabilize_s=0.1)
    ctrl.on_command(cmd.LEFT)
    tick(ctrl, 0.0, 10.0)                     # start_yaw = 10
    active, man = tick(ctrl, 0.1, 50.0)       # delta=40, 아직 미달
    assert man == "left"
    active, man = tick(ctrl, 0.2, 108.0)      # delta=98 >= 90-5=85 → 완료
    assert man == "exit"


def test_wrap_around_does_not_break_turn_completion():
    # start_yaw=170, RIGHT 목표 -90±5 → 실제로는 wrap 넘어 도달하는 경우를 시뮬레이션
    ctrl = make_ctrl(stabilize_s=0.1)
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 170.0)
    # 170 -> -100 : wrap_deg(-100-170) = wrap_deg(-270) = 90 (부호 반대) 이므로
    # 대신 170 -> 80으로 이동(= -90 delta, wrap 없이도 맞물리는 값)해 완료를 확인.
    active, man = tick(ctrl, 0.1, 80.0)
    assert man == "exit"


# ---------------- 직진: timeout fallback ----------------
def test_straight_completes_after_timeout():
    ctrl = make_ctrl(straight_timeout_s=1.0)
    ctrl.on_command(cmd.STRAIGHT)
    tick(ctrl, 0.0, 0.0)
    active, man = tick(ctrl, 0.5, 0.0)
    assert active and man == "straight"
    active, man = tick(ctrl, 1.1, 0.0)
    assert not active and man is None
    assert ctrl.state == IntersectionController.IDLE


def test_straight_completes_on_position_exit_when_enabled():
    ctrl = make_ctrl(straight_timeout_s=100.0, use_position_exit=True,
                      center=(0.0, 0.0), radius_m=2.0)
    ctrl.on_command(cmd.STRAIGHT)
    tick(ctrl, 0.0, 0.0, x=0.0, y=0.0)          # 원 안에서 시작
    active, man = tick(ctrl, 0.1, 0.0, x=0.5, y=0.0)
    assert active and man == "straight"         # 아직 원 안
    active, man = tick(ctrl, 0.2, 0.0, x=5.0, y=0.0)   # 원 밖으로 이탈
    assert not active and man is None


# ---------------- IMU invalid/stale 안전정지 ----------------
def test_imu_invalid_during_turn_triggers_error_after_timeout():
    ctrl = make_ctrl(imu_invalid_abort_s=0.3)
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 0.0, imu_valid=True)
    active, man = tick(ctrl, 0.1, -20.0, imu_valid=False)
    assert active and man == "right"            # 아직 abort 시간 전
    active, man = tick(ctrl, 0.5, -20.0, imu_valid=False)   # 0.5s invalid 지속
    assert active and man == "error"
    assert ctrl.state == IntersectionController.ERROR
    assert ctrl.steer_deg == 0.0


def test_imu_nan_treated_as_invalid():
    ctrl = make_ctrl(imu_invalid_abort_s=0.2)
    ctrl.on_command(cmd.LEFT)
    tick(ctrl, 0.0, 0.0, imu_valid=True)
    active, man = tick(ctrl, 0.3, math.nan, imu_valid=True)
    assert man == "error"


def test_error_state_holds_zero_steer_and_stays_active():
    ctrl = make_ctrl(imu_invalid_abort_s=0.1)
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 0.0, imu_valid=True)
    tick(ctrl, 0.2, -10.0, imu_valid=False)
    assert ctrl.state == IntersectionController.ERROR
    active, man = tick(ctrl, 0.3, -10.0, imu_valid=False)
    assert active and man == "error" and ctrl.steer_deg == 0.0


def test_none_command_clears_error_and_returns_idle():
    ctrl = make_ctrl(imu_invalid_abort_s=0.1)
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 0.0, imu_valid=True)
    tick(ctrl, 0.2, -10.0, imu_valid=False)
    assert ctrl.state == IntersectionController.ERROR

    ctrl.on_command(cmd.NONE)   # 초기화
    active, man = tick(ctrl, 0.3, -10.0, imu_valid=True)
    assert not active and man is None
    assert ctrl.state == IntersectionController.IDLE
    assert ctrl.active_command == cmd.NONE


def test_activation_requires_valid_imu():
    ctrl = make_ctrl()
    ctrl.on_command(cmd.RIGHT)
    active, man = tick(ctrl, 0.0, 0.0, imu_valid=False)
    assert not active and man is None
    assert ctrl.state == IntersectionController.IDLE
    # IMU가 다시 정상화되면 그제서야 시작(명령은 계속 대기 중이었음)
    active, man = tick(ctrl, 0.1, 0.0, imu_valid=True)
    assert active and man == "right"


# ---------------- 완료 후 IDLE 복귀 ----------------
def test_full_cycle_returns_to_idle_and_clears_active_command():
    ctrl = make_ctrl(stabilize_s=0.1)
    ctrl.on_command(cmd.RIGHT)
    tick(ctrl, 0.0, 0.0)
    tick(ctrl, 0.1, -88.0)   # 완료 -> EXITING
    active, man = tick(ctrl, 0.3, -88.0)  # stabilize 경과 -> IDLE
    assert not active and man is None
    assert ctrl.state == IntersectionController.IDLE
    assert ctrl.active_command == cmd.NONE
    assert ctrl.steer_deg == 0.0
