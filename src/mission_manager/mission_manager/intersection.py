"""
교차로 처리 — /intersection/command(직진/좌/우) latch 기반 상태머신
             + IMU 상대yaw(도)로 회전량·회전완료 판정.

★ 설계 변경 (기존 방위 자동판정 폐기) ★
예전에는 진입 heading을 나침반 방위(N/E/S/W)로 양자화해 그 방위의 plan을 자동으로
골랐다. 하지만 같은 방위로 들어와도 상황에 따라 직진/좌/우 어느 쪽이든 필요할 수
있으므로, 기동 선택은 더 이상 heading에서 자동으로 하지 않는다.
기동 선택은 intersection_command.py 가 정의하는 /intersection/command 로 명시적으로
받는다(현재는 사람이 CLI로, 추후 Mission Sequencer가 자동으로 발행).

  IMU = 방향(직진/좌/우) 선택 수단 X
  IMU = 회전량 및 회전 완료 판단 O

IMU 계산 로직 자체(필터링/적분 등)는 이 모듈에 복제하지 않는다. 이 모듈은
imu_manager가 이미 발행하는 /imu/relative_yaw_deg(도), /imu/valid(bool) 값을
node가 넘겨주는 순수 숫자로만 받는다(구독은 node가 함 — 이 모듈은 ROS 비의존).

명령 latch(섹션 7 요구사항):
  기동 시작 순간의 명령을 active_command로 저장(latch)한다. 기동 도중 다른 명령이
  들어와도(같은 값이든 다른 값이든) 현재 진행 중인 기동에는 영향을 주지 않는다.
  기동이 끝나면(IDLE 복귀) active_command를 초기화한다. NONE(0)은 "초기화" 명령으로,
  IDLE에서는 아무 효과가 없고 ERROR 상태에서는 그 상태를 해제해 IDLE로 되돌린다.

좌/우회전 완료 판정(섹션 9~10):
  회전 시작 순간의 yaw를 저장하고, 이후 yaw와의 차이를 ±180° wrap-around를 안전하게
  처리해서 계산한다(179°→-179°가 -358°로 잘못 계산되지 않도록 wrap_deg 사용).
  목표각·허용오차는 YAML로 분리되며, 좌/우 각각의 부호는 실제 IMU 좌표계를
  확인하기 전까지는 알 수 없으므로 코드에 고정하지 않고 설정값(right_target_yaw_deg 등,
  부호 포함)으로 받는다.

직진(STRAIGHT, 섹션 11):
  yaw 회전 판정을 쓰지 않는다. 위치 기반 교차로 이탈 판정(원 안↔밖)이 쓸 수 있으면
  그것을 우선 쓰고, 없으면(예: 학교 시험처럼 GPS가 없을 때) timeout을 fallback으로
  쓴다. 이 클래스는 항상 두 조건을 동시에 감시해서, 위치 판정이 붙어 있어도 어떤
  이유로든 실패하면 timeout이 안전망 역할을 한다.

IMU 무효/오래됨 안전처리(섹션 12):
  node가 매 제어주기 "지금 이 순간 IMU 값을 믿을 수 있는가"(imu_valid AND 유한값
  AND 메시지가 최근에 왔음)를 계산해 넘겨준다. 기동 중 이 상태가
  imu_invalid_abort_s 이상 지속되면 ERROR로 전이해 즉시 drive/steer를 0으로 고정한다
  (무한회전 방지). ERROR는 /intersection/command에 NONE(0)을 보내야 해제된다.
"""
import math

from . import intersection_command as cmd
from .geo_utils import dist_m


# ---------------- 나침반 방위 변환 유틸 ----------------
# (다른 오프라인/모니터링 도구 — analyze_entry_dirs.py, intersection_monitor_node.py,
#  intersection_calib_node.py — 가 계속 재사용하므로 그대로 유지한다. 더 이상
#  기동 자동선택에는 쓰이지 않지만, 로그·진단용 방위 표시에는 여전히 유용하다.)
COMPASS_DIRS = ["N", "E", "S", "W"]


def math_heading_to_compass_deg(heading_rad):
    """수학관례 heading(rad, 동=0 반시계+)을 나침반 방위각(deg, 북=0 시계+)으로."""
    math_deg = math.degrees(heading_rad)
    return (90.0 - math_deg) % 360.0


def compass_deg_to_dir(compass_deg):
    """나침반 방위각(deg)을 N/E/S/W 4분면으로 양자화."""
    idx = int(((compass_deg + 45.0) % 360.0) // 90.0)
    return COMPASS_DIRS[idx]


def wrap_deg(deg):
    """각도(도)를 -180~180 범위로 정규화.
    179 -> -179 로 2° 회전한 상황이 -358°로 잘못 계산되지 않도록 wrap-around 처리."""
    return ((deg + 180.0) % 360.0) - 180.0


class IntersectionController:
    """
    교차로 진입/통과 상태머신 (명령 latch + IMU yaw 기반 회전완료 판정).

    사용 흐름(node.control_loop 안, 매 제어주기):
      controller.on_command(raw_uint8)         # /intersection/command 콜백에서 호출
      active, maneuver = controller.update(
          imu_valid=..., imu_yaw_deg=..., now_s=..., x=.., y=..)
      if active:
          steer = controller.steer_deg
          # maneuver == "error" 이면 node가 speed=0으로 안전정지시켜야 함

    상태:
      IDLE     : 대기. 다음 tick에 latch할 명령(_pending_command)이 있으면 시작.
      TURNING  : 좌/우회전 중. IMU yaw 변화량으로 완료 감지.
      STRAIGHT : 직진 통과 중. 위치기반 이탈판정 또는 timeout으로 완료 감지.
      EXITING  : 회전은 끝났지만 아직 안정화 중(steer=0 직진). STRAIGHT와 동일하게
                 위치기반 이탈판정 또는 timeout으로 완료 감지.
      ERROR    : IMU 무효가 오래 지속되어 안전정지. NONE(0) 명령으로만 해제.
    """
    IDLE = "IDLE"
    TURNING = "TURNING"
    STRAIGHT = "STRAIGHT"
    EXITING = "EXITING"
    ERROR = "ERROR"

    def __init__(self, *,
                 max_steer_deg=25.0,
                 right_target_yaw_deg=-90.0,
                 right_yaw_tolerance_deg=5.0,
                 left_target_yaw_deg=90.0,
                 left_yaw_tolerance_deg=5.0,
                 stabilize_s=1.0,
                 straight_timeout_s=5.0,
                 imu_invalid_abort_s=0.5,
                 use_position_exit=False,
                 center=(0.0, 0.0),
                 radius_m=2.5,
                 logger=None):
        self.max_steer = float(max_steer_deg)
        # 부호 포함 목표각. 실제 IMU 좌표계 확인 후 YAML에서 부호를 정할 것
        # (예: 우회전이 음수 방향이면 right_target_yaw_deg=-90.0).
        self.right_target = float(right_target_yaw_deg)
        self.right_tol = abs(float(right_yaw_tolerance_deg))
        self.left_target = float(left_target_yaw_deg)
        self.left_tol = abs(float(left_yaw_tolerance_deg))
        self.stabilize_s = float(stabilize_s)
        self.straight_timeout_s = float(straight_timeout_s)
        self.imu_invalid_abort_s = float(imu_invalid_abort_s)
        self.use_position_exit = bool(use_position_exit)
        self.cx, self.cy = float(center[0]), float(center[1])
        self.radius = float(radius_m)
        self.log = logger

        self.state = self.IDLE
        self.active_command = cmd.NONE
        self.steer_deg = 0.0
        self._pending_command = cmd.NONE   # IDLE 상태에서 다음 update()에 latch할 명령
        self._start_yaw_deg = 0.0
        self._phase_start_t = None         # 현재 단계(STRAIGHT/EXITING) 시작 시각(now_s)
        self._was_inside = False           # 위치기반 이탈판정: 원 안에 있었는지 추적
        self._last_good_imu_t = None       # 마지막으로 imu_valid=True였던 now_s
        # STRAIGHT/EXITING 종료 시 True 한 틱만. node가 경로 재포착(resync) 트리거로 사용.
        self.needs_path_resync = False

    @property
    def active(self):
        return self.state != self.IDLE

    def _logi(self, msg):
        if self.log:
            self.log.info(msg)

    def _logw(self, msg):
        if self.log:
            self.log.warning(msg)

    # ===================== 명령 입력 =====================
    def on_command(self, raw_value):
        """/intersection/command(UInt8) 콜백에서 매 수신마다 호출.
        잘못된 값(0~3 밖)은 거부(경고 로그만, 기존 active_command는 안전하게 유지)."""
        try:
            raw_value = int(raw_value)
        except (TypeError, ValueError):
            self._logw(f"intersection command 파싱 실패: {raw_value!r} 무시")
            return
        if raw_value not in cmd.VALID:
            self._logw(
                f"잘못된 intersection command={raw_value} 무시 "
                f"(유효값: 0=NONE,1=STRAIGHT,2=RIGHT,3=LEFT)")
            return

        if raw_value == cmd.NONE:
            if self.state == self.ERROR:
                self._logi("NONE 수신 → ERROR 해제, IDLE 복귀")
                self._finish()
            # ACTIVE(회전/직진/안정화) 도중의 NONE은 진행 중인 기동에 영향 없음(latch).
            return

        if self.state == self.IDLE:
            self._pending_command = raw_value
        else:
            # 기동 중 새 명령 수신: 현재 latch를 바꾸지 않는다(섹션 7 요구사항).
            self._logi(
                f"기동 중(active={cmd.name(self.active_command)}) 상태에서 "
                f"{cmd.name(raw_value)} 수신 → 무시(현재 기동 유지)")

    # ===================== 매 제어주기 =====================
    def update(self, *, imu_valid, imu_yaw_deg, now_s, x=None, y=None):
        """
        매 제어주기 호출.
          imu_valid  : 이번 틱에 IMU 값을 신뢰할 수 있는가(node가 /imu/valid +
                       staleness + 유한값 여부를 합쳐 계산해 넘김).
          imu_yaw_deg: /imu/relative_yaw_deg 최신값(도).
          now_s      : 단조 증가 시각(초).
          x, y       : (옵션) 현재 위치(m). use_position_exit=True이고 주어지면
                       원 이탈(교차로 통과 완료) 판정에 사용. 없으면 timeout만 사용.
        반환: (active: bool, maneuver: "straight"|"left"|"right"|"exit"|"error"|None)
        """
        self.needs_path_resync = False

        imu_ok = bool(imu_valid) and math.isfinite(imu_yaw_deg)
        if imu_ok:
            self._last_good_imu_t = now_s

        if self.state == self.IDLE:
            if self._pending_command == cmd.NONE:
                return False, None
            if not imu_ok:
                # IMU 없이 회전량 판정도, 안전한 시작도 할 수 없다 — 시작 보류.
                self._logw("IMU invalid — 교차로 기동 시작 보류(명령은 유지됨)")
                return False, None
            self._activate(self._pending_command, imu_yaw_deg, now_s, x, y)
            self._pending_command = cmd.NONE
            return self._status()

        # ---- ACTIVE 공통: IMU 무효/오래됨 워치독(무한회전 방지) ----
        if self.state != self.ERROR:
            age = (now_s - self._last_good_imu_t) if self._last_good_imu_t is not None else math.inf
            if age > self.imu_invalid_abort_s:
                self._enter_error(f"IMU invalid/stale {age:.2f}s 지속")
                return self._status()

        if self.state == self.TURNING:
            delta = wrap_deg(imu_yaw_deg - self._start_yaw_deg)
            target, tol = self._turn_target()
            done = (delta >= target - tol) if target >= 0 else (delta <= target + tol)
            if done:
                self._logi(
                    f"[{cmd.name(self.active_command)}] 회전완료 "
                    f"(Δyaw={delta:.1f}°, 목표{target:.0f}±{tol:.0f}°) → 안정화 직진")
                self._enter_exiting(now_s, x, y)
                return self._status()
            self.steer_deg = self.max_steer if self.active_command == cmd.LEFT else -self.max_steer
            return self._status()

        if self.state == self.STRAIGHT:
            self.steer_deg = 0.0
            if self._exit_condition_met(now_s, x, y, self.straight_timeout_s):
                self._logi("[STRAIGHT] 교차로 통과 완료")
                self.needs_path_resync = True
                self._finish()
                return False, None
            return self._status()

        if self.state == self.EXITING:
            self.steer_deg = 0.0
            if self._exit_condition_met(now_s, x, y, self.stabilize_s):
                self._logi(
                    f"[{cmd.name(self.active_command)}] 교차로 이탈 완료 → 경로 재포착")
                self.needs_path_resync = True
                self._finish()
                return False, None
            return self._status()

        if self.state == self.ERROR:
            self.steer_deg = 0.0
            return self._status()

        return False, None

    # ===================== 내부 헬퍼 =====================
    def _turn_target(self):
        if self.active_command == cmd.RIGHT:
            return self.right_target, self.right_tol
        return self.left_target, self.left_tol   # LEFT

    def _inside(self, x, y):
        return dist_m(x, y, self.cx, self.cy) <= self.radius

    def _exit_condition_met(self, now_s, x, y, timeout):
        """위치기반 이탈판정(옵션) + timeout fallback을 함께 감시.
        위치판정이 켜져 있고 좌표가 주어지면 원 안→밖 전이를 우선 감지하되,
        어떤 이유로든(설정 안 됨/좌표 없음/판정 실패) timeout이 항상 안전망으로 동작한다."""
        if self.use_position_exit and x is not None and y is not None:
            inside = self._inside(x, y)
            if self._was_inside and not inside:
                return True
            self._was_inside = self._was_inside or inside
        elapsed = now_s - self._phase_start_t
        return elapsed >= timeout

    def _activate(self, command, imu_yaw_deg, now_s, x, y):
        self.active_command = command
        self._start_yaw_deg = imu_yaw_deg
        self._phase_start_t = now_s
        self._was_inside = False
        if self.use_position_exit and x is not None and y is not None:
            self._was_inside = self._inside(x, y)
        if command == cmd.STRAIGHT:
            self.state = self.STRAIGHT
            self.steer_deg = 0.0
        else:
            self.state = self.TURNING
            self.steer_deg = self.max_steer if command == cmd.LEFT else -self.max_steer
        self._logi(f"교차로 기동 시작: command={cmd.name(command)} (yaw0={imu_yaw_deg:.1f}°)")

    def _enter_exiting(self, now_s, x, y):
        self.state = self.EXITING
        self._phase_start_t = now_s
        self._was_inside = False
        if self.use_position_exit and x is not None and y is not None:
            self._was_inside = self._inside(x, y)
        self.steer_deg = 0.0

    def _enter_error(self, reason):
        self._logw(
            f"교차로 안전정지(ERROR): {reason} → drive/steer=0 고정. "
            f"'ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "
            f"\"{{data: 0}}\"' 로 해제하세요.")
        self.state = self.ERROR
        self.steer_deg = 0.0

    def _maneuver_name(self):
        if self.state == self.TURNING:
            return "left" if self.active_command == cmd.LEFT else "right"
        if self.state == self.STRAIGHT:
            return "straight"
        if self.state == self.EXITING:
            return "exit"
        if self.state == self.ERROR:
            return "error"
        return None

    def _status(self):
        return (self.state != self.IDLE), self._maneuver_name()

    def _finish(self):
        self.state = self.IDLE
        self.active_command = cmd.NONE
        self.steer_deg = 0.0
        self._phase_start_t = None
        self._was_inside = False

    def reset(self):
        """외부(node)에서 강제로 IDLE로 되돌린다.
        예: 다른 미션이 우선순위를 가져가 교차로 로직을 더 이상 돌릴 수 없을 때,
        맴돌던 기동 상태를 해제한다. 대기 중이던 명령(_pending_command)도 버린다."""
        self.needs_path_resync = False
        self._pending_command = cmd.NONE
        self._finish()


# ---------------- yaml 파싱 헬퍼 ----------------
def build_from_params(node, get_logger=None):
    """
    mission_manager 파라미터에서 IntersectionController 생성.
    node는 declare_parameter/get_parameter를 가진 rclpy Node.

    기존 설정 호환:
      intersection_names 의 첫 항목의 "<name>.center"/"<name>.radius_m"/"<name>.turn_deg"를
      위치기반 이탈판정(옵션)의 기본 원과, 좌/우 목표각의 기본값(±turn_deg)으로 재사용한다.
      (예전 <name>.plan_N/E/S/W 방위별 자동선택은 더 이상 쓰지 않는다 — 섹션 3 참고.)

    intersection.enabled=false 로 두면 이전처럼 교차로 로직 자체를 끌 수 있다
    (기본은 항상 활성 — 명령 기반이라 GPS 유무와 무관하게 학교 연습에서도 필요).
    """
    d = node.declare_parameter
    d("intersection.enabled", True)
    d("intersection.control_mode", "reference_route")
    if not bool(node.get_parameter("intersection.enabled").value):
        if get_logger:
            get_logger.info("교차로 비활성화(intersection.enabled=false)")
        return None
    if str(node.get_parameter("intersection.control_mode").value) == "reference_route":
        if get_logger:
            get_logger.info(
                "reference_route 교차로: legacy IMU-only controller 비활성화")
        return None

    d("intersection_names", [""])
    names = [n for n in node.get_parameter("intersection_names").value if n]

    cx, cy, radius, turn_deg = 0.0, 0.0, 2.5, 90.0
    if names:
        name0 = names[0]
        d(f"{name0}.center", [0.0, 0.0])
        d(f"{name0}.radius_m", 2.5)
        d(f"{name0}.turn_deg", 90.0)
        c = node.get_parameter(f"{name0}.center").value
        cx, cy = float(c[0]), float(c[1])
        radius = float(node.get_parameter(f"{name0}.radius_m").value)
        turn_deg = float(node.get_parameter(f"{name0}.turn_deg").value)

    max_steer = 25.0
    try:
        max_steer = float(node.get_parameter("max_steering_deg").value)
    except Exception:
        pass

    d("intersection.use_position_exit", False)
    d("intersection.right_target_yaw_deg", -turn_deg)
    d("intersection.right_yaw_tolerance_deg", 5.0)
    d("intersection.left_target_yaw_deg", turn_deg)
    d("intersection.left_yaw_tolerance_deg", 5.0)
    d("intersection.stabilize_s", 1.0)
    d("intersection.straight_timeout_s", 5.0)
    d("intersection.imu_invalid_abort_s", 0.5)

    p = node.get_parameter
    ctrl = IntersectionController(
        max_steer_deg=max_steer,
        right_target_yaw_deg=float(p("intersection.right_target_yaw_deg").value),
        right_yaw_tolerance_deg=float(p("intersection.right_yaw_tolerance_deg").value),
        left_target_yaw_deg=float(p("intersection.left_target_yaw_deg").value),
        left_yaw_tolerance_deg=float(p("intersection.left_yaw_tolerance_deg").value),
        stabilize_s=float(p("intersection.stabilize_s").value),
        straight_timeout_s=float(p("intersection.straight_timeout_s").value),
        imu_invalid_abort_s=float(p("intersection.imu_invalid_abort_s").value),
        use_position_exit=bool(p("intersection.use_position_exit").value),
        center=(cx, cy), radius_m=radius,
        logger=get_logger,
    )
    if get_logger:
        get_logger.info(
            f"교차로 controller 로드: center=({cx:.2f},{cy:.2f}) r={radius:.1f}m "
            f"right={ctrl.right_target:.0f}±{ctrl.right_tol:.0f}° "
            f"left={ctrl.left_target:.0f}±{ctrl.left_tol:.0f}° "
            f"use_position_exit={ctrl.use_position_exit} "
            f"(명령은 /intersection/command 로 받음)")
    return ctrl
