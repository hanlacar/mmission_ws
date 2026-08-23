"""
미션 모드 4종 — 실제 궤적 구현.

각 미션 공통 인터페이스:
  - vehicle_mode: IMU 매니저 등에 방송할 모드 문자열
  - step(cur) -> (speed_mps, steer_deg, done_bool)
      cur = (x, y, heading_rad). 20Hz(control_rate_hz)로 호출.
      done=True 반환 시 미션 종료 → 일반 주행 복귀.

설계 원칙(중요):
  * 미션존 반경(~2m) 안에서는 GPS/heading 정밀도를 믿지 않는다.
    그래서 각 미션은 '진행거리(엔코더)'와 '상대회전각(IMU yaw)'을
    시작시점 기준 증분으로 스스로 적분해 phase를 넘긴다.
    → cur 좌표가 흔들려도 궤적이 반복 재현된다.
  * 거리 판정은 node.enc_ticks * encoder_m_per_tick 로 계산.
    encoder_m_per_tick==0(미보정)이면 거리판정 불가 → 시간기반 폴백.
    (시뮬 sim.yaml은 0.01이라 정상. 실차는 1m 굴려 tick 세서 채울 것.)
  * 조향 부호: 좌 +, 우 - (memo 기준). pure_pursuit(y_v 좌+)와 일관.

각 미션은 _Base의 헬퍼(traveled_m, turned_deg)를 써서
'전진 Xm → 정지 → 조향 후진 Ym → …' 식 phase 머신으로 동작한다.

실차 튜닝 포인트는 클래스 상단 상수(대문자)로 모아둠. yaml로 빼도 됨.

주행 방식 분류(미션 이름) 6종: NORMAL, SLOPE, S_COURSE, T_PARK, PARALLEL_PARK, OUT.
  - T_PARK = TPark.vehicle_mode
  - PARALLEL_PARK = ParallelPark.vehicle_mode
  - SLOPE = SlopeStop.vehicle_mode
  - NORMAL = _Base/Accel/RightTurn.vehicle_mode (기본값)
  - S_COURSE(S자 코스), OUT(출차): 이 파일에 대응하는 미션 클래스 없음.
    gps_route_follower가 경로 CSV의 mode 컬럼 값을 그대로 /drive_mode로
    발행하므로, 코드 구현 없이 CSV 기록 시 mode 컬럼에 "S_COURSE"/"OUT"
    문자열을 써서 표현한다. 여기서는 문자열 상수만 참고용으로 남긴다.
"""
import math

# CSV mode 컬럼 전용 값(이 파일의 미션 클래스로 구현되지 않음). 문자열 상수로만 존재.
S_COURSE = "S_COURSE"
OUT = "OUT"


class _Base:
    vehicle_mode = "NORMAL"

    def __init__(self, node, park_side=None):
        self.node = node
        # 주차 방향: "left"/"right"/None. None이면 클래스 기본 STEER_SIGN 사용.
        # 카메라 /parking_slot 로 현장에서 즉석 지정되는 값을 받는다.
        self.park_side = park_side
        # 시간은 벽시계 대신 '제어주기(step) 호출 횟수'로 잰다.
        #   경과초 = tick수 / control_rate_hz
        # → 실차(실시간)·오프라인 시뮬(가상시간)·배속 모두에서 일관.
        #   step()마다 _tick()을 반드시 먼저 호출할 것.
        self._ticks = 0
        self._phase_tick0 = 0
        self.phase = 0
        # 시작 시점 스냅샷(증분 적분 기준)
        self._enc0 = self._enc_now()
        self._yaw0 = node.imu_rel_yaw          # rad
        self._phase_enc0 = self._enc0
        self._phase_yaw0 = self._yaw0

    def _steer_sign(self):
        """이 미션의 조향 부호. park_side가 지정되면 그에 맞춰 결정,
        아니면 클래스 기본 STEER_SIGN. 부호: 좌조향 +, 우조향 -.
        차고/공간이 우측이면 우조향(-), 좌측이면 좌조향(+)."""
        base = getattr(self, "STEER_SIGN", -1.0)
        if self.park_side == "right":
            return -1.0
        if self.park_side == "left":
            return +1.0
        return base   # unknown/None → 클래스 기본값

    def _rate(self):
        try:
            r = self.node.pf("control_rate_hz")
            return r if r > 0 else 20.0
        except Exception:
            return 20.0

    def _tick(self):
        """각 미션 step() 진입 첫 줄에서 호출. 주기 카운트 증가."""
        self._ticks += 1

    # ---------- 시간 ----------
    def elapsed(self):
        return self._ticks / self._rate()

    def phase_elapsed(self):
        return (self._ticks - self._phase_tick0) / self._rate()

    # ---------- 엔코더 거리 ----------
    def _m_per_tick(self):
        try:
            return self.node.pf("encoder_m_per_tick")
        except Exception:
            return 0.0

    def _enc_now(self):
        # 누적 tick → meter. 미보정(0)이면 None 반환(거리판정 불가 신호).
        mpt = self._m_per_tick()
        if mpt <= 0.0:
            return None
        return self.node.enc_ticks * mpt

    def traveled_m(self):
        """이번 phase 시작 이후 진행한 |거리|(m). 미보정이면 None."""
        now = self._enc_now()
        if now is None or self._phase_enc0 is None:
            return None
        return abs(now - self._phase_enc0)

    def calibrated(self):
        return self._m_per_tick() > 0.0

    # ---------- IMU 상대 회전각 ----------
    def turned_deg(self):
        """이번 phase 시작 이후 회전한 각도(deg, 부호포함). 좌회전 +."""
        d = self.node.imu_rel_yaw - self._phase_yaw0
        # -180~180 정규화
        d = math.atan2(math.sin(d), math.cos(d))
        return math.degrees(d)

    # ---------- phase 전환 ----------
    def next_phase(self):
        self.phase += 1
        self._phase_tick0 = self._ticks
        e = self._enc_now()
        self._phase_enc0 = e
        self._phase_yaw0 = self.node.imu_rel_yaw

    def step(self, cur):
        raise NotImplementedError

    # ---------- 공용 파라미터 ----------
    def _mspeed(self):
        try:
            return self.node.pf("mission_speed_mps")
        except Exception:
            return 0.4

    def _max_steer(self):
        try:
            return self.node.pf("max_steering_deg")
        except Exception:
            return 25.0


# ============================================================
#  T자 주차 (전진 진입 → 조향 후진 진입 → 정렬 → 정지 → 복귀)
# ============================================================
class TPark(_Base):
    """
    T자 코스: 진입로에서 직각으로 꺾어 후진으로 차고에 넣는다.
    phase:
      0) 차고 옆 기준선까지 소폭 전진 (정렬 위치 확보)
      1) 우조향 상태로 후진 → 약 90도 차체 회전 (차고 정면 진입)
      2) 직진 후진으로 차고 안쪽까지 (거리)
      3) 정지 유지(정차 인정 시간)
      4) 직진 전진으로 차고 탈출 (원래 라인 복귀)
      5) 완료
    부호: 우조향은 -, 후진은 speed<0.
    후진+우조향(-)이면 물리상 차 앞머리는 좌로, 뒤는 우 차고로 들어감.
    실차에서 차고가 좌측이면 STEER_SIGN을 +1로 바꾸면 대칭.
    """
    vehicle_mode = "T_PARK"

    APPROACH_M = 0.6      # phase0 전진거리
    TURN_DEG = 80.0       # phase1 후진회전 목표각
    BACK_IN_M = 0.8       # phase2 차고 진입 후진거리
    HOLD_S = 3.0          # phase3 정차시간
    OUT_M = 1.2           # phase4 탈출 전진거리
    STEER_SIGN = -1.0     # 차고가 우측: -1(우조향 후진). 좌측이면 +1.
    APPROACH_T = 1.5      # 미보정시 폴백 시간
    BACK_IN_T = 2.0
    OUT_T = 3.0

    def step(self, cur):
        self._tick()
        spd = self._mspeed()
        st = self._max_steer()

        if self.phase == 0:  # 전진 정렬
            done = (self.traveled_m() is not None and self.traveled_m() >= self.APPROACH_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.APPROACH_T)
            if done:
                self.next_phase()
                return 0.0, 0.0, False
            return spd, 0.0, False

        if self.phase == 1:  # 조향 후진으로 90도 회전
            if abs(self.turned_deg()) >= self.TURN_DEG:
                self.next_phase()
                return 0.0, 0.0, False
            return -spd, self._steer_sign() * st, False

        if self.phase == 2:  # 직진 후진 차고 진입
            done = (self.traveled_m() is not None and self.traveled_m() >= self.BACK_IN_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.BACK_IN_T)
            if done:
                self.next_phase()
                return 0.0, 0.0, False
            return -spd, 0.0, False

        if self.phase == 3:  # 정차
            if self.phase_elapsed() >= self.HOLD_S:
                self.next_phase()
            return 0.0, 0.0, False

        if self.phase == 4:  # 전진 탈출
            done = (self.traveled_m() is not None and self.traveled_m() >= self.OUT_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.OUT_T)
            if done:
                return 0.0, 0.0, True
            return spd, 0.0, False

        return 0.0, 0.0, True


# ============================================================
#  평행 주차 (2단 조향 후진이 핵심)
# ============================================================
class ParallelPark(_Base):
    """
    평행주차 표준 궤적:
      0) 주차공간 옆을 지나 앞차 뒤범퍼선까지 전진 정렬
      1) 우조향 후진 (차 뒷부분을 공간 안쪽으로) — 약 40도 사선
      2) 조향 반대(좌) 후진 (차체 펴며 공간에 평행)
      3) 소폭 전진으로 중앙 정렬
      4) 정차
      5) 완료(탈출은 일반주행이 이어받음)
    """
    vehicle_mode = "PARALLEL_PARK"

    ALIGN_M = 0.5
    PHASE1_DEG = 40.0     # 1단 사선 진입각
    PHASE2_DEG = 40.0     # 2단 반대조향으로 되펴는 각
    CENTER_M = 0.3
    HOLD_S = 3.0
    STEER_SIGN = -1.0     # 공간이 우측: 1단 우조향(-)
    ALIGN_T = 1.2
    CENTER_T = 1.0

    def step(self, cur):
        self._tick()
        spd = self._mspeed()
        st = self._max_steer()

        if self.phase == 0:  # 전진 정렬
            done = (self.traveled_m() is not None and self.traveled_m() >= self.ALIGN_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.ALIGN_T)
            if done:
                self.next_phase()
                return 0.0, 0.0, False
            return spd, 0.0, False

        if self.phase == 1:  # 1단: 우조향 후진(사선 진입)
            if abs(self.turned_deg()) >= self.PHASE1_DEG:
                self.next_phase()
                return 0.0, 0.0, False
            return -spd, self._steer_sign() * st, False

        if self.phase == 2:  # 2단: 좌조향 후진(차체 되펴기)
            # 목표: phase1에서 틀어진 각을 거의 0으로 되돌림
            if abs(self.turned_deg()) >= self.PHASE2_DEG:
                self.next_phase()
                return 0.0, 0.0, False
            return -spd, -self._steer_sign() * st, False

        if self.phase == 3:  # 소폭 전진 정렬
            done = (self.traveled_m() is not None and self.traveled_m() >= self.CENTER_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.CENTER_T)
            if done:
                self.next_phase()
                return 0.0, 0.0, False
            return spd, 0.0, False

        if self.phase == 4:  # 정차
            if self.phase_elapsed() >= self.HOLD_S:
                return 0.0, 0.0, True
            return 0.0, 0.0, False

        return 0.0, 0.0, True


# ============================================================
#  가속 구간 (직진 최고속 → 존 이탈/거리 후 종료)
# ============================================================
class Accel(_Base):
    """
    직진 가속 구간. 정해진 거리만큼 accel_speed로 직진 후 종료.
    거리 판정 불가(미보정)면 시간 폴백.
    조향은 0 고정(직진). 필요시 살짝 pure-pursuit 보정 가능하나
    가속존은 보통 직선이라 0으로 둔다.
    """
    vehicle_mode = "NORMAL"

    RUN_M = 8.0           # 가속 유지 거리
    RUN_T = 3.0           # 폴백 시간

    def _aspeed(self):
        try:
            return self.node.pf("accel_speed_mps")
        except Exception:
            return 2.0

    def step(self, cur):
        self._tick()
        spd = self._aspeed()
        if self.phase == 0:
            done = (self.traveled_m() is not None and self.traveled_m() >= self.RUN_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.RUN_T)
            if done:
                return 0.0, 0.0, True
            return spd, 0.0, False
        return 0.0, 0.0, True


# ============================================================
#  우회전 (IMU yaw로 90도 회전 완료 감지)
# ============================================================
class RightTurn(_Base):
    """
    교차로/코너 우회전. 전진하며 우조향으로 약 90도 돌면 종료.
    우회전 = heading 감소 = turned_deg 음(-). 그래서 |turned| 로 판정.
    """
    vehicle_mode = "NORMAL"

    TURN_DEG = 85.0       # 목표 회전각(살짝 못미쳐 종료 → 오버슛 방지)
    STEER_SIGN = -1.0     # 우조향 -
    FALLBACK_T = 6.0      # 안전 타임아웃

    def step(self, cur):
        self._tick()
        spd = self._mspeed()
        st = self._max_steer()
        turned = abs(self.turned_deg())
        if turned >= self.TURN_DEG or self.phase_elapsed() >= self.FALLBACK_T:
            return 0.0, 0.0, True
        return spd, self.STEER_SIGN * st, False


# ============================================================
#  경사로 정지·출발 (오르막 감지 → 정지선 정차 → 밀림 없이 재출발)
# ============================================================
class SlopeStop(_Base):
    """
    대형 장내기능 경사로 미션.
    IMU pitch(오르막 +)로 오르막 진입을 감지하고, 그 시점부터
    엔코더 거리로 '정지선'까지 전진해 정차한 뒤, 밀림 없이 재출발한다.

    phase:
      0) 오르막 진입 대기 : 전진하며 pitch >= UPHILL_DEG 감지 대기
      1) 정지선까지 전진   : 오르막 진입 후 STOPLINE_M 만큼 더 전진
      2) 정차 홀드        : 정지 + HOLD_S초. 이 구간에서 뒤로 밀리면
                           (엔코더 후진 tick) 재출발 전까지 홀드 유지.
      3) 재출발·등판      : 다시 전진해 정상 통과. pitch가 내리막(-)까지
                           떨어졌다 평지로 돌아오면 경사 벗어난 것으로 종료.
      4) 완료

    pitch 토픽(/imu/pitch_deg)이 아직 없으면 node.imu_pitch_deg가 0으로
    고정 → 오르막을 영영 감지 못 함. 그래서 phase0/3에 '시간 폴백'을 둬서
    pitch 없이도 최소한 진행은 하게 한다(실차 pitch 붙이면 정상 동작).

    부호: 전진 speed>0, 조향 0(직진 경사로 가정). 밀림 감지는 엔코더
    tick 증감으로. STEER_SIGN 없음(직선).
    """
    vehicle_mode = "SLOPE"

    UPHILL_DEG = 5.0       # 이 이상이면 오르막으로 판정(deg)
    FLAT_DEG = 2.0         # 이 이하로 돌아오면 평지 복귀로 판정
    STOPLINE_M = 1.0       # 오르막 진입 후 정지선까지 거리
    HOLD_S = 3.0           # 정차 유지 시간
    CREEP_STEER = 0.0      # 경사로는 직선 가정(곡선이면 조향 넣기)
    ENTER_T = 4.0          # phase0 pitch 미검출 시 폴백(이 시간 후 강제 진입판정)
    STOPLINE_T = 2.0       # phase1 거리 미보정 시 폴백
    CLEAR_T = 5.0          # phase3 pitch 미검출 시 폴백(이 시간 후 종료)
    ROLLBACK_TICKS = 3     # 이만큼 뒤로 밀리면 밀림으로 간주

    def _pitch(self):
        # node에 imu_pitch_deg 속성이 있으면 사용, 없으면 0(폴백 유도)
        return float(getattr(self.node, "imu_pitch_deg", 0.0))

    def step(self, cur):
        self._tick()
        spd = self._mspeed()

        if self.phase == 0:  # 오르막 진입 대기 (전진)
            uphill = self._pitch() >= self.UPHILL_DEG
            fallback = self.phase_elapsed() >= self.ENTER_T
            if uphill or fallback:
                self.next_phase()
                return spd, self.CREEP_STEER, False
            return spd, self.CREEP_STEER, False

        if self.phase == 1:  # 정지선까지 전진
            done = (self.traveled_m() is not None and self.traveled_m() >= self.STOPLINE_M) \
                   or (not self.calibrated() and self.phase_elapsed() >= self.STOPLINE_T)
            if done:
                self.next_phase()
                return 0.0, 0.0, False
            return spd, self.CREEP_STEER, False

        if self.phase == 2:  # 정차 홀드 (밀림 감지)
            # 뒤로 밀리는지 감시: phase 시작 대비 tick이 줄면 밀림.
            # (하드웨어 브레이크가 없으면 speed=0만으로는 못 막으니,
            #  밀림 감지 시 살짝 전진 토크로 버틴다.)
            rolled_back = False
            if self.calibrated():
                # traveled_m은 abs라 방향을 모름 → 원시 tick 증분으로 판정
                d = self.node.enc_ticks - self._phase_start_ticks()
                rolled_back = d <= -self.ROLLBACK_TICKS
            if rolled_back:
                # 밀림 저지: 홀드 타이머 리셋하고 살짝 전진해 버팀
                self._reset_phase_timer()
                return spd * 0.5, 0.0, False
            if self.phase_elapsed() >= self.HOLD_S:
                self.next_phase()
            return 0.0, 0.0, False

        if self.phase == 3:  # 재출발·등판 → 평지 복귀 시 종료
            pitch = self._pitch()
            # 오르막을 넘어 평지(|pitch|<=FLAT_DEG)로 복귀하면 종료.
            cleared = abs(pitch) <= self.FLAT_DEG and self.phase_elapsed() >= 1.0
            fallback = self.phase_elapsed() >= self.CLEAR_T
            if cleared or fallback:
                return 0.0, 0.0, True
            return spd, self.CREEP_STEER, False

        return 0.0, 0.0, True

    # --- phase2 밀림 감지용 원시 tick 기준점 ---
    def _phase_start_ticks(self):
        # next_phase() 직후의 enc_ticks를 기억해두기 위한 헬퍼.
        # _Base에 원시 tick 스냅샷이 없으므로 여기서 지연 초기화.
        if not hasattr(self, "_ptick0") or self._ptick0_phase != self.phase:
            self._ptick0 = self.node.enc_ticks
            self._ptick0_phase = self.phase
        return self._ptick0

    def _reset_phase_timer(self):
        self._phase_tick0 = self._ticks


_REGISTRY = {
    "tpark": TPark,
    "parallel": ParallelPark,
    "accel": Accel,
    "right_turn": RightTurn,
    "slope": SlopeStop,
}


def make_mission(name, node, park_side=None):
    """park_side: "left"/"right"/None. 주차류 미션의 조향 방향을
    카메라 /parking_slot 값으로 현장 지정. None/unknown이면 클래스 기본."""
    cls = _REGISTRY.get(name)
    if cls is None:
        node.get_logger().warn(f"알 수 없는 미션: {name} → 즉시 종료")

        class _NoOp(_Base):
            def step(self, cur): return 0.0, 0.0, True
        return _NoOp(node)
    return cls(node, park_side=park_side)
