"""
gps_stability — GPS 안정성을 0~100%로 점수화.

■ 목적
  GPS 재밍/멀티패스로 값이 튀거나 끊길 때, '지금 GPS를 얼마나 믿을 수
  있는가'를 하나의 숫자(0~100)로 요약해 /gps_stability 토픽으로 발행한다.
  MCU나 상위 로직이 이 값을 보고 스스로 판단한다:
    예) 80% 이상 → GPS 정속 주행
        30~80%   → 감속·주의
        30% 미만 → 정지 또는 다른 소스로 전환

■ 점수 산출 (네 요소를 곱해서 0~1 → ×100)
  1) fix 유무   : NO_FIX면 0 (재밍 최악). 있으면 1.
  2) 공분산     : 위치 불확실성(m²). 작을수록 좋음.
                  good_cov 이하=1.0, max_cov 이상=0.0, 사이는 선형.
  3) 위치 점프  : 직전 위치 대비 이동량이 물리적으로 불가능하게 크면 감점.
                  jump_soft 이하=1.0, jump_hard 이상=0.0, 사이는 선형.
  4) 최신성     : fix가 안 들어온 시간. fresh_s 이하=1.0,
                  timeout_s 이상=0.0, 사이는 선형.

  최종 = fix * cov_score * jump_score * fresh_score * 100
  (하나라도 0이면 전체 0 — 곱이라 '가장 약한 고리'가 지배. 재밍엔 이게 안전)

■ 왜 곱인가
  더하기(평균)면 한 요소가 완전히 망가져도 나머지가 받쳐줘 높은 점수가
  나올 수 있다. 재밍 상황에선 위험. 곱은 어느 하나라도 나쁘면 전체가
  떨어져서, '전부 좋아야 높은 점수'가 된다.

■ 사용 (gps_route_follower의 _on_fix에서)
    self.stability = GpsStability(good_cov_m2=..., max_cov_m2=self.max_cov, ...)
    score = self.stability.score(
        has_fix=..., covariance=..., jump_m=..., age_s=...)
    self.stability_pub.publish(Float32(data=score))
"""
from __future__ import annotations

from dataclasses import dataclass


def _linear_falloff(value: float, best: float, worst: float) -> float:
    """value가 best 이하면 1.0, worst 이상이면 0.0, 사이는 선형(1→0)."""
    if worst <= best:
        return 1.0
    if value <= best:
        return 1.0
    if value >= worst:
        return 0.0
    return 1.0 - (value - best) / (worst - best)


@dataclass
class GpsStability:
    # 공분산(m²): 이 값 이하면 만점, max 이상이면 0점
    good_cov_m2: float = 0.05      # RTK Fixed 수준(약 22cm² → 0.05)
    max_cov_m2: float = 9.0        # follower의 max_gps_covariance_m2와 맞출 것
    # 점프(m): 한 fix 사이 이동량. 이 이하면 만점, 이상이면 0점
    jump_soft_m: float = 1.0
    jump_hard_m: float = 5.0       # follower의 gps_jump_m와 맞출 것
    # 최신성(s): fix 경과시간. 이 이하면 만점, timeout 이상이면 0점
    fresh_s: float = 0.2
    timeout_s: float = 1.0         # follower의 gps_timeout_sec와 맞출 것

    def score(self, *, has_fix: bool, covariance: float,
              jump_m: float, age_s: float) -> float:
        """0.0~100.0 반환. 하나라도 치명적이면 곱에 의해 0에 수렴."""
        if not has_fix:
            return 0.0
        # 공분산이 비정상(inf/nan)이면 신뢰 불가
        if covariance != covariance or covariance == float("inf"):
            return 0.0
        cov_score = _linear_falloff(covariance, self.good_cov_m2, self.max_cov_m2)
        jump_score = _linear_falloff(jump_m, self.jump_soft_m, self.jump_hard_m)
        fresh_score = _linear_falloff(age_s, self.fresh_s, self.timeout_s)
        return round(100.0 * cov_score * jump_score * fresh_score, 1)
