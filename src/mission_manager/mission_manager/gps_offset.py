"""
GPS 안테나 오프셋 보정 (평면 2D, 단위 cm 입력).

■ 왜 필요한가
  GPS는 '안테나가 붙은 위치'의 좌표를 준다. 하지만 Pure Pursuit은
  차량의 특정 기준점(보통 뒷축 중심 = 자전거 모델의 회전 중심)을
  기준으로 조향을 계산해야 정확하다. 안테나가 그 기준점에서
  앞/옆으로 떨어져 있으면, 그만큼 위치가 어긋난다.

  이 모듈은 '안테나 좌표'를 heading을 이용해 '차량 기준점 좌표'로
  평면 변환한다. 높이(z)는 무시 → 완전 평면(2D) 기준.

■ 좌표계 약속 (navigation_controller / geo_utils와 동일)
  - x축 = East(동), y축 = North(북), 단위 = meter
  - heading = 차량이 향한 방향 (라디안). 수학 기준: East=0, 반시계(+).
    즉 차량 정면이 +x(동)를 볼 때 heading=0.

■ 오프셋 정의 (차량 몸체 기준, 단위 cm)
  기준점(뒷축 중심)에서 본 안테나의 상대 위치를 잰다.
      forward_cm : 차량 정면(앞) 방향으로 얼마나 앞에 있나 (앞=+, 뒤=-)
      lateral_cm : 차량 좌측 방향으로 얼마나 왼쪽에 있나 (왼쪽=+, 오른쪽=-)

  예) 안테나가 뒷축 중심보다 40cm 앞, 정중앙:
        forward_cm=40, lateral_cm=0
  예) 안테나가 뒷축 중심보다 55cm 앞, 오른쪽으로 10cm 치우침:
        forward_cm=55, lateral_cm=-10

  ※ 앞바퀴/뒷바퀴 어느 쪽을 기준으로 삼든 이 forward_cm 값만
     바꾸면 된다. 뒷축 기준으로 재는 것을 권장(Pure Pursuit 기본).

■ 사용법
  1) 실차에서 줄자로 안테나~뒷축 중심 거리를 잰다 (cm).
  2) AntennaOffset(forward_cm=..., lateral_cm=...) 로 생성.
  3) 매 GPS 콜백에서:
        rx, ry = offset.antenna_to_reference(ax, ay, heading)
     로 안테나 좌표(ax,ay) → 기준점 좌표(rx,ry) 변환 후 set_position에 넣는다.

  오프셋을 (0,0)으로 두면 아무 보정 없이 그대로 통과한다(현재 동작과 동일).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


CM_PER_M = 100.0


@dataclass(frozen=True)
class AntennaOffset:
    """차량 기준점에서 본 GPS 안테나의 평면 오프셋 (cm 입력)."""

    forward_cm: float = 0.0   # 차량 앞 방향(+). 뒤=음수.
    lateral_cm: float = 0.0   # 차량 왼쪽 방향(+). 오른쪽=음수.

    @property
    def forward_m(self) -> float:
        return self.forward_cm / CM_PER_M

    @property
    def lateral_m(self) -> float:
        return self.lateral_cm / CM_PER_M

    def is_zero(self) -> bool:
        return self.forward_cm == 0.0 and self.lateral_cm == 0.0

    def antenna_to_reference(self, ax: float, ay: float,
                             heading_rad: float) -> tuple[float, float]:
        """
        안테나 좌표(ax, ay) → 차량 기준점 좌표(rx, ry).

        안테나는 기준점보다 (forward_m 앞, lateral_m 왼쪽)에 있으므로,
        기준점 = 안테나 - (그 오프셋을 월드좌표로 회전한 벡터).

        월드좌표 오프셋 벡터 (heading=차량 정면 방향):
            정면(앞) 단위벡터  = ( cos h,  sin h)
            좌측(왼쪽) 단위벡터 = (-sin h,  cos h)
            offset_world = forward_m * 정면 + lateral_m * 좌측
        """
        if self.is_zero():
            return ax, ay
        ch, sh = math.cos(heading_rad), math.sin(heading_rad)
        off_x = self.forward_m * ch - self.lateral_m * sh
        off_y = self.forward_m * sh + self.lateral_m * ch
        return ax - off_x, ay - off_y

    def reference_to_antenna(self, rx: float, ry: float,
                             heading_rad: float) -> tuple[float, float]:
        """역변환: 기준점 좌표 → 안테나 좌표 (시뮬/디버그용)."""
        if self.is_zero():
            return rx, ry
        ch, sh = math.cos(heading_rad), math.sin(heading_rad)
        off_x = self.forward_m * ch - self.lateral_m * sh
        off_y = self.forward_m * sh + self.lateral_m * ch
        return rx + off_x, ry + off_y


def offset_from_wheel_positions(antenna_to_front_cm: float,
                                wheelbase_cm: float,
                                lateral_cm: float = 0.0) -> AntennaOffset:
    """
    편의 함수: '안테나~앞바퀴축' 거리와 축거로 '안테나~뒷축' 오프셋 계산.

    앞바퀴축을 기준으로 재는 게 편할 때 쓴다.
      antenna_to_front_cm : 안테나가 앞바퀴축보다 얼마나 앞인가 (앞=+, 뒤=-)
      wheelbase_cm        : 앞축~뒷축 거리 (양수)
    반환: 뒷축 중심 기준 AntennaOffset

    예) 안테나가 앞축보다 5cm 뒤(=antenna_to_front_cm=-5),
        축거 58cm → 뒷축 기준 forward = -5 + 58 = 53cm
    """
    forward_cm = antenna_to_front_cm + wheelbase_cm
    return AntennaOffset(forward_cm=forward_cm, lateral_cm=lateral_cm)
