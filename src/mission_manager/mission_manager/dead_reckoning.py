"""
추측항법(Dead-Reckoning) — GPS 재밍 시 위치를 이어서 추정.

원리:
  - 엔코더 => 얼마나 나아갔는가 (ds, 이동거리 증분)
  - IMU yaw => 지금 어느 방향을 보는가 (heading)
  두 개를 적분하면 GPS 없이도 (x, y)를 계속 갱신할 수 있다.

    x += ds * cos(heading)
    y += ds * sin(heading)

주의:
  - 시간이 지날수록 오차가 누적된다(드리프트). 그래서 이건 어디까지나
    'GPS가 다시 잡힐 때까지 버티는 용도'다. 짧은 재밍 구간에 유효.
  - GPS가 정상일 때는 매 fix마다 (x,y,heading)을 GPS 값으로 리셋(seed)해서
    드리프트를 0으로 되돌린다.
"""
import math


class DeadReckoning:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0     # rad, ENU 기준(East=0, 반시계 +)
        self._have_seed = False

    def seed(self, x, y, heading_rad):
        """GPS 양호 구간에서 현재 추정을 GPS 실측으로 재설정(드리프트 제거)."""
        self.x = x
        self.y = y
        self.heading = heading_rad
        self._have_seed = True

    def update(self, ds_m, heading_rad):
        """
        엔코더 이동거리 증분 ds_m(+전진/-후진)과
        IMU가 주는 절대 heading(rad)으로 위치 갱신.
        heading은 IMU 상대yaw를 seed시점 heading에 더해 절대화한 값을 넣는다.
        """
        if not self._have_seed:
            return
        self.heading = heading_rad
        self.x += ds_m * math.cos(self.heading)
        self.y += ds_m * math.sin(self.heading)

    @property
    def pose(self):
        return self.x, self.y, self.heading
