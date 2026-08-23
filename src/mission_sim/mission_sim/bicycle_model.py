"""
가상 차량 물리 모델 (자전거 모델, bicycle model).

실차와 동일하게 '속도 + 조향각'을 입력받아 위치/방향을 갱신한다.
mission_manager가 내보낸 명령이 실제로 차를 몰 수 있는지 폐루프로 검증하는 용도.

상태: x, y (m, ENU), heading(rad), v(m/s)
입력: 목표속도 cmd_v(m/s), 조향각 steer(rad)
갱신:
    v      += (cmd_v - v) * accel_gain * dt      (속도는 서서히 추종)
    heading+= v / wheelbase * tan(steer) * dt
    x      += v * cos(heading) * dt
    y      += v * sin(heading) * dt

경사(pitch)가 있으면 오르막에서 v가 덜 붙도록 간단한 감쇠를 준다.
"""
import math


class BicycleModel:
    def __init__(self, wheelbase=0.30, accel_gain=2.0):
        self.wb = wheelbase
        self.accel_gain = accel_gain
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0     # rad, East=0
        self.v = 0.0           # m/s
        self.pitch = 0.0       # rad, +면 오르막

    def set_pose(self, x, y, heading):
        self.x, self.y, self.heading = x, y, heading

    def step(self, cmd_v, steer_rad, dt):
        # 오르막이면 목표속도를 깎음(경사 저항 흉내)
        slope_factor = max(0.3, 1.0 - 1.5 * max(0.0, math.sin(self.pitch)))
        target_v = cmd_v * slope_factor

        self.v += (target_v - self.v) * self.accel_gain * dt
        # 조향 제한
        steer_rad = max(-0.6, min(0.6, steer_rad))
        self.heading += (self.v / self.wb) * math.tan(steer_rad) * dt
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        self.x += self.v * math.cos(self.heading) * dt
        self.y += self.v * math.sin(self.heading) * dt
        return self.x, self.y, self.heading, self.v
