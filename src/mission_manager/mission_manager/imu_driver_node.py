#!/usr/bin/env python3
"""
imu_driver_node.py — RealSense D456 IMU → heading/pitch 재발행

RealSense(realsense2_camera)가 내주는 raw IMU(sensor_msgs/Imu)를 구독해서
mission_manager가 쓰는 규약 토픽으로 변환 발행한다.

발행:
  /imu/relative_yaw_deg (std_msgs/Float32)
      — 노드 시작 시점을 0으로 하는 상대 yaw(도). gyro z축 각속도 적분.
        mission_manager의 추측항법이 (현재 - 시드시점)으로 heading 변화량 계산.
  /imu/pitch_deg (std_msgs/Float32)
      — pitch(도). 오르막 +. accel(중력벡터)로 계산. 경사로 미션이 기다림.

구독 (택1, 상수 IMU_SOURCE로 선택):
  A) 통합 토픽  /camera/camera/imu  (sensor_msgs/Imu)
     ← realsense를 unite_imu_method>0 으로 켰을 때. 권장.
  B) 분리 토픽  /camera/camera/gyro/sample + /camera/camera/accel/sample
     ← unite 안 켰을 때.

★ 실물 오면 확인할 것 (코드 하단 [실차 체크] 참고):
  1) 실제 IMU 토픽명이 /camera/imu 인지 /camera/camera/imu 인지 → IMU_TOPIC 수정
  2) yaw 증가방향이 ENU 반시계(+)와 일치하는지 → YAW_SIGN
  3) pitch 오르막이 +로 나오는지 → PITCH_SIGN
  4) 카메라(IMU) 장착 방향(어느 축이 전진/좌우/상하인지) → 축 매핑 상수
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


# ────────────────────────── 튜닝 상수 ──────────────────────────
# ── 구독 소스 선택 ──
IMU_SOURCE = "united"          # "united"(통합 /imu) | "split"(gyro+accel 분리)

# 통합 토픽명 (united 모드) — 실물에서 ros2 topic list로 확인해 맞출 것
IMU_TOPIC = "/camera/camera/imu"

# 분리 토픽명 (split 모드)
GYRO_TOPIC = "/camera/camera/gyro/sample"
ACCEL_TOPIC = "/camera/camera/accel/sample"

# ── 발행 주기 ──
PUBLISH_HZ = 50.0

# ── 부호/축 (실차에서 반드시 확인) ──
# yaw: gyro z 적분값의 부호. mission_manager는 ENU 반시계(좌회전)를 +로 본다.
#      차를 좌회전시켰을 때 relative_yaw_deg가 증가해야 정상. 반대면 -1.0.
YAW_SIGN = 1.0
# pitch: 오르막에서 +가 나와야 경사로 미션이 감지한다. 반대면 -1.0.
PITCH_SIGN = 1.0

# ── 축 매핑 ──
# RealSense IMU 광학 프레임은 통상 z=광축(전방), x=우, y=하 방향이라
# 차량 z축(수직) 회전(=yaw)이 gyro의 어느 축인지 장착에 따라 다르다.
# 카메라를 정면 보게 세워 달면 '차량 yaw' = 'IMU y축 각속도'인 경우가 많다.
# 아래에서 yaw로 쓸 gyro 축, pitch 계산에 쓸 accel 축을 고른다.
#   값: "x" | "y" | "z"  (부호 반전이 필요하면 위 YAW_SIGN/PITCH_SIGN로)
GYRO_YAW_AXIS = "y"      # 차량 수직축 회전에 해당하는 gyro 축
# pitch = 전방축 기울기. accel 중력 성분으로 계산 (자세한 식은 _compute_pitch).
ACCEL_FWD_AXIS = "z"    # 카메라 광축(차량 전방)에 해당하는 accel 축
ACCEL_UP_AXIS = "y"     # 차량 수직(위)에 해당하는 accel 축 (부호는 상황따라)

# ── 자이로 드리프트 완화 ──
# 정지 시 미세한 gyro bias가 적분되어 heading이 슬슬 도는 것을 막기 위해,
# 각속도 크기가 이 값(rad/s) 미만이면 적분하지 않는다(정지 간주).
GYRO_DEADBAND = 0.01
# ───────────────────────────────────────────────────────────────


def _axis(vec, name):
    return {"x": vec.x, "y": vec.y, "z": vec.z}[name]


class ImuDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver_node")

        self.rel_yaw = 0.0          # 적분된 상대 yaw (rad)
        self.pitch = 0.0            # 최신 pitch (rad)
        self._last_t = None         # gyro 적분용 이전 시각

        # accel 최신값 (split 모드에서 gyro/accel 시각이 달라 저장해둠)
        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0

        self.yaw_pub = self.create_publisher(Float32, "/imu/relative_yaw_deg", 10)
        self.pitch_pub = self.create_publisher(Float32, "/imu/pitch_deg", 10)

        if IMU_SOURCE == "united":
            self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 50)
            self.get_logger().info(f"IMU 구독(통합): {IMU_TOPIC}")
        else:
            from sensor_msgs.msg import Imu as _Imu  # gyro/accel도 Imu 타입
            self.create_subscription(_Imu, GYRO_TOPIC, self._on_gyro, 50)
            self.create_subscription(_Imu, ACCEL_TOPIC, self._on_accel, 50)
            self.get_logger().info(
                f"IMU 구독(분리): gyro={GYRO_TOPIC}, accel={ACCEL_TOPIC}")

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info("imu_driver_node 시작")

    # ---------- 통합 모드 ----------
    def _on_imu(self, msg: Imu):
        t = self._stamp_sec(msg)
        self._integrate_yaw(_axis(msg.angular_velocity, GYRO_YAW_AXIS), t)
        # accel 저장 후 pitch 계산
        self._ax = msg.linear_acceleration.x
        self._ay = msg.linear_acceleration.y
        self._az = msg.linear_acceleration.z
        self._compute_pitch()

    # ---------- 분리 모드 ----------
    def _on_gyro(self, msg: Imu):
        t = self._stamp_sec(msg)
        self._integrate_yaw(_axis(msg.angular_velocity, GYRO_YAW_AXIS), t)

    def _on_accel(self, msg: Imu):
        self._ax = msg.linear_acceleration.x
        self._ay = msg.linear_acceleration.y
        self._az = msg.linear_acceleration.z
        self._compute_pitch()

    # ---------- 계산 ----------
    def _integrate_yaw(self, wz, t):
        """gyro 각속도(rad/s)를 시간 적분해 상대 yaw 갱신."""
        if self._last_t is None:
            self._last_t = t
            return
        dt = t - self._last_t
        self._last_t = t
        if dt <= 0 or dt > 0.5:      # 비정상 dt 방어(끊김/역행)
            return
        if abs(wz) < GYRO_DEADBAND:  # 정지 간주 → 적분 안 함(드리프트 완화)
            return
        self.rel_yaw += wz * dt

    def _compute_pitch(self):
        """중력 가속도 벡터로 pitch(전후 기울기) 계산. 오르막 +(PITCH_SIGN 적용)."""
        fwd = {"x": self._ax, "y": self._ay, "z": self._az}[ACCEL_FWD_AXIS]
        up = {"x": self._ax, "y": self._ay, "z": self._az}[ACCEL_UP_AXIS]
        # 평지에선 중력이 up축에 실리고 fwd축은 ~0.
        # 오르막에선 fwd축에 중력 성분이 생김. atan2로 각도화.
        # 부호/축은 실차에서 [실차 체크]대로 맞춘다.
        self.pitch = math.atan2(fwd, abs(up) + 1e-6)

    def _publish(self):
        yaw_deg = YAW_SIGN * math.degrees(self.rel_yaw)
        pitch_deg = PITCH_SIGN * math.degrees(self.pitch)
        self.yaw_pub.publish(Float32(data=float(yaw_deg)))
        self.pitch_pub.publish(Float32(data=float(pitch_deg)))

    @staticmethod
    def _stamp_sec(msg):
        s = msg.header.stamp
        return s.sec + s.nanosec * 1e-9


def main():
    rclpy.init()
    node = ImuDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
