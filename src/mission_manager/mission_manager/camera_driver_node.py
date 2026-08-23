#!/usr/bin/env python3
"""
camera_driver_node.py — RealSense D456 컬러영상 → 비전 주행/주차 골격

RealSense가 내주는 컬러 이미지(sensor_msgs/Image)를 구독해서
mission_manager가 쓰는 규약 토픽으로 비전 결과를 발행한다.

발행:
  /parking_slot (std_msgs/String)  — "left" | "right" | "unknown"
      주차 미션존 진입 '전에' 미리 발행돼야 함. mission_manager가
      _start_mission에서 이 값을 읽어 주차 방향(park_side)을 정한다.
  /camera/target_speed_mps (std_msgs/Float32)
      카메라 주행(폴백 C: GPS·DR 둘 다 불가할 때) 속도 제안.
  /camera/target_steering_deg (std_msgs/Float32)
      카메라 주행 조향 제안(차선 중앙 추종). +좌 / -우 (mission_manager 규약).

구독:
  컬러 이미지 IMAGE_TOPIC (sensor_msgs/Image)

★ 실물 오면 채울 곳 (전부 TODO):
  - _detect_lane()      : 차선 검출 → 조향/속도 산출
  - _detect_parking()   : 주차공간 좌/우 판정
  실제 CV(OpenCV/딥러닝)는 여기서. 지금은 골격이라 기본값만 발행.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


# ────────────────────────── 튜닝 상수 ──────────────────────────
# ── 구독 토픽 (실물에서 ros2 topic list로 확인해 맞출 것) ──
# realsense 기본 컬러 토픽은 namespace 중첩되어 /camera/camera/color/image_raw
IMAGE_TOPIC = "/camera/camera/color/image_raw"

# ── 발행 주기 ──
PUBLISH_HZ = 20.0        # 주행 제어와 맞춰 20Hz

# ── 카메라 주행 기본값 (비전 미구현 시 안전값) ──
DEFAULT_CAM_SPEED = 0.0    # 골격 단계: 0 = 안 움직임(안전). 비전 붙으면 산출값.
DEFAULT_CAM_STEER = 0.0    # 직진

# ── 조향 한계 (mission_manager max_steering_deg와 맞출 것) ──
MAX_STEER_DEG = 25.0

# ── 주차 판정 유지 ──
# 주차방향은 한번 정하면 미션존 들어갈 때까지 유지돼야 하므로,
# 매 프레임 튀지 않게 마지막 유효 판정을 붙잡는다.
PARKING_HOLD = True
# ───────────────────────────────────────────────────────────────


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__("camera_driver_node")

        # 최신 비전 결과 상태
        self.cam_speed = DEFAULT_CAM_SPEED
        self.cam_steer = DEFAULT_CAM_STEER
        self.parking_slot = "unknown"

        # 최신 이미지 (골격에선 안 씀; 실물 CV에서 사용)
        self._last_image = None

        # 발행
        self.slot_pub = self.create_publisher(String, "/parking_slot", 10)
        self.speed_pub = self.create_publisher(
            Float32, "/camera/target_speed_mps", 10)
        self.steer_pub = self.create_publisher(
            Float32, "/camera/target_steering_deg", 10)

        # 구독 (이미지)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        self.get_logger().info(f"이미지 구독: {IMAGE_TOPIC}")

        # 발행 타이머
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info("camera_driver_node 시작 (골격 모드)")

    def _on_image(self, msg: Image):
        """컬러 프레임 도착 → 비전 처리 (골격: 저장만)."""
        self._last_image = msg
        # ★ TODO(실물): 여기서 프레임을 cv_bridge로 변환해 CV 처리
        #   from cv_bridge import CvBridge
        #   frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #   self._detect_lane(frame)
        #   self._detect_parking(frame)
        self._detect_lane(None)
        self._detect_parking(None)

    # ---------- 비전 (TODO: 실물에서 구현) ----------
    def _detect_lane(self, frame):
        """
        차선 검출 → 카메라 주행 속도/조향 산출.
        ★ TODO(실물): 차선 검출(색/에지/딥러닝) → 차선 중앙과 화면 중앙의
           오차로 조향 계산, 곡률로 속도 조절.
        지금은 골격: 안전 기본값 유지(정지·직진).
        """
        if frame is None:
            self.cam_speed = DEFAULT_CAM_SPEED
            self.cam_steer = DEFAULT_CAM_STEER
            return
        # --- 실물 예시(의사코드) ---
        # offset = lane_center_x - image_center_x     # px
        # self.cam_steer = clamp(-K * offset, -MAX_STEER_DEG, MAX_STEER_DEG)
        # self.cam_speed = base_speed * (1 - abs(offset)/half_width)

    def _detect_parking(self, frame):
        """
        주차공간 좌/우 판정 → /parking_slot.
        ★ TODO(실물): 주차선/빈공간 인식 → "left"|"right" 결정.
        미션존 '진입 전'에 미리 판정돼야 하므로, 확신 있는 프레임에서만
        갱신하고 PARKING_HOLD로 마지막 값을 유지.
        지금은 골격: "unknown" 유지(mission_manager가 기본 우측 처리).
        """
        if frame is None:
            if not PARKING_HOLD:
                self.parking_slot = "unknown"
            return
        # --- 실물 예시(의사코드) ---
        # side = classify_parking(frame)   # "left"|"right"|None
        # if side is not None:
        #     self.parking_slot = side     # PARKING_HOLD면 유효값만 갱신

    def _publish(self):
        self.slot_pub.publish(String(data=self.parking_slot))
        self.speed_pub.publish(Float32(data=float(self.cam_speed)))
        self.steer_pub.publish(Float32(data=float(self.cam_steer)))


def main():
    rclpy.init()
    node = CameraDriverNode()
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
