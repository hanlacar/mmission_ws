#!/usr/bin/env python3
"""
wheel_odom_node.py — 엔코더+IMU → /odom + odom→base_footprint TF 발행.

왜 필요한가:
  팀원 SLAM 스택(slam_toolbox)은 map→odom을 스캔매칭으로 만들지만,
  그 아래 odom→base_footprint TF와 /odom(nav_msgs/Odometry)는 '누군가'
  발행해줘야 한다. 시뮬(Gazebo)에서는 Ackermann 플러그인이 공짜로 줬지만,
  실물 차에는 그게 없다. 그 빈자리를 이 노드가 메운다.

  재료는 이미 mission_manager가 쓰는 것과 동일:
    - /mcu/encoder (Int32, 누적; 파라미터) → 이동거리 증분 ds
    - /imu/relative_yaw_deg (Float32, 도)   → heading (ENU 반시계 +)
  수식은 dead_reckoning.py와 같다:  x += ds·cos(h),  y += ds·sin(h)
  즉 '추측항법 계산을 /odom 발행용으로 재포장'한 것이다.

발행:
  /odom (nav_msgs/Odometry)
      header.frame_id = odom, child_frame_id = base_footprint
      pose: 시작점을 원점(0,0,yaw0=0)으로 하는 상대 위치 (odom 표준)
      twist: body frame 속도 (linear.x 전진속도, angular.z yaw rate)
  TF: odom → base_footprint (위 pose와 동일)

프레임(팀원 slam_toolbox.yaml과 반드시 일치):
  odom_frame  = "odom"
  base_frame  = "base_footprint"   ← base_link 아님! bridge/SLAM 규약에 맞춤

★ 실물 오면:
  - encoder_m_per_tick 실측 보정 (지금 0이면 위치가 안 움직임 → 경고)
    두뇌(mission_manager)의 encoder_m_per_tick과 '같은 값'을 넣을 것.
  - IMU yaw 부호가 ENU 반시계(+)와 맞는지 확인 (imu_driver의 YAW_SIGN에서 이미 맞췄다면 여기선 그대로).

주의:
  IMU relative_yaw_deg는 '노드 시작 시점 0 기준'이다. odom 프레임도
  '시작점 기준 상대'가 표준이므로, 시작 yaw를 0으로 두면 정합이 맞는다.
  SLAM이 map→odom으로 절대 정합을 담당하니 odom 자체는 상대면 충분.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw):
    """ENU yaw(rad) → geometry_msgs/Quaternion (z축 회전만)."""
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(0.5 * yaw),
        w=math.cos(0.5 * yaw),
    )


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_node")

        # ---------------- 파라미터 ----------------
        # encoder_m_per_tick: 두뇌(mission_manager)와 '같은 이름/같은 값'을 쓴다.
        #   0.0이면 미보정 → 위치가 갱신되지 않음(경고 발생). 실물 1m 굴려 보정.
        self.declare_parameter("encoder_m_per_tick", 0.0)
        # 최신 t870_mcu bridge 계약. 구형 /encoder/ticks가 필요하면 override한다.
        self.declare_parameter("encoder_topic", "/mcu/encoder")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        # tick 부호: 전진 시 tick이 증가해야 정상. 반대면 True.
        #   (encoder_driver의 TICK_INVERT에서 이미 맞췄다면 여기선 False 유지)
        self.declare_parameter("tick_invert", False)
        # publish_tf: 다른 곳에서 odom TF를 이미 쏘면 False로 꺼서 충돌 방지.
        self.declare_parameter("publish_tf", True)

        self.m_per_tick = float(self.get_parameter("encoder_m_per_tick").value)
        self.encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tick_invert = bool(self.get_parameter("tick_invert").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # ---------------- 상태 ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0                 # 현재 heading (rad, ENU) = imu_rel_yaw
        self.imu_rel_yaw = 0.0         # 최신 IMU 상대yaw (rad)
        self.enc_ticks = 0             # 최신 누적 tick
        self._enc_last = None          # 직전 tick (증분 계산용)
        self._last_pub_t = None        # twist용 이전 발행시각
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0

        # ---------------- 구독 ----------------
        # 센서 데이터는 SENSOR_DATA(best effort)가 자연스럽지만,
        # encoder/imu 드라이버가 기본(RELIABLE)로 쏘므로 기본 depth로 맞춘다.
        self.create_subscription(Int32, self.encoder_topic, self._on_enc, 10)
        self.create_subscription(
            Float32, "/imu/relative_yaw_deg", self._on_imu_yaw, 10)

        # ---------------- 발행 ----------------
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None

        # ---------------- 타이머 ----------------
        self.create_timer(1.0 / self.publish_hz, self._update)

        # ---------------- 경고 ----------------
        if self.m_per_tick == 0.0:
            self.get_logger().warn(
                "encoder_m_per_tick=0 (미보정). /odom 위치가 갱신되지 않습니다. "
                "실물 1m 실측 후 이 값과 mission_manager의 encoder_m_per_tick을 "
                "함께 갱신하세요.")
        self.get_logger().info(
            f"wheel_odom_node 시작 "
            f"(encoder_topic={self.encoder_topic}, odom_frame={self.odom_frame}, "
            f"base_frame={self.base_frame}, "
            f"m/tick={self.m_per_tick}, publish_tf={self.publish_tf})")

    # ---------------- 콜백 ----------------
    def _on_enc(self, msg: Int32):
        ticks = -msg.data if self.tick_invert else msg.data
        self.enc_ticks = ticks

    def _on_imu_yaw(self, msg: Float32):
        # 도 → 라디안. ENU 반시계(+) 규약은 imu_driver에서 이미 맞춰준 값.
        self.imu_rel_yaw = math.radians(msg.data)

    # ---------------- 메인 적분 ----------------
    def _update(self):
        now = self.get_clock().now()

        # heading은 IMU 상대yaw를 그대로 사용 (시작 시점 0 기준 = odom 상대 규약)
        self.yaw = self.imu_rel_yaw

        # 엔코더 증분 → 이동거리 ds
        if self._enc_last is None:
            self._enc_last = self.enc_ticks
        dticks = self.enc_ticks - self._enc_last
        self._enc_last = self.enc_ticks
        ds = dticks * self.m_per_tick

        # dead_reckoning.py와 동일 수식: 현재 heading 방향으로 ds만큼 전진
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        # ---- twist(body frame 속도) 계산 ----
        t = now.nanoseconds * 1e-9
        vx_body = 0.0
        wz = 0.0
        if self._last_pub_t is not None:
            dt = t - self._last_pub_t
            if dt > 1e-6:
                # 전진속도: 이번 이동거리(부호 포함)를 dt로 나눔.
                # ds가 전진(+)/후진(-) 부호를 그대로 가지므로 body x속도로 적절.
                vx_body = ds / dt
                # yaw rate
                dyaw = math.atan2(
                    math.sin(self.yaw - self._last_yaw),
                    math.cos(self.yaw - self._last_yaw))
                wz = dyaw / dt
        self._last_pub_t = t
        self._last_x = self.x
        self._last_y = self.y
        self._last_yaw = self.yaw

        stamp = now.to_msg()
        quat = yaw_to_quat(self.yaw)

        # ---- /odom 발행 ----
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        # twist는 child(base) 프레임 기준 = body frame
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.angular.z = wz
        # 공분산: 바퀴 odom은 드리프트가 있으니 적당히 큰 값(대각).
        #   SLAM 스캔매칭이 map→odom으로 교정하므로 과신 안 하게 둔다.
        odom.pose.covariance[0] = 0.05      # x
        odom.pose.covariance[7] = 0.05      # y
        odom.pose.covariance[35] = 0.10     # yaw
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        # ---- TF: odom → base_footprint ----
        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = quat
            self.tf_bc.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdomNode()
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
