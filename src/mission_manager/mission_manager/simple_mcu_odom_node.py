#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import TransformBroadcaster


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class SimpleMcuOdom(Node):
    """
    Real T870 lifted-test odometry.

    IMPORTANT:
    - /mcu/encoder is an A-only monotonic counter.
    - Distance scale uses the team's measured counts_per_meter=797.0.
    - Encoder is on the steering/front axle, so rear-axle Ackermann distance
      uses d_rear = d_front * cos(steer).
    """

    def __init__(self):
        super().__init__('simple_mcu_odom')

        params = {
            'encoder_topic': '/mcu/encoder',
            'steer_topic': '/mcu/steer_deg',
            'steer_command_topic': '/mcu/applied_wheel',
            'imu_yaw_topic': '/imu/relative_yaw_deg',
            'drive_topic': '/mcu/applied_drive',
            'stop_topic': '/mcu/stop_active',
            'odom_topic': '/odom',
            'frame_id': 'odom',
            'child_frame_id': 'base_link',

            'wheelbase_m': 0.73,
            'counts_per_meter': 797.0,
            'odom_steer_compensation': True,
            'max_steer_deg': 22.0,

            # Heading source health / fallback
            'imu_fallback_enabled': True,
            'steer_timeout_s': 0.35,
            'imu_timeout_s': 0.35,
            'steer_mismatch_detection_enabled': True,
            'steer_mismatch_threshold_deg': 8.0,
            'steer_mismatch_hold_s': 0.50,

            'publish_rate_hz': 30.0,
            'max_encoder_jump': 100000,
        }
        for k, v in params.items():
            self.declare_parameter(k, v)

        gp = lambda n: self.get_parameter(n).value

        self.encoder_topic = str(gp('encoder_topic'))
        self.steer_topic = str(gp('steer_topic'))
        self.steer_command_topic = str(gp('steer_command_topic'))
        self.imu_yaw_topic = str(gp('imu_yaw_topic'))
        self.drive_topic = str(gp('drive_topic'))
        self.stop_topic = str(gp('stop_topic'))
        self.odom_topic = str(gp('odom_topic'))
        self.frame_id = str(gp('frame_id'))
        self.child_frame_id = str(gp('child_frame_id'))

        self.L = max(0.01, float(gp('wheelbase_m')))
        self.cpm = max(1.0, float(gp('counts_per_meter')))
        self.m_per_count = 1.0 / self.cpm
        self.odom_steer_comp = bool(gp('odom_steer_compensation'))
        self.max_steer = abs(float(gp('max_steer_deg')))
        self.imu_fallback_enabled = bool(gp('imu_fallback_enabled'))
        self.steer_timeout = max(0.05, float(gp('steer_timeout_s')))
        self.imu_timeout = max(0.05, float(gp('imu_timeout_s')))
        self.steer_mismatch_detection_enabled = bool(
            gp('steer_mismatch_detection_enabled'))
        self.steer_mismatch_threshold = abs(
            float(gp('steer_mismatch_threshold_deg')))
        self.steer_mismatch_hold = max(
            0.05, float(gp('steer_mismatch_hold_s')))
        self.rate = max(5.0, float(gp('publish_rate_hz')))
        self.max_jump = max(100, int(gp('max_encoder_jump')))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.steer_deg = 0.0
        self.steer_valid = False
        self.last_steer_time = None

        self.steer_command_deg = 0.0
        self.last_steer_command_time = None
        self.steer_mismatch_started = None

        self.imu_yaw_rad = None
        self.last_imu_time = None
        self.imu_yaw_offset = None
        self.yaw_source = 'NONE'
        self.last_source_log = None

        self.last_encoder = None
        self.last_encoder_time = None

        # A-only encoder has no direction. Direction comes from actual applied drive.
        self.last_nonzero_direction = 1.0
        self.drive_stage = 0.0
        self.stop_active = False

        self.linear_v = 0.0
        self.angular_v = 0.0

        self.pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(Int32, self.encoder_topic, self.on_encoder, 30)
        self.create_subscription(Float32, self.steer_topic, self.on_steer, 30)
        self.create_subscription(
            Int32, self.steer_command_topic, self.on_steer_command, 30)
        self.create_subscription(
            Float32, self.imu_yaw_topic, self.on_imu_yaw, 50)
        self.create_subscription(Float32, self.drive_topic, self.on_drive, 30)
        self.create_subscription(Bool, self.stop_topic, self.on_stop, 30)

        self.create_timer(1.0 / self.rate, self.publish_odom)

        self.get_logger().info(
            'real MCU odom ready: '
            f'wheelbase={self.L:.3f} m, '
            f'counts_per_meter={self.cpm:.1f}, '
            f'm/count={self.m_per_count:.9f}, '
            f'front->rear_comp={self.odom_steer_comp}, '
            f'IMU fallback={self.imu_fallback_enabled}'
        )

    def on_steer(self, msg: Float32):
        value = float(msg.data)
        now = time.monotonic()

        if not math.isfinite(value) or abs(value) > self.max_steer + 5.0:
            self.steer_valid = False
            return

        self.steer_deg = max(
            -self.max_steer,
            min(self.max_steer, value)
        )
        self.steer_valid = True
        self.last_steer_time = now

    def on_steer_command(self, msg: Int32):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.steer_command_deg = max(
            -self.max_steer,
            min(self.max_steer, value)
        )
        self.last_steer_command_time = time.monotonic()

    def on_imu_yaw(self, msg: Float32):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.imu_yaw_rad = math.radians(value)
        self.last_imu_time = time.monotonic()

    def on_drive(self, msg: Float32):
        self.drive_stage = float(msg.data)
        if self.drive_stage > 0.1:
            self.last_nonzero_direction = 1.0
        elif self.drive_stage < -0.1:
            self.last_nonzero_direction = -1.0

    def on_stop(self, msg: Bool):
        self.stop_active = bool(msg.data)

    def _steer_is_healthy(self, now):
        if (
            not self.steer_valid
            or self.last_steer_time is None
            or now - self.last_steer_time > self.steer_timeout
        ):
            self.steer_mismatch_started = None
            return False

        if not self.steer_mismatch_detection_enabled:
            self.steer_mismatch_started = None
            return True

        if (
            self.last_steer_command_time is None
            or now - self.last_steer_command_time > self.steer_timeout
        ):
            self.steer_mismatch_started = None
            return True

        error = abs(self.steer_command_deg - self.steer_deg)

        if error <= self.steer_mismatch_threshold:
            self.steer_mismatch_started = None
            return True

        if self.steer_mismatch_started is None:
            self.steer_mismatch_started = now
            return True

        return (
            now - self.steer_mismatch_started
            < self.steer_mismatch_hold
        )

    def _imu_is_healthy(self, now):
        return (
            self.imu_fallback_enabled
            and self.imu_yaw_rad is not None
            and self.last_imu_time is not None
            and now - self.last_imu_time <= self.imu_timeout
        )

    def _select_yaw_source(self, now):
        steer_ok = self._steer_is_healthy(now)
        imu_ok = self._imu_is_healthy(now)

        if steer_ok:
            source = 'STEER'
        elif imu_ok:
            source = 'IMU'
        else:
            source = 'NONE'

        if source != self.yaw_source:
            if source == 'IMU':
                # IMU relative yaw zero-point를 현재 odom yaw에 연속적으로 정렬.
                self.imu_yaw_offset = norm_angle(
                    self.yaw - float(self.imu_yaw_rad)
                )
            self.yaw_source = source

            if source != self.last_source_log:
                if source == 'IMU':
                    self.get_logger().warning(
                        'steering sensor unhealthy -> IMU yaw fallback ACTIVE'
                    )
                elif source == 'STEER':
                    self.get_logger().info(
                        'steering sensor healthy -> steering yaw source ACTIVE'
                    )
                else:
                    self.get_logger().error(
                        'no valid steering or IMU heading source -> odom halted'
                    )
                self.last_source_log = source

        return source

    def on_encoder(self, msg: Int32):
        now = time.monotonic()
        enc = int(msg.data)

        if self.last_encoder is None:
            self.last_encoder = enc
            self.last_encoder_time = now
            return

        delta = enc - self.last_encoder
        dt = now - float(self.last_encoder_time or now)

        self.last_encoder = enc
        self.last_encoder_time = now

        # A-only counter is monotonic. Negative/huge changes mean reboot/reset/wrap.
        if delta < 0 or delta > self.max_jump:
            self.linear_v = 0.0
            self.angular_v = 0.0
            self.get_logger().warn(f'encoder rebase: delta={delta}')
            return

        if delta == 0:
            self.linear_v = 0.0
            self.angular_v = 0.0
            return

        direction = self.last_nonzero_direction
        source = self._select_yaw_source(now)

        if source == 'NONE':
            self.linear_v = 0.0
            self.angular_v = 0.0
            return

        # Team measured scale: 797 count / meter at the encoder/front axle.
        d_front = (float(delta) / self.cpm) * direction

        if source == 'STEER':
            steer = math.radians(self.steer_deg)

            # Encoder is on the front/steering axle.
            ds = (
                d_front * math.cos(steer)
                if self.odom_steer_comp
                else d_front
            )
            dtheta = ds * math.tan(steer) / self.L
            next_yaw = norm_angle(self.yaw + dtheta)

        else:
            # Steering sensor is unusable.  Do not use its angle for distance
            # compensation or heading.  Encoder gives signed travel distance,
            # and fresh IMU relative yaw gives heading.
            ds = d_front

            if self.imu_yaw_offset is None:
                self.imu_yaw_offset = norm_angle(
                    self.yaw - float(self.imu_yaw_rad)
                )

            next_yaw = norm_angle(
                float(self.imu_yaw_rad) + self.imu_yaw_offset
            )
            dtheta = norm_angle(next_yaw - self.yaw)

        # Mid-heading integration keeps x/y continuous for both sources.
        mid_yaw = norm_angle(self.yaw + 0.5 * dtheta)
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw = next_yaw

        if dt > 1.0e-3:
            self.linear_v = ds / dt
            self.angular_v = dtheta / dt

    def publish_odom(self):
        now = time.monotonic()
        source = self._select_yaw_source(now)

        # 둘 다 죽으면 /odom을 계속 새 timestamp로 내보내지 않는다.
        # downstream odom watchdog가 timeout으로 정지할 수 있게 한다.
        if source == 'NONE':
            return

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.linear_v
        msg.twist.twist.angular.z = self.angular_v
        self.pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMcuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
