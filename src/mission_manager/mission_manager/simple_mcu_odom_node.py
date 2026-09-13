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
            'drive_topic': '/mcu/applied_drive',
            'stop_topic': '/mcu/stop_active',
            'odom_topic': '/odom',
            'frame_id': 'odom',
            'child_frame_id': 'base_link',

            'wheelbase_m': 0.73,
            'counts_per_meter': 797.0,
            'odom_steer_compensation': True,
            'max_steer_deg': 22.0,

            'publish_rate_hz': 30.0,
            'max_encoder_jump': 100000,
        }
        for k, v in params.items():
            self.declare_parameter(k, v)

        gp = lambda n: self.get_parameter(n).value

        self.encoder_topic = str(gp('encoder_topic'))
        self.steer_topic = str(gp('steer_topic'))
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
        self.rate = max(5.0, float(gp('publish_rate_hz')))
        self.max_jump = max(100, int(gp('max_encoder_jump')))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.steer_deg = 0.0
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
        self.create_subscription(Float32, self.drive_topic, self.on_drive, 30)
        self.create_subscription(Bool, self.stop_topic, self.on_stop, 30)

        self.create_timer(1.0 / self.rate, self.publish_odom)

        self.get_logger().info(
            'real MCU odom ready: '
            f'wheelbase={self.L:.3f} m, '
            f'counts_per_meter={self.cpm:.1f}, '
            f'm/count={self.m_per_count:.9f}, '
            f'front->rear_comp={self.odom_steer_comp}'
        )

    def on_steer(self, msg: Float32):
        self.steer_deg = max(
            -self.max_steer,
            min(self.max_steer, float(msg.data))
        )

    def on_drive(self, msg: Float32):
        self.drive_stage = float(msg.data)
        if self.drive_stage > 0.1:
            self.last_nonzero_direction = 1.0
        elif self.drive_stage < -0.1:
            self.last_nonzero_direction = -1.0

    def on_stop(self, msg: Bool):
        self.stop_active = bool(msg.data)

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
        steer = math.radians(self.steer_deg)

        # Team measured scale: 797 count / meter at the encoder/front axle.
        d_front = (float(delta) / self.cpm) * direction

        # Encoder is on the front/steering axle.
        ds = d_front * math.cos(steer) if self.odom_steer_comp else d_front

        dtheta = ds * math.tan(steer) / self.L

        # Exact constant-curvature Ackermann integration.
        if abs(dtheta) > 1.0e-9:
            radius = ds / dtheta
            self.x += radius * (math.sin(self.yaw + dtheta) - math.sin(self.yaw))
            self.y -= radius * (math.cos(self.yaw + dtheta) - math.cos(self.yaw))
        else:
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)

        self.yaw = norm_angle(self.yaw + dtheta)

        if dt > 1.0e-3:
            self.linear_v = ds / dt
            self.angular_v = dtheta / dt

    def publish_odom(self):
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
