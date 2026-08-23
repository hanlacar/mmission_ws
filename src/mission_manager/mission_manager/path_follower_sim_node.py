#!/usr/bin/env python3
"""
path_follower_sim_node.py

전진/후진 방향이 포함된 시뮬레이션 경로를 Pure Pursuit로 추종한다.

발행:
  /cmd_vel
  /t_wheel
  /t_drive

구독:
  /odom

CSV:
  x,y,dir

dir:
  1  = 전진
  -1 = 후진
"""

import csv
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32, String


def yaw_from_quat(q):
    sin_yaw = 2.0 * (
        q.w * q.z +
        q.x * q.y
    )

    cos_yaw = 1.0 - 2.0 * (
        q.y * q.y +
        q.z * q.z
    )

    return math.atan2(
        sin_yaw,
        cos_yaw,
    )


class PathFollowerSim(Node):

    def __init__(self):
        super().__init__('path_follower_sim')

        self.declare_parameter('route_path', '')
        self.declare_parameter('lookahead_m', 0.6)
        self.declare_parameter('speed_mps', 0.5)
        self.declare_parameter('goal_tol_m', 0.25)
        self.declare_parameter('wheelbase_m', 0.30)
        self.declare_parameter('max_steer_deg', 27.0)
        self.declare_parameter('advance_tol_m', 0.35)

        self.route_path = str(
            self.get_parameter('route_path').value
        ).strip()

        if not self.route_path:
            self.get_logger().error(
                'route_path가 지정되지 않았습니다.\n'
                '실행 예:\n'
                'ros2 run mission_manager path_follower_sim '
                '--ros-args '
                '-p route_path:=$HOME/mmission_ws/routes/route.csv'
            )
            raise RuntimeError('route_path is empty')

        self.route_path = os.path.abspath(
            os.path.expanduser(self.route_path)
        )

        self.lookahead = float(
            self.get_parameter('lookahead_m').value
        )

        self.speed = float(
            self.get_parameter('speed_mps').value
        )

        self.goal_tol = float(
            self.get_parameter('goal_tol_m').value
        )

        self.wheelbase = float(
            self.get_parameter('wheelbase_m').value
        )

        self.max_steer = math.radians(
            float(
                self.get_parameter(
                    'max_steer_deg'
                ).value
            )
        )

        self.advance_tol = float(
            self.get_parameter(
                'advance_tol_m'
            ).value
        )

        self.path = self._load(
            self.route_path
        )

        if not self.path:
            raise RuntimeError(
                f'경로 로드 실패: {self.route_path}'
            )

        forward_count = sum(
            1
            for point in self.path
            if point[2] > 0
        )

        reverse_count = sum(
            1
            for point in self.path
            if point[2] < 0
        )

        self.get_logger().info(
            f'경로 로드: {len(self.path)}점 '
            f'(전진 {forward_count}, '
            f'후진 {reverse_count})'
        )

        self._cursor = 0
        self._last_dir = None
        self._finished = False

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.wheel_pub = self.create_publisher(
            Int32,
            '/t_wheel',
            10,
        )

        self.drive_pub = self.create_publisher(
            String,
            '/t_drive',
            10,
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom,
            10,
        )

    def _load(self, path):
        points = []

        try:
            with open(
                path,
                newline='',
                encoding='utf-8',
            ) as file:
                reader = csv.reader(file)

                next(reader, None)

                for row in reader:
                    if len(row) < 2:
                        continue

                    direction = (
                        int(row[2])
                        if len(row) >= 3
                        else 1
                    )

                    points.append(
                        (
                            float(row[0]),
                            float(row[1]),
                            direction,
                        )
                    )

        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f'CSV 로드 오류: {exc}'
            )

        return points

    def _publish_outputs(
        self,
        steer_rad,
        drive_mps,
    ):
        steer_deg = math.degrees(
            steer_rad
        )

        self.wheel_pub.publish(
            Int32(
                data=int(
                    round(steer_deg)
                )
            )
        )

        self.drive_pub.publish(
            String(
                data=f'{drive_mps:.2f}'
            )
        )

    def _on_odom(self, msg: Odometry):
        if self._finished:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        yaw = yaw_from_quat(
            msg.pose.pose.orientation
        )

        while self._cursor < len(self.path) - 1:
            px, py, _ = self.path[self._cursor]

            distance = math.hypot(
                px - x,
                py - y,
            )

            if distance <= self.advance_tol:
                self._cursor += 1
            else:
                break

        goal_x, goal_y, _ = self.path[-1]

        goal_distance = math.hypot(
            goal_x - x,
            goal_y - y,
        )

        if (
            self._cursor >= len(self.path) - 1
            and goal_distance <= self.goal_tol
        ):
            self._stop()
            self._publish_outputs(
                0.0,
                0.0,
            )

            self._finished = True

            self.get_logger().info(
                '최종 목표 도달 → 정지'
            )

            return

        current_direction = (
            self.path[self._cursor][2]
        )

        if current_direction != self._last_dir:
            self.get_logger().info(
                '[방향] '
                f'{"전진" if current_direction > 0 else "후진"} '
                f'구간 진입 '
                f'(idx={self._cursor})'
            )

            self._last_dir = (
                current_direction
            )

        target = None

        for i in range(
            self._cursor,
            len(self.path),
        ):
            px, py, _ = self.path[i]

            distance = math.hypot(
                px - x,
                py - y,
            )

            if distance >= self.lookahead:
                target = (
                    px,
                    py,
                )
                break

        if target is None:
            target = (
                self.path[-1][0],
                self.path[-1][1],
            )

        dx = target[0] - x
        dy = target[1] - y

        local_x = (
            math.cos(-yaw) * dx
            - math.sin(-yaw) * dy
        )

        local_y = (
            math.sin(-yaw) * dx
            + math.cos(-yaw) * dy
        )

        command = Twist()

        if current_direction >= 0:
            ld = math.hypot(
                local_x,
                local_y,
            )

            if ld < 1e-6:
                self._stop()
                self._publish_outputs(
                    0.0,
                    0.0,
                )
                return

            curvature = (
                2.0
                * local_y
                / (ld * ld)
            )

            steer = math.atan(
                self.wheelbase
                * curvature
            )

            steer = max(
                -self.max_steer,
                min(
                    self.max_steer,
                    steer,
                ),
            )

            command.linear.x = self.speed
            command.angular.z = (
                self.speed
                * curvature
            )

            drive = self.speed

        else:
            reverse_x = -local_x
            reverse_y = -local_y

            ld = math.hypot(
                reverse_x,
                reverse_y,
            )

            if ld < 1e-6:
                self._stop()
                self._publish_outputs(
                    0.0,
                    0.0,
                )
                return

            curvature = (
                2.0
                * reverse_y
                / (ld * ld)
            )

            steer = math.atan(
                self.wheelbase
                * curvature
            )

            steer = max(
                -self.max_steer,
                min(
                    self.max_steer,
                    steer,
                ),
            )

            command.linear.x = -self.speed
            command.angular.z = (
                self.speed
                * curvature
            )

            drive = -self.speed

        self.cmd_pub.publish(
            command
        )

        self._publish_outputs(
            steer,
            drive,
        )

    def _stop(self):
        self.cmd_pub.publish(
            Twist()
        )


def main():
    rclpy.init()

    node = None

    try:
        node = PathFollowerSim()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError as exc:
        print(exc)

    finally:
        if node is not None:
            node._stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()