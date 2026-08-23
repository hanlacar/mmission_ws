#!/usr/bin/env python3
"""
path_recorder_sim_node.py

시뮬레이션 차량의 주행 경로를 x, y, dir 형태로 CSV에 저장한다.

구독:
  /odom
  /cmd_vel

CSV:
  x,y,dir

dir:
  1  = 전진
  -1 = 후진
"""

import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PathRecorderSim(Node):

    def __init__(self):
        super().__init__('path_recorder_sim')

        self.declare_parameter('output_path', '')
        self.declare_parameter('min_dist_m', 0.10)
        self.declare_parameter('save_period_s', 2.0)
        self.declare_parameter('dir_deadband', 0.02)

        self.output_path = str(
            self.get_parameter('output_path').value
        ).strip()

        self.min_dist = float(
            self.get_parameter('min_dist_m').value
        )

        save_period = float(
            self.get_parameter('save_period_s').value
        )

        self.dir_deadband = float(
            self.get_parameter('dir_deadband').value
        )

        if not self.output_path:
            self.get_logger().error(
                'output_path가 지정되지 않았습니다.\n'
                '실행 예:\n'
                'ros2 run mission_manager path_recorder_sim '
                '--ros-args '
                '-p output_path:=$HOME/mmission_ws/routes/route.csv'
            )
            raise RuntimeError('output_path is empty')

        self.output_path = os.path.abspath(
            os.path.expanduser(self.output_path)
        )

        output_dir = os.path.dirname(self.output_path)

        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        self.points = []
        self._last = None
        self._cur_dir = 1

        self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom,
            10,
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._on_cmd,
            10,
        )

        self.create_timer(
            save_period,
            self._flush,
        )

        self.get_logger().info(
            f'경로 저장 시작 → {self.output_path} '
            f'(min_dist={self.min_dist:.2f} m)'
        )

    def _on_cmd(self, msg: Twist):
        vx = msg.linear.x

        if vx > self.dir_deadband:
            self._cur_dir = 1

        elif vx < -self.dir_deadband:
            self._cur_dir = -1

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last is None:
            self._record(x, y)
            return

        distance = math.hypot(
            x - self._last[0],
            y - self._last[1],
        )

        if distance >= self.min_dist:
            self._record(x, y)

    def _record(self, x, y):
        self.points.append(
            (
                float(x),
                float(y),
                int(self._cur_dir),
            )
        )

        self._last = (x, y)

    def _flush(self):
        if not self.points:
            return

        try:
            with open(
                self.output_path,
                'w',
                encoding='utf-8',
            ) as file:
                file.write('x,y,dir\n')

                for x, y, direction in self.points:
                    file.write(
                        f'{x:.4f},{y:.4f},{direction}\n'
                    )

        except OSError as exc:
            self.get_logger().error(
                f'경로 저장 실패: {exc}'
            )

    def destroy_node(self):
        self._flush()

        switches = sum(
            1
            for i in range(1, len(self.points))
            if self.points[i][2] != self.points[i - 1][2]
        )

        self.get_logger().info(
            f'최종 저장: {len(self.points)} waypoint, '
            f'방향전환 {switches}회 → {self.output_path}'
        )

        super().destroy_node()


def main():
    rclpy.init()

    node = None

    try:
        node = PathRecorderSim()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError as exc:
        print(exc)

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()