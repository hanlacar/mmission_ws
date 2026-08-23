#!/usr/bin/env python3
"""
twist_mux.launch.py — GPS 명령과 nav2 명령을 우선순위로 합쳐 /cmd_vel 발행.

입력:
  /cmd_vel_gps   (cmd_bridge, GPS 추종)   priority 10
  /cmd_vel_nav2  (nav2 velocity_smoother) priority 100  ← 재밍 시 이김
출력:
  /cmd_vel       (시뮬 AckermannSteering / 실차 모터가 구독)

전제:
  - nav2_params.yaml 의 velocity_smoother:
        cmd_vel_out_topic: cmd_vel_nav2   (기존 cmd_vel 에서 변경)
  - cmd_bridge(또는 GPS 명령원)가 /cmd_vel_gps 로 내도록 리매핑
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('mission_manager'),
        'config', 'twist_mux.yaml')

    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[cfg],
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),
    ])
