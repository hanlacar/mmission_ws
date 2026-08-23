"""
통합 시뮬레이션 런치.
sim(가짜센서+차량) + mission_manager(두뇌) + monitor(시각화)를 한번에 띄운다.

실행:
  ros2 launch mission_sim sim_test.launch.py

실차 전환 시:
  - sim 노드만 빼고, 실제 rtk_node / imu_manager / 엔코더노드 / camera로 교체
  - mission_manager와 monitor는 그대로 사용
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_cfg = os.path.join(
        get_package_share_directory('mission_sim'), 'config', 'sim.yaml')
    mm_cfg = os.path.join(
        get_package_share_directory('mission_manager'), 'config', 'mission.yaml')

    return LaunchDescription([
        Node(package='mission_sim', executable='sim', name='mission_sim',
             output='screen', parameters=[sim_cfg]),
        Node(package='mission_manager', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[mm_cfg]),
        Node(package='mission_sim', executable='sim_monitor',
             name='sim_monitor', output='screen',
             parameters=[{'route_csv': mm_cfg and ''}]),
    ])
