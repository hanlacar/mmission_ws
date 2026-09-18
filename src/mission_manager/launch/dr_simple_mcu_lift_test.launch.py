#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('mission_manager'))
    default_route = str(share / 'routes' / 'route_network_segmented_10.csv')
    installed_rviz = share / 'rviz' / 'dr_lifted_route_test.rviz'
    source_rviz = Path.home() / 'mmission_ws' / 'src' / 'mission_manager' / 'rviz' / 'dr_lifted_route_test.rviz'
    default_rviz = str(installed_rviz if installed_rviz.is_file() else source_rviz)
    pad_config = str(share / 'config' / 'pad_takeover.yaml')
    route = LaunchConfiguration('network_path')
    start = LaunchConfiguration('start_segment')
    return LaunchDescription([
        DeclareLaunchArgument('network_path', default_value=default_route),
        DeclareLaunchArgument('start_segment', default_value='START_A'),
        Node(package='mission_manager', executable='simple_mcu_odom', name='simple_mcu_odom', output='screen',
             parameters=[{
                 'wheelbase_m': 0.73,
                 'counts_per_meter': 797.0,
                 'odom_steer_compensation': True,
                 'max_steer_deg': 22.0,
                 'publish_rate_hz': 30.0,
             }]),
        Node(package='mission_manager', executable='dr_real_segmented_follower', name='dr_real_segmented_follower', output='screen',
             parameters=[{
                 'network_path': route, 'start_segment': start, 'auto_start': False, 'end_branch_topic': '',
                 'odom_topic': '/odom', 'gps_drive_topic': '/cmd_drive', 'gps_wheel_topic': '/cmd_wheel',
                 'lidar_drive_topic': '/lidar_drive', 'lidar_wheel_topic': '/lidar_wheel', 'drive_mode_topic': '/drive_mode',
                 'wheelbase_m': 0.73, 'max_steer_deg': 22.0, 'steering_sign': 1,
                 'intersection_wait_sec': 3.0, 'end_wait_sec': 5.0, 'odom_timeout_s': 0.5,
                 'off_route_stop_m': 2.0, 'transition_gap_stop_m': 1.05,
             }]),
        Node(package='mission_manager', executable='gamepad_takeover', name='t870_gamepad_takeover', output='screen', parameters=[pad_config]),
        Node(package='mission_manager', executable='simple_mcu_command_adapter', name='simple_mcu_command_adapter', output='screen', parameters=[pad_config]),
        Node(package='mission_manager', executable='intersection_timeout_bridge',
             name='intersection_timeout_bridge', output='screen'),
        Node(package='mission_manager', executable='dr_segmented_visualizer', name='dr_segmented_visualizer', output='screen',
             parameters=[{'network_path': route, 'start_segment': start, 'frame_id': 'odom'}]),
        Node(package='rviz2', executable='rviz2', name='dr_lifted_rviz', output='screen', arguments=['-d', default_rviz]),
    ])
