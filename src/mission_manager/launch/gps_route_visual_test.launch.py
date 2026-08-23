"""Visual max-steering tests using the unchanged production follower."""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    share = get_package_share_directory('mission_manager')
    selection = LaunchConfiguration('test_route').perform(context)
    choices = {
        'forward_circle': ('08_circle_forward_max27.csv', 90.0),
        'reverse_circle': ('09_circle_reverse_max27.csv', -90.0),
    }
    if selection not in choices:
        raise RuntimeError(
            f'Unknown test_route {selection!r}; choose forward_circle or reverse_circle')
    filename, heading = choices[selection]
    route = os.path.join(share, 'routes', 'test', filename)
    config = os.path.join(share, 'config', 'gps_route.yaml')
    rviz = os.path.join(share, 'rviz', 'gps_route.rviz')
    with open(config, encoding='utf-8') as stream:
        wheelbase = float(yaml.safe_load(stream)['gps_route_follower']['ros__parameters']['wheelbase_m'])
    return [
        Node(package='mission_manager', executable='gps_route_follower',
             parameters=[config, {
                 'route_path': route,
                 # 최대 조향각 검증에서 경로에 지정된 drive level을 유지합니다.
                 'steering_slowdown_deg': 28.0,
             }], output='screen'),
        Node(package='mission_manager', executable='fake_gps_sim',
             parameters=[{
                 'route_csv': route,
                 'start_heading_deg': heading,
                 'wheelbase_m': wheelbase,
                 'drive_level_1_mps': 0.15,
                 'drive_level_2_mps': 0.15,
             }], output='screen'),
        Node(package='mission_manager', executable='gps_test_imu_publisher',
             condition=IfCondition(LaunchConfiguration('use_test_imu')), output='screen'),
        Node(package='mission_manager', executable='gps_route_visualizer',
             parameters=[{'route_path': route, 'frame_id': 'map'}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_test_imu', default_value='false',
                              description='Publish test-only mission-facing IMU topics'),
        DeclareLaunchArgument('test_route', default_value='forward_circle',
                              description='forward_circle or reverse_circle'),
        OpaqueFunction(function=_launch_nodes),
    ])
