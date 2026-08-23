import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sim = get_package_share_directory('mission_sim')
    pkg_manager = get_package_share_directory('mission_manager')

    sim_cfg = os.path.join(
        pkg_sim,
        'config',
        'sim_loop_test.yaml',
    )

    mission_cfg = os.path.join(
        pkg_manager,
        'config',
        'mission_sequencer.yaml',
    )

    route_csv = os.path.join(
        pkg_manager,
        'routes',
        'test',
        '07_circle_loop.csv',
    )

    return LaunchDescription([
        Node(
            package='mission_sim',
            executable='sim',
            name='mission_sim',
            output='screen',
            parameters=[sim_cfg],
        ),

        Node(
            package='mission_manager',
            executable='mission_sequencer',
            name='mission_sequencer',
            output='screen',
            parameters=[mission_cfg],
        ),

        Node(
            package='mission_sim',
            executable='sim_live',
            name='sim_live',
            output='screen',
            parameters=[{
                'route_csv': route_csv,
                'intersections': [
                    '0.0,0.0,2.5',
                ],
            }],
        ),
    ])