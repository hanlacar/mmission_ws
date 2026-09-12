#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    config = os.path.join(
        share, "config", "real_parking_slot_selector.yaml")

    return LaunchDescription([
        Node(
            package="mission_manager",
            executable="real_parking_slot_selector",
            name="real_parking_slot_selector",
            output="screen",
            parameters=[config],
        ),
    ])
