#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    config = os.path.join(
        share, "config", "end_branch_adapter.yaml")

    return LaunchDescription([
        Node(
            package="mission_manager",
            executable="end_branch_adapter",
            name="end_branch_adapter",
            output="screen",
            parameters=[config],
        ),
    ])
