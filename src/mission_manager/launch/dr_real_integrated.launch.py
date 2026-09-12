#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    launch_dir = os.path.join(share, "launch")

    start_segment = LaunchConfiguration("start_segment")
    auto_start = LaunchConfiguration("auto_start")

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "dr_real_segmented_follower.launch.py",
            )
        ),
        launch_arguments={
            "start_segment": start_segment,
            "auto_start": auto_start,
            "end_branch_topic": "/dr/end_branch",
        }.items(),
    )

    parking_selector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "real_parking_slot_selector.launch.py",
            )
        )
    )

    end_branch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "end_branch_adapter.launch.py",
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_segment",
            default_value="START_A",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
        ),

        follower,
        parking_selector,
        end_branch,
    ])
