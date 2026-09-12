#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("mission_manager"))
    default_route = str(
        share / "routes" / "route_network_segmented_10.csv"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_path", default_value=default_route),
        DeclareLaunchArgument(
            "start_segment", default_value="START_A"),
        DeclareLaunchArgument(
            "auto_start", default_value="false"),
        DeclareLaunchArgument(
            "end_branch_topic", default_value=""),

        Node(
            package="mission_manager",
            executable="dr_real_segmented_follower",
            name="dr_real_segmented_follower",
            output="screen",
            parameters=[{
                "network_path": LaunchConfiguration("network_path"),
                "start_segment": LaunchConfiguration("start_segment"),
                "auto_start": LaunchConfiguration("auto_start"),
                "end_branch_topic": LaunchConfiguration("end_branch_topic"),

                "odom_topic": "/odom",
                "t_slot_topic": "/t_parking/selected_slot",
                "v_slot_topic": "/parallel_parking/selected_slot",
                "intersection_go_topic": "/mission/intersection_go",

                "gps_drive_topic": "/gps_drive",
                "gps_wheel_topic": "/gps_wheel",
                "lidar_drive_topic": "/lidar_drive",
                "lidar_wheel_topic": "/lidar_wheel",
                "drive_mode_topic": "/drive_mode",

                "wheelbase_m": 0.73,
                "max_steer_deg": 22.0,
                "steering_sign": -1,
                "intersection_wait_sec": 3.0,
                "end_wait_sec": 5.0,
                "odom_timeout_s": 0.5,
                "off_route_stop_m": 2.0,
                "transition_gap_stop_m": 1.05,
            }],
        ),
    ])
