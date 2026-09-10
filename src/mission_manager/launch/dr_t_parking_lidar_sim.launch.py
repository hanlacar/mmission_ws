from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("mission_manager"))
    rviz_cfg = share / "rviz" / "t_parking_lidar_sim.rviz"

    network_csv = LaunchConfiguration("network_csv")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_csv",
            default_value=str(
                Path.home() / "mmission_ws" / "routes" / "T_ONLY_segmented_1.csv"
            ),
        ),
        DeclareLaunchArgument("auto_start", default_value="false"),

        Node(
            package="mission_manager",
            executable="dr_odom_sim",
            name="dr_odom_sim",
            output="screen",
            parameters=[{
                "odom_topic": "/odom",
                "drive_topic": "/gps_drive",
                "wheel_topic": "/gps_wheel",
                "wheelbase_m": 0.73,
                "max_steer_deg": 22.0,
                "rate_hz": 30.0,
            }],
        ),

        Node(
            package="mission_manager",
            executable="dr_segmented_branch_follower",
            name="dr_segmented_branch_follower",
            output="screen",
            parameters=[{
                "network_path": network_csv,
                "odom_topic": "/odom",
                "drive_topic": "/gps_drive",
                "wheel_topic": "/gps_wheel",
                "forward_segment": "T_foword",
                "branch_a_segment": "T_A",
                "branch_b_segment": "T_B",
                "wheelbase_m": 0.73,
                "max_steer_deg": 22.0,
                "align_route_to_start": True,
                "auto_start": ParameterValue(auto_start, value_type=bool),
            }],
        ),

        Node(
            package="mission_manager",
            executable="fake_rear_lidar",
            name="fake_rear_lidar",
            output="screen",
            parameters=[{
                "scan_topic": "/scan_rear",
                "odom_topic": "/odom",
                "default_scenario": "BOTH",
            }],
        ),

        Node(
            package="mission_manager",
            executable="t_parking_lidar_selector",
            name="t_parking_lidar_selector",
            output="screen",
            parameters=[{
                "scan_topic": "/scan_rear",
                "segment_topic": "/dr_navigation/current_segment",
                "branch_select_topic": "/dr_branch/select",
                "active_segment": "T_foword",
                "branch_a": "T_A",
                "branch_b": "T_B",
                "default_free_branch": "T_A",
                "confirmation_frames": 3,
                "occupied_min_points": 3,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="t_parking_lidar_rviz",
            output="screen",
            arguments=["-d", str(rviz_cfg)],
        ),
    ])
