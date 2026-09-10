from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("mission_manager"))
    rviz_cfg = share / "rviz" / "dr_branch_test.rviz"

    network_csv = LaunchConfiguration("network_csv")
    use_sim_odom = LaunchConfiguration("use_sim_odom")
    auto_start = LaunchConfiguration("auto_start")
    open_rviz = LaunchConfiguration("open_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_csv",
            description="segmented route network CSV",
        ),
        DeclareLaunchArgument(
            "use_sim_odom",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "open_rviz",
            default_value="true",
        ),

        # Existing hardware-free simulator added in the previous RViz package.
        Node(
            package="mission_manager",
            executable="dr_odom_sim",
            name="dr_odom_sim",
            output="screen",
            condition=IfCondition(use_sim_odom),
            parameters=[{
                "odom_topic": "/odom",
                "drive_topic": "/gps_drive",
                "wheel_topic": "/gps_wheel",
                "wheelbase_m": 0.73,
                "max_steer_deg": 22.0,
                "rate_hz": 30.0,
            }],
        ),

        # Give RViz a valid odom TF frame during hardware-free testing.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="dr_sim_map_to_odom",
            output="screen",
            condition=IfCondition(use_sim_odom),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
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
            package="rviz2",
            executable="rviz2",
            name="dr_branch_rviz",
            output="screen",
            condition=IfCondition(open_rviz),
            arguments=["-d", str(rviz_cfg)],
        ),
    ])
