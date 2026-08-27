from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    config = os.path.join(share, "config", "gps_route.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "route_csv",
            default_value="/home/ww/mmission_ws/routes/recorded_route.csv"),
        Node(package="mission_manager", executable="route_recorder",
             parameters=[config, {"out_csv": LaunchConfiguration("route_csv")}],
             output="screen"),
    ])
