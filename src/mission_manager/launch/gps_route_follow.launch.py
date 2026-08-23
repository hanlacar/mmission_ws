from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    config = os.path.join(share, "config", "gps_route.yaml")
    accuracy_config = os.path.join(share, "config", "route_accuracy.yaml")
    default_route = os.path.join(share, "routes", "test", "04_forward_reverse_cusp.csv")
    ix_dir = os.path.join(share, "routes", "intersection")
    return LaunchDescription([
        DeclareLaunchArgument("route_csv", default_value=default_route),
        DeclareLaunchArgument("enable_accuracy_monitor", default_value="false"),
        Node(package="mission_manager", executable="gps_route_follower",
             parameters=[config, {
                 "route_path": LaunchConfiguration("route_csv"),
                 "intersection.route_dir": ix_dir,
             }], output="screen"),
        Node(package="mission_manager", executable="route_accuracy_monitor",
             condition=IfCondition(LaunchConfiguration("enable_accuracy_monitor")),
             parameters=[accuracy_config, {
                 "route_csv": LaunchConfiguration("route_csv"),
             }], output="screen"),
    ])
