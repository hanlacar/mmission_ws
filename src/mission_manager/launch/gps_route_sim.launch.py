"""Fake GPS + production follower. IMU must come from imu_ws or a test publisher."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    config = os.path.join(share, "config", "gps_route.yaml")
    route = os.path.join(share, "routes", "test", "04_forward_reverse_cusp.csv")
    return LaunchDescription([
        Node(package="mission_manager", executable="gps_route_follower",
             parameters=[config, {"route_path": route}], output="screen"),
        Node(package="mission_manager", executable="fake_gps_sim",
             parameters=[{"route_csv": route}], output="screen"),
    ])
