from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share=Path(get_package_share_directory('mission_manager'))
    route=LaunchConfiguration('route_csv'); use_sim=LaunchConfiguration('use_sim_odom')
    auto=LaunchConfiguration('auto_start'); open_rviz=LaunchConfiguration('open_rviz')
    return LaunchDescription([
        DeclareLaunchArgument('route_csv',description='DR CSV path'),
        DeclareLaunchArgument('use_sim_odom',default_value='true'),
        DeclareLaunchArgument('auto_start',default_value='false'),
        DeclareLaunchArgument('open_rviz',default_value='true'),
        Node(package='mission_manager',executable='dr_odom_sim',name='dr_odom_sim',output='screen',condition=IfCondition(use_sim),
             parameters=[{'odom_topic':'/odom','drive_topic':'/gps_drive','wheel_topic':'/gps_wheel','wheelbase_m':0.73,'max_steer_deg':22.0}]),
        Node(package='mission_manager',executable='dr_route_follower',name='dr_route_follower',output='screen',
             parameters=[{'route_path':route,'odom_topic':'/odom','drive_topic':'/gps_drive','wheel_topic':'/gps_wheel',
                          'status_topic':'/dr_navigation/status','wheelbase_m':0.73,'max_steer_deg':22.0,
                          'align_route_to_start':True,'auto_start':ParameterValue(auto,value_type=bool)}]),
        Node(package='mission_manager',executable='dr_route_visualizer',name='dr_route_visualizer',output='screen',
             parameters=[{'route_path':route,'odom_topic':'/odom','status_topic':'/dr_navigation/status','frame_id':'odom','align_route_to_start':True}]),
        Node(package='rviz2',executable='rviz2',name='dr_route_rviz',output='screen',condition=IfCondition(open_rviz),
             arguments=['-d',str(share/'rviz'/'dr_route_live.rviz')]),
    ])
