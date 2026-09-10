import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'mission_manager'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ],
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ],
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml'),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.py'),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'rviz',
            ),
            glob('rviz/*.rviz'),
        ),

        # routes 바로 아래 CSV/YAML
        (
            os.path.join(
                'share',
                package_name,
                'routes',
            ),
            glob('routes/*.csv')
            + glob('routes/*.yaml'),
        ),

        # 테스트 경로
        (
            os.path.join(
                'share',
                package_name,
                'routes',
                'test',
            ),
            glob('routes/test/*.csv')
            + glob('routes/test/*.yaml'),
        ),

        # 실제 교차로 경로
        (
            os.path.join(
                'share',
                package_name,
                'routes',
                'intersection',
            ),
            glob('routes/intersection/*'),
        ),

        # 테스트 교차로 경로
        (
            os.path.join(
                'share',
                package_name,
                'routes',
                'test',
                'intersection',
            ),
            glob('routes/test/intersection/*'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='team',
    maintainer_email='you@example.com',

    description='최상위 미션 상태머신',

    license='MIT',

    extras_require={
        'test': [
            'pytest'
        ]
    },

    entry_points={
        'console_scripts': [
            'odom_route_recorder = mission_manager.odom_route_recorder_node:main',
            'dr_route_follower = mission_manager.dr_route_follower_node:main',
            't_parking_lidar_selector = mission_manager.t_parking_lidar_selector_node:main',
            'fake_rear_lidar = mission_manager.fake_rear_lidar_node:main',
            'dr_segmented_branch_follower = mission_manager.dr_segmented_branch_follower_node:main',
            'dr_route_visualizer = mission_manager.dr_route_visualizer_node:main',
            'dr_odom_sim = mission_manager.dr_odom_sim_node:main',
            'route_recorder = mission_manager.route_recorder_node:main',
            'route_merge = mission_manager.route_merge:main',
            'mission_sequencer = mission_manager.mission_sequencer_node:main',
            'gps_driver = mission_manager.gps_driver_node:main',
            'encoder_driver = mission_manager.encoder_driver_node:main',
            'imu_driver = mission_manager.imu_driver_node:main',
            'camera_driver = mission_manager.camera_driver_node:main',
            'wheel_odom = mission_manager.wheel_odom_node:main',
            'gps_dr_supervisor = mission_manager.gps_dr_supervisor_node:main',
            'path_recorder_sim = mission_manager.path_recorder_sim_node:main',
            'path_follower_sim = mission_manager.path_follower_sim_node:main',
            'fake_gps_sim = mission_manager.fake_gps_sim_node:main',
            'gps_route_follower = mission_manager.gps_route_follower_node:main',
            'route_accuracy_monitor = mission_manager.route_accuracy_monitor_node:main',
            'gps_route_visualizer = mission_manager.gps_route_visualizer_node:main',
            'gps_test_imu_publisher = mission_manager.test_imu_publisher_node:main',
            'intersection_calib = mission_manager.intersection_calib_node:main',
            'analyze_entry_dirs = mission_manager.analyze_entry_dirs:main',
        ],
    },
)
