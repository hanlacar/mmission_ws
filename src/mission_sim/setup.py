import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'mission_sim'


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
                'launch',
            ),
            glob('launch/*.py'),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml')
            + glob('config/*.rviz'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='team',
    maintainer_email='you@example.com',

    description='Mission simulation package',

    license='MIT',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'sim = mission_sim.sim_node:main',
            'sim_live = mission_sim.sim_live_node:main',
            'rviz_viz = mission_sim.rviz_viz_node:main',
        ],
    },
)