from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lidar_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srinath',
    maintainer_email='srinath@example.com',
    description='ROS 2 LiDAR obstacle detection and PX4 integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_processor.lidar_node:main',
            'px4_obstacle_bridge = lidar_processor.px4_obstacle_bridge:main',
            'avoidance_controller = lidar_processor.avoidance_controller:main',
            'goto_node = lidar_processor.goto_node:main',
        ],
    },
)
