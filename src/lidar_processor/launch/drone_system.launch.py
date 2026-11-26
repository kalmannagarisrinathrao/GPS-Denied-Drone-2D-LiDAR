import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GZ_SCAN_SUFFIX = (
    '/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan'
)


def generate_launch_description():

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz for clicking goals'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='walls',
        description='Gazebo world name (the PX4_GZ_WORLD you started)'
    )

    gz_scan_topic = ['/world/', LaunchConfiguration('world'), GZ_SCAN_SUFFIX]

    gz_scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_scan_bridge',
        arguments=[
            gz_scan_topic
            + ['@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        ],
        remappings=[(gz_scan_topic, '/scan')],
        output='screen'
    )

    map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    lidar_node = Node(
        package='lidar_processor',
        executable='lidar_node',
        name='lidar_processor',
        output='screen',
        emulate_tty=True
    )

    px4_bridge_node = Node(
        package='lidar_processor',
        executable='px4_obstacle_bridge',
        name='px4_obstacle_bridge',
        output='screen',
        emulate_tty=True
    )

    avoidance_controller_node = Node(
        package='lidar_processor',
        executable='avoidance_controller',
        name='avoidance_controller',
        output='screen',
        emulate_tty=True
    )

    goto_node = Node(
        package='lidar_processor',
        executable='goto_node',
        name='goto_node',
        output='screen',
        emulate_tty=True
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('lidar_processor'),
            'rviz', 'goto.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        world_arg,
        gz_scan_bridge,
        map_frame,
        lidar_node,
        px4_bridge_node,
        avoidance_controller_node,
        goto_node,
        rviz
    ])
