from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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

    return LaunchDescription([
        lidar_node,
        px4_bridge_node,
        avoidance_controller_node
    ])
