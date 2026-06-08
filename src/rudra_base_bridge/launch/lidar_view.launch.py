import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rudra_base_bridge')
    install_config = os.path.join(
        package_share,
        'config',
        'lidar_obstacle_view.rviz',
    )
    source_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_share)))),
        'src',
        'rudra_base_bridge',
        'config',
        'lidar_obstacle_view.rviz',
    )
    config_path = source_config if os.path.exists(source_config) else install_config

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rudra_lidar_view',
        output='screen',
        arguments=['-d', config_path],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rudra_laser_static_tf_view',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    return LaunchDescription([static_tf, rviz])
