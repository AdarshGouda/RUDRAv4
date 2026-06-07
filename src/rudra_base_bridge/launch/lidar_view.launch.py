import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('rudra_base_bridge'),
        'config',
        'lidar_obstacle_view.rviz',
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='RViz config for viewing the LiDAR and obstacle guard',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rudra_lidar_view',
        output='screen',
        arguments=['-d', LaunchConfiguration('config')],
    )

    return LaunchDescription([
        config_arg,
        rviz,
    ])
