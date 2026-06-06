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
        'rudra_v4_hardware.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Full path to rudra_v4_hardware.yaml',
    )

    node = Node(
        package='rudra_base_bridge',
        executable='ps2_uno_to_teensy',
        name='ps2_uno_to_teensy',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
