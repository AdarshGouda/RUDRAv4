import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rudra_base_bridge')
    default_robot_config = os.path.join(
        package_share,
        'config',
        'rudra_v4_hardware.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_robot_config,
        description='Full path to rudra_v4_hardware.yaml',
    )

    dcdc_node = Node(
        package='rudra_base_bridge',
        executable='dcdc_usb_monitor',
        name='dcdc_usb_monitor',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([
        config_arg,
        dcdc_node,
    ])
