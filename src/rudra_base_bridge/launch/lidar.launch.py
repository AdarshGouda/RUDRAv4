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
        'ydlidar_g2b.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Full path to the YDLIDAR parameter YAML',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB serial port for the YDLIDAR adapter',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame id used in the LaserScan header',
    )

    node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
            },
        ],
    )

    return LaunchDescription([
        config_arg,
        serial_port_arg,
        frame_id_arg,
        node,
    ])
