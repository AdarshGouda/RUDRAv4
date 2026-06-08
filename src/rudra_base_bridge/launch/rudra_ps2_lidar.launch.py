import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_serial_port = (
        '/dev/serial/by-id/'
        'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    )

    package_share = get_package_share_directory('rudra_base_bridge')

    default_robot_config = os.path.join(
        package_share,
        'config',
        'rudra_v4_hardware.yaml',
    )
    default_lidar_config = os.path.join(
        package_share,
        'config',
        'ydlidar_g2b.yaml',
    )

    robot_config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_robot_config,
        description='Full path to rudra_v4_hardware.yaml',
    )
    lidar_config_arg = DeclareLaunchArgument(
        'lidar_config',
        default_value=default_lidar_config,
        description='Full path to the YDLIDAR parameter YAML',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial_port,
        description='USB serial port for the YDLIDAR adapter',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame id used in the LaserScan header',
    )

    ps2_node = Node(
        package='rudra_base_bridge',
        executable='ps2_uno_to_teensy',
        name='ps2_uno_to_teensy',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('lidar_config'),
            {
                'port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
            },
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rudra_laser_static_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('frame_id'),
        ],
    )

    return LaunchDescription([
        robot_config_arg,
        lidar_config_arg,
        serial_port_arg,
        frame_id_arg,
        ps2_node,
        lidar_node,
        static_tf,
    ])
