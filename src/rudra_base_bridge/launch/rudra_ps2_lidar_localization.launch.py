import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
    default_ekf_config = os.path.join(
        package_share,
        'config',
        'localization_ekf.yaml',
    )

    config_arg = DeclareLaunchArgument('config', default_value=default_robot_config)
    lidar_config_arg = DeclareLaunchArgument(
        'lidar_config',
        default_value=default_lidar_config,
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
    )
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='laser')
    ekf_config_arg = DeclareLaunchArgument('ekf_config', default_value=default_ekf_config)
    enable_dcdc_monitor_arg = DeclareLaunchArgument(
        'enable_dcdc_monitor',
        default_value='false',
        description='Start Mini-Box DCDC-USB monitor for the NUC power rail',
    )

    ps2_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'rudra_ps2_lidar.launch.py')
        ),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'lidar_config': LaunchConfiguration('lidar_config'),
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': LaunchConfiguration('frame_id'),
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'ekf_config': LaunchConfiguration('ekf_config'),
        }.items(),
    )

    dcdc_monitor_node = Node(
        package='rudra_base_bridge',
        executable='dcdc_usb_monitor',
        name='dcdc_usb_monitor',
        output='screen',
        parameters=[LaunchConfiguration('config')],
        condition=IfCondition(LaunchConfiguration('enable_dcdc_monitor')),
    )

    return LaunchDescription([
        config_arg,
        lidar_config_arg,
        serial_port_arg,
        frame_id_arg,
        ekf_config_arg,
        enable_dcdc_monitor_arg,
        ps2_lidar_launch,
        localization_launch,
        dcdc_monitor_node,
    ])
