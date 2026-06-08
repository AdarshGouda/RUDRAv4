import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rudra_base_bridge')
    default_config = os.path.join(
        package_share,
        'config',
        'localization_ekf.yaml',
    )

    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=default_config,
        description='Full path to the EKF localization parameter YAML',
    )
    filtered_odom_topic_arg = DeclareLaunchArgument(
        'filtered_odom_topic',
        default_value='/odometry/filtered',
        description='Odometry topic used for TF broadcasting',
    )

    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rudra_imu_static_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
    )

    odom_tf_node = Node(
        package='rudra_base_bridge',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'odom_topic': LaunchConfiguration('filtered_odom_topic')}],
    )

    return LaunchDescription([
        ekf_config_arg,
        filtered_odom_topic_arg,
        imu_static_tf,
        ekf_node,
        odom_tf_node,
    ])
