"""Full RUDRAv4 robot bringup: base, LiDAR, localization, and voice."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _workspace_root(package_share: str) -> str:
    env_workspace = os.environ.get('RUDRA_WS', '')
    if os.path.exists(os.path.join(env_workspace, 'src', 'rudra_base_bridge')):
        return env_workspace

    parts = package_share.split(os.sep)
    if 'install' in parts:
        install_index = parts.index('install')
        return os.sep.join(parts[:install_index]) or os.sep
    if 'src' in parts:
        src_index = parts.index('src')
        return os.sep.join(parts[:src_index]) or os.sep
    return '/home/rudra/Projects/RUDRAv4'


def generate_launch_description():
    base_share = get_package_share_directory('rudra_base_bridge')
    voice_share = get_package_share_directory('rudra_voice')
    workspace_root = _workspace_root(base_share)

    default_robot_config = os.path.join(
        base_share,
        'config',
        'rudra_v4_hardware.yaml',
    )
    default_lidar_config = os.path.join(
        base_share,
        'config',
        'ydlidar_g2b.yaml',
    )
    default_ekf_config = os.path.join(
        base_share,
        'config',
        'localization_ekf.yaml',
    )
    default_venv_python = os.path.join(
        workspace_root,
        '.venv_voice',
        'bin',
        'python',
    )
    default_ollama_script = os.path.join(
        workspace_root,
        'scripts',
        'start_ollama.sh',
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'rudra_ps2_lidar_localization.launch.py')
        ),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'lidar_config': LaunchConfiguration('lidar_config'),
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'ekf_config': LaunchConfiguration('ekf_config'),
            'enable_dcdc_monitor': LaunchConfiguration('enable_dcdc_monitor'),
        }.items(),
    )

    voice_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(voice_share, 'launch', 'rudra_voice.launch.py')
        ),
        launch_arguments={
            'use_venv': LaunchConfiguration('use_voice_venv'),
            'venv_python': LaunchConfiguration('voice_venv_python'),
            'use_llm_router': LaunchConfiguration('voice_use_llm_router'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_voice')),
    )

    rviz_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'localization_view.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    ollama_helper = ExecuteProcess(
        cmd=[
            LaunchConfiguration('ollama_script'),
            LaunchConfiguration('ollama_model'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ollama')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_robot_config,
            description='Full path to rudra_v4_hardware.yaml',
        ),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=default_lidar_config,
            description='Full path to the YDLIDAR parameter YAML',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
            ),
            description='USB serial port for the YDLIDAR adapter',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame id used in the LaserScan header',
        ),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=default_ekf_config,
            description='Full path to the EKF localization parameter YAML',
        ),
        DeclareLaunchArgument(
            'enable_dcdc_monitor',
            default_value='false',
            description='Start Mini-Box DCDC-USB monitor for the NUC power rail',
        ),
        DeclareLaunchArgument(
            'enable_voice',
            default_value='true',
            description='Start RUDRA voice node and voice command guard',
        ),
        DeclareLaunchArgument(
            'voice_use_llm_router',
            default_value='false',
            description='Let voice use Ollama for uncertain phrases in full launch.',
        ),
        DeclareLaunchArgument(
            'use_voice_venv',
            default_value='true',
            description='Run voice nodes through .venv_voice/bin/python',
        ),
        DeclareLaunchArgument(
            'voice_venv_python',
            default_value=default_venv_python,
            description='Python executable with vosk, sounddevice, and requests',
        ),
        DeclareLaunchArgument(
            'enable_ollama',
            default_value='false',
            description='Start/check Ollama before launching voice LLM routing',
        ),
        DeclareLaunchArgument(
            'ollama_script',
            default_value=default_ollama_script,
            description='Helper script that starts Ollama and pulls the configured model',
        ),
        DeclareLaunchArgument(
            'ollama_model',
            default_value='qwen2.5:3b',
            description='Ollama model used by rudra_voice when LLM routing is needed',
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Start localization RViz on this machine',
        ),
        ollama_helper,
        robot_bringup,
        voice_bringup,
        rviz_view,
    ])
