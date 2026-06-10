"""Launch RUDRA voice and command guard nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _make_nodes(context, *args, **kwargs):
    package_share = get_package_share_directory('rudra_voice')
    config_path = os.path.join(package_share, 'config', 'rudra_voice.yaml')
    workspace_root = os.path.abspath(
        os.path.join(package_share, '..', '..', '..', '..')
    )
    default_venv_python = os.path.join(workspace_root, '.venv_voice', 'bin', 'python')
    configured_venv_python = LaunchConfiguration('venv_python').perform(context)
    use_venv = _as_bool(LaunchConfiguration('use_venv').perform(context))
    venv_python = configured_venv_python or default_venv_python
    python_prefix = f'{venv_python} ' if use_venv and os.path.exists(venv_python) else None

    node_kwargs = {}
    launch_actions = []
    if python_prefix is not None:
        node_kwargs['prefix'] = python_prefix
        launch_actions.append(
            LogInfo(msg=f'RUDRA voice launch using Python: {venv_python}')
        )
    else:
        launch_actions.append(
            LogInfo(
                msg=(
                    'RUDRA voice launch using console-script default Python. '
                    'Create .venv_voice or pass use_venv:=false if this is intentional.'
                )
            )
        )

    voice_node = Node(
        package='rudra_voice',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[
            config_path,
            {'use_llm_router': LaunchConfiguration('use_llm_router')},
        ],
        **node_kwargs,
    )

    command_guard_node = Node(
        package='rudra_voice',
        executable='command_guard_node',
        name='command_guard_node',
        output='screen',
        parameters=[config_path],
        **node_kwargs,
    )

    launch_actions.extend([voice_node, command_guard_node])
    return launch_actions


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('rudra_voice')
    workspace_root = os.path.abspath(
        os.path.join(package_share, '..', '..', '..', '..')
    )
    default_venv_python = os.path.join(workspace_root, '.venv_voice', 'bin', 'python')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_venv',
                default_value='true',
                description='Run nodes through .venv_voice/bin/python when available.',
            ),
            DeclareLaunchArgument(
                'venv_python',
                default_value=default_venv_python,
                description='Python executable that has vosk, sounddevice, and requests.',
            ),
            DeclareLaunchArgument(
                'use_llm_router',
                default_value='true',
                description='Route uncertain phrases through the local Ollama model.',
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
