"""Independent PX4 uXRCE-DDS adapter; optional owned serial/UDP Agent."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav_bringup.xrce import agent_actions


def compose(context):
    config_dir = str(Path(LaunchConfiguration('config_dir').perform(context)).resolve())
    start_agent = LaunchConfiguration('start_agent').perform(context)
    if start_agent not in ('true', 'false'):
        raise ValueError('start_agent must be true or false')
    actions = agent_actions(config_dir) if start_agent == 'true' else []
    actions.append(Node(package='px4_adapter_ros2', executable='px4_adapter_node', name='px4_adapter',
                        namespace=LaunchConfiguration('uav_namespace'), output='screen',
                        parameters=[{'config_file': str(Path(config_dir) / 'node/px4_adapter/px4_adapter.xml')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uav_namespace', default_value='uav1'),
        DeclareLaunchArgument('config_dir', default_value=str(Path(get_package_share_directory('bringup')) / 'config')),
        DeclareLaunchArgument('start_agent', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=compose),
    ])
