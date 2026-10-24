"""Start just the independent video gateway using config/node/scene/scene.xml."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uav_namespace', default_value='uav1'),
        DeclareLaunchArgument('config_dir', default_value=str(Path(get_package_share_directory('bringup')) / 'config')),
        Node(package='video_gateway_ros2', executable='episode_node', name='video_gateway',
             namespace=LaunchConfiguration('uav_namespace'), output='screen', respawn=True,
             respawn_delay=2.0, parameters=[{'config_file': PathJoinSubstitution([
                 LaunchConfiguration('config_dir'), 'node', 'scene', 'scene.xml'])}]),
    ])
