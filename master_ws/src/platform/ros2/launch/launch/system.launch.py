"""Compose fleet and vehicle adapters without embedding business orchestration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def role_condition(role, expected):
    values = ", ".join(f"'{value}'" for value in expected)
    return IfCondition(PythonExpression(["'", role, f"' in [{values}]"]))


def managed_node(*, package, executable, name):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )


def generate_launch_description():
    role = LaunchConfiguration("role")
    uav_namespace = LaunchConfiguration("uav_namespace")

    master_group = GroupAction(
        condition=role_condition(role, ["master", "master_uav"]),
        actions=[
            PushRosNamespace("fleet"),
            Node(
                package="mission_executor_ros2",
                executable="mission_executor_node",
                name="mission_executor",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"target_namespace": uav_namespace}],
            ),
        ],
    )

    uav_group = GroupAction(
        condition=role_condition(role, ["uav", "master_uav"]),
        actions=[
            PushRosNamespace(uav_namespace),
            managed_node(
                package="flight_controller_ros2",
                executable="controller_node",
                name="flight_controller_gateway",
            ),
            managed_node(
                package="video_gateway_ros2",
                executable="episode_node",
                name="video_gateway",
            ),
            managed_node(
                package="find_executor_ros2",
                executable="find_executor_node",
                name="find_executor",
            ),
            managed_node(
                package="observe_executor_ros2",
                executable="observe_executor_node",
                name="observe_executor",
            ),
            managed_node(
                package="track_executor_ros2",
                executable="track_executor_node",
                name="track_executor",
            ),
            managed_node(
                package="refind_executor_ros2",
                executable="refind_executor_node",
                name="refind_executor",
            ),
            managed_node(
                package="report_executor_ros2",
                executable="report_executor_node",
                name="report_executor",
            ),
            managed_node(
                package="executor",
                executable="behavior_executor_node",
                name="behavior_executor",
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "role",
                default_value="master_uav",
                description="master, uav, or master_uav",
            ),
            DeclareLaunchArgument("uav_namespace", default_value="uav1"),
            master_group,
            uav_group,
        ]
    )
