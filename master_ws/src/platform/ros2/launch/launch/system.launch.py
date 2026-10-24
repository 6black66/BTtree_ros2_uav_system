"""Compose nodes with typed XML configuration from config/node/<name>/<name>.xml."""

from pathlib import Path
from uav_bringup.xml_config import load_node_config

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def managed_node(*, package, executable, name, parameters):
    # Explicit types prevent launch YAML coercion of XML string values.
    typed = {key: ParameterValue(value, value_type=type(value)) for key, value in parameters.items()}
    return Node(package=package, executable=executable, name=name,
                parameters=[typed], output="screen", respawn=True, respawn_delay=2.0)


def compose(context):
    role = LaunchConfiguration("role").perform(context)
    namespace = LaunchConfiguration("uav_namespace").perform(context)
    config_dir = str(Path(LaunchConfiguration("config_dir").perform(context)).resolve())
    if role not in ("master", "uav", "master_uav"):
        raise ValueError("role must be master, uav or master_uav")
    variables = {"config_dir": config_dir, "uav_namespace": namespace}
    if role in ("uav", "master_uav"):
        variables["plugin_dir"] = str(Path(get_package_prefix("bt_capability_plugins")) / "lib")
    def node(config_name, package, executable, name):
        parameters = load_node_config(config_dir, config_name, variables)
        if config_name == "observe":
            # Allows model paths relative to the XML, including installed config.
            parameters["config_file"] = str(Path(config_dir) / "node/observe/observe.xml")
        return managed_node(package=package, executable=executable, name=name,
                            parameters=parameters)
    groups = []
    if role in ("master", "master_uav"):
        groups.append(GroupAction(actions=[PushRosNamespace("fleet"),
            node("mission", "mission_executor_ros2", "mission_executor_node", "mission_executor")]))
    if role in ("uav", "master_uav"):
        nodes = [
            ("controller", "flight_controller_ros2", "controller_node", "flight_controller_gateway"),
            ("scene", "video_gateway_ros2", "episode_node", "video_gateway"),
            ("px4_adapter", "px4_adapter_ros2", "px4_adapter_node", "px4_adapter"),
            ("find", "find_executor_ros2", "find_executor_node", "find_executor"),
            ("observe", "observe_executor_ros2", "observe_executor_node", "observe_executor"),
            ("track", "track_executor_ros2", "track_executor_node", "track_executor"),
            ("refind", "refind_executor_ros2", "refind_executor_node", "refind_executor"),
            ("report", "report_executor_ros2", "report_executor_node", "report_executor"),
            ("behavior", "executor", "behavior_executor_node", "behavior_executor"),
        ]
        groups.append(GroupAction(actions=[PushRosNamespace(namespace)] + [node(*spec) for spec in nodes]))
    return groups


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("role", default_value="master_uav", choices=["master", "uav", "master_uav"]),
        DeclareLaunchArgument("uav_namespace", default_value="uav1"),
        DeclareLaunchArgument("config_dir", default_value=str(Path(get_package_share_directory("bringup")) / "config")),
        OpaqueFunction(function=compose),
    ])
