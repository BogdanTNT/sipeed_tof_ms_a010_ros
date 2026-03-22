from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _create_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    node_name = LaunchConfiguration("node_name").perform(context)

    instance_name = os.path.splitext(os.path.basename(params_file))[0]
    if instance_name.endswith(".params"):
        instance_name = instance_name[:-7]

    resolved_namespace = namespace or instance_name
    resolved_node_name = node_name or f"{instance_name}_node"

    return [
        Node(
            package="sipeed_tof_ms_a010",
            executable="sipeed_tof_node",
            namespace=resolved_namespace,
            name=resolved_node_name,
            output="screen",
            emulate_tty=True,
            parameters=[params_file],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("sipeed_tof_ms_a010")
    default_params = os.path.join(package_share, "config", "tof_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description=(
                    "Namespace for this MS-A010 instance. If empty, the YAML "
                    "filename stem is used."
                ),
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="",
                description=(
                    "Node name for this MS-A010 instance. If empty, the YAML "
                    "filename stem plus '_node' is used."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the MS-A010 parameter YAML file.",
            ),
            OpaqueFunction(function=_create_node),
        ]
    )
