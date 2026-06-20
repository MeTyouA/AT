import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("mid360_chunk_cloud")
    default_params_file = os.path.join(
        package_share_dir,
        "config",
        "mid360_chunk_cloud.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the YAML parameter file for mid360_chunk_cloud_node.",
        ),
        Node(
            package="mid360_chunk_cloud",
            executable="mid360_chunk_cloud_node",
            name="mid360_chunk_cloud_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
