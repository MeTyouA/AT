from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('smooth_cmd_node').find('smooth_cmd_node')
    config_file = os.path.join(pkg_share, 'config', 'smooth_cmd_node.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='YAML 参数文件路径'
        ),

        Node(
            package='smooth_cmd_node',
            executable='smooth_cmd_node',
            name='smooth_cmd_node',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        )
    ])

