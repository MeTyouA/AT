"""
Topo Path Planner Launch File
启动拓扑路径规划器节点（组件版本）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('topo_path_planner')
    config_file = os.path.join(pkg_share, 'config', 'topo_planner_params.yaml')

    return LaunchDescription([
        # 声明launch参数
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Global frame id'
        ),
        DeclareLaunchArgument(
            'use_skeleton',
            default_value='true',
            description='Use skeleton extraction for sampling'
        ),
        DeclareLaunchArgument(
            'enable_smoothing',
            default_value='true',
            description='Enable path smoothing'
        ),

        # 组件容器
        ComposableNodeContainer(
            name='topo_planner_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='topo_path_planner',
                    plugin='topo_path_planner::TopoPlannerNode',
                    name='topo_planner_node',
                    parameters=[
                        config_file,
                        {
                            'global_frame': LaunchConfiguration('global_frame'),
                            'use_skeleton': LaunchConfiguration('use_skeleton'),
                            'enable_smoothing': LaunchConfiguration('enable_smoothing'),
                        }
                    ],
                    remappings=[
                        ('/topo_plan_path', '/planned_path'),
                    ]
                ),
            ],
        ),
    ])
