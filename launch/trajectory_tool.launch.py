"""Launch trajectory tool node with example parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_localization')
    parameter_file = os.path.join(package_share, 'params', 'trajectory_tool.yaml')

    trajectory_node = Node(
        package='robot_localization',
        executable='trajectory_tool.py',
        name='trajectory_tool',
        output='screen',
        parameters=[parameter_file],
    )

    return LaunchDescription([trajectory_node])
