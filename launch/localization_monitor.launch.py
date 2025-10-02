"""Launch localization monitor node with example parameters."""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_localization')
    parameter_file = os.path.join(package_share, 'params', 'localization_monitor.yaml')

    monitor_node = Node(
        package='robot_localization',
        executable='localization_monitor_node',
        name='localization_monitor',
        output='screen',
        parameters=[parameter_file],
    )

    return LaunchDescription([monitor_node])
