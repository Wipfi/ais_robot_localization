"""Launch alignment filter node with example parameters."""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_localization')
    parameter_file = os.path.join(package_share, 'params', 'alignment_filter.yaml')

    alignment_node = Node(
        package='robot_localization',
        executable='alignment_filter_node',
        name='alignment_filter',
        output='screen',
        parameters=[parameter_file],
    )

    return LaunchDescription([alignment_node])
