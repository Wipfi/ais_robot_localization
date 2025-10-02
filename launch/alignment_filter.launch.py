from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    local_odom_topic = LaunchConfiguration('local_odom_topic')
    monitor_result_topic = LaunchConfiguration('monitor_result_topic')
    alignment_odom_topic = LaunchConfiguration('alignment_odom_topic')
    alignment_global_path_topic = LaunchConfiguration('alignment_global_path_topic')
    alignment_local_path_topic = LaunchConfiguration('alignment_local_path_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true.')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_localization'),
            'params',
            'alignment_filter.yaml',
        ]),
        description='Parameter file for the alignment filter node.')
    declare_local_odom = DeclareLaunchArgument(
        'local_odom_topic',
        default_value='localization/odometry/odom_lidar',
        description='Input topic for the locally referenced odometry.')
    declare_monitor_result = DeclareLaunchArgument(
        'monitor_result_topic',
        default_value='localization/monitor/result',
        description='Localization monitor result topic to listen to.')
    declare_alignment_odom = DeclareLaunchArgument(
        'alignment_odom_topic',
        default_value='localization/alignment_filter/odom_map',
        description='Output topic for the globally aligned odometry.')
    declare_alignment_global_path = DeclareLaunchArgument(
        'alignment_global_path_topic',
        default_value='localization/alignment_filter/global_path',
        description='Output topic for the global reference path used by the filter.')
    declare_alignment_local_path = DeclareLaunchArgument(
        'alignment_local_path_topic',
        default_value='localization/alignment_filter/local_path',
        description='Output topic for the transformed local path.')

    alignment_filter = Node(
        package='robot_localization',
        executable='alignment_filter_node',
        name='alignment_filter',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('local_odom', local_odom_topic),
            ('localization_result', monitor_result_topic),
            ('alignment_odometry', alignment_odom_topic),
            ('alignment_global_path', alignment_global_path_topic),
            ('alignment_local_path_transformed', alignment_local_path_topic),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_local_odom,
        declare_monitor_result,
        declare_alignment_odom,
        declare_alignment_global_path,
        declare_alignment_local_path,
        alignment_filter,
    ])
