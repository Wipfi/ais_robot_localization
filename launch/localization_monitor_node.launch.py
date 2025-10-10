from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    global_odom_topic = LaunchConfiguration('global_odom_topic')
    local_odom_topic = LaunchConfiguration('local_odom_topic')
    global_path_topic = LaunchConfiguration('global_path_topic')
    local_path_topic = LaunchConfiguration('local_path_topic')
    monitor_result_topic = LaunchConfiguration('monitor_result_topic')
    rpe_topic = LaunchConfiguration('rpe_topic')
    scaled_gps_topic = LaunchConfiguration('scaled_gps_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true.')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_robot_localization'),
            'params',
            'localization_monitor.yaml',
        ]),
        description='Parameter file for the localization monitor node.')
    declare_global_odom = DeclareLaunchArgument(
        'global_odom_topic',
        default_value='localization/preprocessing/global_odom_fix_cov',
        description='Globally referenced odometry topic to monitor.')
    declare_local_odom = DeclareLaunchArgument(
        'local_odom_topic',
        default_value='localization/odometry/odom_lidar',
        description='Locally referenced odometry topic to monitor.')
    declare_global_path = DeclareLaunchArgument(
        'global_path_topic',
        default_value='localization/monitor/global_reference_path',
        description='Output topic for the global reference path.')
    declare_local_path = DeclareLaunchArgument(
        'local_path_topic',
        default_value='localization/monitor/local_reference_path',
        description='Output topic for the transformed local path.')
    declare_monitor_result = DeclareLaunchArgument(
        'monitor_result_topic',
        default_value='localization/monitor/result',
        description='Output topic for aggregated monitor statistics.')
    declare_rpe_topic = DeclareLaunchArgument(
        'rpe_topic',
        default_value='localization/monitor/RPE_Values',
        description='Output topic for RPE feature vectors.')
    declare_scaled_gps = DeclareLaunchArgument(
        'scaled_gps_topic',
        default_value='localization/monitor/gnss_with_scaled_covariance',
        description='Output topic for the GNSS message with scaled covariance.')

    localization_monitor = Node(
        package='ais_robot_localization',
        executable='localization_monitor_node',
        name='localization_monitor',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('global_odom', global_odom_topic),
            ('local_odom', local_odom_topic),
            ('global_odom_path', global_path_topic),
            ('local_odom_path', local_path_topic),
            ('localization_result', monitor_result_topic),
            ('RPE_Values', rpe_topic),
            ('gnss_with_scaled_covariance', scaled_gps_topic),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_global_odom,
        declare_local_odom,
        declare_global_path,
        declare_local_path,
        declare_monitor_result,
        declare_rpe_topic,
        declare_scaled_gps,
        localization_monitor,
    ])
