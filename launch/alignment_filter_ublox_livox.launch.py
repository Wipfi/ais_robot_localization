from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    monitor_params = LaunchConfiguration('monitor_params')
    alignment_params = LaunchConfiguration('alignment_params')
    navsat_params = LaunchConfiguration('navsat_params')
    navsat_transform_params = LaunchConfiguration('navsat_transform_params')

    global_odom_topic = LaunchConfiguration('global_odom_topic')
    local_odom_topic = LaunchConfiguration('local_odom_topic')
    monitor_result_topic = LaunchConfiguration('monitor_result_topic')
    monitor_rpe_topic = LaunchConfiguration('monitor_rpe_topic')
    monitor_global_path_topic = LaunchConfiguration('monitor_global_path_topic')
    monitor_local_path_topic = LaunchConfiguration('monitor_local_path_topic')
    scaled_gps_topic = LaunchConfiguration('scaled_gps_topic')

    navsat_pre_input_topic = LaunchConfiguration('navsat_input_topic')
    navsat_orientation_topic = LaunchConfiguration('navsat_orientation_topic')
    navsat_output_topic = LaunchConfiguration('navsat_output_topic')
    navsat_imu_topic = LaunchConfiguration('navsat_imu_topic')
    navsat_fix_topic = LaunchConfiguration('navsat_fix_topic')

    alignment_odom_topic = LaunchConfiguration('alignment_odom_topic')
    alignment_global_path_topic = LaunchConfiguration('alignment_global_path_topic')
    alignment_local_path_topic = LaunchConfiguration('alignment_local_path_topic')

    start_navsat_preprocessing = LaunchConfiguration('start_navsat_preprocessing')
    start_alignment_filter = LaunchConfiguration('start_alignment_filter')
    start_navsat_transform = LaunchConfiguration('start_navsat_transform')
    start_rviz = LaunchConfiguration('start_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true.')
    declare_monitor_params = DeclareLaunchArgument(
        'monitor_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_robot_localization'),
            'params',
            'localization_monitor.yaml',
        ]),
        description='Parameter file for the localization monitor node.')
    declare_alignment_params = DeclareLaunchArgument(
        'alignment_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_robot_localization'),
            'params',
            'alignment_filter.yaml',
        ]),
        description='Parameter file for the alignment filter node.')
    declare_navsat_params = DeclareLaunchArgument(
        'navsat_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_robot_localization'),
            'params',
            'navsat_preprocessing_ublox_livox.yaml',
        ]),
        description='Parameter file for the u-blox/Livox navsat preprocessing node.')
    declare_navsat_transform_params = DeclareLaunchArgument(
        'navsat_transform_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_robot_localization'),
            'params',
            'navsat_transform.yaml',
        ]),
        description='Parameter file for the navsat_transform_node.')

    declare_global_odom = DeclareLaunchArgument(
        'global_odom_topic',
        default_value='localization/preprocessing/global_odom_fix_cov',
        description='Globally referenced odometry topic to monitor.')
    declare_local_odom = DeclareLaunchArgument(
        'local_odom_topic',
        default_value='localization/odometry/odom_lidar',
        description='Locally referenced odometry topic to monitor and align.')
    declare_monitor_result = DeclareLaunchArgument(
        'monitor_result_topic',
        default_value='localization/monitor/result',
        description='Output topic for aggregated monitor statistics.')
    declare_monitor_rpe = DeclareLaunchArgument(
        'monitor_rpe_topic',
        default_value='localization/monitor/RPE_Values',
        description='Output topic for RPE feature vectors.')
    declare_monitor_global_path = DeclareLaunchArgument(
        'monitor_global_path_topic',
        default_value='localization/monitor/global_reference_path',
        description='Output topic for the global reference path.')
    declare_monitor_local_path = DeclareLaunchArgument(
        'monitor_local_path_topic',
        default_value='localization/monitor/local_reference_path',
        description='Output topic for the transformed local path.')
    declare_scaled_gps = DeclareLaunchArgument(
        'scaled_gps_topic',
        default_value='localization/monitor/gnss_with_scaled_covariance',
        description='Output topic for the GNSS message with scaled covariance.')

    declare_navsat_input = DeclareLaunchArgument(
        'navsat_input_topic',
        default_value='localization/navsat/odometry/gps',
        description='Input odometry topic for navsat preprocessing and transform nodes.')
    declare_navsat_orientation = DeclareLaunchArgument(
        'navsat_orientation_topic',
        default_value='/livox/imu',
        description='External orientation topic for the Livox IMU input.')
    declare_navsat_output = DeclareLaunchArgument(
        'navsat_output_topic',
        default_value='localization/preprocessing/global_odom_fix_cov',
        description='Output odometry topic for navsat preprocessing and navsat transform.')
    declare_navsat_imu = DeclareLaunchArgument(
        'navsat_imu_topic',
        default_value='localization/preprocessing/global_orientation_fix_cov',
        description='Output IMU topic containing the corrected orientation.')
    declare_navsat_fix = DeclareLaunchArgument(
        'navsat_fix_topic',
        default_value='/ublox/fix',
        description='Raw NavSatFix topic for the navsat transform node.')

    declare_alignment_odom = DeclareLaunchArgument(
        'alignment_odom_topic',
        default_value='localization/alignment_filter/odom_map',
        description='Output odometry topic for the alignment filter.')
    declare_alignment_global_path = DeclareLaunchArgument(
        'alignment_global_path_topic',
        default_value='localization/alignment_filter/global_path',
        description='Output topic for the alignment filter global path.')
    declare_alignment_local_path = DeclareLaunchArgument(
        'alignment_local_path_topic',
        default_value='localization/alignment_filter/local_path',
        description='Output topic for the transformed local path.')

    declare_start_navsat_preprocessing = DeclareLaunchArgument(
        'start_navsat_preprocessing',
        default_value='true',
        description='Start the navsat preprocessing node as part of the pipeline.')
    declare_start_alignment_filter = DeclareLaunchArgument(
        'start_alignment_filter',
        default_value='true',
        description='Start the alignment filter node as part of the pipeline.')
    declare_start_navsat_transform = DeclareLaunchArgument(
        'start_navsat_transform',
        default_value='true',
        description='Start the navsat_transform_node as part of the pipeline.')
    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Request RViz to start when using the pipeline helper script.')

    localization_monitor = Node(
        package='ais_robot_localization',
        executable='localization_monitor_node',
        name='localization_monitor',
        output='screen',
        parameters=[monitor_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('global_odom', global_odom_topic),
            ('local_odom', local_odom_topic),
            ('global_odom_path', monitor_global_path_topic),
            ('local_odom_path', monitor_local_path_topic),
            ('localization_result', monitor_result_topic),
            ('RPE_Values', monitor_rpe_topic),
            ('gnss_with_scaled_covariance', scaled_gps_topic),
        ],
    )

    alignment_filter = Node(
        package='ais_robot_localization',
        executable='alignment_filter_node',
        name='alignment_filter',
        output='screen',
        parameters=[alignment_params, {'use_sim_time': use_sim_time}, {'ignore_global_yaw': True}],
        remappings=[
            ('local_odom', local_odom_topic),
            ('localization_result', monitor_result_topic),
            ('alignment_odometry', alignment_odom_topic),
            ('alignment_global_path', alignment_global_path_topic),
            ('alignment_local_path_transformed', alignment_local_path_topic),
        ],
        condition=IfCondition(start_alignment_filter),
    )

    navsat_preprocessing = Node(
        package='ais_robot_localization',
        executable='navsat_preprocessing_node_ublox_livox.py',
        name='navsat_preprocessing_ublox_livox',
        output='screen',
        parameters=[navsat_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('localization/preprocessing/input/gps_odometry', navsat_pre_input_topic),
            ('localization/preprocessing/input/imu', navsat_orientation_topic),
            ('localization/preprocessing/output/odometry', navsat_output_topic),
            ('localization/preprocessing/output/imu_with_fix_cov', navsat_imu_topic),
        ],
        condition=IfCondition(start_navsat_preprocessing),
    )

    navsat_transform = Node(
        package='ais_robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_transform_params,
            {'use_sim_time': use_sim_time},
            {'use_odometry_yaw': True},
            {'publish_filtered_gps': True},
            {'broadcast_cartesian_transform': True},
        ],
        remappings=[
            ('odometry/filtered', alignment_odom_topic),
            ('gps/fix', navsat_fix_topic),
            ('odometry/gps', navsat_pre_input_topic),
            ('gps/filtered', 'localization/navsat/filtered_gps'),
        ],
        condition=IfCondition(start_navsat_transform),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        declare_use_sim_time,
        declare_monitor_params,
        declare_alignment_params,
        declare_navsat_params,
        declare_navsat_transform_params,
        declare_global_odom,
        declare_local_odom,
        declare_monitor_result,
        declare_monitor_rpe,
        declare_monitor_global_path,
        declare_monitor_local_path,
        declare_scaled_gps,
        declare_navsat_input,
        declare_navsat_orientation,
        declare_navsat_output,
        declare_navsat_imu,
        declare_navsat_fix,
        declare_alignment_odom,
        declare_alignment_global_path,
        declare_alignment_local_path,
        declare_start_navsat_preprocessing,
        declare_start_alignment_filter,
        declare_start_navsat_transform,
        declare_start_rviz,
        localization_monitor,
        navsat_preprocessing,
        alignment_filter,
        navsat_transform,
    ])
