from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    gps_topic = LaunchConfiguration('gps_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odometry_output_topic = LaunchConfiguration('odometry_output_topic')
    imu_output_topic = LaunchConfiguration('imu_output_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true.')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_robot_localization'),
            'params',
            'navsat_preprocessing_ublox_livox.yaml',
        ]),
        description='Parameter file for the navsat preprocessing u-blox/Livox node.')
    declare_gps_topic = DeclareLaunchArgument(
        'gps_topic',
        default_value='localization/navsat/odometry/gps',
        description='Topic providing GNSS odometry messages.')
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu',
        description='Topic providing Livox IMU measurements.')
    declare_odometry_output = DeclareLaunchArgument(
        'odometry_output_topic',
        default_value='localization/preprocessing/global_odom_fix_cov',
        description='Output topic for the fused global odometry.')
    declare_imu_output = DeclareLaunchArgument(
        'imu_output_topic',
        default_value='localization/preprocessing/global_orientation_fix_cov',
        description='Optional IMU topic with the corrected orientation.')

    navsat_preprocessing = Node(
        package='ais_robot_localization',
        executable='navsat_preprocessing_node_ublox_livox.py',
        name='navsat_preprocessing_ublox_livox',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('localization/preprocessing/input/gps_odometry', gps_topic),
            ('localization/preprocessing/input/imu', imu_topic),
            ('localization/preprocessing/output/odometry', odometry_output_topic),
            ('localization/preprocessing/output/imu_with_fix_cov', imu_output_topic),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_gps_topic,
        declare_imu_topic,
        declare_odometry_output,
        declare_imu_output,
        navsat_preprocessing,
    ])
