from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    gps_topic = LaunchConfiguration('gps_topic')
    heading_topic = LaunchConfiguration('heading_topic')
    odometry_output_topic = LaunchConfiguration('odometry_output_topic')
    imu_output_topic = LaunchConfiguration('imu_output_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true.')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_localization'),
            'params',
            'navsat_preprocessing.yaml',
        ]),
        description='Parameter file for the navsat preprocessing node.')
    declare_gps_topic = DeclareLaunchArgument(
        'gps_topic',
        default_value='localization/navsat/odometry/gps',
        description='Topic providing GNSS odometry messages.')
    declare_heading_topic = DeclareLaunchArgument(
        'heading_topic',
        default_value='navsat/orientation',
        description='Topic providing heading information as QuaternionStamped.')
    declare_odometry_output = DeclareLaunchArgument(
        'odometry_output_topic',
        default_value='localization/preprocessing/global_odom_fix_cov',
        description='Output topic for the fused global odometry.')
    declare_imu_output = DeclareLaunchArgument(
        'imu_output_topic',
        default_value='localization/preprocessing/global_orientation_fix_cov',
        description='Optional IMU topic with the corrected orientation.')

    navsat_preprocessing = Node(
        package='robot_localization',
        executable='navsat_preprocessing_node.py',
        name='navsat_preprocessing',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('localization/preprocessing/input/gps_odometry', gps_topic),
            (
                'localization/preprocessing/input/orientation_with_global_heading',
                heading_topic,
            ),
            ('localization/preprocessing/output/odometry', odometry_output_topic),
            ('localization/preprocessing/output/imu_with_fix_cov', imu_output_topic),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_gps_topic,
        declare_heading_topic,
        declare_odometry_output,
        declare_imu_output,
        navsat_preprocessing,
    ])
