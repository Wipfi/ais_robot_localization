from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dlo_yaml = PathJoinSubstitution([
        FindPackageShare('direct_lidar_odometry'),
        'config',
        'dlo.yaml'
    ])
    params_yaml = PathJoinSubstitution([
        FindPackageShare('direct_lidar_odometry'),
        'config',
        'params.yaml'
    ])

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='lidar'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu'
    )

    dlo_odom_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_odom_node',
        name='dlo_odom',
        output='screen',
        parameters=[dlo_yaml, params_yaml],
        remappings=[
            ('pointcloud', LaunchConfiguration('pointcloud_topic')),
            ('imu', LaunchConfiguration('imu_topic')),
            ('odom', '/localization/odometry/odom_lidar'),
            ('pose', 'dlo/odom_node/pose'),
            ('kfs', 'dlo/odom_node/odom/keyframe'),
            ('keyframe', 'dlo/odom_node/pointcloud/keyframe'),
        ]
    )

    return LaunchDescription([
        pointcloud_topic_arg,
        imu_topic_arg,
        dlo_odom_node
    ])
