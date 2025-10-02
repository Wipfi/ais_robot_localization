#!/usr/bin/env python3

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from geometry_msgs.msg import Quaternion, QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import tf_transformations as tf_trans


class NavsatPreprocessingNode(Node):
    """Fuse GNSS odometry with an externally provided heading source."""

    def __init__(self) -> None:
        super().__init__('navsat_preprocessing_node')
        self.get_logger().info('Initializing navsat preprocessing node...')
        self.declare_parameter('use_sim_time', False)

        self.base_link_frame = self.declare_parameter('base_link_frame', 'base_link').value
        self.use_external_heading = self.declare_parameter('use_external_heading', True).value
        self.calculate_heading_from_trajectory = self.declare_parameter(
            'calculate_heading_from_trajectory', False).value
        self.overwrite_covariance_matrix = self.declare_parameter(
            'overwrite_covariance_matrix', False).value
        self.heading_reference_length = float(self.declare_parameter('heading_reference_length', 10).value)
        self.publish_imu_message_with_orientation = self.declare_parameter(
            'publish_imu_message_with_orientation', False).value

        default_covariance = [
            1000.0, 0.0,   0.0,   0.0, 0.0, 0.0,
            0.0,   1000.0, 0.0,   0.0, 0.0, 0.0,
            0.0,   0.0,   1500.0, 0.0, 0.0, 0.0,
            0.0,   0.0,     0.0,  0.5, 0.0, 0.0,
            0.0,   0.0,     0.0,  0.0, 0.5, 0.0,
            0.0,   0.0,     0.0,  0.0, 0.0, 0.5,
        ]
        self.pose_covariance = self.declare_parameter(
            'pose_covariance', default_covariance).value  # type: ignore[assignment]

        qos = rclpy.qos.QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'localization/preprocessing/input/gps_odometry',
            self.odom_callback,
            qos)

        self.orientation_sub: Optional[Subscription] = None
        if self.use_external_heading:
            self.orientation_sub = self.create_subscription(
                QuaternionStamped,
                'localization/preprocessing/input/orientation_with_global_heading',
                self.orientation_with_heading_callback,
                qos)
        elif self.calculate_heading_from_trajectory:
            self.get_logger().error('calculate_heading_from_trajectory is not implemented yet')

        self.odom_with_pose_pub = self.create_publisher(
            Odometry, 'localization/preprocessing/output/odometry', 10)

        self.imu_pub: Optional[Publisher] = None
        if self.publish_imu_message_with_orientation:
            self.imu_pub = self.create_publisher(
                Imu, 'localization/preprocessing/output/imu_with_fix_cov', 10)

        self.latest_odom: Optional[Odometry] = None
        self.latest_orientation: Optional[QuaternionStamped] = None

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.latest_orientation is None:
            return
        self.publish_odom_with_pose_and_covariance()

    def orientation_with_heading_callback(self, msg: QuaternionStamped) -> None:
        self.latest_orientation = msg
        if self.publish_imu_message_with_orientation and self.imu_pub is not None:
            self.publish_imu_with_orientation()

    def apply_static_transform(self, quaternion: Quaternion) -> Quaternion:
        rotation_matrix = tf_trans.quaternion_matrix(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        local_z_rotation = tf_trans.quaternion_matrix([0.0, 0.0, 0.0, 1.0])
        rotated_matrix = tf_trans.concatenate_matrices(rotation_matrix, local_z_rotation)
        rotated_quaternion = tf_trans.quaternion_from_matrix(rotated_matrix)
        return Quaternion(x=rotated_quaternion[0], y=rotated_quaternion[1],
                          z=rotated_quaternion[2], w=rotated_quaternion[3])

    def publish_odom_with_pose_and_covariance(self) -> None:
        if self.latest_odom is None or self.latest_orientation is None:
            return

        odom_with_pose = Odometry()
        odom_with_pose.header = self.latest_odom.header
        odom_with_pose.child_frame_id = self.base_link_frame
        odom_with_pose.pose = self.latest_odom.pose

        if self.overwrite_covariance_matrix:
            odom_with_pose.pose.covariance = list(self.pose_covariance)

        odom_with_pose.pose.pose.orientation = self.apply_static_transform(
            self.latest_orientation.quaternion)

        self.odom_with_pose_pub.publish(odom_with_pose)

    def publish_imu_with_orientation(self) -> None:
        if self.latest_orientation is None or self.imu_pub is None:
            return

        imu_msg = Imu()
        imu_msg.header = self.latest_orientation.header
        imu_msg.orientation = self.latest_orientation.quaternion
        imu_msg.orientation_covariance = self._orientation_covariance_from_pose(self.pose_covariance)

        self.imu_pub.publish(imu_msg)

    @staticmethod
    def _orientation_covariance_from_pose(covariance: Sequence[float]) -> Sequence[float]:
        if len(covariance) >= 36:
            return [
                covariance[21], covariance[22], covariance[23],
                covariance[27], covariance[28], covariance[29],
                covariance[33], covariance[34], covariance[35],
            ]
        return [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavsatPreprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
