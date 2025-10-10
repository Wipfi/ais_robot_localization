#!/usr/bin/env python3
import numpy as np
if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "int"):
    np.int = int


from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from geometry_msgs.msg import Quaternion, QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import numpy as np
import transforms3d.quaternions as quat




class NavsatPreprocessingNode(Node):
    """Fuse GNSS odometry with an externally provided heading source."""

    def __init__(self) -> None:
        super().__init__('navsat_preprocessing_node')
        self.get_logger().info('Initializing navsat preprocessing node...')

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        # Offset-Quaternion aus Parametern (ROS-Format [x,y,z,w])
        self.q_off = self.declare_parameter(
            "quat_offset", [0.0, 0.0, 0.0, 1.0]  # Default = Identität [0.0, 0.0, 0.0, 1.0]
        ).value


        self.base_link_frame = self.declare_parameter('base_link_frame', 'base_link').value
        self.use_external_heading = self.declare_parameter('use_external_heading', True).value
        self.calculate_heading_from_trajectory = self.declare_parameter(
            'calculate_heading_from_trajectory', False).value
        self.overwrite_covariance_matrix = self.declare_parameter(
            'overwrite_covariance_matrix', False).value
        param = self.declare_parameter('heading_reference_length', 10.0)
        self.heading_reference_length = float(param.value)

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

    
    def quat_to_mat4(self, q: Quaternion) -> np.ndarray:
        """Convert ROS Quaternion [x,y,z,w] to 4x4 rotation matrix."""
        x, y, z, w = q.x, q.y, q.z, q.w
        n = x*x + y*y + z*z + w*w
        if n < 1e-16:
            return np.eye(4)
        s = 2.0 / n
        xx, yy, zz = x*x*s, y*y*s, z*z*s
        xy, xz, yz = x*y*s, x*z*s, y*z*s
        wx, wy, wz = w*x*s, w*y*s, w*z*s
        R = np.array([
            [1.0 - (yy + zz), xy - wz,       xz + wy,       0.0],
            [xy + wz,         1.0 - (xx + zz), yz - wx,     0.0],
            [xz - wy,         yz + wx,       1.0 - (xx + yy), 0.0],
            [0.0,             0.0,           0.0,           1.0],
        ])
        return R
    
    def mat4_to_quat(self, M: np.ndarray) -> Quaternion:
        """Convert 4x4 (or 3x3) rotation matrix to ROS Quaternion [x,y,z,w]."""
        m = M[:3, :3]
        tr = np.trace(m)
        if tr > 0.0:
            S = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (m[2,1] - m[1,2]) / S
            y = (m[0,2] - m[2,0]) / S
            z = (m[1,0] - m[0,1]) / S
        elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
            S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2.0
            w = (m[2,1] - m[1,2]) / S
            x = 0.25 * S
            y = (m[0,1] + m[1,0]) / S
            z = (m[0,2] + m[2,0]) / S
        elif m[1,1] > m[2,2]:
            S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2.0
            w = (m[0,2] - m[2,0]) / S
            x = (m[0,1] + m[1,0]) / S
            y = 0.25 * S
            z = (m[1,2] + m[2,1]) / S
        else:
            S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2.0
            w = (m[1,0] - m[0,1]) / S
            x = (m[0,2] + m[2,0]) / S
            y = (m[1,2] + m[2,1]) / S
            z = 0.25 * S
        return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


    def apply_static_transform(self, quaternion: Quaternion) -> Quaternion:
        # Eingangs-Quaternion → Matrix
        R_in = self.quat_to_mat4(quaternion)

        q_off_ros = Quaternion(x=self.q_off[0], y=self.q_off[1], z=self.q_off[2], w=self.q_off[3])
        R_off = self.quat_to_mat4(q_off_ros)

        # Multiplizieren (lokale Rotation)
        R_new = np.dot(R_in, R_off)

        # Matrix zurück → Quaternion
        return self.mat4_to_quat(R_new)



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
