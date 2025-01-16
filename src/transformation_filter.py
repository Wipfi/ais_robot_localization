#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_matrix
import tf as tf
import tf_conversions


class SE3KalmanFilter:
    def __init__(self, initial_state, process_noise_cov, measurement_noise_cov):
        self.state = initial_state
        self.covariance = np.eye(len(initial_state))
        self.Q = process_noise_cov
        self.R = measurement_noise_cov

    def predict(self, delta_s):
        F = np.eye(len(self.state))
        F[0, 6] = delta_s
        F[1, 7] = delta_s
        F[2, 8] = delta_s
        F[3, 9] = delta_s
        F[4, 10] = delta_s
        F[5, 11] = delta_s

        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement):
        H = np.zeros((6, len(self.state)))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        H[3, 3] = 1
        H[4, 4] = 1
        H[5, 5] = 1

        z_pred = H @ self.state
        y = measurement - z_pred

        S = H @ self.covariance @ H.T + self.R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance

    def get_state(self):
        return self.state

class SE3KalmanFilterROSNode:
    def __init__(self):
        # Parameters
        self.delta_s_threshold = rospy.get_param("~delta_s_threshold", 1)  # Threshold for delta_s
        self.initial_state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # Initial filter state
        self.Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001])
        self.R = np.diag([50, 50, 50, 5, 5, 5])
        self.filter = SE3KalmanFilter(self.initial_state, self.Q, self.R)

        # Variables
        self.last_odom_position = None  # To store the last odometry position
        self.total_delta_s = 0  # Accumulated delta_s

        # ROS Subscribers and TF
        self.odom_sub = rospy.Subscriber("/localization/odometry/odom_lidar", Odometry, self.odometry_callback)
        self.odom_pub = rospy.Publisher("/transform_filtered_odom", Odometry, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_transformed_odom(self, odom_msg):
        """
        Transform the incoming odometry message from the odom frame to the map frame
        using the filtered transform and publish the result.
        """
        # Extract the filtered transform (translation and rotation)
        filtered_state = self.filter.get_state()
        filtered_translation = filtered_state[:3]
        filtered_rotation = filtered_state[3:6]  # Euler angles: roll, pitch, yaw

        # Convert Euler angles to quaternion
        filtered_quaternion = tf_conversions.transformations.quaternion_from_euler(
            filtered_rotation[0], filtered_rotation[1], filtered_rotation[2]
        )

        # Transform the odometry pose into the map frame
        odom_position = odom_msg.pose.pose.position
        odom_orientation = odom_msg.pose.pose.orientation

        # Convert odometry pose to a 4x4 transformation matrix
        odom_quaternion = [odom_orientation.x, odom_orientation.y, odom_orientation.z, odom_orientation.w]
        odom_rotation_matrix = tf_conversions.transformations.quaternion_matrix(odom_quaternion)
        odom_rotation_matrix[:3, 3] = [odom_position.x, odom_position.y, odom_position.z]

        # Create the filtered transform as a 4x4 matrix
        filtered_rotation_matrix = tf_conversions.transformations.quaternion_matrix(filtered_quaternion)
        filtered_rotation_matrix[:3, 3] = filtered_translation

        # Apply the filtered transform to the odometry pose
        transformed_pose_matrix = np.dot(filtered_rotation_matrix, odom_rotation_matrix)

        # Extract the transformed position and quaternion
        transformed_position = transformed_pose_matrix[:3, 3]
        transformed_quaternion = tf_conversions.transformations.quaternion_from_matrix(transformed_pose_matrix)

        # Create the transformed Odometry message
        transformed_odom = Odometry()
        transformed_odom.header.stamp = rospy.Time.now()
        transformed_odom.header.frame_id = "map"  # New frame: map
        transformed_odom.child_frame_id = odom_msg.child_frame_id  # Preserve the child frame ID

        # Set the transformed pose
        transformed_odom.pose.pose.position.x = transformed_position[0]
        transformed_odom.pose.pose.position.y = transformed_position[1]
        transformed_odom.pose.pose.position.z = transformed_position[2]
        transformed_odom.pose.pose.orientation.x = transformed_quaternion[0]
        transformed_odom.pose.pose.orientation.y = transformed_quaternion[1]
        transformed_odom.pose.pose.orientation.z = transformed_quaternion[2]
        transformed_odom.pose.pose.orientation.w = transformed_quaternion[3]

        # Copy the original twist (velocity) data
        transformed_odom.twist = odom_msg.twist

        # Publish the transformed odometry message
        self.odom_pub.publish(transformed_odom)
        #rospy.loginfo("Published transformed odometry in map frame.")



    def odometry_callback(self, msg):
        # Compute delta_s from the odometry message
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        if self.last_odom_position is not None:
            delta_s = np.linalg.norm(position - self.last_odom_position)
            self.total_delta_s += delta_s

        if self.total_delta_s > self.delta_s_threshold:
            self.total_delta_s = 0  # Reset accumulated delta_s
            self.process_map_to_odom_transform()

        self.publish_transformed_odom(msg)

        self.last_odom_position = position

    def process_map_to_odom_transform(self):
        try:
            # Request the map -> odom transform
            transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0))

            # Extract translation from the transform
            translation = transform.transform.translation
            translation_vector = np.array([translation.x, translation.y, translation.z])

            # Extract rotation (quaternion) and convert to Euler angles
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            euler_angles = tf.transformations.euler_from_quaternion(quaternion)  # Convert to roll, pitch, yaw

            # Combine translation and rotation into the measurement vector
            measurement = np.concatenate((translation_vector, euler_angles))

            # Update the Kalman filter with the measurement
            self.filter.predict(self.delta_s_threshold)  # Predict with delta_s
            self.filter.update(measurement)

            # Get and print the filtered state
            filtered_state = self.filter.get_state()
            filtered_translation = filtered_state[:3]
            filtered_rotation = filtered_state[3:6]  # Filtered Euler angles
            drift_translation = filtered_state[6:9]  # Drift rates for translation
            drift_rotation = filtered_state[9:12]  # Drift rates for rotation

            rospy.loginfo("Unfildered Transform:")
            rospy.loginfo(f"Translation: x={measurement[0]:.3f}, y={measurement[1]:.3f}, z={measurement[2]:.3f}")
            rospy.loginfo(f"Rotation (Euler): roll={measurement[3]:.3f}, pitch={measurement[4]:.3f}, yaw={measurement[5]:.3f}")

            rospy.loginfo("Filtered Transform:")
            rospy.loginfo(f"Translation: x={filtered_translation[0]:.3f}, y={filtered_translation[1]:.3f}, z={filtered_translation[2]:.3f}")
            rospy.loginfo(f"Rotation (Euler): roll={filtered_rotation[0]:.3f}, pitch={filtered_rotation[1]:.3f}, yaw={filtered_rotation[2]:.3f}")
            rospy.loginfo("Drift Rates:")
            rospy.loginfo(f"Translation Drift: dx={drift_translation[0]:.6f}, dy={drift_translation[1]:.6f}, dz={drift_translation[2]:.6f}")
            rospy.loginfo(f"Rotation Drift: droll={drift_rotation[0]:.6f}, dpitch={drift_rotation[1]:.6f}, dyaw={drift_rotation[2]:.6f}")

        except tf2_ros.LookupException:
            rospy.logwarn("Transform map -> odom not available!")
        except tf2_ros.ConnectivityException:
            rospy.logwarn("TF connectivity issue!")
        except tf2_ros.ExtrapolationException:
            rospy.logwarn("Transform extrapolation error!")


if __name__ == "__main__":
    rospy.init_node("se3_kalman_filter_node")
    try:
        node = SE3KalmanFilterROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
