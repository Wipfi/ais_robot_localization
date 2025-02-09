#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32MultiArray
from evo.core.trajectory import PoseTrajectory3D
from ais_robot_localization.msg import LocalizationMonitorResult
from utils import odom_to_se3, se3_to_posestamped, snake_alignment, gaussian_weight, euclidean_distance
import threading
import numpy as np
import tf.transformations as tf
import tf2_ros
from geometry_msgs.msg import Quaternion

class AlignmentBasedFilterNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('alignment_filter_node')

        self.publish_tf = rospy.get_param('~publish_tf', True)

        # Initialize data storage
        self.data_list = []
        self.error_trans_avg = []
        self.error_rot_avg = []
        self.distances_current_estimate = []
        self.time_stamp_list = []
        self.global_path = Path()
        self.global_poses = []
        self.local_poses = []
        self.local_path = Path()
        self.local_path_transformed = Path()
        self.used_length = 0.0

        self.current_odom_state = None
        
        # Frame IDs for global and local paths
        self.global_frame = "map"
        self.local_frame = "odom"
        self.last_selected = -float('inf')

        self.current_transform = None

        # Transform broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers for path visualization
        self.global_path_pub = rospy.Publisher('/alignment_global_path', Path, queue_size=10)
        self.local_path_transformed_pub = rospy.Publisher('/alignment_local_path_transformed', Path, queue_size=10)
        self.odom_publisher = rospy.Publisher('/alignment_odometry', Odometry, queue_size=10)
        
        # Subscriber for LocalizationMonitorResult messages
        self.sub = rospy.Subscriber('/localization_result', LocalizationMonitorResult, self.localization_monitor_callback)

        # Initialize lock for thread safety
        self.lock = threading.Lock()

        self.local_odom_sub = rospy.Subscriber("/local_odom", Odometry, self.local_odom_callback)

        rospy.loginfo("AlignmentBasedFilterNode initialized")


    def publish_transform(self, odom):
        """Publish the inverse of the current transformation as map -> odom."""
        if self.current_transform is None:
            return

        try:
            # Compute the inverse transform
            transform = self.current_transform#tf.inverse_matrix(self.current_transform)

            # Create a TransformStamped message
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.global_frame  # Parent frame
            t.child_frame_id = self.local_frame    # Child frame


            # Extract translation and rotation
            translation = transform[:3, 3]
            rotation_matrix = transform[:3, :3]

            # Convert rotation matrix to quaternion
            quaternion = tf.quaternion_from_matrix(transform)

            # Fill the TransformStamped message
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)
            #rospy.loginfo("Published transform: map -> odom")
        except Exception as e:
            rospy.logerr(f"Error publishing transform: {e}")


    def local_odom_callback(self, odom_msg):
        with self.lock:
            # Extract the current position and orientation from the Odometry message
            position = np.array([odom_msg.pose.pose.position.x,
                                odom_msg.pose.pose.position.y,
                                odom_msg.pose.pose.position.z, 1])
            
            # Convert quaternion to a 3x3 rotation matrix
            orientation_q = odom_msg.pose.pose.orientation
            rotation_matrix = tf.quaternion_matrix([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            
            # Create the current pose 4x4 matrix
            current_pose_matrix = np.eye(4)
            current_pose_matrix[:3, :3] = rotation_matrix[:3, :3]
            current_pose_matrix[:3, 3] = position[:3]
            
            # Apply the transformation
            if self.current_transform is None or self.used_length < 10.0:
                transformed_pose_matrix = current_pose_matrix
            else:
                if self.publish_tf:
                    self.publish_transform(odom_msg)
                transformed_pose_matrix = np.dot(self.current_transform, current_pose_matrix)
            
            # Extract the transformed position
            transformed_position = transformed_pose_matrix[:3, 3]
            
            # Convert the transformed rotation matrix back to a quaternion
            transformed_quaternion = tf.quaternion_from_matrix(transformed_pose_matrix)
            
            # Update the Odometry message
            transformed_odom = Odometry()
            transformed_odom.header = odom_msg.header  # Keep the same header
            transformed_odom.child_frame_id = odom_msg.child_frame_id
            transformed_odom.header.frame_id = self.global_frame
            
            # Update position
            transformed_odom.pose.pose.position.x = transformed_position[0]
            transformed_odom.pose.pose.position.y = transformed_position[1]
            transformed_odom.pose.pose.position.z = transformed_position[2]
            
            # Update orientation
            transformed_odom.pose.pose.orientation = Quaternion(
                x=transformed_quaternion[0],
                y=transformed_quaternion[1],
                z=transformed_quaternion[2],
                w=transformed_quaternion[3]
            )

            self.current_odom_state = transformed_odom
            self.odom_publisher.publish(transformed_odom)


    def localization_monitor_callback(self, msg):
        with self.lock:

            # Extract frames from the first message and set them
            if not self.global_frame:
                self.global_frame = msg.pose_global.header.frame_id
            if not self.local_frame:
                self.local_frame = msg.pose_local.header.frame_id

            features = msg.float_array

            # Store the message in the data list
            self.data_list.append(msg)

            # Add global pose to the global path
            global_pose_stamped = PoseStamped()
            global_pose_stamped.header = msg.pose_global.header
            global_pose_stamped.pose = msg.pose_global.pose
            self.global_path.poses.append(global_pose_stamped)
            self.global_poses.append(odom_to_se3(msg.pose_global.pose))
            self.time_stamp_list.append(msg.pose_global.header.stamp)

            # Add local pose to the local path
            local_pose_stamped = PoseStamped()
            local_pose_stamped.header = msg.pose_local.header
            local_pose_stamped.pose = msg.pose_local.pose
            self.local_path.poses.append(local_pose_stamped)
            self.local_poses.append(odom_to_se3(msg.pose_local.pose))

            self.error_trans_avg.append(features[0])

            #if self.current_transform is None:
            self.distances_current_estimate.append(0.0)
            #else:
                #self.distances_current_estimate.append(euclidean_distance(msg.pose_global.pose, self.current_odom_state.pose.pose))

            rospy.loginfo("Added new global and local poses to paths with error %f^, distance to current estimate %f", self.error_trans_avg[-1], self.distances_current_estimate[-1])
            
            self.clean_up()
            if(len(self.data_list) > 3):
                self.align_local_to_global_odom_and_calc_errors(msg.pose_global.header)
                self.publish_paths()
                self.used_length = self.data_list[-1].cumulative_length - self.data_list[0].cumulative_length

    def clean_up(self):
        length = self.data_list[-1].cumulative_length - self.data_list[0].cumulative_length
        rospy.loginfo(length)
        while (self.data_list[-1].cumulative_length - self.data_list[0].cumulative_length) > 150:
            self.data_list.pop(0)
            self.time_stamp_list.pop(0)
            self.global_path.poses.pop(0)
            self.global_poses.pop(0)
            self.local_poses.pop(0)
            self.local_path.poses.pop(0)
            self.error_trans_avg.pop(0)
            self.distances_current_estimate.pop(0)



    def align_local_to_global_odom_and_calc_errors(self, header):
        rospy.loginfo("Align local´odom path to global odom path")
        self.local_odom_path = Path()

        local_trajectory = PoseTrajectory3D(poses_se3=np.copy(self.local_poses), timestamps=self.time_stamp_list)
        local_trajectory_no_snake = PoseTrajectory3D(poses_se3=np.copy(self.local_poses), timestamps=self.time_stamp_list)
        global_trajectory = PoseTrajectory3D(poses_se3=np.copy(self.global_poses), timestamps=self.time_stamp_list)

        #error_mean = np.square(np.mean(self.error_trans_avg))
        #error_distance_squared = np.square(np.array(self.distances_current_estimate))
        #error_distance_squared_mean = np.mean(error_distance_squared)

        penalty_values = np.array(self.error_trans_avg)
        #if(self.used_length > 30.0):
        #    penalty_values += error_distance_squared

        ref_value =  np.percentile(penalty_values, 25)

        weights = gaussian_weight(penalty_values, ref_value)
        rospy.loginfo("ref_val: %f, min weight: %f, max weigth: %f", ref_value, np.min(weights), np.max(weights))
        self.current_transform = snake_alignment(local_trajectory, global_trajectory, weights=weights)

        local_trajectory_no_snake.transform(self.current_transform)
        self.local_path_transformed.poses.clear()      
        for i, pose_se3 in enumerate(local_trajectory_no_snake.poses_se3):
            pose_stamped = se3_to_posestamped(pose_se3, self.time_stamp_list[i], z_to_zero=False)
            pose_stamped.header = header
            pose_stamped.header.frame_id = self.global_frame
            self.local_path_transformed.poses.append(pose_stamped)

    def publish_paths(self):
        # Update path headers with the correct frames and publish
        self.global_path.header.frame_id = self.global_frame
        self.global_path.header.stamp = rospy.Time.now()
        self.global_path_pub.publish(self.global_path)
        
        self.local_path_transformed.header.frame_id = self.global_frame
        self.local_path_transformed.header.stamp = rospy.Time.now()
        self.local_path_transformed_pub.publish(self.local_path_transformed)

        rospy.loginfo("Published global and local paths with %d and %d poses", len(self.global_path.poses), len(self.local_path.poses))

if __name__ == '__main__':
    try:
        node = AlignmentBasedFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AlignmentBasedFilterNode has been interrupted.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
