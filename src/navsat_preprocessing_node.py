#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped, Quaternion
from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
import tf.transformations as tf_transform
from enum import Enum

#class State(Enum):
#    ERROR = -1
#    WAIT_FOR_GPS = 0
#    WAIT_FOR_EXTERNA_ATTITDUE = 1
#    WAIT_FOR_EXTERNAL_ORIENTATION = 2
#    WAIT_FOR_HEADING_CALCULATIION = 3
#    READY = 10

class NavsatPreprocessingNode:
    def __init__(self):
        rospy.init_node('navsat_preprocessing_node')
        rospy.loginfo("Initializing navsat preprocessing node node...")

        #self.orientation_source_frame = rospy.get_param('~orientation_source_frame', 'imu_frame')
        #self.gps_source_frame = rospy.get_param('~gps_source_frame', 'gps_frame')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')
        self.use_external_heading = rospy.get_param('~use_external_heading', True)
        self.calculate_heading_from_trajectory = rospy.get_param('~calculate_heading_from_trajectory', False)
        self.overwrite_covariance_matrix = rospy.get_param('~overwrite_covariance_matrix', False)
        self.heading_reference_length = rospy.get_param('~heading_reference_length', 10)
        self.publish_imu_message_with_orientation = rospy.get_param('~publish_imu_message_with_orientation', False)

        # Load covariance matrix parameter (fallback to default if not provided)
        self.pose_covariance = [1000, 0.0, 0.0,  0.0, 0.0, 0.0,
                                0.0, 1000, 0.0,  0.0, 0.0, 0.0,
                                0.0,  0.0, 1500, 0.0, 0.0, 0.0,
                                0.0,  0.0, 0.0,  0.1, 0.0, 0.0,
                                0.0,  0.0, 0.0,  0.0, 0.1, 0.0,
                                0.0,  0.0, 0.0,  0.0, 0.0, 0.2]

        # Subscribe to the odometry and orientation topics
        rospy.Subscriber('/localization/preprocessing/input/gps_odometry', Odometry, self.odom_callback)

        if self.use_external_heading:
            rospy.Subscriber('/localization/preprocessing/input/orientation_with_global_heading', QuaternionStamped, self.orientation_with_heading_callback)

        elif self.calculate_heading_from_trajectory:
            rospy.logerr("Heading_from_trajectory not implented yet")
            #rospy.Subscriber('/localization/preprocessing/input/orientation', QuaternionStamped, self.orientation_callback)
            #rospy.Subscriber('/localization/preprocessing/input/local_reference', Odometry, self.local_odom_callback)
        

        # Publishers for combined odometry/orientation and IMU messages
        self.odom_with_pose_pub = rospy.Publisher('/localization/preprocessing/output/odometry', Odometry, queue_size=10)
        #self.stat_pub = rospy.Publisher('/localization/preprocessing/state', Int8, queue_size=10)

        if(self.publish_imu_with_orientation):
            self.imu_pub = rospy.Publisher('/localization/preprocessing/output/imu_with_fix_cov', Imu, queue_size=10)

        # TF listener to get the static transform
        self.tf_listener = tf.TransformListener()
        
        # Placeholder for the latest received odometry and orientation messages
        self.latest_odom = None
        self.latest_orientation = None

        # Placeholder for the static transform matrix
        self.orientation_transform = "Not used for now, every thing related to geokombi, needs to be changed LOL"

    #def get_transform_orientation(self):
    #    return


    #def get_transform_odom(self):
    #    return

    def odom_callback(self, msg):
        # Store only do if orientation is available
        self.latest_odom = msg
        if(self.latest_orientation is None or self.orientation_transform is None):
            return
        self.publish_odom_with_pose_and_covariance()


    def orientation_with_heading_callback(self, msg):
        # Store the latest orientation message
        self.latest_orientation = msg

        if(self.publish_imu_message_with_orientation):
            self.publish_imu_with_orientation()


    def apply_static_transform(self, quaternion):   
        # Convert the orientation quaternion to a matrix
        #orientation_matrix = tf_transform.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Apply the stored transform
        #transformed_matrix = tf_transform.concatenate_matrices(self.base_link_transform, orientation_matrix)
        
        # Convert back to quaternion
        #transformed_quaternion = tf_transform.quaternion_from_matrix(transformed_matrix)

        # Convert the input quaternion to a 4x4 rotation matrix
        rotation_matrix = tf_transform.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    
        # Define a 180-degree rotation around the Z-axis in the local coordinate system
        local_z_rotation = tf_transform.quaternion_matrix([0, 0, 0, 1])#tf_transform.quaternion_matrix([0, 0, 1, 0])  # 180 degrees around Z-axis
    
        # Apply the rotation in the local frame by post-multiplying the local Z rotation
        # This rotates the orientation in its own local frame
        rotated_matrix = tf_transform.concatenate_matrices(rotation_matrix, local_z_rotation)
    
        # Convert the resulting matrix back to a quaternion
        rotated_quaternion = tf_transform.quaternion_from_matrix(rotated_matrix)
    
        # Return the rotated quaternion as a ROS Quaternion message
        return Quaternion(*rotated_quaternion)


    #def orientation_callback(self, msg):
    #    return
    

    #def local_odom_callback(self, msg):
    #    return


    #def apply_static_transform(self, quaternion):
    #    if self.base_link_transform is None:
    #        rospy.logwarn("Static transform is not available. Using original quaternion.")
    #        return quaternion  # If no transform, return the original quaternion
        
        # Convert the orientation quaternion to a matrix
    #    rotation_matrix = tf_transform.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
    #    local_z_rotation = tf_transform.quaternion_matrix(self.base_link_transform)#[0, 0, 1, 0])  # 180 degrees around Z-axis
    
        # Apply the rotation in the local frame by post-multiplying the local Z rotation
        # This rotates the orientation in its own local frame
    #    rotated_matrix = tf_transform.concatenate_matrices(rotation_matrix, local_z_rotation)
    
        # Convert the resulting matrix back to a quaternion
    #    rotated_quaternion = tf_transform.quaternion_from_matrix(rotated_matrix)
    
        # Return the rotated quaternion as a ROS Quaternion message
    #    return Quaternion(*rotated_quaternion)
        #return Quaternion(*transformed_quaternion)

    def publish_odom_with_pose_and_covariance(self):
        # Check if both odometry and orientation have been received
        if self.latest_odom is None:
            return
        
        # Copy the odometry message
        odom_with_pose = self.latest_odom
        
        # Update the orientation and set the fixed covariance in the odometry message
        odom_with_pose.header.stamp = self.latest_odom.header.stamp  # Use the timestamp from the odometry message
        odom_with_pose.child_frame_id = "geo_kombi_link"
        odom_with_pose.pose.covariance = self.pose_covariance
        odom_with_pose.pose.pose.orientation = self.apply_static_transform(self.latest_orientation.quaternion)
        
        # Publish the modified odometry message
        self.odom_with_pose_pub.publish(odom_with_pose)

    def publish_imu_with_orientation(self):
        # Check if orientation has been received
        if self.latest_orientation is None:
            return

        # Create an Imu message with the orientation in its original frame
        imu_msg = Imu()
        imu_msg.header.stamp = self.latest_orientation.header.stamp  # Use the timestamp from the orientation message
        imu_msg.header.frame_id = self.latest_orientation.header.frame_id  # Use the original frame of the orientation topic
        imu_msg.orientation = self.latest_orientation.quaternion
        imu_msg.orientation_covariance = self.pose_covariance[21:24] + self.pose_covariance[27:30] + self.pose_covariance[33:36]
        #rospy.loginfo(self.latest_orientation.header.frame_id)

        # Publish the IMU message
        self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    try:
        node = NavsatPreprocessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("navsat_preprocessing_node terminated.")
