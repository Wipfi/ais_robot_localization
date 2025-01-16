#!/usr/bin/env python

import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import Vector3, Quaternion, Transform, Point, Pose

class TransformPublisherNode:
    def __init__(self, frame_to_connect_from, root_to_connect_to, target_frame, costmap_refernce_frame_ENU, costmap_reference_frame_z, costmap_frame):
        rospy.init_node('tf_tree_connector')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.frame_to_connect_from = frame_to_connect_from
        self.root_to_connect_to = root_to_connect_to
        self.target_frame = target_frame
        self.costmap_refernce_frame_ENU = costmap_refernce_frame_ENU
        self.costmap_reference_frame_z = costmap_reference_frame_z
        self.costmap_frame = costmap_frame

        self.rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            try:                
                # Get the transform from tree root (e.g. odom) to the mounting frame of the gnss module (e.g. FP_link)
                transform_root_to_mounting_point = self.tf_buffer.lookup_transform(self.target_frame,self.root_to_connect_to,rospy.Time())
 
                # Create resulting Transfomation to connect Trees
                Transform_to_connect_trees = geometry_msgs.msg.TransformStamped()
                Transform_to_connect_trees.header.frame_id = self.frame_to_connect_from
                Transform_to_connect_trees.child_frame_id = self.root_to_connect_to
                Transform_to_connect_trees.transform = transform_root_to_mounting_point.transform
                

                # Create resulting Transfomation for costmap
                transform_FP_POI_to_FP_ENU0 = self.tf_buffer.lookup_transform(self.costmap_refernce_frame_ENU, self.frame_to_connect_from, rospy.Time())
                transform_base_link_to_FP_LINK = self.tf_buffer.lookup_transform(self.target_frame, self.costmap_reference_frame_z, rospy.Time())
                
                Transform_costmap = geometry_msgs.msg.TransformStamped()
                Transform_costmap.header.frame_id = costmap_refernce_frame_ENU
                Transform_costmap.child_frame_id = costmap_frame
                Transform_costmap.transform.rotation.x = 0
                Transform_costmap.transform.rotation.y = 0
                Transform_costmap.transform.rotation.z = 0
                Transform_costmap.transform.rotation.w = 1
                Transform_costmap.transform.translation.z = transform_FP_POI_to_FP_ENU0.transform.translation.z + transform_base_link_to_FP_LINK.transform.translation.z


                # Set the timestamp if provided, otherwise use the current time
                Transform_to_connect_trees.header.stamp = rospy.Time.now()
                Transform_costmap.header.stamp = rospy.Time.now()

                # Broadcast the combined transform
                self.tf_broadcaster.sendTransform(Transform_to_connect_trees)
                self.tf_broadcaster.sendTransform(Transform_costmap)   
                

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Transform lookup failed.")

            self.rate.sleep()



if __name__ == '__main__':
    try:
        node = TransformPublisherNode(root_to_connect_to='map', target_frame='FP_link', frame_to_connect_from='FP_POI'\
        , costmap_refernce_frame_ENU="FP_ENU0", costmap_reference_frame_z="base_link", costmap_frame="cmr_costmap")
    except rospy.ROSInterruptException:
        pass
