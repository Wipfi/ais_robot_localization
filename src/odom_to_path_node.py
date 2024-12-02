#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class OdomToPath:
    def __init__(self):
        rospy.init_node('odom_to_path_node', anonymous=True)
        self.odom_sub = rospy.Subscriber('/alignment_odometry', Odometry, self.odom_callback_gt)
        self.path_pub = rospy.Publisher('/path_alignment_odometry', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odometry/gps', Odometry, self.odom_callback_dlo)
        self.path_pub_dlo = rospy.Publisher('/path_gnss', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg_dlo = Path()

    def odom_callback_gt(self, odom_msg):
        #self.path_msg.header = odom_msg.header
        self.path_msg.header = odom_msg.header
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def odom_callback_dlo(self, odom_msg):
        #self.path_msg.header = odom_msg.header
        self.path_msg_dlo.header = odom_msg.header
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        self.path_msg_dlo.poses.append(pose)
        self.path_pub_dlo.publish(self.path_msg_dlo)

if __name__ == '__main__':
    try:
        odom_to_path = OdomToPath()
        rospy.spin()
        #plt.show()
    except rospy.ROSInterruptException:
        pass
