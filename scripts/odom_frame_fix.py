#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

class OdomChildFrameModifier:
    def __init__(self):
        rospy.init_node('odom_child_frame_fix')

        # Parameters
        self.new_child_frame = rospy.get_param("~new_child_frame_id", "base_link")
        self.input_topic = rospy.get_param("~input_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/odom_modified")

        # Publisher and Subscriber
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.callback)

        rospy.loginfo("Subscribed to %s, publishing modified messages to %s with child_frame_id = '%s'",
                      self.input_topic, self.output_topic, self.new_child_frame)

    def callback(self, msg: Odometry):
        msg.child_frame_id = self.new_child_frame
        msg.twist.covariance =[ 0.7,  0.0, 0.0,  0.0, 0.0, 0.0,
                                0.0,  0.2, 0.0,  0.0, 0.0, 0.0,
                                0.0,  0.0, 0.2,  0.0, 0.0, 0.0,
                                0.0,  0.0, 0.0,  0.1, 0.0, 0.0,
                                0.0,  0.0, 0.0,  0.0, 0.1, 0.0,
                                0.0,  0.0, 0.0,  0.0, 0.0, 0.1]


        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        OdomChildFrameModifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
