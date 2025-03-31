import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odometry_callback(msg):
    try:
        # Create a TransformListener
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        
        # Wait for transform from odom to utm
        transform = tf_buffer.lookup_transform("utm", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        # Transform the odometry pose
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        
        # Overwrite frame_id and pose in the original message
        msg.header.frame_id = "utm"
        msg.pose.pose = transformed_pose.pose
        
        odom_pub.publish(msg)
    except tf2_ros.LookupException as e:
        rospy.logwarn("Could not find transform: {}".format(e))
    except tf2_ros.ExtrapolationException as e:
        rospy.logwarn("Extrapolation Exception: {}".format(e))

if __name__ == '__main__':
    rospy.init_node('odom_to_utm_transformer', anonymous=True)
    
    # Publisher for transformed odometry
    odom_pub = rospy.Publisher('/localization/fusion_utm', Odometry, queue_size=10)
    
    # Subscriber to original odometry topic
    rospy.Subscriber('/odometry/raw', Odometry, odometry_callback)
    
    rospy.spin()
