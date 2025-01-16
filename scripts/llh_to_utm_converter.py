#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from pyproj import Proj, transform

class LLHToUTMConverter:
    def __init__(self):
        rospy.init_node('llh_to_utm_converter', anonymous=True)

        # Subscribe to LLH Odometry topic
        rospy.Subscriber('/fixposition/odometry_llh', NavSatFix, self.llh_callback)

        # Publisher for UTM Odometry
        self.utm_pub = rospy.Publisher('/localization/fixposition/odom_utm', Odometry, queue_size=10)


    def llh_callback(self, msg):
        # Extract latitude, longitude, and height from the message
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        # Convert LLH to UTM
        zone = int((lon + 180) // 6) + 1
        south = lat < 0  # Check if in the southern hemisphere
        self.utm_proj = Proj(proj="utm", zone=zone, ellps="WGS84", south=south)
        
        easting, northing = self.utm_proj(lon, lat)

        # Create a new Odometry message with UTM coordinates
        utm_msg = Odometry()
        utm_msg.header.stamp = rospy.Time.now()
        utm_msg.header.frame_id = "utm"  # Fixed world frame (UTM coordinates)
        utm_msg.child_frame_id = "base_link"  # Robot's moving frame

        # Assign UTM coordinates
        utm_msg.pose.pose.position.x = easting
        utm_msg.pose.pose.position.y = northing
        utm_msg.pose.pose.position.z = alt  # Keep altitude unchanged

        # Preserve orientation
        utm_msg.pose.pose.orientation.w = 1.0  # Identity quaternion

        # Publish the UTM message
        self.utm_pub.publish(utm_msg)

if __name__ == '__main__':
    try:
        converter = LLHToUTMConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
