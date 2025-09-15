#include "robot_localization/localization_monitor.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor");
  ros::NodeHandle nh;

  RobotLocalization::LocalizationMonitor monitor;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(
      "odometry", 1,
      [&monitor](const nav_msgs::Odometry::ConstPtr& msg){ monitor.process(*msg); });

  ros::spin();
  return 0;
}
