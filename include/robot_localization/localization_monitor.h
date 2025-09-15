#ifndef ROBOT_LOCALIZATION_LOCALIZATION_MONITOR_H
#define ROBOT_LOCALIZATION_LOCALIZATION_MONITOR_H

#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

namespace RobotLocalization
{
class LocalizationMonitor
{
public:
  LocalizationMonitor();

  void process(const nav_msgs::Odometry& odom);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_LOCALIZATION_MONITOR_H
