#include "robot_localization/localization_monitor.h"

#include <eigen_conversions/eigen_msg.h>

namespace RobotLocalization
{
LocalizationMonitor::LocalizationMonitor()
  : tf_listener_(tf_buffer_)
{
}

void LocalizationMonitor::process(const nav_msgs::Odometry& /*odom*/)
{
  // Placeholder implementation
}
}  // namespace RobotLocalization
