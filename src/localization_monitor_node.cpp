#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_localization/localization_monitor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robot_localization::LocalizationMonitor>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
