#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_localization/alignment_filter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robot_localization::AlignmentFilter>(options);
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
