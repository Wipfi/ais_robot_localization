#ifndef ROBOT_LOCALIZATION__LOCALIZATION_MONITOR_HPP_
#define ROBOT_LOCALIZATION__LOCALIZATION_MONITOR_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_localization
{

class LocalizationMonitor : public rclcpp::Node
{
public:
  explicit LocalizationMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void referenceCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  double computePositionError(const nav_msgs::msg::Odometry & target,
    const nav_msgs::msg::Odometry & reference) const;
  double computeOrientationError(const nav_msgs::msg::Odometry & target,
    const nav_msgs::msg::Odometry & reference) const;
  double computeLinearVelocityError(const nav_msgs::msg::Odometry & target,
    const nav_msgs::msg::Odometry & reference) const;
  double computeAngularVelocityError(const nav_msgs::msg::Odometry & target,
    const nav_msgs::msg::Odometry & reference) const;

  void publishDiagnostic(
    const diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & summary_message);

  nav_msgs::msg::Odometry::SharedPtr target_state_;
  nav_msgs::msg::Odometry::SharedPtr reference_state_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::string target_topic_;
  std::string reference_topic_;
  std::string diagnostic_name_;
  std::string hardware_id_;

  double position_tolerance_;
  double orientation_tolerance_;
  double linear_velocity_tolerance_;
  double angular_velocity_tolerance_;
  double timeout_sec_;
  double monitor_frequency_;

  bool publish_diagnostics_;

};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__LOCALIZATION_MONITOR_HPP_
