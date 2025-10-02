#include "robot_localization/localization_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_localization
{

namespace
{
constexpr double kDefaultFrequency = 5.0;
constexpr double kDefaultTolerance = 1.0;
constexpr double kDefaultAngularTolerance = 0.5;  // radians
constexpr double kDefaultTimeout = 1.0;
}  // namespace

LocalizationMonitor::LocalizationMonitor(const rclcpp::NodeOptions & options)
: rclcpp::Node("localization_monitor", options),
  position_tolerance_(0.0),
  orientation_tolerance_(0.0),
  linear_velocity_tolerance_(0.0),
  angular_velocity_tolerance_(0.0),
  timeout_sec_(0.0),
  monitor_frequency_(0.0),
  publish_diagnostics_(false)
{
  target_topic_ = this->declare_parameter<std::string>("target_topic", "odometry/filtered");
  reference_topic_ = this->declare_parameter<std::string>("reference_topic", "ground_truth");
  diagnostic_name_ = this->declare_parameter<std::string>("diagnostic_name", "Localization monitor");
  hardware_id_ = this->declare_parameter<std::string>("hardware_id", "robot_localization");

  position_tolerance_ = this->declare_parameter<double>("position_tolerance", kDefaultTolerance);
  orientation_tolerance_ = this->declare_parameter<double>(
    "orientation_tolerance", kDefaultAngularTolerance);
  linear_velocity_tolerance_ = this->declare_parameter<double>(
    "linear_velocity_tolerance", kDefaultTolerance);
  angular_velocity_tolerance_ = this->declare_parameter<double>(
    "angular_velocity_tolerance", kDefaultAngularTolerance);
  timeout_sec_ = this->declare_parameter<double>("timeout", kDefaultTimeout);
  monitor_frequency_ = this->declare_parameter<double>("monitor_frequency", kDefaultFrequency);
  publish_diagnostics_ = this->declare_parameter<bool>("publish_diagnostics", true);

  if (monitor_frequency_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "Monitor frequency must be positive, resetting to %.2f Hz",
      kDefaultFrequency);
    monitor_frequency_ = kDefaultFrequency;
  }

  if (publish_diagnostics_) {
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(1));
  }

  target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    target_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&LocalizationMonitor::targetCallback, this, std::placeholders::_1));

  reference_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    reference_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&LocalizationMonitor::referenceCallback, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration<double>(1.0 / monitor_frequency_);
  timer_ = this->create_wall_timer(timer_period, std::bind(&LocalizationMonitor::onTimer, this));
}

void LocalizationMonitor::targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  target_state_ = msg;
}

void LocalizationMonitor::referenceCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  reference_state_ = msg;
}

void LocalizationMonitor::onTimer()
{
  if (!target_state_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticStatus status_msg;
  status_msg.name = diagnostic_name_;
  status_msg.hardware_id = hardware_id_;
  status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status_msg.message = "Localization is within tolerance";
  status_msg.values.reserve(4);

  const auto now = this->now();
  if (timeout_sec_ > 0.0) {
    const double target_age = (now - target_state_->header.stamp).seconds();
    if (target_age > timeout_sec_) {
      status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status_msg.message = "Target localization data timeout";
      status_msg.values.emplace_back();
      status_msg.values.back().key = "target_age";
      status_msg.values.back().value = std::to_string(target_age);
      publishDiagnostic(status_msg, status_msg.message);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Localization monitor did not receive target data for %.2f seconds", target_age);
      return;
    }

    if (reference_state_) {
      const double reference_age = (now - reference_state_->header.stamp).seconds();
      if (reference_age > timeout_sec_) {
        status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Reference localization data timeout";
        status_msg.values.emplace_back();
        status_msg.values.back().key = "reference_age";
        status_msg.values.back().value = std::to_string(reference_age);
        publishDiagnostic(status_msg, status_msg.message);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 2000,
          "Localization monitor did not receive reference data for %.2f seconds", reference_age);
        return;
      }
    }
  }

  if (!reference_state_) {
    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status_msg.message = "Waiting for reference localization";
    publishDiagnostic(status_msg, status_msg.message);
    return;
  }

  const double position_error = computePositionError(*target_state_, *reference_state_);
  const double orientation_error = computeOrientationError(*target_state_, *reference_state_);
  const double linear_error = computeLinearVelocityError(*target_state_, *reference_state_);
  const double angular_error = computeAngularVelocityError(*target_state_, *reference_state_);

  status_msg.values.emplace_back();
  status_msg.values.back().key = "position_error";
  status_msg.values.back().value = std::to_string(position_error);

  status_msg.values.emplace_back();
  status_msg.values.back().key = "orientation_error";
  status_msg.values.back().value = std::to_string(orientation_error);

  status_msg.values.emplace_back();
  status_msg.values.back().key = "linear_velocity_error";
  status_msg.values.back().value = std::to_string(linear_error);

  status_msg.values.emplace_back();
  status_msg.values.back().key = "angular_velocity_error";
  status_msg.values.back().value = std::to_string(angular_error);

  auto check_threshold = [&](double value, double limit, const std::string & field) {
      if (limit >= 0.0 && value > limit) {
        status_msg.level = std::max(
          status_msg.level,
          diagnostic_msgs::msg::DiagnosticStatus::WARN);
        std::ostringstream stream;
        stream << field << " exceeds tolerance: " << value << " > " << limit;
        status_msg.message = stream.str();
      }
    };

  check_threshold(position_error, position_tolerance_, "Position error");
  check_threshold(orientation_error, orientation_tolerance_, "Orientation error");
  check_threshold(linear_error, linear_velocity_tolerance_, "Linear velocity error");
  check_threshold(angular_error, angular_velocity_tolerance_, "Angular velocity error");

  if (status_msg.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "Localization monitor detected deviation: position %.3f (tol %.3f), orientation %.3f (tol %.3f)",
      position_error, position_tolerance_, orientation_error, orientation_tolerance_);
  }

  publishDiagnostic(status_msg, status_msg.message);
}

double LocalizationMonitor::computePositionError(
  const nav_msgs::msg::Odometry & target,
  const nav_msgs::msg::Odometry & reference) const
{
  const double dx = target.pose.pose.position.x - reference.pose.pose.position.x;
  const double dy = target.pose.pose.position.y - reference.pose.pose.position.y;
  const double dz = target.pose.pose.position.z - reference.pose.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double LocalizationMonitor::computeOrientationError(
  const nav_msgs::msg::Odometry & target,
  const nav_msgs::msg::Odometry & reference) const
{
  tf2::Quaternion target_q;
  tf2::Quaternion reference_q;
  tf2::fromMsg(target.pose.pose.orientation, target_q);
  tf2::fromMsg(reference.pose.pose.orientation, reference_q);

  return reference_q.angularDistance(target_q);
}

double LocalizationMonitor::computeLinearVelocityError(
  const nav_msgs::msg::Odometry & target,
  const nav_msgs::msg::Odometry & reference) const
{
  const double dx = target.twist.twist.linear.x - reference.twist.twist.linear.x;
  const double dy = target.twist.twist.linear.y - reference.twist.twist.linear.y;
  const double dz = target.twist.twist.linear.z - reference.twist.twist.linear.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double LocalizationMonitor::computeAngularVelocityError(
  const nav_msgs::msg::Odometry & target,
  const nav_msgs::msg::Odometry & reference) const
{
  const double dx = target.twist.twist.angular.x - reference.twist.twist.angular.x;
  const double dy = target.twist.twist.angular.y - reference.twist.twist.angular.y;
  const double dz = target.twist.twist.angular.z - reference.twist.twist.angular.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void LocalizationMonitor::publishDiagnostic(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & summary_message)
{
  if (!publish_diagnostics_ || diagnostics_pub_ == nullptr) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray array_msg;
  array_msg.header.stamp = this->now();
  array_msg.status.push_back(status);
  if (array_msg.status.front().message.empty()) {
    array_msg.status.front().message = summary_message;
  }
  diagnostics_pub_->publish(array_msg);
}

}  // namespace robot_localization

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization::LocalizationMonitor)
