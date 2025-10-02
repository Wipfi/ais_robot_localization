#include "robot_localization/alignment_filter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_localization
{

namespace
{
constexpr double kDefaultTimeout = 1.0;
}  // namespace

AlignmentFilter::AlignmentFilter(const rclcpp::NodeOptions & options)
: rclcpp::Node("alignment_filter", options),
  publish_aligned_odometry_(true),
  publish_tf_(true),
  publish_transform_topic_(false),
  timeout_sec_(kDefaultTimeout)
{
  reference_topic_ = this->declare_parameter<std::string>("reference_topic", "map_odometry");
  subject_topic_ = this->declare_parameter<std::string>("subject_topic", "odom");
  aligned_topic_ = this->declare_parameter<std::string>("aligned_topic", "odometry/aligned");
  transform_topic_ = this->declare_parameter<std::string>("transform_topic", "alignment_transform");
  fixed_frame_id_ = this->declare_parameter<std::string>("fixed_frame_id", "map");
  moving_frame_id_ = this->declare_parameter<std::string>("moving_frame_id", "odom");
  publish_aligned_odometry_ = this->declare_parameter<bool>("publish_aligned_odometry", true);
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
  publish_transform_topic_ = this->declare_parameter<bool>("publish_transform_topic", false);
  timeout_sec_ = this->declare_parameter<double>("timeout", kDefaultTimeout);

  reference_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    reference_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&AlignmentFilter::referenceCallback, this, std::placeholders::_1));

  subject_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    subject_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&AlignmentFilter::subjectCallback, this, std::placeholders::_1));

  if (publish_aligned_odometry_) {
    aligned_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      aligned_topic_, rclcpp::QoS(10));
  }

  if (publish_transform_topic_) {
    transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      transform_topic_, rclcpp::QoS(10));
  }
}

void AlignmentFilter::initialize()
{
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  }
}

void AlignmentFilter::referenceCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_reference_ = msg;
  evaluateAlignment();
}

void AlignmentFilter::subjectCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_subject_ = msg;
  evaluateAlignment();
}

void AlignmentFilter::evaluateAlignment()
{
  if (!latest_reference_ || !latest_subject_) {
    return;
  }

  const rclcpp::Time now = this->now();
  if (timeout_sec_ > 0.0) {
    const double reference_age = (now - latest_reference_->header.stamp).seconds();
    const double subject_age = (now - latest_subject_->header.stamp).seconds();
    if (reference_age > timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Alignment filter reference data timeout: %.2f seconds", reference_age);
      return;
    }
    if (subject_age > timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Alignment filter subject data timeout: %.2f seconds", subject_age);
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform = computeAlignmentTransform(
    *latest_reference_, *latest_subject_);

  publishTransform(transform);

  if (publish_aligned_odometry_) {
    nav_msgs::msg::Odometry aligned = computeAlignedOdometry(*latest_subject_, transform);
    publishAlignedOdometry(aligned);
  }
}

geometry_msgs::msg::TransformStamped AlignmentFilter::computeAlignmentTransform(
  const nav_msgs::msg::Odometry & reference,
  const nav_msgs::msg::Odometry & subject) const
{
  tf2::Transform reference_tf;
  tf2::Transform subject_tf;
  tf2::fromMsg(reference.pose.pose, reference_tf);
  tf2::fromMsg(subject.pose.pose, subject_tf);

  tf2::Transform alignment_tf = reference_tf * subject_tf.inverse();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = subject.header.stamp;
  transform.header.frame_id = fixed_frame_id_.empty() ? reference.header.frame_id : fixed_frame_id_;
  transform.child_frame_id = moving_frame_id_.empty() ? subject.header.frame_id : moving_frame_id_;
  transform.transform = tf2::toMsg(alignment_tf);

  return transform;
}

nav_msgs::msg::Odometry AlignmentFilter::computeAlignedOdometry(
  const nav_msgs::msg::Odometry & subject,
  const geometry_msgs::msg::TransformStamped & transform) const
{
  tf2::Transform subject_tf;
  tf2::fromMsg(subject.pose.pose, subject_tf);

  tf2::Transform alignment_tf;
  tf2::fromMsg(transform.transform, alignment_tf);

  tf2::Transform aligned_tf = alignment_tf * subject_tf;

  nav_msgs::msg::Odometry aligned = subject;
  aligned.header.frame_id = transform.header.frame_id;
  aligned.pose.pose = tf2::toMsg(aligned_tf);
  aligned.pose.covariance = subject.pose.covariance;
  rotateCovariance(transform.transform.rotation, aligned.pose.covariance);

  tf2::Vector3 linear(subject.twist.twist.linear.x, subject.twist.twist.linear.y,
    subject.twist.twist.linear.z);
  tf2::Vector3 angular(subject.twist.twist.angular.x, subject.twist.twist.angular.y,
    subject.twist.twist.angular.z);
  const tf2::Matrix3x3 basis = alignment_tf.getBasis();
  linear = basis * linear;
  angular = basis * angular;

  aligned.twist.twist.linear.x = linear.x();
  aligned.twist.twist.linear.y = linear.y();
  aligned.twist.twist.linear.z = linear.z();
  aligned.twist.twist.angular.x = angular.x();
  aligned.twist.twist.angular.y = angular.y();
  aligned.twist.twist.angular.z = angular.z();
  aligned.twist.covariance = subject.twist.covariance;
  rotateTwistCovariance(transform.transform.rotation, aligned.twist.covariance);

  return aligned;
}

void AlignmentFilter::rotateCovariance(
  const geometry_msgs::msg::Quaternion & rotation,
  std::array<double, 36> & covariance) const
{
  Eigen::Matrix<double, 6, 6> cov =
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(covariance.data());

  tf2::Quaternion tf_quat;
  tf2::fromMsg(rotation, tf_quat);
  tf_quat.normalize();

  Eigen::Quaterniond eigen_quat(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());
  Eigen::Matrix3d rotation_matrix = eigen_quat.toRotationMatrix();

  Eigen::Matrix<double, 6, 6> rotation_matrix6 = Eigen::Matrix<double, 6, 6>::Identity();
  rotation_matrix6.block<3, 3>(0, 0) = rotation_matrix;
  rotation_matrix6.block<3, 3>(3, 3) = rotation_matrix;

  Eigen::Matrix<double, 6, 6> rotated = rotation_matrix6 * cov * rotation_matrix6.transpose();

  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(covariance.data()) = rotated;
}

void AlignmentFilter::rotateTwistCovariance(
  const geometry_msgs::msg::Quaternion & rotation,
  std::array<double, 36> & covariance) const
{
  rotateCovariance(rotation, covariance);
}

void AlignmentFilter::publishTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  if (publish_tf_ && tf_broadcaster_) {
    tf_broadcaster_->sendTransform(transform);
  }

  if (publish_transform_topic_ && transform_pub_) {
    transform_pub_->publish(transform);
  }
}

void AlignmentFilter::publishAlignedOdometry(const nav_msgs::msg::Odometry & aligned)
{
  if (aligned_pub_) {
    aligned_pub_->publish(aligned);
  }
}

}  // namespace robot_localization

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization::AlignmentFilter)
