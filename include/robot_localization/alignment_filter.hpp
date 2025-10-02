#ifndef ROBOT_LOCALIZATION__ALIGNMENT_FILTER_HPP_
#define ROBOT_LOCALIZATION__ALIGNMENT_FILTER_HPP_

#include <array>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace robot_localization
{

class AlignmentFilter : public rclcpp::Node
{
public:
  explicit AlignmentFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void initialize();

private:
  void referenceCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void subjectCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void evaluateAlignment();
  void publishTransform(const geometry_msgs::msg::TransformStamped & transform);
  void publishAlignedOdometry(const nav_msgs::msg::Odometry & aligned);

  geometry_msgs::msg::TransformStamped computeAlignmentTransform(
    const nav_msgs::msg::Odometry & reference,
    const nav_msgs::msg::Odometry & subject) const;

  nav_msgs::msg::Odometry computeAlignedOdometry(
    const nav_msgs::msg::Odometry & subject,
    const geometry_msgs::msg::TransformStamped & transform) const;

  void rotateCovariance(
    const geometry_msgs::msg::Quaternion & rotation,
    std::array<double, 36> & covariance) const;
  void rotateTwistCovariance(
    const geometry_msgs::msg::Quaternion & rotation,
    std::array<double, 36> & covariance) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subject_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::Odometry::SharedPtr latest_reference_;
  nav_msgs::msg::Odometry::SharedPtr latest_subject_;

  std::string reference_topic_;
  std::string subject_topic_;
  std::string aligned_topic_;
  std::string transform_topic_;
  std::string fixed_frame_id_;
  std::string moving_frame_id_;

  bool publish_aligned_odometry_;
  bool publish_tf_;
  bool publish_transform_topic_;

  double timeout_sec_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__ALIGNMENT_FILTER_HPP_
