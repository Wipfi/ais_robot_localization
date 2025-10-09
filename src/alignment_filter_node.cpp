#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "ais_robot_localization/alignment_filter.hpp"
#include "ais_robot_localization/se3_algorithms.hpp"
#include "robot_localization/msg/localization_monitor_result.hpp"

namespace ais_robot_localization
{
namespace
{

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

Eigen::Isometry3d projectToXYPlane(const Eigen::Isometry3d & transform)
{
  Eigen::Isometry3d projected = Eigen::Isometry3d::Identity();
  projected.translation().x() = transform.translation().x();
  projected.translation().y() = transform.translation().y();
  projected.translation().z() = 0.0;

  const Eigen::Matrix3d & rotation = transform.linear();
  double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  projected.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return projected;
}

void projectTrajectoryToXYPlane(std::vector<Eigen::Isometry3d> & trajectory)
{
  for (auto & pose : trajectory) {
    pose = projectToXYPlane(pose);
  }
}

Eigen::Isometry3d poseMsgToIsometry(const geometry_msgs::msg::Pose & pose_msg)
{
  Eigen::Vector3d position(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  Eigen::Quaterniond orientation = toEigenQuaternion(pose_msg.orientation);
  return odomToSE3(position, orientation);
}

geometry_msgs::msg::PoseStamped toRosPoseStamped(
  const PoseStamped & pose,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.position.x = pose.position.x();
  msg.pose.position.y = pose.position.y();
  msg.pose.position.z = pose.position.z();
  msg.pose.orientation.x = pose.orientation.x();
  msg.pose.orientation.y = pose.orientation.y();
  msg.pose.orientation.z = pose.orientation.z();
  msg.pose.orientation.w = pose.orientation.w();
  return msg;
}

geometry_msgs::msg::Pose toRosPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose_msg;
  Eigen::Quaterniond q(transform.linear());
  q.normalize();
  pose_msg.position.x = transform.translation().x();
  pose_msg.position.y = transform.translation().y();
  pose_msg.position.z = transform.translation().z();
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();
  return pose_msg;
}

rclcpp::Time secondsToTime(double stamp_sec)
{
  return rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9));
}

}  // namespace

class AlignmentFilterNode : public rclcpp::Node
{
public:
  AlignmentFilterNode()
  : rclcpp::Node("alignment_filter_node"),
    publish_tf_(this->declare_parameter("publish_tf", true)),
    two_d_mode_(this->declare_parameter("2D_mode", false)),
    current_transform_(Eigen::Isometry3d::Identity())
  {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    local_frame_ = this->declare_parameter<std::string>("local_frame", "odom");

    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("alignment_global_path", 10);
    local_path_transformed_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("alignment_local_path_transformed", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("alignment_odometry", 10);

    localization_result_sub_ = this->create_subscription<robot_localization::msg::LocalizationMonitorResult>(
      "localization_result", 50,
      std::bind(&AlignmentFilterNode::localizationMonitorCallback, this, std::placeholders::_1));
    local_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "local_odom", rclcpp::SensorDataQoS(),
      std::bind(&AlignmentFilterNode::localOdomCallback, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&AlignmentFilterNode::publishTransformTimer, this));
    }

    RCLCPP_INFO(this->get_logger(), "AlignmentFilterNode initialized");
  }

private:
  void localizationMonitorCallback(
    const robot_localization::msg::LocalizationMonitorResult::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!msg->pose_global.header.frame_id.empty()) {
      global_frame_ = msg->pose_global.header.frame_id;
    }
    if (!msg->pose_local.header.frame_id.empty()) {
      local_frame_ = msg->pose_local.header.frame_id;
    }

    global_path_.poses.push_back(msg->pose_global);
    local_path_.poses.push_back(msg->pose_local);

    Eigen::Isometry3d global_pose = poseMsgToIsometry(msg->pose_global.pose);
    Eigen::Isometry3d local_pose = poseMsgToIsometry(msg->pose_local.pose);
    double timestamp_sec = rclcpp::Time(msg->pose_global.header.stamp).seconds();
    double cumulative_length = msg->cumulative_length;
    double trans_error = msg->float_array.empty() ? 0.0 : static_cast<double>(msg->float_array.front());

    std::size_t removed = filter_.addMeasurement(global_pose, local_pose, timestamp_sec, trans_error,
        cumulative_length);

    if (removed > 0) {
      if (global_path_.poses.size() > removed) {
        global_path_.poses.erase(global_path_.poses.begin(), global_path_.poses.begin() + removed);
      } else {
        global_path_.poses.clear();
      }

      if (local_path_.poses.size() > removed) {
        local_path_.poses.erase(local_path_.poses.begin(), local_path_.poses.begin() + removed);
      } else {
        local_path_.poses.clear();
      }
    }

    if (filter_.hasSufficientData()) {
      AlignmentResult result;
      if (filter_.computeAlignment(result)) {
        Eigen::Isometry3d transform_to_use = two_d_mode_ ? projectToXYPlane(result.transform) : result.transform;
        if (two_d_mode_) {
          projectTrajectoryToXYPlane(result.transformed_local);
        }
        current_transform_ = transform_to_use;
        updateTransformedPath(result.transformed_local, filter_.timestamps(), msg->pose_global.header);
        publishPaths();
      }
    }
  }

  void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation = toEigenQuaternion(msg->pose.pose.orientation);
    Eigen::Isometry3d current_pose = odomToSE3(position, orientation);
    Eigen::Isometry3d transformed_pose = current_transform_ * current_pose;
    if (two_d_mode_) {
      transformed_pose = projectToXYPlane(transformed_pose);
    }

    auto transformed_msg = *msg;
    transformed_msg.header.frame_id = global_frame_;
    transformed_msg.pose.pose = toRosPose(transformed_pose);
    transformed_msg.pose.covariance = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    };

    odom_pub_->publish(transformed_msg);
  }

  void updateTransformedPath(
    const std::vector<Eigen::Isometry3d> & transformed_local,
    const std::vector<double> & time_stamps,
    const std_msgs::msg::Header & header)
  {
    local_path_transformed_.poses.clear();
    local_path_transformed_.header.frame_id = global_frame_;
    local_path_transformed_.header.stamp = this->now();

    for (std::size_t i = 0; i < transformed_local.size(); ++i) {
      double stamp = (i < time_stamps.size()) ? time_stamps[i] : rclcpp::Time(header.stamp).seconds();
      PoseStamped pose = se3ToPoseStamped(transformed_local[i], stamp, two_d_mode_);
      local_path_transformed_.poses.push_back(
        toRosPoseStamped(pose, global_frame_, secondsToTime(stamp)));
    }
  }

  void publishPaths()
  {
    rclcpp::Time now = this->now();
    global_path_.header.frame_id = global_frame_;
    global_path_.header.stamp = now;
    local_path_transformed_.header.stamp = now;

    global_path_pub_->publish(global_path_);
    local_path_transformed_pub_->publish(local_path_transformed_);
  }

  void publishTransformTimer()
  {
    if (!publish_tf_ || !tf_buffer_ || !tf_listener_ || !tf_broadcaster_) {
      return;
    }

    try {
      geometry_msgs::msg::TransformStamped latest_tf = tf_buffer_->lookupTransform(
        local_frame_, "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = latest_tf.header.stamp;
      transform_msg.header.frame_id = global_frame_;
      transform_msg.child_frame_id = local_frame_;

      Eigen::Quaterniond q(current_transform_.linear());
      q.normalize();
      Eigen::Vector3d t = current_transform_.translation();

      transform_msg.transform.translation.x = t.x();
      transform_msg.transform.translation.y = t.y();
      transform_msg.transform.translation.z = t.z();
      transform_msg.transform.rotation.x = q.x();
      transform_msg.transform.rotation.y = q.y();
      transform_msg.transform.rotation.z = q.z();
      transform_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transform_msg);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error publishing transform: %s", ex.what());
    }
  }

  bool publish_tf_;
  bool two_d_mode_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_transformed_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<robot_localization::msg::LocalizationMonitorResult>::SharedPtr
    localization_result_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path local_path_;
  nav_msgs::msg::Path local_path_transformed_;

  std::string global_frame_;
  std::string local_frame_;

  AlignmentFilter filter_;
  Eigen::Isometry3d current_transform_;

  std::mutex mutex_;
};

}  // namespace ais_robot_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ais_robot_localization::AlignmentFilterNode>());
  rclcpp::shutdown();
  return 0;
}

