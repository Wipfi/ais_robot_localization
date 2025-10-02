#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "ais_robot_localization/LocalizationMonitorResult.h"
#include "ais_robot_localization/alignment_filter.hpp"
#include "ais_robot_localization/se3_algorithms.hpp"

namespace ais_robot_localization {
namespace {

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::Quaternion& q_msg) {
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

Eigen::Isometry3d poseMsgToIsometry(const geometry_msgs::Pose& pose_msg) {
  Eigen::Vector3d position(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  Eigen::Quaterniond orientation = toEigenQuaternion(pose_msg.orientation);
  return odomToSE3(position, orientation);
}

geometry_msgs::PoseStamped toRosPoseStamped(const PoseStamped& pose, const std::string& frame_id, const ros::Time& stamp) {
  geometry_msgs::PoseStamped msg;
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

geometry_msgs::Pose toRosPose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose pose_msg;
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

}  // namespace

class AlignmentFilterNode {
 public:
  AlignmentFilterNode()
      : nh_(),
        private_nh_("~"),
        publish_tf_(private_nh_.param("publish_tf", true)),
        tf_buffer_(),
        tf_listener_(nullptr),
        current_transform_(Eigen::Isometry3d::Identity()) {
    global_frame_ = "map";
    local_frame_ = "odom";

    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/alignment_global_path", 10);
    local_path_transformed_pub_ = nh_.advertise<nav_msgs::Path>("/alignment_local_path_transformed", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/alignment_odometry", 10);

    localization_result_sub_ = nh_.subscribe("/localization_result", 50, &AlignmentFilterNode::localizationMonitorCallback, this);
    local_odom_sub_ = nh_.subscribe("/local_odom", 50, &AlignmentFilterNode::localOdomCallback, this);

    if (publish_tf_) {
      tf_buffer_.reset(new tf2_ros::Buffer());
      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
      tf_timer_ = nh_.createTimer(ros::Duration(0.2), &AlignmentFilterNode::publishTransformTimer, this);
    }

    ROS_INFO("AlignmentFilterNode initialized (CPP)");
  }

  void spin() const {
    ros::spin();
  }

 private:
  void localizationMonitorCallback(const ais_robot_localization::LocalizationMonitorResult::ConstPtr& msg) {
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
    double timestamp_sec = msg->pose_global.header.stamp.toSec();
    double cumulative_length = msg->cumulative_length;
    double trans_error = msg->float_array.empty() ? 0.0 : static_cast<double>(msg->float_array.front());

    std::size_t removed = filter_.addMeasurement(global_pose, local_pose, timestamp_sec, trans_error, cumulative_length);

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
        current_transform_ = result.transform;
        updateTransformedPath(result.transformed_local, filter_.timestamps(), msg->pose_global.header);
        publishPaths();
      }
    }
  }

  void localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation = toEigenQuaternion(msg->pose.pose.orientation);
    Eigen::Isometry3d current_pose = odomToSE3(position, orientation);
    Eigen::Isometry3d transformed_pose = current_transform_ * current_pose;

    nav_msgs::Odometry transformed_msg;
    transformed_msg = *msg;
    transformed_msg.header.frame_id = global_frame_;
    transformed_msg.pose.pose = toRosPose(transformed_pose);
    transformed_msg.pose.covariance = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1};

    odom_pub_.publish(transformed_msg);
  }

  void updateTransformedPath(const std::vector<Eigen::Isometry3d>& transformed_local,
                             const std::vector<double>& time_stamps,
                             const std_msgs::Header& header) {
    local_path_transformed_.poses.clear();
    local_path_transformed_.header.frame_id = global_frame_;
    local_path_transformed_.header.stamp = ros::Time::now();

    for (std::size_t i = 0; i < transformed_local.size(); ++i) {
      double stamp = (i < time_stamps.size()) ? time_stamps[i] : header.stamp.toSec();
      PoseStamped pose = se3ToPoseStamped(transformed_local[i], stamp, false);
      geometry_msgs::PoseStamped ros_pose = toRosPoseStamped(pose, global_frame_, ros::Time(stamp));
      local_path_transformed_.poses.push_back(ros_pose);
    }
  }

  void publishPaths() {
    ros::Time now = ros::Time::now();
    global_path_.header.frame_id = global_frame_;
    global_path_.header.stamp = now;
    local_path_transformed_.header.stamp = now;

    global_path_pub_.publish(global_path_);
    local_path_transformed_pub_.publish(local_path_transformed_);

    ROS_INFO("Published global and local transformed paths with %zu poses", global_path_.poses.size());
  }

  void publishTransformTimer(const ros::TimerEvent&) {
    if (!publish_tf_ || !tf_buffer_ || !tf_listener_) {
      return;
    }

    try {
      geometry_msgs::TransformStamped latest_tf = tf_buffer_->lookupTransform(local_frame_, "base_link", ros::Time(0), ros::Duration(1.0));
      geometry_msgs::TransformStamped transform_msg;
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

      tf_broadcaster_.sendTransform(transform_msg);
    } catch (const tf2::TransformException& ex) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Error publishing transform: " << ex.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  bool publish_tf_;

  ros::Publisher global_path_pub_;
  ros::Publisher local_path_transformed_pub_;
  ros::Publisher odom_pub_;

  ros::Subscriber localization_result_sub_;
  ros::Subscriber local_odom_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;

  nav_msgs::Path global_path_;
  nav_msgs::Path local_path_;
  nav_msgs::Path local_path_transformed_;

  std::string global_frame_;
  std::string local_frame_;

  AlignmentFilter filter_;
  Eigen::Isometry3d current_transform_;

  std::mutex mutex_;
};

}  // namespace ais_robot_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "alignment_filter_node");
  ais_robot_localization::AlignmentFilterNode node;
  node.spin();
  return 0;
}

