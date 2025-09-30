#include <algorithm>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>

#include "ais_robot_localization/LocalizationMonitorResult.h"
#include "ais_robot_localization/localization_monitor.hpp"
#include "ais_robot_localization/se3_algorithms.hpp"

namespace ais_robot_localization {
namespace {

double stampToDouble(const ros::Time& stamp) {
  return stamp.toSec();
}

geometry_msgs::PoseStamped toRosPoseStamped(const PoseStamped& pose, const std::string& frame_id) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.fromSec(pose.stamp);
  msg.pose.position.x = pose.position.x();
  msg.pose.position.y = pose.position.y();
  msg.pose.position.z = pose.position.z();
  msg.pose.orientation.x = pose.orientation.x();
  msg.pose.orientation.y = pose.orientation.y();
  msg.pose.orientation.z = pose.orientation.z();
  msg.pose.orientation.w = pose.orientation.w();
  return msg;
}

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::Quaternion& q) {
  Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
  if (eigen_q.norm() == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  eigen_q.normalize();
  return eigen_q;
}

}  // namespace

class LocalizationMonitorNode {
 public:
  LocalizationMonitorNode()
      : nh_(), private_nh_("~"), last_publish_time_sec_(0.0) {
    dist_cum_threshold_ = private_nh_.param("dist_cum_threshold", 15.0);
    publish_rate_ = private_nh_.param("publish_rate", 1.0);
    int max_poses_param = private_nh_.param("max_poses_threshold", 500);
    max_poses_threshold_ = max_poses_param < 0 ? 0 : static_cast<std::size_t>(max_poses_param);
    time_based_ = private_nh_.param("time_based", false);
    publish_gnss_with_scaled_covariance_ = private_nh_.param("publish_gnss_with_scaled_covariance", false);

    feature_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/RPE_Values", 10);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_odom_path", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_odom_path", 10);
    localization_result_pub_ = nh_.advertise<ais_robot_localization::LocalizationMonitorResult>("/localization_result", 10);
    gnss_with_scaled_covariance_pub_ = nh_.advertise<nav_msgs::Odometry>("/gnss_with_scaled_covariance", 10);

    global_odom_sub_ = nh_.subscribe("/global_odom", 50, &LocalizationMonitorNode::globalOdomCallback, this);
    local_odom_sub_ = nh_.subscribe("/local_odom", 50, &LocalizationMonitorNode::localOdomCallback, this);

    ROS_INFO("LocalizationMonitorNode initialized");
  }

  void spin() const {
    ros::spin();
  }

 private:
  void globalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation = toEigenQuaternion(msg->pose.pose.orientation);
    Eigen::Isometry3d pose = odomToSE3(position, orientation);

    global_poses_.push_back(pose);
    global_times_.push_back(stampToDouble(msg->header.stamp));
    newest_global_time_ = msg->header.stamp;
    global_frame_ = msg->header.frame_id;
    newest_global_odom_ = *msg;
    have_newest_global_odom_ = true;
  }

  void localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation = toEigenQuaternion(msg->pose.pose.orientation);
    Eigen::Isometry3d pose = odomToSE3(position, orientation);

    double current_time = stampToDouble(msg->header.stamp);
    double min_interval = publish_rate_ > 0.0 ? 1.0 / publish_rate_ : 0.0;

    if (!local_poses_.empty()) {
      double dist = calculateEuclideanDistance(local_poses_.back(), pose);
      if (!time_based_ && dist < 0.2) {
        return;
      }
      if (time_based_ && min_interval > 0.0 && (current_time - last_publish_time_sec_) < min_interval) {
        return;
      }
      cumulative_distance_ += dist;
      cumulative_length_ += dist;
    }

    local_frame_ = msg->header.frame_id;
    local_poses_.push_back(pose);
    local_times_.push_back(current_time);

    if (min_interval == 0.0 || (current_time - last_publish_time_sec_) >= min_interval) {
      if (cumulative_distance_ > 1.0 || time_based_) {
        processTrajectories(msg->header);
        last_publish_time_sec_ = current_time;
      }
    }
  }

  void processTrajectories(const std_msgs::Header& header) {
    if (local_poses_.empty() || global_poses_.empty()) {
      publishResults(nullptr);
      return;
    }

    checkCumulativeDistance(local_poses_, local_times_, cumulative_distance_, dist_cum_threshold_, max_poses_threshold_);
    cleanupGlobalOdomPoses(global_poses_, global_times_, local_times_);

    if (global_poses_.empty()) {
      publishResults(nullptr);
      return;
    }

    std::vector<Eigen::Isometry3d> aligned_global;
    std::vector<double> aligned_times;
    findCorrespondingPoses(local_times_, global_poses_, global_times_, aligned_global, aligned_times);

    if (aligned_global.empty()) {
      publishResults(nullptr);
      return;
    }

    global_poses_ = aligned_global;
    global_times_ = aligned_times;

    std::vector<Eigen::Isometry3d> local_aligned = local_poses_;
    std::vector<Eigen::Isometry3d> reference = global_poses_;
    alignTrajectories(local_aligned, reference);

    updatePaths(reference, local_aligned, header);

    RPEStats stats;
    const RPEStats* stats_ptr = nullptr;
    if (reference.size() >= 2 && local_aligned.size() >= 2 && cumulative_distance_ >= 2.0) {
      stats = calculateRPE(reference, local_aligned);
      stats_ptr = &stats;
    }

    publishResults(stats_ptr);

    if (publish_gnss_with_scaled_covariance_ && have_newest_global_odom_) {
      ROS_WARN_ONCE("Publishing GNSS odometry without covariance scaling; feature not yet implemented.");
      gnss_with_scaled_covariance_pub_.publish(newest_global_odom_);
    }
  }

  void updatePaths(const std::vector<Eigen::Isometry3d>& reference,
                   const std::vector<Eigen::Isometry3d>& local_aligned,
                   const std_msgs::Header& header) {
    global_path_.poses.clear();
    local_path_.poses.clear();

    for (std::size_t i = 0; i < reference.size(); ++i) {
      PoseStamped global_pose = se3ToPoseStamped(reference[i], global_times_[i], false);
      geometry_msgs::PoseStamped global_msg = toRosPoseStamped(global_pose, global_frame_);
      global_path_.poses.push_back(global_msg);
    }

    for (std::size_t i = 0; i < local_aligned.size(); ++i) {
      PoseStamped local_pose = se3ToPoseStamped(local_aligned[i], local_times_[i], false);
      geometry_msgs::PoseStamped local_msg = toRosPoseStamped(local_pose, global_frame_);
      local_msg.header.stamp = header.stamp;
      local_path_.poses.push_back(local_msg);
    }

    ros::Time publish_stamp = newest_global_time_;
    if (publish_stamp.isZero() && !global_times_.empty()) {
      publish_stamp.fromSec(global_times_.back());
    }

    global_path_.header.frame_id = global_frame_;
    global_path_.header.stamp = publish_stamp;
    local_path_.header.frame_id = global_frame_;
    local_path_.header.stamp = publish_stamp;

    global_path_pub_.publish(global_path_);
    local_path_pub_.publish(local_path_);
  }

  void publishResults(const RPEStats* stats) {
    std_msgs::Float32MultiArray feature_msg;
    feature_msg.data.resize(4, std::numeric_limits<float>::infinity());

    if (stats != nullptr) {
      feature_msg.data[0] = static_cast<float>(stats->avg_trans);
      feature_msg.data[1] = static_cast<float>(stats->avg_rot);
      feature_msg.data[2] = static_cast<float>(stats->max_trans);
      feature_msg.data[3] = static_cast<float>(stats->max_rot);
    }

    feature_pub_.publish(feature_msg);

    if (global_poses_.empty() || local_poses_.empty() || local_times_.empty()) {
      return;
    }

    std::size_t index = global_poses_.size() - 1;
    PoseStamped global_pose = se3ToPoseStamped(global_poses_[index], local_times_[index], false);
    PoseStamped local_pose = se3ToPoseStamped(local_poses_[index], local_times_[index], false);

    ais_robot_localization::LocalizationMonitorResult result;
    result.float_array = feature_msg.data;
    result.pose_global = toRosPoseStamped(global_pose, global_frame_);
    result.pose_local = toRosPoseStamped(local_pose, local_frame_);
    result.cumulative_length = cumulative_length_;

    localization_result_pub_.publish(result);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher feature_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher localization_result_pub_;
  ros::Publisher gnss_with_scaled_covariance_pub_;

  ros::Subscriber global_odom_sub_;
  ros::Subscriber local_odom_sub_;

  std::vector<Eigen::Isometry3d> global_poses_;
  std::vector<double> global_times_;
  std::vector<Eigen::Isometry3d> local_poses_;
  std::vector<double> local_times_;

  nav_msgs::Path global_path_;
  nav_msgs::Path local_path_;

  double cumulative_distance_{0.0};
  double cumulative_length_{0.0};
  double last_publish_time_sec_;

  double dist_cum_threshold_{15.0};
  double publish_rate_{1.0};
  std::size_t max_poses_threshold_{500};
  bool time_based_{false};
  bool publish_gnss_with_scaled_covariance_{false};

  std::string global_frame_;
  std::string local_frame_;
  ros::Time newest_global_time_;
  nav_msgs::Odometry newest_global_odom_;
  bool have_newest_global_odom_{false};

  std::mutex mutex_;
};

}  // namespace ais_robot_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_monitor");

  ais_robot_localization::LocalizationMonitorNode node;
  node.spin();

  return 0;
}

