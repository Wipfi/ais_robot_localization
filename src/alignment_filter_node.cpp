#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <ais_robot_localization/LocalizationMonitorResult.h>

#include <vector>
#include <algorithm>
#include <numeric>
#include <mutex>

#include "ais_robot_localization/se3_algorithms.h"

namespace arl = ais_robot_localization;

class AlignmentFilterNode
{
public:
  AlignmentFilterNode()
      : nh_(),
        nh_private_("~"),
        publish_tf_(true),
        current_transform_(Eigen::Isometry3d::Identity()),
        used_length_(0.0)
  {
    nh_private_.param("publish_tf", publish_tf_, true);

    if (publish_tf_)
    {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
      timer_ = nh_.createTimer(ros::Duration(0.2), &AlignmentFilterNode::publishTransform, this);
    }

    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/alignment_global_path", 10);
    local_path_transformed_pub_ = nh_.advertise<nav_msgs::Path>("/alignment_local_path_transformed", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/alignment_odometry", 10);

    sub_ = nh_.subscribe("/localization_result", 10, &AlignmentFilterNode::localizationCallback, this);
    local_odom_sub_ = nh_.subscribe("/local_odom", 10, &AlignmentFilterNode::localOdomCallback, this);

    global_frame_ = "map";
    local_frame_ = "odom";
  }

private:
  void publishTransform(const ros::TimerEvent&)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!publish_tf_)
    {
      return;
    }

    try
    {
      geometry_msgs::TransformStamped latest_tf = tf_buffer_.lookupTransform(local_frame_, "base_link", ros::Time(0),
                                                                             ros::Duration(1.0));
      geometry_msgs::TransformStamped t;
      t.header.stamp = latest_tf.header.stamp;
      t.header.frame_id = global_frame_;
      t.child_frame_id = local_frame_;
      Eigen::Quaterniond q(current_transform_.rotation());
      Eigen::Vector3d trans = current_transform_.translation();
      t.transform.translation.x = trans.x();
      t.transform.translation.y = trans.y();
      t.transform.translation.z = trans.z();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_.sendTransform(t);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("Error publishing transform: " << ex.what());
    }
  }

  void localOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    Eigen::Isometry3d pose = arl::odomToSe3(odom_msg->pose.pose);
    Eigen::Isometry3d transformed = current_transform_ * pose;

    nav_msgs::Odometry transformed_odom = *odom_msg;
    transformed_odom.header.frame_id = global_frame_;
    Eigen::Quaterniond q(transformed.rotation());
    transformed_odom.pose.pose.position.x = transformed.translation().x();
    transformed_odom.pose.pose.position.y = transformed.translation().y();
    transformed_odom.pose.pose.position.z = transformed.translation().z();
    transformed_odom.pose.pose.orientation.x = q.x();
    transformed_odom.pose.pose.orientation.y = q.y();
    transformed_odom.pose.pose.orientation.z = q.z();
    transformed_odom.pose.pose.orientation.w = q.w();
    // set fixed covariance like python implementation
    double cov[36] = {1.0, 0, 0, 0, 0, 0,
                      0, 1.0, 0, 0, 0, 0,
                      0, 0, 1.0, 0, 0, 0,
                      0, 0, 0, 0.1, 0, 0,
                      0, 0, 0, 0, 0.1, 0,
                      0, 0, 0, 0, 0, 0.1};
    std::copy(std::begin(cov), std::end(cov), transformed_odom.pose.covariance.begin());
    current_odom_state_ = transformed_odom;
    odom_pub_.publish(transformed_odom);
  }

  void localizationCallback(const ais_robot_localization::LocalizationMonitorResult::ConstPtr& msg)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (global_frame_.empty())
    {
      global_frame_ = msg->pose_global.header.frame_id;
    }
    if (local_frame_.empty())
    {
      local_frame_ = msg->pose_local.header.frame_id;
    }

    data_list_.push_back(*msg);

    global_path_.poses.push_back(msg->pose_global);
    global_poses_.push_back(arl::odomToSe3(msg->pose_global.pose));
    time_stamps_.push_back(msg->pose_global.header.stamp);

    local_path_.poses.push_back(msg->pose_local);
    local_poses_.push_back(arl::odomToSe3(msg->pose_local.pose));

    if (!msg->float_array.data.empty())
    {
      error_trans_avg_.push_back(msg->float_array.data[0]);
    }
    else
    {
      error_trans_avg_.push_back(0.0);
    }
    distances_current_estimate_.push_back(0.0);

    cleanUp();

    if (data_list_.size() > 3)
    {
      alignLocalToGlobal(msg->pose_global.header);
      publishPaths();
      used_length_ = data_list_.back().cumulative_length - data_list_.front().cumulative_length;
    }
  }

  void cleanUp()
  {
    if (data_list_.empty())
    {
      return;
    }
    double length = data_list_.back().cumulative_length - data_list_.front().cumulative_length;
    while (length > 150.0 && !data_list_.empty())
    {
      data_list_.erase(data_list_.begin());
      time_stamps_.erase(time_stamps_.begin());
      global_path_.poses.erase(global_path_.poses.begin());
      global_poses_.erase(global_poses_.begin());
      local_path_.poses.erase(local_path_.poses.begin());
      local_poses_.erase(local_poses_.begin());
      error_trans_avg_.erase(error_trans_avg_.begin());
      distances_current_estimate_.erase(distances_current_estimate_.begin());
      if (data_list_.empty())
      {
        break;
      }
      length = data_list_.back().cumulative_length - data_list_.front().cumulative_length;
    }
  }

  void alignLocalToGlobal(const std_msgs::Header& header)
  {
    std::vector<double> penalty = error_trans_avg_;
    std::vector<double> sorted = penalty;
    if (!sorted.empty())
    {
      std::sort(sorted.begin(), sorted.end());
      double ref_value = sorted[sorted.size() / 4];
      std::vector<double> weights = arl::gaussianWeight(penalty, ref_value);
      current_transform_ = arl::snakeAlignment(local_poses_, global_poses_, weights);
    }
    else
    {
      current_transform_ = Eigen::Isometry3d::Identity();
    }

    local_path_transformed_.poses.clear();
    for (size_t i = 0; i < local_poses_.size(); ++i)
    {
      Eigen::Isometry3d pose = current_transform_ * local_poses_[i];
      geometry_msgs::PoseStamped pose_stamped = arl::se3ToPoseStamped(pose, time_stamps_[i], global_frame_, false);
      geometry_msgs::PoseStamped out = pose_stamped;
      out.header = header;
      out.header.frame_id = global_frame_;
      local_path_transformed_.poses.push_back(out);
    }
  }

  void publishPaths()
  {
    global_path_.header.frame_id = global_frame_;
    global_path_.header.stamp = ros::Time::now();
    global_path_pub_.publish(global_path_);

    local_path_transformed_.header.frame_id = global_frame_;
    local_path_transformed_.header.stamp = ros::Time::now();
    local_path_transformed_pub_.publish(local_path_transformed_);
  }

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_;
  ros::Subscriber local_odom_sub_;
  ros::Publisher global_path_pub_;
  ros::Publisher local_path_transformed_pub_;
  ros::Publisher odom_pub_;
  bool publish_tf_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Timer timer_;

  // Data storage
  std::vector<ais_robot_localization::LocalizationMonitorResult> data_list_;
  std::vector<ros::Time> time_stamps_;
  std::vector<Eigen::Isometry3d> global_poses_;
  std::vector<Eigen::Isometry3d> local_poses_;
  std::vector<double> error_trans_avg_;
  std::vector<double> distances_current_estimate_;
  nav_msgs::Path global_path_;
  nav_msgs::Path local_path_;
  nav_msgs::Path local_path_transformed_;
  nav_msgs::Odometry current_odom_state_;

  std::string global_frame_;
  std::string local_frame_;
  Eigen::Isometry3d current_transform_;
  double used_length_;

  std::recursive_mutex mutex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "alignment_filter_node");
  AlignmentFilterNode node;
  ros::spin();
  return 0;
}

