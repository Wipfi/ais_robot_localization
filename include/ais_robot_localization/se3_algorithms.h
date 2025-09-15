#ifndef AIS_ROBOT_LOCALIZATION_SE3_ALGORITHMS_H
#define AIS_ROBOT_LOCALIZATION_SE3_ALGORITHMS_H

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>

namespace ais_robot_localization
{

// Convert ROS pose to Eigen Isometry3d
Eigen::Isometry3d odomToSe3(const geometry_msgs::Pose& pose);

// Convert Eigen Isometry3d to PoseStamped
geometry_msgs::PoseStamped se3ToPoseStamped(const Eigen::Isometry3d& se3,
                                            const ros::Time& stamp,
                                            const std::string& frame_id,
                                            bool z_to_zero = false);

// Weighted Kabsch-based alignment with snake pre-processing
Eigen::Isometry3d snakeAlignment(const std::vector<Eigen::Isometry3d>& trajectory,
                                 const std::vector<Eigen::Isometry3d>& reference,
                                 const std::vector<double>& weights);

// Gaussian weighting helper
std::vector<double> gaussianWeight(const std::vector<double>& errors, double half_life);

// Euclidean distance helper for geometry_msgs::Pose
double euclideanDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);

}  // namespace ais_robot_localization

#endif  // AIS_ROBOT_LOCALIZATION_SE3_ALGORITHMS_H
