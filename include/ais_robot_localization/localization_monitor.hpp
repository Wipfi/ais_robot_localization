#ifndef AIS_ROBOT_LOCALIZATION_LOCALIZATION_MONITOR_HPP
#define AIS_ROBOT_LOCALIZATION_LOCALIZATION_MONITOR_HPP

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

namespace ais_robot_localization
{
void checkCumulativeDistance(
  std::vector<Eigen::Isometry3d> & local_poses,
  std::vector<double> & local_times,
  double & cumulative_distance,
  double dist_cum_threshold,
  std::size_t max_poses_threshold);

void cleanupGlobalOdomPoses(
  std::vector<Eigen::Isometry3d> & global_poses,
  std::vector<double> & global_times,
  const std::vector<double> & local_times);

std::size_t findNearestTime(const std::vector<double> & times, double ref_time);

void findCorrespondingPoses(
  const std::vector<double> & local_times,
  const std::vector<Eigen::Isometry3d> & global_poses,
  const std::vector<double> & global_times,
  std::vector<Eigen::Isometry3d> & aligned_poses,
  std::vector<double> & aligned_times);

}  // namespace ais_robot_localization

#endif  // AIS_ROBOT_LOCALIZATION_LOCALIZATION_MONITOR_HPP
