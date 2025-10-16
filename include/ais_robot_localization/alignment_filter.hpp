#ifndef AIS_ROBOT_LOCALIZATION_ALIGNMENT_FILTER_HPP
#define AIS_ROBOT_LOCALIZATION_ALIGNMENT_FILTER_HPP

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

namespace ais_robot_localization
{
struct AlignmentResult
{
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Isometry3d> transformed_local;
  double used_length{0.0};
};

class AlignmentFilter
{
public:
  AlignmentFilter();

  void setMaxWindowLength(double length);
  void setIgnoreGlobalYaw(bool ignore) {ignore_global_yaw_ = ignore;}
  bool ignoreGlobalYaw() const {return ignore_global_yaw_;}

  std::size_t addMeasurement(
    const Eigen::Isometry3d & global_pose,
    const Eigen::Isometry3d & local_pose,
    double timestamp_sec,
    double translational_error,
    double cumulative_length);

  bool hasSufficientData() const;

  bool computeAlignment(AlignmentResult & result);

  const std::vector<Eigen::Isometry3d> & globalPoses() const {return global_poses_;}
  const std::vector<Eigen::Isometry3d> & localPoses() const {return local_poses_;}
  const std::vector<double> & timestamps() const {return timestamps_;}
  const std::vector<double> & translationalErrors() const {return translational_errors_;}
  const Eigen::Isometry3d & currentTransform() const {return current_transform_;}
  double usedLength() const {return used_length_;}

private:
  std::size_t cleanup();
  static double percentile(const std::vector<double> & values, double percent);
  void alignGlobalYawWithLocal(const std::vector<Eigen::Isometry3d> & local_for_alignment);

  double max_window_length_;
  std::vector<Eigen::Isometry3d> global_poses_;
  std::vector<Eigen::Isometry3d> local_poses_;
  std::vector<double> timestamps_;
  std::vector<double> translational_errors_;
  std::vector<double> cumulative_lengths_;

  Eigen::Isometry3d current_transform_;
  double used_length_;
  bool ignore_global_yaw_;
};

}  // namespace ais_robot_localization

#endif  // AIS_ROBOT_LOCALIZATION_ALIGNMENT_FILTER_HPP
