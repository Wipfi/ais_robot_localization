#include "ais_robot_localization/alignment_filter.hpp"

#include <algorithm>
#include <cmath>

#include "ais_robot_localization/se3_algorithms.hpp"

namespace ais_robot_localization
{
namespace
{
constexpr std::size_t kMinSamples = 4;
constexpr double kDefaultWindowLength = 150.0;
}  // namespace

AlignmentFilter::AlignmentFilter()
: max_window_length_(kDefaultWindowLength),
  current_transform_(Eigen::Isometry3d::Identity()),
  used_length_(0.0),
  ignore_global_yaw_(false)
{
}

void AlignmentFilter::setMaxWindowLength(double length)
{
  if (length > 0.0) {
    max_window_length_ = length;
  }
}

std::size_t AlignmentFilter::addMeasurement(
  const Eigen::Isometry3d & global_pose,
  const Eigen::Isometry3d & local_pose,
  double timestamp_sec,
  double translational_error,
  double cumulative_length)
{
  global_poses_.push_back(global_pose);
  local_poses_.push_back(local_pose);
  timestamps_.push_back(timestamp_sec);
  translational_errors_.push_back(translational_error);
  cumulative_lengths_.push_back(cumulative_length);

  std::size_t removed = cleanup();

  if (cumulative_lengths_.size() >= 2) {
    used_length_ = cumulative_lengths_.back() - cumulative_lengths_.front();
  } else {
    used_length_ = 0.0;
  }

  return removed;
}

bool AlignmentFilter::hasSufficientData() const
{
  return global_poses_.size() >= kMinSamples && local_poses_.size() >= kMinSamples;
}

bool AlignmentFilter::computeAlignment(AlignmentResult & result)
{
  if (!hasSufficientData()) {
    return false;
  }

  std::vector<Eigen::Isometry3d> local_for_alignment = local_poses_;
  if (ignore_global_yaw_) {
    const std::size_t count = std::min(global_poses_.size(), local_for_alignment.size());
    for (std::size_t i = 0; i < count; ++i) {
      const Eigen::Matrix3d & global_rotation = global_poses_[i].linear();
      const Eigen::Matrix3d & local_rotation = local_for_alignment[i].linear();
      const double global_yaw = std::atan2(global_rotation(1, 0), global_rotation(0, 0));
      const double local_yaw = std::atan2(local_rotation(1, 0), local_rotation(0, 0));
      const double yaw_delta = local_yaw - global_yaw;
      Eigen::Matrix3d yaw_alignment = Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      global_poses_[i].linear() = yaw_alignment * global_poses_[i].linear();
    }
  }
  std::vector<double> weights = gaussianWeights(translational_errors_, percentile(translational_errors_, 25.0));
  const std::vector<double> * weights_ptr = nullptr;
  if (weights.size() == local_for_alignment.size()) {
    weights_ptr = &weights;
  }

  Eigen::Isometry3d transform = alignTrajectories(local_for_alignment, global_poses_, weights_ptr);

  current_transform_ = transform;
  result.transform = transform;
  result.transformed_local = std::move(local_for_alignment);
  result.used_length = used_length_;

  return true;
}

std::size_t AlignmentFilter::cleanup()
{
  if (cumulative_lengths_.empty()) {
    return 0U;
  }

  std::size_t removed = 0U;
  while (cumulative_lengths_.size() > 1 &&
    cumulative_lengths_.back() - cumulative_lengths_.front() > max_window_length_)
  {
    global_poses_.erase(global_poses_.begin());
    local_poses_.erase(local_poses_.begin());
    timestamps_.erase(timestamps_.begin());
    translational_errors_.erase(translational_errors_.begin());
    cumulative_lengths_.erase(cumulative_lengths_.begin());
    ++removed;
  }

  return removed;
}

double AlignmentFilter::percentile(const std::vector<double> & values, double percent)
{
  if (values.empty()) {
    return 0.0;
  }

  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());

  if (sorted.size() == 1U) {
    return sorted.front();
  }

  double rank = percent / 100.0 * static_cast<double>(sorted.size() - 1U);
  double lower_idx = std::floor(rank);
  double upper_idx = std::ceil(rank);
  double fraction = rank - lower_idx;

  double lower_value = sorted[static_cast<std::size_t>(lower_idx)];
  double upper_value = sorted[static_cast<std::size_t>(upper_idx)];

  return lower_value + (upper_value - lower_value) * fraction;
}

}  // namespace ais_robot_localization

