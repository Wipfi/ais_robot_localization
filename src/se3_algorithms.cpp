#include "ais_robot_localization/se3_algorithms.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace ais_robot_localization
{

Eigen::Isometry3d odomToSe3(const geometry_msgs::Pose& pose)
{
  Eigen::Isometry3d se3 = Eigen::Isometry3d::Identity();
  se3.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  se3.linear() = q.toRotationMatrix();
  return se3;
}

geometry_msgs::PoseStamped se3ToPoseStamped(const Eigen::Isometry3d& se3,
                                            const ros::Time& stamp,
                                            const std::string& frame_id,
                                            bool z_to_zero)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = se3.translation().x();
  pose.pose.position.y = se3.translation().y();
  pose.pose.position.z = z_to_zero ? 0.0 : se3.translation().z();
  Eigen::Quaterniond q(se3.rotation());
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

static void makeSnake(std::vector<Eigen::Isometry3d>& traj, double fatness = 3.0)
{
  if (traj.size() < 3)
  {
    throw std::runtime_error("To short to be a snake :(");
  }

  Eigen::Vector4d translation_vector(0.0, 0.0, fatness, 1.0);
  for (size_t i = 0; i < traj.size() - 1; ++i)
  {
    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    rotation.block<3,3>(0,0) = traj[i].rotation();
    Eigen::Vector4d point_translation = rotation * translation_vector;
    if (i % 2 == 0)
    {
      point_translation *= -1.0;
    }
    traj[i].translation() += point_translation.head<3>();
  }
}

static Eigen::Isometry3d kabschAlgorithm(const std::vector<Eigen::Isometry3d>& a,
                                         const std::vector<Eigen::Isometry3d>& b,
                                         const std::vector<double>& weights)
{
  size_t N = std::min(a.size(), b.size());
  Eigen::MatrixXd P(3, N), Q(3, N);
  for (size_t i = 0; i < N; ++i)
  {
    P.col(i) = a[i].translation();
    Q.col(i) = b[i].translation();
  }

  Eigen::VectorXd w;
  if (!weights.empty())
  {
    w = Eigen::Map<const Eigen::VectorXd>(weights.data(), N);
  }

  double wsum = weights.empty() ? static_cast<double>(N) : w.sum();
  Eigen::Vector3d centroidP = weights.empty() ? P.rowwise().mean() : (P * w / wsum);
  Eigen::Vector3d centroidQ = weights.empty() ? Q.rowwise().mean() : (Q * w / wsum);

  P.colwise() -= centroidP;
  Q.colwise() -= centroidQ;

  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  if (weights.empty())
  {
    H = P * Q.transpose();
  }
  else
  {
    for (size_t i = 0; i < N; ++i)
    {
      H += weights[i] * P.col(i) * Q.col(i).transpose();
    }
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0)
  {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1.0;
    R = V * svd.matrixU().transpose();
  }

  Eigen::Vector3d t = centroidQ - R * centroidP;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = R;
  transform.translation() = t;
  return transform;
}

Eigen::Isometry3d snakeAlignment(const std::vector<Eigen::Isometry3d>& trajectory_in,
                                 const std::vector<Eigen::Isometry3d>& reference_in,
                                 const std::vector<double>& weights)
{
  std::vector<Eigen::Isometry3d> trajectory = trajectory_in;
  std::vector<Eigen::Isometry3d> reference = reference_in;
  makeSnake(trajectory);
  makeSnake(reference);
  return kabschAlgorithm(trajectory, reference, weights);
}

std::vector<double> gaussianWeight(const std::vector<double>& errors, double half_life)
{
  std::vector<double> weights;
  weights.reserve(errors.size());
  double sigma = half_life / std::sqrt(2.0 * std::log(2.0));
  for (double e : errors)
  {
    double w = std::exp(-0.5 * std::pow(e / sigma, 2.0));
    weights.push_back(w);
  }
  return weights;
}

double euclideanDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  double dx = b.position.x - a.position.x;
  double dy = b.position.y - a.position.y;
  double dz = b.position.z - a.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace ais_robot_localization

