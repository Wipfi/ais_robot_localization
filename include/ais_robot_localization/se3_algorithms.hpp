#ifndef AIS_ROBOT_LOCALIZATION_SE3_ALGORITHMS_HPP
#define AIS_ROBOT_LOCALIZATION_SE3_ALGORITHMS_HPP

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ais_robot_localization {

struct PoseStamped {
    double stamp{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

Eigen::Isometry3d odomToSE3(const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation);

PoseStamped se3ToPoseStamped(const Eigen::Isometry3d& transform,
                             double stamp,
                             bool z_to_zero = false);

double calculateEuclideanDistance(const Eigen::Isometry3d& pose1,
                                  const Eigen::Isometry3d& pose2);

void kabschAlgorithm(const std::vector<Eigen::Vector3d>& src,
                     const std::vector<Eigen::Vector3d>& dst,
                     Eigen::Matrix3d& rotation,
                     Eigen::Vector3d& translation,
                     const std::vector<double>* weights = nullptr);

void makeSnake(std::vector<Eigen::Isometry3d>& trajectory,
               double fatness = 3.0);

Eigen::Isometry3d alignTrajectories(std::vector<Eigen::Isometry3d>& trajectory,
                                    const std::vector<Eigen::Isometry3d>& reference,
                                    const std::vector<double>* weights = nullptr);

struct RPEStats {
    double avg_trans{0.0};
    double avg_rot{0.0};
    double max_trans{0.0};
    double max_rot{0.0};
};

RPEStats calculateRPE(const std::vector<Eigen::Isometry3d>& reference,
                      const std::vector<Eigen::Isometry3d>& comparison);

}  // namespace ais_robot_localization

#endif  // AIS_ROBOT_LOCALIZATION_SE3_ALGORITHMS_HPP
