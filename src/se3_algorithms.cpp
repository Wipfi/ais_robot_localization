#include "ais_robot_localization/se3_algorithms.hpp"

#include <algorithm>
#include <numeric>

namespace ais_robot_localization {

Eigen::Isometry3d odomToSE3(const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = orientation.toRotationMatrix();
    transform.translation() = position;
    return transform;
}

PoseStamped se3ToPoseStamped(const Eigen::Isometry3d& transform,
                             double stamp,
                             bool z_to_zero) {
    PoseStamped pose;
    pose.stamp = stamp;
    pose.position = transform.translation();
    if (z_to_zero) {
        pose.position.z() = 0.0;
    }
    pose.orientation = Eigen::Quaterniond(transform.linear());
    pose.orientation.normalize();
    return pose;
}

double calculateEuclideanDistance(const Eigen::Isometry3d& pose1,
                                  const Eigen::Isometry3d& pose2) {
    return (pose2.translation() - pose1.translation()).norm();
}

void kabschAlgorithm(const std::vector<Eigen::Vector3d>& src,
                     const std::vector<Eigen::Vector3d>& dst,
                     Eigen::Matrix3d& rotation,
                     Eigen::Vector3d& translation,
                     const std::vector<double>* weights) {
    if (src.size() != dst.size() || src.empty()) {
        rotation.setIdentity();
        translation.setZero();
        return;
    }
    std::vector<Eigen::Vector3d> src_pts = src;
    std::vector<Eigen::Vector3d> dst_pts = dst;

    Eigen::Vector3d centroid_src = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_dst = Eigen::Vector3d::Zero();

    if (weights == nullptr) {
        for (size_t i = 0; i < src_pts.size(); ++i) {
            centroid_src += src_pts[i];
            centroid_dst += dst_pts[i];
        }
        centroid_src /= static_cast<double>(src_pts.size());
        centroid_dst /= static_cast<double>(dst_pts.size());
    } else {
        double weight_sum = std::accumulate(weights->begin(), weights->end(), 0.0);
        for (size_t i = 0; i < src_pts.size(); ++i) {
            double w = (*weights)[i] / weight_sum;
            centroid_src += w * src_pts[i];
            centroid_dst += w * dst_pts[i];
        }
    }

    for (size_t i = 0; i < src_pts.size(); ++i) {
        src_pts[i] -= centroid_src;
        dst_pts[i] -= centroid_dst;
    }

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    if (weights == nullptr) {
        for (size_t i = 0; i < src_pts.size(); ++i) {
            H += src_pts[i] * dst_pts[i].transpose();
        }
    } else {
        for (size_t i = 0; i < src_pts.size(); ++i) {
            H += (*weights)[i] * src_pts[i] * dst_pts[i].transpose();
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1.0; // reflection handling
        rotation = V * svd.matrixU().transpose();
    }
    translation = centroid_dst - rotation * centroid_src;
}

void makeSnake(std::vector<Eigen::Isometry3d>& trajectory,
               double fatness) {
    if (trajectory.size() < 3) {
        return;
    }
    Eigen::Vector4d translation_vec(0.0, 0.0, fatness, 1.0);
    for (size_t i = 0; i + 1 < trajectory.size(); ++i) {
        Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();
        rot.block<3,3>(0,0) = trajectory[i].linear();
        Eigen::Vector4d offset = rot * translation_vec;
        if (i % 2 == 0) {
            offset *= -1.0;
        }
        trajectory[i].translation() += offset.head<3>();
    }
}

Eigen::Isometry3d alignTrajectories(std::vector<Eigen::Isometry3d>& trajectory,
                                    const std::vector<Eigen::Isometry3d>& reference,
                                    const std::vector<double>* weights) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (trajectory.size() != reference.size() || trajectory.empty()) {
        return transform;
    }

    std::vector<Eigen::Isometry3d> ref_snake = reference;
    makeSnake(trajectory);
    makeSnake(ref_snake);

    std::vector<Eigen::Vector3d> src_pts, dst_pts;
    src_pts.reserve(trajectory.size());
    dst_pts.reserve(ref_snake.size());
    for (size_t i = 0; i < trajectory.size(); ++i) {
        src_pts.push_back(trajectory[i].translation());
        dst_pts.push_back(ref_snake[i].translation());
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    kabschAlgorithm(src_pts, dst_pts, R, t, weights);

    transform.linear() = R;
    transform.translation() = t;

    for (auto& pose : trajectory) {
        pose = transform * pose;
    }
    return transform;
}

RPEStats calculateRPE(const std::vector<Eigen::Isometry3d>& reference,
                      const std::vector<Eigen::Isometry3d>& comparison) {
    RPEStats stats;
    if (reference.size() != comparison.size() || reference.size() < 2) {
        return stats;
    }

    double total_trans = 0.0;
    double total_rot = 0.0;
    double max_trans = 0.0;
    double max_rot = 0.0;
    size_t count = reference.size() - 1;

    for (size_t i = 1; i < reference.size(); ++i) {
        Eigen::Isometry3d ref_rel = reference[i-1].inverse() * reference[i];
        Eigen::Isometry3d comp_rel = comparison[i-1].inverse() * comparison[i];
        Eigen::Isometry3d error = ref_rel.inverse() * comp_rel;

        double trans_error = error.translation().norm();
        double rot_error = Eigen::AngleAxisd(error.linear()).angle();

        total_trans += trans_error;
        total_rot += rot_error;
        max_trans = std::max(max_trans, trans_error);
        max_rot = std::max(max_rot, rot_error);
    }

    stats.avg_trans = total_trans / static_cast<double>(count);
    stats.avg_rot = total_rot / static_cast<double>(count);
    stats.max_trans = max_trans;
    stats.max_rot = max_rot;
    return stats;
}

}  // namespace ais_robot_localization

