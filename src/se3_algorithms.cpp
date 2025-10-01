#include "ais_robot_localization/se3_algorithms.hpp"

#include <algorithm>
#include <cmath>
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

std::vector<double> gaussianWeights(const std::vector<double>& errors,
                                    double half_life) {
    std::vector<double> weights;
    weights.reserve(errors.size());

    if (errors.empty()) {
        return weights;
    }

    if (half_life <= 0.0) {
        weights.assign(errors.size(), 1.0);
        return weights;
    }

    double sigma = half_life / std::sqrt(2.0 * std::log(2.0));
    if (!std::isfinite(sigma) || sigma <= 0.0) {
        weights.assign(errors.size(), 1.0);
        return weights;
    }

    for (double error : errors) {
        double ratio = error / sigma;
        weights.push_back(std::exp(-0.5 * ratio * ratio));
    }

    return weights;
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

    const size_t N = reference.size();

    // (Alg. 9, l. 7) Anchor is the last pose of the sub-trajectory
    const Eigen::Isometry3d& P_Ga = reference.back();
    const Eigen::Isometry3d& P_La = comparison.back();

    // (Alg. 9, l. 13–14) Compute cumulative arc-length s_k
    std::vector<double> s_vals(N, 0.0);
    for (size_t i = 1; i < N; ++i) {
        double ds = (reference[i].translation() - reference[i-1].translation()).norm();
        s_vals[i] = s_vals[i-1] + ds;
    }
    double total_length = s_vals.back() - s_vals.front();
    if (total_length <= 1e-9) {
        return stats;
    }

    auto relativeTransform = [](const Eigen::Isometry3d& A, const Eigen::Isometry3d& B) {
        return A.inverse() * B;
    };

    // (Alg. 9, l. 17–18) SE(3) error definition
    auto se3_error = [&](const Eigen::Isometry3d& P_Gk,
                         const Eigen::Isometry3d& P_Lk) {
        Eigen::Isometry3d ref_rel  = relativeTransform(P_Ga, P_Gk);
        Eigen::Isometry3d comp_rel = relativeTransform(P_La, P_Lk);
        return ref_rel.inverse() * comp_rel;
    };

    auto translationNorm = [](const Eigen::Isometry3d& T) {
        return T.translation().norm();
    };

    auto rotationNorm = [](const Eigen::Isometry3d& T) {
        Eigen::AngleAxisd aa(T.linear());
        return std::abs(aa.angle());
    };

    double e_T = 0.0;
    double e_R = 0.0;
    double max_trans = 0.0;
    double max_rot   = 0.0;

    // (Alg. 9, l. 15–20) Loop over all segments with trapezoidal integration
    for (size_t i = 0; i < N - 1; ++i) {
        Eigen::Isometry3d E_k  = se3_error(reference[i], comparison[i]);
        Eigen::Isometry3d E_k1 = se3_error(reference[i+1], comparison[i+1]);

        double t_err_k  = translationNorm(E_k);
        double t_err_k1 = translationNorm(E_k1);
        double r_err_k  = rotationNorm(E_k);
        double r_err_k1 = rotationNorm(E_k1);

        double ds = s_vals[i+1] - s_vals[i];

        // (Alg. 9, l. 19) Trapezoidal integration
        e_T += ds * 0.5 * (t_err_k + t_err_k1);
        e_R += ds * 0.5 * (r_err_k + r_err_k1);

        // track maximum errors
        max_trans = std::max({max_trans, t_err_k, t_err_k1});
        max_rot   = std::max({max_rot, r_err_k, r_err_k1});
    }

    // (Alg. 9, l. 21) Normalize by total arc length
    stats.avg_trans = e_T / total_length;
    stats.avg_rot   = e_R / total_length;
    stats.max_trans = max_trans;
    stats.max_rot   = max_rot;

    return stats;
}


}  // namespace ais_robot_localization

