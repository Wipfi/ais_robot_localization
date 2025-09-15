#include "ais_robot_localization/localization_monitor.hpp"
#include "ais_robot_localization/se3_algorithms.hpp"

#include <algorithm>

namespace ais_robot_localization {

void checkCumulativeDistance(std::vector<Eigen::Isometry3d>& local_poses,
                             std::vector<double>& local_times,
                             double& cumulative_distance,
                             double dist_cum_threshold,
                             std::size_t max_poses_threshold) {
    while (((cumulative_distance > dist_cum_threshold) && local_poses.size() > 1) ||
           local_poses.size() > max_poses_threshold) {
        double dist_removed = calculateEuclideanDistance(local_poses[0], local_poses[1]);
        local_poses.erase(local_poses.begin());
        local_times.erase(local_times.begin());
        cumulative_distance -= dist_removed;
    }
}

void cleanupGlobalOdomPoses(std::vector<Eigen::Isometry3d>& global_poses,
                            std::vector<double>& global_times,
                            const std::vector<double>& local_times) {
    if (local_times.empty()) {
        return;
    }
    double oldest_local = local_times.front();
    while (!global_times.empty() && global_times.front() < oldest_local) {
        global_poses.erase(global_poses.begin());
        global_times.erase(global_times.begin());
    }
}

std::size_t findNearestTime(const std::vector<double>& times,
                            double ref_time) {
    if (times.empty()) {
        return 0;
    }
    auto it = std::lower_bound(times.begin(), times.end(), ref_time);
    if (it == times.begin()) {
        return 0;
    }
    if (it == times.end()) {
        return times.size() - 1;
    }
    std::size_t pos = std::distance(times.begin(), it);
    double before = times[pos - 1];
    double after = times[pos];
    return (std::abs(after - ref_time) < std::abs(before - ref_time)) ? pos : pos - 1;
}

void findCorrespondingPoses(const std::vector<double>& local_times,
                            const std::vector<Eigen::Isometry3d>& global_poses,
                            const std::vector<double>& global_times,
                            std::vector<Eigen::Isometry3d>& aligned_poses,
                            std::vector<double>& aligned_times) {
    aligned_poses.clear();
    aligned_times.clear();
    if (local_times.empty() || global_times.empty()) {
        return;
    }
    for (double ref_time : local_times) {
        std::size_t idx = findNearestTime(global_times, ref_time);
        aligned_poses.push_back(global_poses[idx]);
        aligned_times.push_back(global_times[idx]);
    }
}

}  // namespace ais_robot_localization

