#include "ouster/impl/utils.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>

#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "ouster/impl/logging.h"
#include "ouster/impl/transformation.h"

using ouster::sdk::core::logger;
using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseV;

namespace ouster {
namespace sdk {
namespace mapping {

double pose_dist(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
    return (point1 - point2).norm();
}

std::vector<Eigen::Vector3d> convert_array_to_vector(
    Eigen::Ref<const Eigen::ArrayX3d> arr) {
    std::vector<Eigen::Vector3d> result;
    result.reserve(arr.rows());  // Reserve space
    for (Eigen::Index i = 0; i < arr.rows(); ++i) {
        result.emplace_back(arr(i, 0), arr(i, 1), arr(i, 2));
    }
    return result;
}

Eigen::ArrayX3d convert_vector_to_array(
    const std::vector<Eigen::Vector3d>& vec) {
    Eigen::ArrayX3d arr(vec.size(), 3);
    for (size_t i = 0; i < vec.size(); ++i) {
        arr.row(i) << vec[i].x(), vec[i].y(), vec[i].z();
    }
    return arr;
}

Eigen::ArrayX3d run_kiss_icp_downsample(
    Eigen::Ref<const Eigen::ArrayX3d> source_points, double voxel_size) {
    std::vector<Eigen::Vector3d> points_vector =
        convert_array_to_vector(source_points);
    std::vector<Eigen::Vector3d> pts_downsample_vector =
        kiss_icp::VoxelDownsample(points_vector, voxel_size);
    Eigen::ArrayX3d pts_downsample_array =
        convert_vector_to_array(pts_downsample_vector);

    return pts_downsample_array;
}

Eigen::Matrix4d run_kiss_icp_matching(
    Eigen::Ref<const Eigen::ArrayX3d> source_points,
    Eigen::Ref<const Eigen::ArrayX3d> target_points,
    const PoseH& initial_guess_pose) {
    Sophus::SE3d identity_matrix;
    // max_num_iterations, convergence_criterion, max_num_threads
    kiss_icp::Registration registration(1000, 0.0001, 0);
    // Voxel_size, max_range, max_points_per_voxel
    kiss_icp::VoxelHashMap local_map(1.0, 200, 20);

    auto source_array = convert_array_to_vector(source_points);
    auto target_array = convert_array_to_vector(target_points);

    auto source_array_down = kiss_icp::VoxelDownsample(source_array, 0.5);
    auto target_array_down = kiss_icp::VoxelDownsample(target_array, 0.5);

    local_map.Update(target_array_down, identity_matrix);

    const auto new_pose = registration.AlignPointsToMap(
        source_array_down,                 // frame
        local_map,                         // voxel_map
        Sophus::SE3d(initial_guess_pose),  // initial_guess
        10,                                // max_correspondence_dist
        2);
    return new_pose.matrix();
}

void save_to_ply(Eigen::Ref<const ouster::sdk::core::MatrixX3dR> points,
                 const std::string& filename) {
    std::ofstream ply_file(filename);

    if (!ply_file.is_open()) {
        logger().error("Could not open the file {}", filename);
        return;
    }

    ply_file << "ply\n";
    ply_file << "format ascii 1.0\n";
    ply_file << "element vertex " << points.rows() << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";
    ply_file << "end_header\n";

    for (int i = 0; i < points.rows(); ++i) {
        ply_file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2)
                 << "\n";
    }

    ply_file.close();
    logger().info("File saved as {}", filename);
}

void save_pts_and_color(Eigen::Ref<const ouster::sdk::core::MatrixX3dR> points,
                        const std::string& filename, int index) {
    std::ofstream ply_file(filename);

    if (!ply_file.is_open()) {
        logger().error("Could not open the file {}", filename);
        return;
    }

    ply_file << "ply\n";
    ply_file << "format ascii 1.0\n";
    ply_file << "element vertex " << points.rows() << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";
    ply_file << "property uchar red\n";
    ply_file << "property uchar green\n";
    ply_file << "property uchar blue\n";
    ply_file << "end_header\n";

    for (int i = 0; i < points.rows(); ++i) {
        // Set color to red for the specified index point, and black for others
        int red = (i == index) ? 255 : 20;
        int green = (i == index) ? 0 : 20;
        int blue = (i == index) ? 0 : 20;

        ply_file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2)
                 << " " << red << " " << green << " " << blue << "\n";
    }

    ply_file.close();
    logger().info("File saved as {}", filename);
}

std::vector<PoseH> deform_trajectory_relative_poses(
    const std::vector<PoseH>& original_poses,
    const std::vector<uint64_t>& timestamps, const PoseH& new_start_pose,
    const PoseH& new_end_pose) {
    const size_t num_poses = original_poses.size();
    if (num_poses <= 1 || timestamps.size() != num_poses) {
        logger().error(
            "Invalid input: need at least 2 poses and matching timestamps (got "
            "{} poses, {} timestamps)",
            num_poses, timestamps.size());
        return {};
    }

    // extract the original relative pose and sum them
    std::vector<PoseV> original_posevs;
    original_posevs.reserve(num_poses - 1);
    PoseV sum_orig(Eigen::Vector6d::Zero());
    for (size_t i = 0; i + 1 < num_poses; ++i) {
        PoseH delta_h =
            PoseH(original_poses[i].inverse()) * original_poses[i + 1];
        PoseV pose_vector = delta_h.log();
        original_posevs.push_back(pose_vector);
        sum_orig = sum_orig + pose_vector;
    }

    // compute the total desired posev
    PoseH total_delta = new_start_pose.inverse() * new_end_pose;
    PoseV desired_posev = total_delta.log();

    // distribute the correction across steps based on time deltas
    PoseV full_correction = desired_posev - sum_orig;
    const uint64_t start_time = timestamps.front();
    const uint64_t end_time = timestamps.back();
    const double total_dt = static_cast<double>(end_time - start_time);

    // build per‐interval corrections
    std::vector<PoseV> per_step_corrections;
    per_step_corrections.reserve(num_poses - 1);
    for (size_t i = 0; i + 1 < num_poses; ++i) {
        double alpha =
            (total_dt > 0.0)
                ? (static_cast<double>(timestamps[i + 1] - timestamps[i]) /
                   total_dt)
                : (1.0 / static_cast<double>(num_poses - 1));
        per_step_corrections.push_back(full_correction * alpha);
    }

    // re‐integrate the corrected pose to build a first-pass new_poses[]
    std::vector<PoseH> new_poses(num_poses);
    new_poses[0] = new_start_pose;
    PoseH curr = new_start_pose;
    for (size_t i = 0; i + 1 < num_poses; ++i) {
        PoseV mod_posev = original_posevs[i] + per_step_corrections[i];
        PoseH delta_mod = mod_posev.exp();
        curr = curr * delta_mod;
        new_poses[i + 1] = curr;
    }
    // compute any remaining end‐error
    PoseH end_error_h = new_end_pose * PoseH(new_poses.back().inverse());
    PoseV end_error_v = end_error_h.log();

    for (size_t i = 0; i < num_poses; ++i) {
        double alpha{};
        if (total_dt > 0.0) {
            alpha =
                (static_cast<double>(timestamps[i] - start_time)) / total_dt;
        } else {
            alpha = static_cast<double>(i) / static_cast<double>(num_poses - 1);
        }
        PoseH corr_h = PoseV(alpha * end_error_v).exp();
        new_poses[i] = corr_h * new_poses[i];
    }

    return new_poses;
}

std::string expand_home_path(const std::string& path) {
    // If the path is empty or doesn't start with '~', return it as is.
    if (path.empty() || path[0] != '~') {
        return path;
    }

    std::string home_path_str;

#ifdef _WIN32
    // On Windows, USERPROFILE is the most reliable environment variable.
    const char* home_env = std::getenv("USERPROFILE");

    // As a fallback, try to construct the path from HOMEDRIVE and HOMEPATH.
    if (home_env == nullptr) {
        const char* home_drive = std::getenv("HOMEDRIVE");
        const char* home_path = std::getenv("HOMEPATH");
        if (home_drive && home_path) {
            home_path_str = std::string(home_drive) + std::string(home_path);
        }
    } else {
        home_path_str = home_env;
    }
#else
    // On Linux, macOS, and other POSIX-compliant systems, HOME is standard.
    const char* home_env = std::getenv("HOME");
    if (home_env) {
        home_path_str = home_env;
    }
#endif

    // If a home directory was found, construct and return the final path.
    if (!home_path_str.empty()) {
        // Append the rest of the original path, skipping the '~' character.
        return home_path_str + path.substr(1);
    }

    return path;
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
