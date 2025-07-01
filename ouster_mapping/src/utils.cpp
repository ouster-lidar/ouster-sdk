#include "ouster/impl/utils.h"

#include <fstream>

#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "ouster/impl/logging.h"
#include "ouster/impl/transformation.h"

namespace ouster {
namespace mapping {

using ouster::sensor::logger;

double pose_dist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return (p1 - p2).norm();
}

std::vector<Eigen::Vector3d> convert_array_to_vector(
    const Eigen::Array<double, Eigen::Dynamic, 3>& arr) {
    std::vector<Eigen::Vector3d> result;
    result.reserve(arr.rows());  // Reserve space
    for (Eigen::Index i = 0; i < arr.rows(); ++i) {
        result.emplace_back(arr(i, 0), arr(i, 1), arr(i, 2));
    }
    return result;
}

Eigen::Array<double, Eigen::Dynamic, 3> convert_vector_to_array(
    const std::vector<Eigen::Vector3d>& vec) {
    Eigen::Array<double, Eigen::Dynamic, 3> arr(vec.size(), 3);
    for (size_t i = 0; i < vec.size(); ++i) {
        arr.row(i) << vec[i].x(), vec[i].y(), vec[i].z();
    }
    return arr;
}

Eigen::Array<double, Eigen::Dynamic, 3> run_KISS_ICP_downsample(
    const Eigen::Array<double, Eigen::Dynamic, 3>& source_points,
    double voxel_size) {
    std::vector<Eigen::Vector3d> points_vector =
        convert_array_to_vector(source_points);
    std::vector<Eigen::Vector3d> pts_downsample_vector =
        kiss_icp::VoxelDownsample(points_vector, voxel_size);
    Eigen::Array<double, Eigen::Dynamic, 3> pts_downsample_array =
        convert_vector_to_array(pts_downsample_vector);

    return pts_downsample_array;
}

Points transform_points(const Points& points, const ouster::impl::PoseH& pose) {
    Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

    Eigen::MatrixXd points_matrix = points.matrix();
    Eigen::MatrixXd transformed_points_matrix =
        (points_matrix * rotation.transpose()).rowwise() +
        translation.transpose();

    Points transformed_points = transformed_points_matrix.array();
    return transformed_points;
}

Eigen::Matrix4d run_KISS_ICP_matching(
    const Eigen::Array<double, Eigen::Dynamic, 3>& source_points,
    const Eigen::Array<double, Eigen::Dynamic, 3>& target_points) {
    Sophus::SE3d identity_matrix;
    // max_num_iterations, convergence_criterion, max_num_threads
    kiss_icp::Registration registration(500, 0.0001, 0);
    // Voxel_size, max_range, max_points_per_voxel
    kiss_icp::VoxelHashMap local_map(1.0, 200, 20);

    auto source_array = convert_array_to_vector(source_points);
    auto target_array = convert_array_to_vector(target_points);

    auto source_array_down = kiss_icp::VoxelDownsample(source_array, 0.5);
    auto target_array_down = kiss_icp::VoxelDownsample(target_array, 0.5);

    local_map.Update(target_array_down, identity_matrix);

    const auto new_pose =
        registration.AlignPointsToMap(source_array_down,  // frame
                                      local_map,          // voxel_map
                                      identity_matrix,    // initial_guess
                                      6,  // max_correspondence_dist
                                      2);
    return new_pose.matrix();
}

void save_to_PLY(const Eigen::Array<double, Eigen::Dynamic, 3>& points,
                 const std::string& filename) {
    std::ofstream plyFile(filename);

    if (!plyFile.is_open()) {
        logger().error("Could not open the file {}", filename);
        return;
    }

    plyFile << "ply\n";
    plyFile << "format ascii 1.0\n";
    plyFile << "element vertex " << points.rows() << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "end_header\n";

    for (int i = 0; i < points.rows(); ++i) {
        plyFile << points(i, 0) << " " << points(i, 1) << " " << points(i, 2)
                << "\n";
    }

    plyFile.close();
    logger().info("File saved as {}", filename);
}

void save_pts_and_color(const Eigen::Array<double, Eigen::Dynamic, 3>& points,
                        const std::string& filename, int index) {
    std::ofstream plyFile(filename);

    if (!plyFile.is_open()) {
        logger().error("Could not open the file {}", filename);
        return;
    }

    plyFile << "ply\n";
    plyFile << "format ascii 1.0\n";
    plyFile << "element vertex " << points.rows() << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "property uchar red\n";
    plyFile << "property uchar green\n";
    plyFile << "property uchar blue\n";
    plyFile << "end_header\n";

    for (int i = 0; i < points.rows(); ++i) {
        // Set color to red for the specified index point, and black for others
        int red = (i == index) ? 255 : 20;
        int green = (i == index) ? 0 : 20;
        int blue = (i == index) ? 0 : 20;

        plyFile << points(i, 0) << " " << points(i, 1) << " " << points(i, 2)
                << " " << red << " " << green << " " << blue << "\n";
    }

    plyFile.close();
    logger().info("File saved as {}", filename);
}

int first_valid_col(const ouster::LidarScan& ls) {
    auto status = ls.status();
    for (size_t i = 0; i < ls.w; ++i) {
        if (status[i] & 0x01) {
            return i;
        }
    }
    return -1;
}

int last_valid_col(const ouster::LidarScan& ls) {
    auto status = ls.status();
    for (int i = ls.w - 1; i >= 0; --i) {
        if (status[i] & 0x01) {
            return i;
        }
    }
    return -1;
}

std::vector<ouster::impl::PoseH> deform_trajectory_relative_poses(
    const std::vector<ouster::impl::PoseH>& original_poses,
    const std::vector<uint64_t>& timestamps,
    const ouster::impl::PoseH& new_start_pose,
    const ouster::impl::PoseH& new_end_pose) {
    const size_t n = original_poses.size();
    if (n <= 1 || timestamps.size() != n) {
        logger().error(
            "Invalid input: need at least 2 poses and matching timestamps (got "
            "{} poses, {} timestamps)",
            n, timestamps.size());
        return {};
    }

    // extract the original relative pose and sum them
    std::vector<ouster::impl::PoseV> original_posevs;
    original_posevs.reserve(n - 1);
    ouster::impl::PoseV sum_orig(Eigen::Vector6d::Zero());
    for (size_t i = 0; i + 1 < n; ++i) {
        ouster::impl::PoseH delta_h =
            ouster::impl::PoseH(original_poses[i].inverse()) *
            original_poses[i + 1];
        ouster::impl::PoseV pv = delta_h.log();
        original_posevs.push_back(pv);
        sum_orig = sum_orig + pv;
    }

    // compute the total desired posev
    ouster::impl::PoseH total_delta = new_start_pose.inverse() * new_end_pose;
    ouster::impl::PoseV desired_posev = total_delta.log();

    // distribute the correction across steps based on time deltas
    ouster::impl::PoseV full_correction = desired_posev - sum_orig;
    const uint64_t t0 = timestamps.front();
    const uint64_t tn = timestamps.back();
    const double total_dt = static_cast<double>(tn - t0);

    // build per‐interval corrections
    std::vector<ouster::impl::PoseV> per_step_corrections;
    per_step_corrections.reserve(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        double alpha =
            (total_dt > 0.0)
                ? (static_cast<double>(timestamps[i + 1] - timestamps[i]) /
                   total_dt)
                : (1.0 / static_cast<double>(n - 1));
        per_step_corrections.push_back(full_correction * alpha);
    }

    // re‐integrate the corrected pose to build a first-pass new_poses[]
    std::vector<ouster::impl::PoseH> new_poses(n);
    new_poses[0] = new_start_pose;
    ouster::impl::PoseH curr = new_start_pose;
    for (size_t i = 0; i + 1 < n; ++i) {
        ouster::impl::PoseV mod_posev =
            original_posevs[i] + per_step_corrections[i];
        ouster::impl::PoseH delta_mod = mod_posev.exp();
        curr = curr * delta_mod;
        new_poses[i + 1] = curr;
    }

    // compute any remaining end‐error
    ouster::impl::PoseH end_error_h =
        new_end_pose * ouster::impl::PoseH(new_poses.back().inverse());
    ouster::impl::PoseV end_error_v = end_error_h.log();

    for (size_t i = 0; i < n; ++i) {
        double alpha;
        if (total_dt > 0.0) {
            alpha = (static_cast<double>(timestamps[i] - t0)) / total_dt;
        } else {
            alpha = static_cast<double>(i) / static_cast<double>(n - 1);
        }
        ouster::impl::PoseH corr_h =
            (ouster::impl::PoseV(alpha * end_error_v)).exp();
        new_poses[i] = corr_h * new_poses[i];
    }

    return new_poses;
}

}  // namespace mapping
}  // namespace ouster
