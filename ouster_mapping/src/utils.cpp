#include "ouster/mapping/impl/utils.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "ouster/core/impl/logging.h"

using ouster::sdk::core::logger;
using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseV;
namespace ouster {
namespace sdk {
namespace mapping {

double pose_dist(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
    return (point1 - point2).norm();
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
        ply_file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2) << "\n";
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

        ply_file << points(i, 0) << " " << points(i, 1) << " " << points(i, 2) << " " << red << " "
                 << green << " " << blue << "\n";
    }

    ply_file.close();
    logger().info("File saved as {}", filename);
}

std::vector<PoseH> deform_trajectory_relative_poses(const std::vector<PoseH>& original_poses,
                                                    const std::vector<uint64_t>& timestamps,
                                                    const PoseH& new_start_pose,
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
        PoseH delta_h = PoseH(original_poses[i].inverse()) * original_poses[i + 1];
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
        double alpha = (total_dt > 0.0)
                           ? (static_cast<double>(timestamps[i + 1] - timestamps[i]) / total_dt)
                           : (1.0 / static_cast<double>(num_poses - 1));
        per_step_corrections.emplace_back(full_correction * alpha);
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
            alpha = (static_cast<double>(timestamps[i] - start_time)) / total_dt;
        } else {
            alpha = static_cast<double>(i) / static_cast<double>(num_poses - 1);
        }
        PoseH corr_h = PoseV(alpha * end_error_v).exp();
        new_poses[i] = corr_h * new_poses[i];
    }

    return new_poses;
}

Eigen::Vector2d relative_xy_from_wgs84(double lat, double lon, double lat0, double lon0) {
    constexpr double earth_equator_radius = 6378137.0;
    constexpr double earth_eccentricity = 0.08181919084261;
    constexpr double deg_to_rad = M_PI / 180.0;

    const double lat0_rad = lat0 * deg_to_rad;
    const double lon0_rad = lon0 * deg_to_rad;
    const double lat_rad = lat * deg_to_rad;
    const double lon_rad = lon * deg_to_rad;

    double ecc_term = earth_eccentricity * std::sin(lat0_rad);
    ecc_term = 1.0 - ecc_term * ecc_term;
    const double rho_e = earth_equator_radius * (1.0 - earth_eccentricity * earth_eccentricity) /
                         (std::sqrt(ecc_term) * ecc_term);
    const double rho_n = earth_equator_radius / std::sqrt(ecc_term);
    const double rho_lat = rho_e;
    const double rho_lon = rho_n * std::cos(lat0_rad);

    const double d_lat = (lat_rad - lat0_rad) * rho_lat;
    const double d_lon = (lon_rad - lon0_rad) * rho_lon;

    return Eigen::Vector2d(d_lon, d_lat);
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
    if (home_env != nullptr) {
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
