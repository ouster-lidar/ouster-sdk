#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "ouster/core/impl/transformation.h"
#include "ouster/core/typedefs.h"

namespace ouster {
namespace sdk {

namespace mapping {

double pose_dist(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);

void save_to_ply(Eigen::Ref<const ouster::sdk::core::MatrixX3dR> points,
                 const std::string& filename);

void save_pts_and_color(Eigen::Ref<const ouster::sdk::core::MatrixX3dR> points,
                        const std::string& filename, int index);

std::vector<ouster::sdk::core::impl::PoseH> deform_trajectory_relative_poses(
    const std::vector<ouster::sdk::core::impl::PoseH>& original_poses,
    const std::vector<uint64_t>& timestamps, const ouster::sdk::core::impl::PoseH& new_start_pose,
    const ouster::sdk::core::impl::PoseH& new_end_pose);

Eigen::Vector2d relative_xy_from_wgs84(double lat, double lon, double lat0, double lon0);

// resolve home-relative paths in file names
std::string expand_home_path(const std::string& path);

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
