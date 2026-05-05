#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "ouster/impl/transformation.h"
#include "ouster/typedefs.h"

namespace ouster {
namespace sdk {

namespace mapping {

double pose_dist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

Eigen::ArrayX3d run_kiss_icp_downsample(
    Eigen::Ref<const Eigen::ArrayX3d> source_points, double voxel_size = 0.15);

Eigen::Matrix4d run_kiss_icp_matching(
    Eigen::Ref<const Eigen::ArrayX3d> source_points,
    Eigen::Ref<const Eigen::ArrayX3d> target_points,
    const ouster::sdk::core::impl::PoseH& initial_guess =
        ouster::sdk::core::impl::PoseH());

void save_to_ply(Eigen::Ref<const ouster::sdk::core::MatrixX3dR> points,
                 const std::string& filename);

void save_pts_and_color(Eigen::Ref<const ouster::sdk::core::MatrixX3dR> points,
                        const std::string& filename, int index);

std::vector<ouster::sdk::core::impl::PoseH> deform_trajectory_relative_poses(
    const std::vector<ouster::sdk::core::impl::PoseH>& original_poses,
    const std::vector<uint64_t>& timestamps,
    const ouster::sdk::core::impl::PoseH& new_start_pose,
    const ouster::sdk::core::impl::PoseH& new_end_pose);

// resolve home-relative paths in file names
std::string expand_home_path(const std::string& path);

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
