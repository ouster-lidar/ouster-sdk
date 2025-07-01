#pragma once

#include <ouster/lidar_scan.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <string>

#include "ouster/impl/transformation.h"

namespace ouster {
namespace mapping {
using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

// TODO replace it with the ouster tranform function later.
Points transform_points(const Points& points, const ouster::impl::PoseH& pose);

double pose_dist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

Eigen::Array<double, Eigen::Dynamic, 3> run_KISS_ICP_downsample(
    const Eigen::Array<double, Eigen::Dynamic, 3>& source_points,
    double voxel_size = 0.15);

Eigen::Matrix4d run_KISS_ICP_matching(
    const Eigen::Array<double, Eigen::Dynamic, 3>& source_points,
    const Eigen::Array<double, Eigen::Dynamic, 3>& target_points);

void save_to_PLY(const Eigen::Array<double, Eigen::Dynamic, 3>& points,
                 const std::string& filename);

void save_pts_and_color(const Eigen::Array<double, Eigen::Dynamic, 3>& points,
                        const std::string& filename, int index);

// TODO move it to the ouster-sw util somewhere

int first_valid_col(const ouster::LidarScan& ls);

int last_valid_col(const ouster::LidarScan& ls);

std::vector<ouster::impl::PoseH> deform_trajectory_relative_poses(
    const std::vector<ouster::impl::PoseH>& original_poses,
    const std::vector<uint64_t>& timestamps,
    const ouster::impl::PoseH& new_start_pose,
    const ouster::impl::PoseH& new_end_pose);

}  // namespace mapping
}  // namespace ouster
