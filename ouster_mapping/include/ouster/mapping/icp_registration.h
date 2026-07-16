/** Copyright (c) 2026, Ouster Inc. All rights reserved.
 */

#pragma once

#include <ouster/core/typedefs.h>
#include <ouster/core/voxel_hash_map.h>

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Correspondences = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Matrix6d, Vector6d>;

/// @brief Build linear system for ICP registration.
/// @param[in] correspondences Correspondences.
/// @param[in] kernel_scale Kernel scale.
/// @return Linear system.
OUSTER_API_FUNCTION
LinearSystem build_linear_system(const Correspondences& correspondences, double kernel_scale);

/// @brief ICP registration for point cloud mapping.
struct OUSTER_API_CLASS ICPRegistration {
    /// @brief Constructor for ICP registration.
    /// @param[in] max_num_iteration Maximum number of ICP iterations.
    /// @param[in] convergence_criterion Convergence criterion.
    /// @param[in] max_num_threads Maximum number of threads.
    OUSTER_API_FUNCTION
    explicit ICPRegistration(int max_num_iteration = 50, double convergence_criterion = 0.0001,
                             int max_num_threads = 0);

    /// @brief Align points to map using 3D voxel map.
    /// @param[in] frame Points to align.
    /// @param[in] voxel_map 3D voxel map.
    /// @param[in] max_distance Maximum distance to search for correspondences.
    /// @param[in] kernel_scale Kernel scale.
    /// @return Aligned points.
    OUSTER_API_FUNCTION
    core::Matrix4dR align_points_to_map(const std::vector<Eigen::Vector3d>& frame,
                                        const ouster::sdk::core::VoxelHashMap3d& voxel_map,
                                        const double max_distance, const double kernel_scale) const;

    /// @brief Align points to map using X-dimensional voxel map.
    /// @param[in] frame Points to align.
    /// @param[in] voxel_map X-dimensional voxel map.
    /// @param[in] max_distance Maximum distance to search for correspondences.
    /// @param[in] kernel_scale Kernel scale.
    /// @return Aligned points.
    OUSTER_API_FUNCTION
    core::Matrix4dR align_points_to_map(const std::vector<Eigen::Vector3d>& frame,
                                        const ouster::sdk::core::VoxelHashMapXd& voxel_map,
                                        const double max_distance, const double kernel_scale) const;

    /// @brief Maximum number of ICP iterations.
    int max_num_iterations_;
    /// @brief Convergence criterion.
    double convergence_criterion_;
    /// @brief Maximum number of threads.
    int max_num_threads_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
