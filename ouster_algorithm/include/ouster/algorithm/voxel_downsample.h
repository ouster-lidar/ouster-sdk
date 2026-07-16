#pragma once

#include <Eigen/Core>
#include <utility>

#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace algorithm {

/**
 * Voxel-grid downsample points and normals.
 *
 * Points are averaged per voxel. Input normals are normalized, accumulated,
 * and the resulting per-voxel normal is normalized before output.
 *
 * @throws std::invalid_argument If either input does not have three columns,
 * their row counts differ, or @p voxel_size is not greater than zero.
 */
OUSTER_API_FUNCTION
std::pair<core::ArrayX3dR, core::ArrayX3dR> voxel_downsample_with_normals(
    Eigen::Ref<const core::ArrayX3dR> points, Eigen::Ref<const core::ArrayX3dR> normals,
    double voxel_size);

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
