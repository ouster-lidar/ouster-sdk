#pragma once

// Internal to the Ouster SDK: not part of the supported public API and may change
// without notice. Use voxel_downsample() / VoxelHashMap instead.

#include <Eigen/Core>
#include <cstdint>
#include <utility>
#include <vector>

#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace algorithm {
namespace impl {

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

/**
 * Reservoir-sample one representative point (and its original row index) per
 * voxel, via the same configurable random selection strategy used by
 * voxel_downsample_xd. Operates on a row-major Nx3 array so the whole frame
 * is packed and unpacked in a couple of bulk Eigen ops rather than one
 * heap-allocating Eigen::VectorXd per point.
 *
 * @param[in] frame       Row-major Nx3 array of points.
 * @param[in] voxel_size  Size of each voxel.
 * @return (kept points, original index of each kept point in @p frame).
 */
OUSTER_API_FUNCTION
std::pair<std::vector<Eigen::Vector3d>, std::vector<std::uint32_t>> voxel_downsample_with_index(
    const Eigen::Ref<const core::ArrayX3dR>& frame, double voxel_size);

/**
 * Convenience overload for callers already holding a std::vector<Eigen::Vector3d>.
 * Forwards to the array-based overload through a zero-copy Eigen::Map -- no
 * intermediate copy of @p frame is made.
 */
inline std::pair<std::vector<Eigen::Vector3d>, std::vector<std::uint32_t>>
voxel_downsample_with_index(const std::vector<Eigen::Vector3d>& frame, const double voxel_size) {
    if (frame.empty()) {
        return {};
    }
    const Eigen::Map<const core::ArrayX3dR> frame_view(frame.front().data(),
                                                       static_cast<Eigen::Index>(frame.size()), 3);
    return voxel_downsample_with_index(frame_view, voxel_size);
}

}  // namespace impl
}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
