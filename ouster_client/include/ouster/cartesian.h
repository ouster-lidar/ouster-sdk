#pragma once

#include <ouster/lidar_scan.h>

#include <cassert>
#include <cstdint>

#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace core {

// Users can enable OpenMP within ouster_client by first enabling omp through
// the compiler and then adding '-DOUSTER_OMP' option to the DCMAKE_CXX_FLAGS
#if defined(OUSTER_OMP)
#if defined(_OPENMP)
#define __OUSTER_UTILIZE_OPENMP__
#else
#pragma message("OUSTER_OMP was defined but openmp is not detected!")
#endif
#endif

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in, out] points The resulting point cloud, should be pre-allocated and
 * have the same dimensions as the direction array.
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 */
template <typename T>
void cartesianT(PointCloudXYZ<T>& points,
                const Eigen::Ref<const img_t<uint32_t>>& range,
                const ArrayX3R<T>& direction, const ArrayX3R<T>& offset) {
    assert(points.rows() == direction.rows() &&
           "points & direction row count mismatch");
    assert(points.rows() == offset.rows() &&
           "points & offset row count mismatch");
    assert(points.rows() == range.size() &&
           "points and range image size mismatch");

    const auto pts = points.data();
    const auto* const rng = range.data();
    const auto* const dir = direction.data();
    const auto* const ofs = offset.data();

    const auto N = range.size();

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (auto i = 0; i < N; ++i) {
        const auto r = rng[i];
        const auto idx_x = (i * 3) + 0;
        const auto idx_y = (i * 3) + 1;
        const auto idx_z = (i * 3) + 2;
        if (r == 0) {
            pts[idx_x] = pts[idx_y] = pts[idx_z] = static_cast<T>(0.0);
        } else {
            pts[idx_x] = r * dir[idx_x] + ofs[idx_x];
            pts[idx_y] = r * dir[idx_y] + ofs[idx_y];
            pts[idx_z] = r * dir[idx_z] + ofs[idx_z];
        }
    }
}

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 * @throw std::invalid_argument if the range image dimensions do not match
 */
template <typename T>
PointCloudXYZ<T> cartesianT(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const ArrayX3R<T>& direction,
                            const ArrayX3R<T>& offset) {
    if (range.cols() * range.rows() != direction.rows()) {
        throw std::invalid_argument("unexpected image dimensions");
    }
    PointCloudXYZ<T> points(direction.rows(), 3);
    cartesianT(points, range, direction, offset);
    return points;
}

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in] scan a LidarScan constaining a range image
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
template <typename T>
PointCloudXYZ<T> cartesianT(const LidarScan& scan, const ArrayX3R<T>& direction,
                            const ArrayX3R<T>& offset) {
    return cartesianT(scan.field(ChanField::RANGE), direction, offset);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
