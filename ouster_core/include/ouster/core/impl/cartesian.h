#pragma once

#include <ouster/core/lidar_frame.h>

#include <cassert>
#include <cmath>
#include <cstdint>

#include "ouster/core/types.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

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
 * Converts the range measurement at `index` to a Cartesian point; zero range
 * yields the origin.
 */
template <typename T>
inline void cartesian_point(T* pts, const uint32_t* rng, const T* dir, const T* ofs,
                            Eigen::Index index) {
    const auto raw = rng[index];
    const T range = static_cast<T>(raw);
    const Eigen::Index base = index * 3;
    pts[base + 0] = raw != 0 ? (range * dir[base + 0]) + ofs[base + 0] : static_cast<T>(0.0);
    pts[base + 1] = raw != 0 ? (range * dir[base + 1]) + ofs[base + 1] : static_cast<T>(0.0);
    pts[base + 2] = raw != 0 ? (range * dir[base + 2]) + ofs[base + 2] : static_cast<T>(0.0);
}

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in, out] points The resulting point cloud, should be pre-allocated and
 * have the same dimensions as the direction array.
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarFrame.
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 */
template <typename T>
void cartesianT(Eigen::Ref<PointCloudXYZ<T>> points, const Eigen::Ref<const img_t<uint32_t>>& range,
                const ArrayX3R<T>& direction, const ArrayX3R<T>& offset) {
    assert(points.rows() == direction.rows() && "points & direction row count mismatch");
    assert(points.rows() == offset.rows() && "points & offset row count mismatch");
    assert(points.rows() == range.size() && "points and range image size mismatch");

    const auto pts = points.data();
    const auto* const rng = range.data();
    const auto* const dir = direction.data();
    const auto* const ofs = offset.data();

    const Eigen::Index num_points = range.size();
    const Eigen::Index num_pairs = num_points / 2;

    // Branch-free and two points per iteration: a branchy one-point loop is
    // front-end bound, and its speed swings by up to ~20% with where the
    // compiler happens to place it in the binary.
#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (Eigen::Index pair = 0; pair < num_pairs; ++pair) {
        cartesian_point<T>(pts, rng, dir, ofs, 2 * pair);
        cartesian_point<T>(pts, rng, dir, ofs, (2 * pair) + 1);
    }
    if (num_points % 2 != 0) {
        cartesian_point<T>(pts, rng, dir, ofs, num_points - 1);
    }
}

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarFrame.
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarFrame where i = row * w + col.
 * @throw std::invalid_argument if the range image dimensions do not match
 */
template <typename T>
PointCloudXYZ<T> cartesianT(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const ArrayX3R<T>& direction, const ArrayX3R<T>& offset) {
    if (range.cols() * range.rows() != direction.rows()) {
        throw std::invalid_argument("unexpected image dimensions");
    }
    PointCloudXYZ<T> points(direction.rows(), 3);
    cartesianT<T>(points, range, direction, offset);
    return points;
}

/**
 * Converts a staggered range image to Cartesian points.
 *
 * @param[in] frame a LidarFrame containing a range image
 * @param[in] direction the direction of an xyz lut.
 * @param[in] offset the offset of an xyz lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarFrame where i = row * w + col.
 */
template <typename T>
PointCloudXYZ<T> cartesianT(const LidarFrame& frame, const ArrayX3R<T>& direction,
                            const ArrayX3R<T>& offset) {
    return cartesianT<T>(frame.field(ChanField::RANGE), direction, offset);
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
