#pragma once

#include <ouster/lidar_scan.h>

namespace ouster {

// Users can enable OpenMP within ouster_client by first enabling omp through
// the compiler and then adding '-DOUSTER_OMP' option to the DCMAKE_CXX_FLAGS
#if defined(OUSTER_OMP)
#if defined(_OPENMP)
#define __OUSTER_UTILIZE_OPENMP__
#else
#pragma message("OUSTER_OMP was defined but openmp is not detected!")
#endif
#endif

template <typename T>
using PointsT = Eigen::Array<T, -1, 3>;
using PointsD = PointsT<double>;
using PointsF = PointsT<float>;

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
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
template <typename T>
void cartesianT(PointsT<T>& points,
                const Eigen::Ref<const img_t<uint32_t>>& range,
                const PointsT<T>& direction, const PointsT<T>& offset) {
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
    const auto col_x = 0 * N;  // 1st column of points (x)
    const auto col_y = 1 * N;  // 2nd column of points (y)
    const auto col_z = 2 * N;  // 3rd column of points (z)

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (auto i = 0; i < N; ++i) {
        const auto r = rng[i];
        const auto idx_x = col_x + i;
        const auto idx_y = col_y + i;
        const auto idx_z = col_z + i;
        if (r == 0) {
            pts[idx_x] = pts[idx_y] = pts[idx_z] = static_cast<T>(0.0);
        } else {
            pts[idx_x] = r * dir[idx_x] + ofs[idx_x];
            pts[idx_y] = r * dir[idx_y] + ofs[idx_y];
            pts[idx_z] = r * dir[idx_z] + ofs[idx_z];
        }
    }
}

}  // namespace ouster
