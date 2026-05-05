#pragma once

#include <stdexcept>

#include "ouster/impl/downsample_impl.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * [BETA] Downsample a pointcloud using a voxel grid of the requested
 * resolution.
 *
 * @tparam DerivedPts type of input Eigen points matrix
 * @tparam DerivedAttrib type of input Eigen attributes matrix
 * @tparam DerivedOutPts type of output Eigen points matrix
 * @tparam DerivedOutAttrib type of output Eigen attributes matrix
 *
 * @param[in]  voxel_size3        downsample voxel size
 * @param[in]  pts                Nx3 array of points to downsample
 * @param[in]  attribs            NxM array of attributes for each point to
 * downsample
 * @param[out] out_pts            Px3 downsampled points array
 * @param[in] out_attribs        PxM downsampled attributes array
 * @param[in]  min_pts_per_voxel  minimum number a points a voxel must contain
 * to count
 *
 * @remarks this is a beta feature and may change in future releases.
 */
template <typename DerivedPts, typename DerivedAttrib, typename DerivedOutPts,
          typename DerivedOutAttrib>
void voxel_downsample(
    const Eigen::Matrix<typename DerivedPts::Scalar, 3, 1>& voxel_size3,
    const DerivedPts& pts, const DerivedAttrib& attribs, DerivedOutPts& out_pts,
    DerivedOutAttrib& out_attribs, int min_pts_per_voxel = 1) {
    for (int i = 0; i < 3; i++) {
        if (voxel_size3[i] <= 0.0) {
            throw std::invalid_argument("Voxel size is zero or negative");
        }
    }

    bool has_attribute = (attribs.rows() != 0 && attribs.cols() != 0);
    if (has_attribute && attribs.rows() != pts.rows()) {
        throw std::invalid_argument("Invalid number of attributes (" +
                                    std::to_string(attribs.rows()) +
                                    "). Must match number of input points (" +
                                    std::to_string(pts.rows()) + ") or be 0.");
    }

    using AccPointT = ouster::sdk::core::impl::AccumulatedPoint<
        typename DerivedPts::Scalar, typename DerivedAttrib::Scalar>;
    using AttribVecT = typename AccPointT::AttribVecT;
    using PointT = typename AccPointT::PointT;
    std::unordered_map<Eigen::Vector3i, AccPointT,
                       ouster::sdk::core::impl::hash_eigen<Eigen::Vector3i>>
        voxel_grid;

    // Accumulate each point into the grid
    AttribVecT ref_attrib;
    Eigen::Vector3i voxel_index;
    for (int i = 0; i < pts.rows(); i++) {
        PointT ref_point(pts(i, 0), pts(i, 1), pts(i, 2));
        PointT ref_coord = ref_point.array() / voxel_size3.array();
        voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))),
            int(floor(ref_coord(2)));
        if (has_attribute) {
            ref_attrib = attribs.row(i);
        }
        voxel_grid[voxel_index].add_point(ref_point, ref_attrib);
    }

    // Reserve output
    int num_output_points = voxel_grid.size();
    out_pts.resize(num_output_points, 3);
    if (has_attribute) {
        out_attribs.resize(num_output_points, attribs.cols());
    }

    // Convert voxels back to output points if they have enough points inside
    int count = 0;
    for (const auto& point : voxel_grid) {
        if (point.second.num_points() < min_pts_per_voxel) {
            continue;
        }
        auto point_pos = point.second.average_point();
        for (int i = 0; i < 3; i++) {
            out_pts(count, i) = point_pos[i];
        }

        if (has_attribute) {
            // Assign the entire attribute row at once
            out_attribs.row(count) = point.second.average_attrib();
        }
        count += 1;
    }

    // Resize output back to actual size if we dropped some voxels
    if (count < num_output_points) {
        out_pts.conservativeResize(count, pts.cols());
        if (has_attribute) {
            out_attribs.conservativeResize(count, attribs.cols());
        }
    }
}

/**
 * [BETA] Downsample a pointcloud using a voxel grid of the requested
 * resolution.
 *
 * @tparam DerivedPts type of input Eigen points matrix
 * @tparam DerivedAttrib type of input Eigen attributes matrix
 * @tparam DerivedOutPts type of output Eigen points matrix
 * @tparam DerivedOutAttrib type of output Eigen attributes matrix
 *
 * @param[in]  voxel_size         downsample voxel size
 * @param[in]  pts                Nx3 array of points to downsample
 * @param[in]  attribs            NxM array of attributes for each point to
 * downsample
 * @param[out] out_pts            Px3 downsampled points array
 * @param[in] out_attribs        PxM downsampled attributes array
 * @param[in]  min_pts_per_voxel  minimum number of points a voxel must contain
 * to count
 *
 * @remarks this is a beta feature and may change in future releases.
 */
template <typename DerivedPts, typename DerivedAttrib, typename DerivedOutPts,
          typename DerivedOutAttrib>
void voxel_downsample(typename DerivedPts::Scalar voxel_size,
                      const DerivedPts& pts, const DerivedAttrib& attribs,
                      DerivedOutPts& out_pts, DerivedOutAttrib& out_attribs,
                      int min_pts_per_voxel = 1) {
    Eigen::Matrix<typename DerivedPts::Scalar, 3, 1> voxel_size3;
    voxel_size3(0, 0) = voxel_size;
    voxel_size3(1, 0) = voxel_size;
    voxel_size3(2, 0) = voxel_size;
    voxel_downsample(voxel_size3, pts, attribs, out_pts, out_attribs,
                     min_pts_per_voxel);
}
}  // namespace core
}  // namespace sdk
}  // namespace ouster
