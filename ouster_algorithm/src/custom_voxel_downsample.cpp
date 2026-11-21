#include "ouster/algorithm/impl/custom_voxel_downsample.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

// PointNormalVoxelHashMap3d is internal to the SDK; see
// ouster/core/impl/voxel_hash_map_t.h.
#include <ouster/core/impl/voxel_hash_map_t.h>

namespace ouster {
namespace sdk {
namespace algorithm {
namespace impl {

namespace {
inline bool finite_vec3(const Eigen::Vector3d& vec) {
    return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z());
}
}  // namespace

std::pair<core::ArrayX3dR, core::ArrayX3dR> voxel_downsample_with_normals(
    Eigen::Ref<const core::ArrayX3dR> points, Eigen::Ref<const core::ArrayX3dR> normals,
    double voxel_size) {
    if (points.cols() != 3 || normals.cols() != 3) {
        throw std::invalid_argument("voxel_downsample_with_normals expects Nx3 inputs");
    }
    if (points.rows() != normals.rows()) {
        throw std::invalid_argument("voxel_downsample_with_normals points/normals size mismatch");
    }
    if (!(voxel_size > 0.0)) {
        throw std::invalid_argument("voxel_downsample_with_normals voxel_size must be > 0");
    }

    core::PointNormalVoxelHashMap3d map(voxel_size, std::numeric_limits<double>::max() / 2.0, 1, 1);

    core::PointNormalVoxelHashMap3d::point_type packed;
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3d point = points.row(i).matrix();
        Eigen::Vector3d normal = normals.row(i).matrix();
        if (!finite_vec3(point) || !finite_vec3(normal)) {
            continue;
        }
        const double n_len = normal.norm();
        if (n_len <= 1e-12) {
            continue;
        }
        normal /= n_len;
        packed.head<3>() = point;
        packed.tail<3>() = normal;
        map.add_point(packed);
    }

    const auto combined = map.pointcloud_vector();
    core::ArrayX3dR out_points(static_cast<Eigen::Index>(combined.size()), 3);
    core::ArrayX3dR out_normals(static_cast<Eigen::Index>(combined.size()), 3);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(combined.size()); ++i) {
        out_points.row(i) = combined[static_cast<std::size_t>(i)].head<3>().array();
        out_normals.row(i) = combined[static_cast<std::size_t>(i)].tail<3>().array();
    }
    return {std::move(out_points), std::move(out_normals)};
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<std::uint32_t>> voxel_downsample_with_index(
    const Eigen::Ref<const core::ArrayX3dR>& frame, const double voxel_size) {
    const Eigen::Index n_elems = frame.rows();
    if (n_elems == 0) {
        return {};
    }

    // Vector4d (fixed-size) rather than VectorXd: bucket points stay
    // stack-allocated end to end (packing, insertion, and reconstruction),
    // where VectorXd would heap-allocate on every single point.
    using IndexedRandomVoxelHashMap = core::VoxelHashMapT<Eigen::Vector3i, Eigen::Vector4d,
                                                          core::DefaultVoxelBucket<Eigen::Vector4d>,
                                                          core::impl::random_selection_strategy>;

    IndexedRandomVoxelHashMap map(
        voxel_size, std::numeric_limits<double>::max() / 2.0, /*max_points_per_voxel=*/1,
        /*min_points_per_voxel=*/1, core::AttributeLayout(), core::VHMInternalStorageMode::ABS);

    for (Eigen::Index idx = 0; idx < n_elems; ++idx) {
        Eigen::Vector4d point;
        point.head<3>() = frame.row(idx);
        point[3] = static_cast<double>(idx);
        map.add_point(point);
    }

    const core::ArrayXXdR points_with_idx = map.pointcloud_double();
    const Eigen::Index n_out = points_with_idx.rows();
    std::vector<Eigen::Vector3d> points_out(n_out);
    std::vector<std::uint32_t> indices_out(n_out);

    if (n_out > 0) {
        Eigen::Map<core::MatrixX3dR> points_map(reinterpret_cast<double*>(points_out.data()), n_out,
                                                3);
        Eigen::Map<Eigen::Matrix<std::uint32_t, Eigen::Dynamic, 1>> indices_map(indices_out.data(),
                                                                                n_out);

        points_map = points_with_idx.leftCols<3>();
        indices_map = points_with_idx.col(3).cast<std::uint32_t>();
    }

    return {std::move(points_out), std::move(indices_out)};
}

}  // namespace impl
}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
