#include "ouster/algorithm/voxel_downsample.h"

#include <ouster/core/voxel_hash_map.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace algorithm {

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

    Eigen::VectorXd packed(6);
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3d point = points.row(i).matrix();
        Eigen::Vector3d normal = normals.row(i).matrix();
        if (!finite_vec3(point) || !finite_vec3(normal)) continue;
        const double n_len = normal.norm();
        if (n_len <= 1e-12) continue;
        normal /= n_len;
        packed.head<3>() = point;
        packed.tail<3>() = normal;
        map.add_point(packed);
    }

    const std::vector<Eigen::VectorXd> combined = map.pointcloud_vector();
    core::ArrayX3dR out_points(static_cast<Eigen::Index>(combined.size()), 3);
    core::ArrayX3dR out_normals(static_cast<Eigen::Index>(combined.size()), 3);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(combined.size()); ++i) {
        out_points.row(i) = combined[static_cast<std::size_t>(i)].head<3>().array();
        out_normals.row(i) = combined[static_cast<std::size_t>(i)].tail<3>().array();
    }
    return {std::move(out_points), std::move(out_normals)};
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
