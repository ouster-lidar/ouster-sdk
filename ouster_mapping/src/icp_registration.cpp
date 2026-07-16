/** Copyright (c) 2026, Ouster Inc. All rights reserved.
 */

#include "ouster/mapping/icp_registration.h"

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sophus/se3.hpp>
#include <tuple>

using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::VoxelHashMap3d;
using ouster::sdk::core::VoxelHashMapXd;

namespace {
inline double square(double x) {
    return x * x;
}

void transform_points(const Sophus::SE3d& pose, std::vector<Eigen::Vector3d>& points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto& point) { return pose * point; });
}

template <typename MapT>
ouster::sdk::mapping::Correspondences data_association(const std::vector<Eigen::Vector3d>& points,
                                                       const MapT& voxel_map,
                                                       const double max_correspondance_distance) {
    const size_t num_pts = points.size();
    ouster::sdk::mapping::Correspondences output(num_pts);
    std::vector<uint8_t> valid(num_pts, 0);

    const double max_dist_sq = square(max_correspondance_distance);
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, num_pts), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                const auto& point = points[i];
                auto closest_neighbor_tuple = voxel_map.get_closest_neighbor(point, max_dist_sq);
                const auto& closest_neighbor = std::get<0>(closest_neighbor_tuple);
                const auto& distance_sq = std::get<1>(closest_neighbor_tuple);
                if (distance_sq < max_dist_sq) {
                    output[i] = {point, closest_neighbor.template head<3>()};
                    valid[i] = 1;
                }
            }
        });

    size_t out = 0;
    for (size_t i = 0; i < num_pts; ++i) {
        if (valid[i] != 0) {
            output[out++] = std::move(output[i]);
        }
    }
    output.resize(out);
    return output;
}

template <typename MapT>
Matrix4dR align_points_to_map_impl(const ouster::sdk::mapping::ICPRegistration& registration,
                                   const std::vector<Eigen::Vector3d>& frame, const MapT& voxel_map,
                                   const double max_correspondence_distance,
                                   const double kernel_scale) {
    if (voxel_map.empty()) {
        return Matrix4dR::Identity();
    }

    std::vector<Eigen::Vector3d> source = frame;

    Sophus::SE3d t_icp = Sophus::SE3d();
    for (int j = 0; j < registration.max_num_iterations_; ++j) {
        const auto correspondences =
            data_association(source, voxel_map, max_correspondence_distance);
        const auto linear_system =
            ouster::sdk::mapping::build_linear_system(correspondences, kernel_scale);
        const auto& jtj = linear_system.first;
        const auto& jtr = linear_system.second;
        const ouster::sdk::mapping::Vector6d dx = jtj.ldlt().solve(-jtr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        transform_points(estimation, source);
        t_icp = estimation * t_icp;
        if (dx.squaredNorm() < square(registration.convergence_criterion_)) {
            break;
        }
    }
    return t_icp.matrix();
}

}  // namespace

namespace ouster {
namespace sdk {
namespace mapping {

LinearSystem build_linear_system(const Correspondences& correspondences,
                                 const double kernel_scale) {
    auto sum_linear_systems = [](LinearSystem a, const LinearSystem& b) {
        a.first += b.first;
        a.second += b.second;
        return a;
    };

    // Geman-McClure robust kernel: down-weights large residuals (outliers)
    auto gm_weight = [&](const double& residual2) {
        return square(kernel_scale) / square(kernel_scale + residual2);
    };

    using correspondence_iterator = Correspondences::const_iterator;
    auto linear_system = tbb::parallel_deterministic_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend(), 128},
        // Identity
        LinearSystem(Matrix6d::Zero(), Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator>& r,
            LinearSystem linear_sys) -> LinearSystem {
            auto& jtj = linear_sys.first;
            auto& jtr = linear_sys.second;
            for (const auto& correspondence : r) {
                const auto& source = correspondence.first;
                const auto& target = correspondence.second;
                const Eigen::Vector3d residual = source - target;
                const double w = gm_weight(residual.squaredNorm());

                const double sx = source.x();
                const double sy = source.y();
                const double sz = source.z();
                const double wsx = w * sx;
                const double wsy = w * sy;
                const double wsz = w * sz;

                // Accumulate lower triangle of JᵀJ directly, exploiting the
                // known sparsity of J_r = [I₃ | -hat(s)]:
                //
                //   JᵀJ = [    I₃           -hat(s)       ]
                //          [  hat(s)    |s|²·I₃ − s·sᵀ     ]
                //
                // Only the lower triangle is filled; Eigen LDLT reads Lower
                // by default.

                // Top-left 3×3: w·I₃
                jtj(0, 0) += w;
                jtj(1, 1) += w;
                jtj(2, 2) += w;

                // Bottom-left 3×3: w·hat(s)
                jtj(3, 1) -= wsz;
                jtj(3, 2) += wsy;
                jtj(4, 0) += wsz;
                jtj(4, 2) -= wsx;
                jtj(5, 0) -= wsy;
                jtj(5, 1) += wsx;

                // Bottom-right 3×3: w·(|s|²·I₃ − s·sᵀ)
                const double wsx2 = wsx * sx;
                const double wsy2 = wsy * sy;
                const double wsz2 = wsz * sz;
                jtj(3, 3) += wsy2 + wsz2;
                jtj(4, 3) -= wsx * sy;
                jtj(4, 4) += wsx2 + wsz2;
                jtj(5, 3) -= wsx * sz;
                jtj(5, 4) -= wsy * sz;
                jtj(5, 5) += wsx2 + wsy2;

                // Jᵀr = w·[r ; s × r]
                jtr.head<3>() += w * residual;
                jtr.tail<3>() += w * source.cross(residual);
            }
            return linear_sys;
        },
        // 2nd Lambda: Parallel reduction
        sum_linear_systems);

    return linear_system;
}

ICPRegistration::ICPRegistration(int max_num_iteration, double convergence_criterion,
                                 int max_num_threads)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      // Only manipulate the number of threads if the user specifies something
      // greater than 0
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()) {
    // This global variable requires static duration storage to be able to
    // manipulate the max concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

Matrix4dR ICPRegistration::align_points_to_map(const std::vector<Eigen::Vector3d>& frame,
                                               const VoxelHashMap3d& voxel_map,
                                               const double max_distance,
                                               const double kernel_scale) const {
    return align_points_to_map_impl(*this, frame, voxel_map, max_distance, kernel_scale);
}

Matrix4dR ICPRegistration::align_points_to_map(const std::vector<Eigen::Vector3d>& frame,
                                               const VoxelHashMapXd& voxel_map,
                                               const double max_distance,
                                               const double kernel_scale) const {
    return align_points_to_map_impl(*this, frame, voxel_map, max_distance, kernel_scale);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
