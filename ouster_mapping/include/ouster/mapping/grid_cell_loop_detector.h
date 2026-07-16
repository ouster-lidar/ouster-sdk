#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/algorithm/impl/spatial_hash.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @brief Simple spatial-hash based loop detector for poses.
 *
 * Uses a 3D grid (cell size = cell_size_m) to find candidate revisits.
 *
 * Detection algorithm (for each incoming pose):
 * 1) Convert pose to a grid cell.
 * 2) Search the current cell and its 26 neighbors.
 * 3) Keep candidates within a Euclidean radius of one cell size.
 * 4) Keep only candidates that are sufficiently separated along the
 *    trajectory (path length difference >= min_distance_m).
 * 5) Sort candidates by spatial distance and return best-first indices.
 *
 * The detector also enforces a global gate: at least min_distance_m path
 * travel must occur between two successive loop-triggering insertions.
 */
class OUSTER_API_CLASS GridCellLoopDetector {
   public:
    /** @brief Spatial hash cell descriptor type. */
    using CellDescriptor = algorithm::impl::SpatialHashCell3D;
    /** @brief Hash functor for CellDescriptor. */
    using CellDescriptorHash = algorithm::impl::SpatialHashCell3DHash;

    /**
     * @param[in] min_distance_m Minimum travelled distance required between
     *            successive loop-closure additions (meters). This same value
     *            is also used as the minimum along-trajectory separation
     *            between the current pose and a candidate pose.
     * @param[in] cell_size_m Spatial hash grid cell size (meters).
     */
    OUSTER_API_FUNCTION
    GridCellLoopDetector(double min_distance_m, double cell_size_m = 1.0)
        : min_loop_distance_(min_distance_m), cell_size_(cell_size_m) {
        if (min_loop_distance_ <= 0.0) {
            throw std::invalid_argument("min_distance_m must be > 0");
        }
        if (cell_size_ <= 0.0) {
            throw std::invalid_argument("cell_size_m must be > 0");
        }
    }

    /**
     * @brief Add a pose and return loop candidates found.
     *
     * @param[in] pose 3D position (x,y,z) in meters.
     * @param[in] dist_travelled Cumulative path length in meters.
     * @return Indices of previously added poses that satisfy all loop criteria,
     * sorted from nearest to farthest in Euclidean distance.
     */
    OUSTER_API_FUNCTION
    std::vector<size_t> add_pose(const Eigen::Vector3d& pose, double dist_travelled) {
        // Keep cumulative distance non-decreasing even if callers provide
        // slightly noisy values.
        if (!trajectory_distances_.empty() && dist_travelled < trajectory_distances_.back()) {
            dist_travelled = trajectory_distances_.back();
        }

        const size_t curr_idx = trajectory_.size();
        const CellDescriptor curr_cell = get_cell(pose);
        std::vector<std::pair<size_t, double>> scored_candidates;
        scored_candidates.reserve(32);

        // Global gate: avoid adding loop-triggering poses too frequently.
        const bool allow_new_loop =
            (!has_last_loop_) || ((dist_travelled - last_loop_distance_) >= min_loop_distance_);
        if (allow_new_loop) {
            const double max_sq_dist = cell_size_ * cell_size_;

            // Search current cell and all 26 adjacent cells. The spatial hash
            // limits comparisons to local neighborhoods.
            algorithm::impl::for_each_neighbor_cell(curr_cell, [&](const CellDescriptor& neighbor) {
                auto it = grid_.find(neighbor);
                if (it == grid_.end()) return;

                for (const size_t old_idx : it->second) {
                    // Along-trajectory separation suppresses trivial
                    // matches with nearby sequential frames.
                    const double path_separation = dist_travelled - trajectory_distances_[old_idx];
                    if (path_separation < min_loop_distance_) {
                        continue;
                    }

                    const auto& old_pos = trajectory_[old_idx];
                    const double sq_dist = (pose - old_pos).squaredNorm();
                    // Radius gate keeps only geometrically-close revisits
                    // around the current pose.
                    if (sq_dist <= max_sq_dist) {
                        scored_candidates.emplace_back(old_idx, sq_dist);
                    }
                }
            });
        }

        // Prefer nearest candidates first. Tie-break on lower index for
        // deterministic output.
        std::sort(
            scored_candidates.begin(), scored_candidates.end(),
            [](const auto& lhs, const auto& rhs) {
                if (std::abs(lhs.second - rhs.second) > std::numeric_limits<double>::epsilon()) {
                    return lhs.second < rhs.second;
                }
                return lhs.first < rhs.first;
            });

        std::vector<size_t> found_loops;
        found_loops.reserve(scored_candidates.size());
        for (const auto& candidate : scored_candidates) {
            found_loops.push_back(candidate.first);
        }

        // Add current pose to the hash map after search to prevent self-match.
        trajectory_.push_back(pose);
        trajectory_distances_.push_back(dist_travelled);
        grid_[curr_cell].push_back(curr_idx);

        if (!found_loops.empty()) {
            last_loop_distance_ = dist_travelled;
            has_last_loop_ = true;
        }

        return found_loops;
    }

    /**
     * @brief Compute the grid cell for a given position.
     *
     * @param[in] pos 3D position.
     * @param[in] cell_size Grid cell size in meters.
     * @return The CellDescriptor for the cell containing @p pos.
     */
    OUSTER_API_FUNCTION
    static CellDescriptor compute_cell(const Eigen::Vector3d& pos, double cell_size) {
        return algorithm::impl::compute_cell_xyz(pos, cell_size);
    }

   private:
    CellDescriptor get_cell(const Eigen::Vector3d& pos) const {
        return compute_cell(pos, cell_size_);
    }

    double min_loop_distance_;
    double cell_size_;
    std::unordered_map<CellDescriptor, std::vector<size_t>, CellDescriptorHash> grid_;
    std::vector<Eigen::Vector3d> trajectory_;
    std::vector<double> trajectory_distances_;
    double last_loop_distance_ = 0.0;
    bool has_last_loop_ = false;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
