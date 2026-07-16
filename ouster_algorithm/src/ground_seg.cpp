#include "ouster/algorithm/ground_seg.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/algorithm/impl/ground_seg_detail.h"
#include "ouster/algorithm/normals.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/xyzlut.h"

using ouster::sdk::core::logger;
using ouster::sdk::core::MatrixX3dR;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::impl::PoseH;

namespace ouster {
namespace sdk {
namespace algorithm {

namespace {

// 8-connected neighbor offsets, shared by BFS pruning and component analysis.
constexpr std::array<std::array<int, 2>, 8> NEIGHBOR_DELTAS = {
    {{{-1, -1}}, {{-1, 0}}, {{-1, 1}}, {{0, -1}}, {{0, 1}}, {{1, -1}}, {{1, 0}}, {{1, 1}}}};

// -----------------------------------------------------------------------
// Shared numeric guards and robust-statistics scale factors.
// -----------------------------------------------------------------------
constexpr double MIN_RANGE_M = 0.15;
constexpr double NORMAL_EPS = 1e-6;
constexpr double MAD_TO_SIGMA = 1.4826;

// -----------------------------------------------------------------------
// Bounds used to size the XY ground grid and classify compact indoor scenes.
// -----------------------------------------------------------------------
constexpr double XY_BOUNDS_PERCENTILE = 0.95;
constexpr double XY_GROUND_INDOOR_SCENE_BOUND_M = 25.0;
constexpr double XY_GROUND_INDOOR_MAX_Z_ABOVE_ESTIMATE_M = 0.3;

// -----------------------------------------------------------------------
// Low-tail range used for the global fallback ground-height estimate.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_TAIL_LOW_PCT = 0.01;
constexpr double XY_GROUND_TAIL_HIGH_PCT = 0.20;

// -----------------------------------------------------------------------
// Per-cell lower-envelope sampling and roughness estimation.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_CELL_LOW_PERCENTILE = 0.15;
constexpr double XY_GROUND_CELL_ROUGHNESS_BAND_M = 0.45;

// -----------------------------------------------------------------------
// Normal gates prefer near-horizontal surfaces when normals are available.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_NORMAL_Z_PREFILTER_MIN = 0.15;
constexpr double XY_GROUND_POINT_NORMAL_Z_MIN = 0.15;
// Max abs(normal.z) for an elevated point to be treated as wall-like.
constexpr double XY_GROUND_WALL_NORMAL_Z_MAX = 0.45;
// Height above local ground required before the wall-like normal veto applies.
constexpr double XY_GROUND_WALL_ABOVE_LOCAL_M = 0.20;
constexpr size_t XY_GROUND_NORMAL_FILTERED_MIN_POINTS = 4u;

// -----------------------------------------------------------------------
// Hole filling and smoothing keep sparse ground connected without crossing
// sharp height edges. The final fill after component rejection uses a
// smaller radius to avoid over-expanding into object boundaries.
// -----------------------------------------------------------------------
constexpr int XY_GROUND_INITIAL_FILL_RADIUS_CELLS = 6;
constexpr int XY_GROUND_FINAL_FILL_RADIUS_CELLS = 3;
constexpr double XY_GROUND_FILL_MAX_NEIGHBOR_SPREAD_M = 0.65;
constexpr double XY_GROUND_SMOOTH_MAX_HEIGHT_DIFF_M = 0.55;

// -----------------------------------------------------------------------
// Connectivity pruning removes isolated elevated surfaces while preserving
// sloped ground connected to the global fallback height.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_MODEL_ANCHOR_ABOVE_FALLBACK_M = 1.10;
constexpr double XY_GROUND_MODEL_MAX_NEIGHBOR_STEP_M = 0.75;
constexpr double XY_GROUND_MODEL_SLOPE_PER_M = 0.80;
constexpr double XY_GROUND_MODEL_PRUNE_MIN_ABOVE_FALLBACK_M = 1.50;

// -----------------------------------------------------------------------
// Connected-component height validation: after model construction, reject
// small elevated components that are disconnected from the main ground
// surface.  This catches rooftops, truck tops, and elevated platforms that
// BFS pruning alone may miss (because they can be locally connected).
// -----------------------------------------------------------------------
constexpr double XY_GROUND_COMPONENT_CLOSE_TO_MAIN_M = 0.50;
constexpr double XY_GROUND_COMPONENT_HIGH_ABOVE_MAIN_M = 1.20;
constexpr double XY_GROUND_COMPONENT_MODERATE_MIN_AREA_FRAC = 0.20;
// Very elevated components (> HIGH_ABOVE_MAIN_M) are kept only when they
// are near-dominant in area — 0.80 effectively rejects rooftops / elevated
// platforms in all typical street / parking-lot / yard scenes.
constexpr double XY_GROUND_COMPONENT_HIGH_MIN_AREA_FRAC = 0.80;
constexpr double XY_GROUND_COMPONENT_MAX_STEP_M = 0.45;

// -----------------------------------------------------------------------
// Local lookup and classification tolerances for testing points against
// the 2.5-D ground model.
// -----------------------------------------------------------------------
constexpr int XY_GROUND_LOOKUP_RADIUS_CELLS = 8;
constexpr double XY_GROUND_LOCAL_BASE_TOL_M = 0.50;
constexpr double XY_GROUND_LOCAL_ROUGHNESS_K = 2.5;
constexpr double XY_GROUND_RANGE_NOISE_K = 0.01;
constexpr double XY_GROUND_RANGE_NOISE_MAX_M = 0.50;
constexpr double XY_GROUND_LOOKUP_SLOPE_TOL_PER_M = 0.20;
constexpr double XY_GROUND_LOOKUP_SLOPE_TOL_MAX_M = 0.45;
constexpr double XY_GROUND_OUTDOOR_MAX_Z_ABOVE_ESTIMATE_M = 1.20;
constexpr double XY_GROUND_MAX_ABOVE_LOCAL_M = 0.50;
constexpr double XY_GROUND_MAX_BELOW_LOCAL_M = 1.20;
constexpr double XY_GROUND_UNSUPPORTED_FALLBACK_ABOVE_M = 0.90;

// -----------------------------------------------------------------------
// Raw floor estimate retained per cell for obstacle-column checks.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_CELL_FLOOR_PERCENTILE = 0.05;

// -----------------------------------------------------------------------
// Obstacle-column detection: cells with enough vertical span above the
// local floor are treated as objects, and their model height is pinned to
// the floor.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_OBSTACLE_COLUMN_Z_SPAN_M = 0.55;
constexpr double XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_FLOOR_M = 0.25;
constexpr size_t XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_POINTS = 2u;
constexpr double XY_GROUND_OBSTACLE_COLUMN_ROUGHNESS_CAP_M = 0.10;
constexpr double XY_GROUND_OBSTACLE_WALL_NORMAL_Z_MAX = 0.65;
constexpr double XY_GROUND_OBSTACLE_WALL_ABOVE_FLOOR_M = 0.20;

// -----------------------------------------------------------------------
// Do not let normal-filtered samples lift the model far above the raw
// lower envelope; vertical structure can otherwise masquerade as smooth
// ground.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_FILTERED_HEIGHT_LIFT_MAX_M = 0.25;

// -----------------------------------------------------------------------
// Final point-level veto for wall/object points inside obstacle-column
// cells.  Hard threshold is only used as a fallback when no normal is
// available.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_WALL_FLOOR_HARD_ABOVE_M = 0.35;

// -----------------------------------------------------------------------
// Intermediate 2.5-D ground height map built from one frame.
// -----------------------------------------------------------------------
struct GroundModel {
    bool valid = false;
    double fallback_ground_z = std::numeric_limits<double>::quiet_NaN();
    double cell_size_m = 0.5;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double inv_cell_size = 2.0;  // 1.0 / cell_size_m (avoid per-point division)
    int rows = 0;
    int cols = 0;
    std::vector<uint8_t> cell_valid;
    std::vector<uint8_t> obstacle_column_cell;
    std::vector<double> cell_floor_z;
    std::vector<double> height;
    std::vector<double> roughness;
    bool indoor_scene = false;
    double footprint_bound = 0.0;  // 95th-pct max(|x|,|y|) extent
};

double point_abs_normal_z(const Eigen::Vector3d* normal) {
    if (normal == nullptr || !normal->allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double normal_norm = normal->norm();
    if (normal_norm <= NORMAL_EPS) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::abs(normal->z() / normal_norm);
}

bool point_normal_is_clearly_non_ground(const Eigen::Vector3d* normal) {
    const double normal_z = point_abs_normal_z(normal);
    return std::isfinite(normal_z) && normal_z < XY_GROUND_POINT_NORMAL_Z_MIN;
}

bool point_normal_is_wall_like(const Eigen::Vector3d& normal) {
    const double normal_z = point_abs_normal_z(&normal);
    return std::isfinite(normal_z) && normal_z < XY_GROUND_WALL_NORMAL_Z_MAX;
}

double percentile_value(std::vector<double>& values, double pct) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    pct = std::min(1.0, std::max(0.0, pct));
    const size_t num_vals = values.size();
    const size_t idx = std::min(
        num_vals - 1, static_cast<size_t>(std::floor(pct * static_cast<double>(num_vals - 1))));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx),
                     values.end());
    return values[idx];
}

double median_value(std::vector<double>& values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const size_t mid = values.size() / 2u;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid),
                     values.end());
    return values[mid];
}

double robust_spread(std::vector<double>& values, double center) {
    if (values.size() < 2u || !std::isfinite(center)) {
        return 0.0;
    }
    for (double& value : values) {
        value = std::abs(value - center);
    }
    const double mad = median_value(values);
    if (!std::isfinite(mad)) {
        return 0.0;
    }
    return MAD_TO_SIGMA * mad;
}

double estimate_xy_ground_z_from_samples(std::vector<double>& zs) {
    if (zs.empty()) {
        return 0.0;
    }

    const size_t num_vals = zs.size();
    const size_t low_idx =
        std::min(num_vals - 1, static_cast<size_t>(std::floor(XY_GROUND_TAIL_LOW_PCT *
                                                              static_cast<double>(num_vals - 1))));
    size_t high_idx =
        std::min(num_vals - 1, static_cast<size_t>(std::ceil(XY_GROUND_TAIL_HIGH_PCT *
                                                             static_cast<double>(num_vals - 1))));
    if (high_idx < low_idx) {
        high_idx = low_idx;
    }

    // Partial sort: only need elements in [low_idx, high_idx].
    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(low_idx), zs.end());
    if (high_idx > low_idx) {
        std::nth_element(zs.begin() + static_cast<std::ptrdiff_t>(low_idx + 1),
                         zs.begin() + static_cast<std::ptrdiff_t>(high_idx), zs.end());
    }
    double sum = 0.0;
    size_t count = 0u;
    for (size_t i = low_idx; i <= high_idx; ++i) {
        sum += zs[i];
        ++count;
    }
    return (count > 0u) ? (sum / static_cast<double>(count)) : zs.front();
}

bool ground_model_cell_for_point(const GroundModel& model, const Eigen::Vector3d& point, int& row,
                                 int& col) {
    if (!model.valid || !point.allFinite()) {
        return false;
    }
    col = static_cast<int>(std::floor((point.x() - model.origin_x) * model.inv_cell_size));
    row = static_cast<int>(std::floor((point.y() - model.origin_y) * model.inv_cell_size));
    return row >= 0 && row < model.rows && col >= 0 && col < model.cols;
}

double ground_model_local_tolerance(double range, double roughness,
                                    double lookup_distance_m = 0.0) {
    const double finite_roughness = std::isfinite(roughness) ? std::max(0.0, roughness) : 0.0;
    const double range_noise =
        std::isfinite(range)
            ? std::min(XY_GROUND_RANGE_NOISE_MAX_M, XY_GROUND_RANGE_NOISE_K * std::max(0.0, range))
            : XY_GROUND_RANGE_NOISE_MAX_M;
    const double lookup_extra =
        std::isfinite(lookup_distance_m)
            ? std::min(XY_GROUND_LOOKUP_SLOPE_TOL_MAX_M,
                       XY_GROUND_LOOKUP_SLOPE_TOL_PER_M * std::max(0.0, lookup_distance_m))
            : 0.0;
    return XY_GROUND_LOCAL_BASE_TOL_M + XY_GROUND_LOCAL_ROUGHNESS_K * finite_roughness +
           range_noise + lookup_extra;
}

void fill_ground_model_holes(GroundModel& model,
                             int fill_radius = XY_GROUND_INITIAL_FILL_RADIUS_CELLS) {
    if (!model.valid) {
        return;
    }

    std::vector<uint8_t> filled_valid = model.cell_valid;
    std::vector<double> filled_height = model.height;
    std::vector<double> filled_roughness = model.roughness;

    std::vector<double> heights;
    std::vector<double> roughnesses;
    std::vector<double> height_copy;
    for (int row = 0; row < model.rows; ++row) {
        for (int col = 0; col < model.cols; ++col) {
            const size_t idx = static_cast<size_t>(row) * model.cols + static_cast<size_t>(col);
            if (model.cell_valid[idx] != 0u) {
                continue;
            }

            heights.clear();
            roughnesses.clear();
            for (int dr = -fill_radius; dr <= fill_radius; ++dr) {
                for (int dc = -fill_radius; dc <= fill_radius; ++dc) {
                    if (dr == 0 && dc == 0) {
                        continue;
                    }
                    const int rr = row + dr;
                    const int ncol = col + dc;
                    if (rr < 0 || rr >= model.rows || ncol < 0 || ncol >= model.cols) {
                        continue;
                    }
                    const size_t nidx =
                        static_cast<size_t>(rr) * model.cols + static_cast<size_t>(ncol);
                    if (model.cell_valid[nidx] == 0u || !std::isfinite(model.height[nidx])) {
                        continue;
                    }
                    heights.push_back(model.height[nidx]);
                    roughnesses.push_back(model.roughness[nidx]);
                }
            }
            if (heights.empty()) {
                continue;
            }

            height_copy = heights;
            const double fill_h = median_value(height_copy);
            const double spread = robust_spread(heights, fill_h);
            if (!std::isfinite(fill_h) || spread > XY_GROUND_FILL_MAX_NEIGHBOR_SPREAD_M) {
                continue;
            }

            filled_height[idx] = fill_h;
            filled_roughness[idx] = median_value(roughnesses);
            filled_valid[idx] = 1u;
        }
    }

    model.cell_valid = std::move(filled_valid);
    model.height = std::move(filled_height);
    model.roughness = std::move(filled_roughness);
}

void smooth_ground_model_edge_preserving(GroundModel& model) {
    if (!model.valid) {
        return;
    }

    std::vector<double> smooth_height = model.height;
    std::vector<double> smooth_roughness = model.roughness;
    std::vector<double> heights;
    std::vector<double> roughnesses;

    for (int row = 0; row < model.rows; ++row) {
        for (int col = 0; col < model.cols; ++col) {
            const size_t idx = static_cast<size_t>(row) * model.cols + static_cast<size_t>(col);
            if (model.cell_valid[idx] == 0u || !std::isfinite(model.height[idx])) {
                continue;
            }

            const double center_h = model.height[idx];
            heights.clear();
            roughnesses.clear();
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    const int rr = row + dr;
                    const int ncol = col + dc;
                    if (rr < 0 || rr >= model.rows || ncol < 0 || ncol >= model.cols) {
                        continue;
                    }
                    const size_t nidx =
                        static_cast<size_t>(rr) * model.cols + static_cast<size_t>(ncol);
                    if (model.cell_valid[nidx] == 0u || !std::isfinite(model.height[nidx])) {
                        continue;
                    }
                    if (std::abs(model.height[nidx] - center_h) >
                        XY_GROUND_SMOOTH_MAX_HEIGHT_DIFF_M) {
                        continue;
                    }
                    heights.push_back(model.height[nidx]);
                    roughnesses.push_back(model.roughness[nidx]);
                }
            }
            if (!heights.empty()) {
                smooth_height[idx] = median_value(heights);
                smooth_roughness[idx] = median_value(roughnesses);
            }
        }
    }

    model.height = std::move(smooth_height);
    model.roughness = std::move(smooth_roughness);
}

void prune_ground_model_to_low_connected_surface(GroundModel& model) {
    if (!model.valid || model.rows <= 0 || model.cols <= 0 ||
        !std::isfinite(model.fallback_ground_z)) {
        return;
    }

    const int rows = model.rows;
    const int cols = model.cols;
    const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    if (model.cell_valid.size() != cell_count || model.height.size() != cell_count) {
        return;
    }

    std::vector<uint8_t> reachable(cell_count, 0u);
    std::vector<size_t> queue;
    queue.reserve(cell_count);

    const auto cell_index = [cols](int row_idx, int col_idx) {
        return static_cast<size_t>(row_idx) * static_cast<size_t>(cols) +
               static_cast<size_t>(col_idx);
    };

    for (int row_idx = 0; row_idx < rows; ++row_idx) {
        for (int col_idx = 0; col_idx < cols; ++col_idx) {
            const size_t idx = cell_index(row_idx, col_idx);
            if (model.cell_valid[idx] == 0u || !std::isfinite(model.height[idx])) {
                continue;
            }
            if (model.height[idx] <=
                model.fallback_ground_z + XY_GROUND_MODEL_ANCHOR_ABOVE_FALLBACK_M) {
                reachable[idx] = 1u;
                queue.push_back(idx);
            }
        }
    }

    if (queue.empty()) {
        return;
    }

    std::array<double, 8> neighbor_dists{};
    for (size_t i = 0; i < 8; ++i) {
        const auto& delta = NEIGHBOR_DELTAS[i];
        neighbor_dists[i] =
            model.cell_size_m *
            std::sqrt(static_cast<double>(delta[0] * delta[0] + delta[1] * delta[1]));
    }

    for (size_t head = 0; head < queue.size(); ++head) {
        const size_t idx = queue[head];
        const int row_bfs = static_cast<int>(idx / static_cast<size_t>(cols));
        const int col_bfs = static_cast<int>(idx % static_cast<size_t>(cols));
        const double h = model.height[idx];

        for (size_t di = 0; di < 8; ++di) {
            const auto& delta = NEIGHBOR_DELTAS[di];
            const int rr = row_bfs + delta[0];
            const int ncol = col_bfs + delta[1];
            if (rr < 0 || rr >= rows || ncol < 0 || ncol >= cols) {
                continue;
            }
            const size_t nidx = cell_index(rr, ncol);
            if (reachable[nidx] != 0u || model.cell_valid[nidx] == 0u ||
                !std::isfinite(model.height[nidx])) {
                continue;
            }

            const double cell_dist = neighbor_dists[di];
            const double allowed_step = std::min(XY_GROUND_MODEL_MAX_NEIGHBOR_STEP_M,
                                                 0.25 + XY_GROUND_MODEL_SLOPE_PER_M * cell_dist);
            if (std::abs(model.height[nidx] - h) > allowed_step) {
                continue;
            }

            // Normal-aware gate: do not propagate BFS into a cell that is
            // climbing significantly and is flagged as an obstacle column
            // (proxy for non-ground-like vertical structure).
            if (model.height[nidx] - h > 0.50 && nidx < model.obstacle_column_cell.size() &&
                model.obstacle_column_cell[nidx] != 0u) {
                continue;
            }

            reachable[nidx] = 1u;
            queue.push_back(nidx);
        }
    }

    for (size_t idx = 0; idx < cell_count; ++idx) {
        if (model.cell_valid[idx] == 0u || !std::isfinite(model.height[idx])) {
            continue;
        }
        const bool elevated = model.height[idx] >
                              model.fallback_ground_z + XY_GROUND_MODEL_PRUNE_MIN_ABOVE_FALLBACK_M;
        if (reachable[idx] == 0u && elevated) {
            model.cell_valid[idx] = 0u;
            model.height[idx] = std::numeric_limits<double>::quiet_NaN();
            model.roughness[idx] = 0.0;
        }
    }
}

void reject_elevated_components(GroundModel& model) {
    if (!model.valid || model.rows <= 0 || model.cols <= 0 ||
        !std::isfinite(model.fallback_ground_z)) {
        return;
    }

    const int rows = model.rows;
    const int cols = model.cols;
    const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    if (model.cell_valid.size() != cell_count || model.height.size() != cell_count) {
        return;
    }

    const auto cell_index = [cols](int r, int col) {
        return static_cast<size_t>(r) * static_cast<size_t>(cols) + static_cast<size_t>(col);
    };

    // Label connected components of valid cells.
    std::vector<int> labels(cell_count, -1);
    std::vector<std::vector<size_t>> components;
    std::vector<size_t> queue;

    for (size_t start = 0; start < cell_count; ++start) {
        if (model.cell_valid[start] == 0u || !std::isfinite(model.height[start]) ||
            labels[start] >= 0) {
            continue;
        }

        const int comp_id = static_cast<int>(components.size());
        components.emplace_back();
        auto& comp = components.back();

        queue.clear();
        queue.push_back(start);
        labels[start] = comp_id;

        for (size_t head = 0; head < queue.size(); ++head) {
            const size_t idx = queue[head];
            comp.push_back(idx);

            const int r = static_cast<int>(idx / static_cast<size_t>(cols));
            const int col = static_cast<int>(idx % static_cast<size_t>(cols));

            for (const auto& delta : NEIGHBOR_DELTAS) {
                const int rr = r + delta[0];
                const int neighbor_col = col + delta[1];
                if (rr < 0 || rr >= rows || neighbor_col < 0 || neighbor_col >= cols) {
                    continue;
                }
                const size_t nidx = cell_index(rr, neighbor_col);
                if (labels[nidx] >= 0 || model.cell_valid[nidx] == 0u ||
                    !std::isfinite(model.height[nidx])) {
                    continue;
                }
                // Only connect cells with similar heights to keep elevated
                // surfaces as separate components.
                if (std::abs(model.height[nidx] - model.height[idx]) >
                    XY_GROUND_COMPONENT_MAX_STEP_M) {
                    continue;
                }
                labels[nidx] = comp_id;
                queue.push_back(nidx);
            }
        }
    }

    if (components.empty()) {
        return;
    }

    // Find the main ground component: the largest component whose median
    // height is close to fallback_ground_z.
    int main_comp_id = -1;
    size_t main_comp_size = 0u;
    double main_median_z = model.fallback_ground_z;

    for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
        const auto& comp = components[static_cast<size_t>(ci)];
        std::vector<double> heights;
        heights.reserve(comp.size());
        for (size_t idx : comp) {
            heights.push_back(model.height[idx]);
        }
        const double med_z = median_value(heights);
        if (!std::isfinite(med_z)) {
            continue;
        }
        // Candidate for main ground: median z within anchor distance of
        // fallback, and larger than current best.
        if (med_z <= model.fallback_ground_z + XY_GROUND_MODEL_ANCHOR_ABOVE_FALLBACK_M &&
            comp.size() > main_comp_size) {
            main_comp_id = ci;
            main_comp_size = comp.size();
            main_median_z = med_z;
        }
    }

    if (main_comp_id < 0) {
        // No clear main ground — do not reject anything.
        return;
    }

    // Evaluate each non-main component.
    for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
        if (ci == main_comp_id) {
            continue;
        }
        const auto& comp = components[static_cast<size_t>(ci)];
        std::vector<double> heights;
        heights.reserve(comp.size());
        for (size_t idx : comp) {
            heights.push_back(model.height[idx]);
        }
        const double comp_median_z = median_value(heights);
        if (!std::isfinite(comp_median_z)) {
            continue;
        }

        const double delta_z = comp_median_z - main_median_z;
        const double area_frac =
            static_cast<double>(comp.size()) / static_cast<double>(main_comp_size);

        // Keep components close in height to main ground.
        if (std::abs(delta_z) <= XY_GROUND_COMPONENT_CLOSE_TO_MAIN_M) {
            continue;
        }

        // Moderately elevated: keep only if large enough.
        if (delta_z > XY_GROUND_COMPONENT_CLOSE_TO_MAIN_M &&
            delta_z <= XY_GROUND_COMPONENT_HIGH_ABOVE_MAIN_M &&
            area_frac >= XY_GROUND_COMPONENT_MODERATE_MIN_AREA_FRAC) {
            continue;
        }

        // Very elevated: keep only if very large.
        if (delta_z > XY_GROUND_COMPONENT_HIGH_ABOVE_MAIN_M &&
            area_frac >= XY_GROUND_COMPONENT_HIGH_MIN_AREA_FRAC) {
            continue;
        }

        // Below main ground by more than close threshold but not too far:
        // keep (could be a ditch or lower terrain).
        if (delta_z < -XY_GROUND_COMPONENT_CLOSE_TO_MAIN_M &&
            delta_z >= -XY_GROUND_COMPONENT_HIGH_ABOVE_MAIN_M) {
            continue;
        }

        // Reject this component.
        for (size_t idx : comp) {
            model.cell_valid[idx] = 0u;
            model.height[idx] = std::numeric_limits<double>::quiet_NaN();
            model.roughness[idx] = 0.0;
        }
    }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
GroundModel build_lower_envelope_ground_model(
    Eigen::Ref<const PointCloudXYZd> points,
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& range,
    Eigen::Ref<const Eigen::Array<uint32_t, Eigen::Dynamic, 1>> status, int first_col, int last_col,
    double grid_size, const MatrixX3dR* normals = nullptr, const PointCloudXYZd* points2 = nullptr,
    const ouster::sdk::core::img_t<uint32_t>* range2 = nullptr,
    const MatrixX3dR* normals2 = nullptr) {
    GroundModel model;

    const int frame_height = static_cast<int>(range.rows());
    const int frame_width = static_cast<int>(range.cols());
    const Eigen::Index frame_size =
        static_cast<Eigen::Index>(frame_height) * static_cast<Eigen::Index>(frame_width);
    const bool have_normals =
        normals != nullptr && normals->rows() == frame_size && normals->cols() == 3;
    const bool have_second_return =
        points2 != nullptr && range2 != nullptr && points2->rows() == frame_size &&
        points2->cols() == points.cols() && range2->rows() == range.rows() &&
        range2->cols() == range.cols();
    const bool have_second_normals = have_second_return && normals2 != nullptr &&
                                     normals2->rows() == frame_size && normals2->cols() == 3;
    if (points.rows() != frame_size || last_col < first_col || last_col >= frame_width) {
        return model;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    std::vector<double> all_zs;
    all_zs.reserve(static_cast<size_t>(frame_size) * (have_second_return ? 2u : 1u));
    std::vector<double> footprint_extents;
    footprint_extents.reserve(static_cast<size_t>(frame_size) * (have_second_return ? 2u : 1u));

    const auto visit_valid_return_points =
        [&](const PointCloudXYZd& return_points,
            const ouster::sdk::core::img_t<uint32_t>& return_range, const auto& visitor) {
            for (int col = first_col; col <= last_col; ++col) {
                if (status(col) == 0u) {
                    continue;
                }
                for (int row = 0; row < frame_height; ++row) {
                    if (return_range(row, col) == 0u) {
                        continue;
                    }
                    const Eigen::Index idx = static_cast<Eigen::Index>(row) * frame_width + col;
                    const Eigen::Vector3d point = return_points.row(idx).matrix();
                    if (!point.allFinite() || point.norm() < MIN_RANGE_M) {
                        continue;
                    }
                    visitor(idx, point);
                }
            }
        };

    const auto update_extents = [&](Eigen::Index /*idx*/, const Eigen::Vector3d& point) {
        min_x = std::min(min_x, point.x());
        min_y = std::min(min_y, point.y());
        max_x = std::max(max_x, point.x());
        max_y = std::max(max_y, point.y());
        all_zs.push_back(point.z());
        footprint_extents.push_back(std::max(std::abs(point.x()), std::abs(point.y())));
    };

    visit_valid_return_points(points, range, update_extents);
    if (have_second_return) {
        visit_valid_return_points(*points2, *range2, update_extents);
    }

    if (all_zs.empty() || !std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) ||
        !std::isfinite(max_y)) {
        return model;
    }

    // Compute footprint bound (95th percentile of max(|x|,|y|)).
    {
        const size_t nth_idx = std::min(
            footprint_extents.size() - 1,
            static_cast<size_t>(std::floor(XY_BOUNDS_PERCENTILE *
                                           static_cast<double>(footprint_extents.size() - 1))));
        std::nth_element(footprint_extents.begin(),
                         footprint_extents.begin() + static_cast<std::ptrdiff_t>(nth_idx),
                         footprint_extents.end());
        model.footprint_bound = footprint_extents[nth_idx];
    }

    model.fallback_ground_z = estimate_xy_ground_z_from_samples(all_zs);
    model.cell_size_m = grid_size;
    model.inv_cell_size = 1.0 / grid_size;
    model.origin_x = std::floor(min_x * model.inv_cell_size) * model.cell_size_m;
    model.origin_y = std::floor(min_y * model.inv_cell_size) * model.cell_size_m;
    model.cols =
        std::max(1, static_cast<int>(std::ceil((max_x - model.origin_x) / model.cell_size_m)) + 1);
    model.rows =
        std::max(1, static_cast<int>(std::ceil((max_y - model.origin_y) / model.cell_size_m)) + 1);

    const size_t cell_count = static_cast<size_t>(model.rows) * static_cast<size_t>(model.cols);
    std::vector<std::vector<double>> cell_zs(cell_count);
    std::vector<std::vector<double>> normal_filtered_cell_zs(cell_count);
    model.cell_valid.assign(cell_count, 0u);
    model.obstacle_column_cell.assign(cell_count, 0u);
    model.cell_floor_z.assign(cell_count, std::numeric_limits<double>::quiet_NaN());
    model.height.assign(cell_count, std::numeric_limits<double>::quiet_NaN());
    model.roughness.assign(cell_count, 0.0);

    const auto cell_index_for_point = [&](const Eigen::Vector3d& point, size_t& idx) {
        const int col =
            static_cast<int>(std::floor((point.x() - model.origin_x) * model.inv_cell_size));
        const int row =
            static_cast<int>(std::floor((point.y() - model.origin_y) * model.inv_cell_size));
        if (row < 0 || row >= model.rows || col < 0 || col >= model.cols) {
            return false;
        }
        idx = static_cast<size_t>(row) * static_cast<size_t>(model.cols) + static_cast<size_t>(col);
        return true;
    };

    const auto add_model_sample = [&](Eigen::Index flat_idx, const Eigen::Vector3d& point,
                                      const MatrixX3dR* return_normals) {
        size_t cell_idx = 0u;
        if (!cell_index_for_point(point, cell_idx)) {
            return;
        }
        cell_zs[cell_idx].push_back(point.z());
        if (return_normals != nullptr) {
            const Eigen::Vector3d normal = return_normals->row(flat_idx).matrix();
            const double normal_norm = normal.norm();
            if (normal.allFinite() && normal_norm > NORMAL_EPS) {
                const double normal_z = std::abs(normal.z() / normal_norm);
                if (normal_z >= XY_GROUND_NORMAL_Z_PREFILTER_MIN) {
                    normal_filtered_cell_zs[cell_idx].push_back(point.z());
                }
            }
        }
    };

    visit_valid_return_points(
        points, range, [&](Eigen::Index flat_idx, const Eigen::Vector3d& point) {
            add_model_sample(flat_idx, point, have_normals ? normals : nullptr);
        });
    if (have_second_return) {
        visit_valid_return_points(
            *points2, *range2, [&](Eigen::Index flat_idx, const Eigen::Vector3d& point) {
                add_model_sample(flat_idx, point, have_second_normals ? normals2 : nullptr);
            });
    }

    size_t valid_cells = 0u;
    std::vector<double> cell_zs_work;
    for (size_t idx = 0; idx < cell_count; ++idx) {
        if (cell_zs[idx].empty()) {
            continue;
        }

        const auto z_minmax = std::minmax_element(cell_zs[idx].begin(), cell_zs[idx].end());
        const double z_max = *z_minmax.second;

        // Compute floor (5th %ile) and height (15th %ile) from one copy.
        // nth_element for floor partitions correctly, then we can do
        // nth_element on [floor_idx+1..end) for the height percentile.
        cell_zs_work = cell_zs[idx];
        const size_t num_zs = cell_zs_work.size();
        const size_t floor_idx =
            std::min(num_zs - 1, static_cast<size_t>(std::floor(XY_GROUND_CELL_FLOOR_PERCENTILE *
                                                                static_cast<double>(num_zs - 1))));
        std::nth_element(cell_zs_work.begin(),
                         cell_zs_work.begin() + static_cast<std::ptrdiff_t>(floor_idx),
                         cell_zs_work.end());
        const double cell_floor = cell_zs_work[floor_idx];

        const size_t height_idx =
            std::min(num_zs - 1, static_cast<size_t>(std::floor(XY_GROUND_CELL_LOW_PERCENTILE *
                                                                static_cast<double>(num_zs - 1))));
        // height_idx >= floor_idx always, so partition from floor_idx onward.
        if (height_idx > floor_idx) {
            std::nth_element(cell_zs_work.begin() + static_cast<std::ptrdiff_t>(floor_idx + 1),
                             cell_zs_work.begin() + static_cast<std::ptrdiff_t>(height_idx),
                             cell_zs_work.end());
        }
        const double h_all = cell_zs_work[height_idx];

        if (!std::isfinite(h_all) || !std::isfinite(cell_floor)) {
            continue;
        }

        model.cell_floor_z[idx] = cell_floor;

        size_t above_floor_count = 0u;
        for (double z : cell_zs[idx]) {
            if (std::isfinite(z) && z > cell_floor + XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_FLOOR_M) {
                ++above_floor_count;
            }
        }

        const double z_span_from_floor = z_max - cell_floor;
        const bool obstacle_column =
            z_span_from_floor > XY_GROUND_OBSTACLE_COLUMN_Z_SPAN_M &&
            above_floor_count >= XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_POINTS;

        if (obstacle_column) {
            model.obstacle_column_cell[idx] = 1u;
        }

        double h_filtered = std::numeric_limits<double>::quiet_NaN();
        bool use_filtered = false;

        if ((have_normals || have_second_normals) &&
            normal_filtered_cell_zs[idx].size() >= XY_GROUND_NORMAL_FILTERED_MIN_POINTS) {
            std::vector<double> filtered_zs_for_height = normal_filtered_cell_zs[idx];

            h_filtered = percentile_value(filtered_zs_for_height, XY_GROUND_CELL_LOW_PERCENTILE);

            const bool filtered_lifted = std::isfinite(h_filtered) &&
                                         h_filtered > h_all + XY_GROUND_FILTERED_HEIGHT_LIFT_MAX_M;

            use_filtered = std::isfinite(h_filtered) && !filtered_lifted && !obstacle_column;
        }

        std::vector<double>* selected_zs =
            use_filtered ? &normal_filtered_cell_zs[idx] : &cell_zs[idx];

        const double h = obstacle_column ? cell_floor : (use_filtered ? h_filtered : h_all);

        if (!std::isfinite(h) || selected_zs->empty()) {
            continue;
        }

        std::vector<double> lower_band;
        lower_band.reserve(selected_zs->size());
        for (double z : *selected_zs) {
            if (std::isfinite(z) && z <= h + XY_GROUND_CELL_ROUGHNESS_BAND_M) {
                lower_band.push_back(z);
            }
        }
        if (lower_band.empty()) {
            lower_band.push_back(h);
        }

        double cell_roughness = robust_spread(lower_band, h);
        if (obstacle_column) {
            cell_roughness = std::min(cell_roughness, XY_GROUND_OBSTACLE_COLUMN_ROUGHNESS_CAP_M);
        }

        model.cell_valid[idx] = 1u;
        model.height[idx] = h;
        model.roughness[idx] = cell_roughness;
        ++valid_cells;
    }

    if (valid_cells == 0u) {
        return model;
    }

    model.valid = true;

    // Fill first so sparse real ground remains connected. The spread gate
    // avoids filling across object edges.
    fill_ground_model_holes(model);
    smooth_ground_model_edge_preserving(model);

    // Remove elevated platforms after the surface has enough continuity.
    prune_ground_model_to_low_connected_surface(model);

    // Fill pruned platform cells with nearby low ground where appropriate, so
    // object tops compare against ground height and get rejected.
    fill_ground_model_holes(model);
    smooth_ground_model_edge_preserving(model);

    // Connected-component height validation: reject small elevated components
    // that are not part of the main ground surface (rooftops, platforms).
    reject_elevated_components(model);

    // Final fill so that rejected elevated cells get replaced by nearby low
    // ground heights, causing points on those surfaces to have large residuals
    // and be correctly classified as non-ground. Use a smaller radius here to
    // avoid over-expanding the ground model into nearby object boundaries.
    fill_ground_model_holes(model, XY_GROUND_FINAL_FILL_RADIUS_CELLS);

    return model;
}

bool xy_point_is_ground_like(const Eigen::Vector3d& point, double range,
                             const GroundModel& ground_model, double fallback_ground_z,
                             const Eigen::Vector3d* normal = nullptr) {
    const double effective_ground_z = std::isfinite(ground_model.fallback_ground_z)
                                          ? ground_model.fallback_ground_z
                                          : fallback_ground_z;

    if (ground_model.indoor_scene && std::isfinite(effective_ground_z) &&
        point.z() > effective_ground_z + XY_GROUND_INDOOR_MAX_Z_ABOVE_ESTIMATE_M) {
        return false;
    }

    // Compute grid cell ONCE and reuse for both obstacle and lookup checks.
    int cell_row = 0;
    int cell_col = 0;
    const bool in_grid = ground_model_cell_for_point(ground_model, point, cell_row, cell_col);

    if (in_grid) {
        const size_t cell_idx =
            static_cast<size_t>(cell_row) * static_cast<size_t>(ground_model.cols) +
            static_cast<size_t>(cell_col);

        // Obstacle-column check.
        if (cell_idx < ground_model.cell_floor_z.size() &&
            cell_idx < ground_model.obstacle_column_cell.size() &&
            ground_model.obstacle_column_cell[cell_idx] != 0u &&
            std::isfinite(ground_model.cell_floor_z[cell_idx])) {
            const double above_cell_floor = point.z() - ground_model.cell_floor_z[cell_idx];

            if (above_cell_floor > XY_GROUND_WALL_FLOOR_HARD_ABOVE_M) {
                return false;
            }

            const double nz = point_abs_normal_z(normal);
            if (std::isfinite(nz)) {
                if (nz < XY_GROUND_OBSTACLE_WALL_NORMAL_Z_MAX &&
                    above_cell_floor > XY_GROUND_OBSTACLE_WALL_ABOVE_FLOOR_M) {
                    return false;
                }
            }
        }

        // Ground model lookup starting from the already-computed cell.
        const auto cell_index = [&](int r, int col) {
            return static_cast<size_t>(r) * static_cast<size_t>(ground_model.cols) +
                   static_cast<size_t>(col);
        };

        double local_height = std::numeric_limits<double>::quiet_NaN();
        double local_roughness = 0.0;
        double lookup_distance_m = 0.0;
        bool lookup_found = false;

        if (ground_model.cell_valid[cell_idx] != 0u &&
            std::isfinite(ground_model.height[cell_idx])) {
            local_height = ground_model.height[cell_idx];
            local_roughness = ground_model.roughness[cell_idx];
            lookup_distance_m = 0.0;
            lookup_found = true;
        } else {
            double best_dist_sq = std::numeric_limits<double>::infinity();
            size_t best_idx = 0u;
            bool found = false;
            for (int radius = 1; radius <= XY_GROUND_LOOKUP_RADIUS_CELLS; ++radius) {
                for (int dr = -radius; dr <= radius; ++dr) {
                    for (int dc = -radius; dc <= radius; ++dc) {
                        if (std::max(std::abs(dr), std::abs(dc)) != radius) {
                            continue;
                        }
                        const int rr = cell_row + dr;
                        const int ncol = cell_col + dc;
                        if (rr < 0 || rr >= ground_model.rows || ncol < 0 ||
                            ncol >= ground_model.cols) {
                            continue;
                        }
                        const size_t nidx = cell_index(rr, ncol);
                        if (ground_model.cell_valid[nidx] == 0u ||
                            !std::isfinite(ground_model.height[nidx])) {
                            continue;
                        }
                        const double dist_sq = static_cast<double>(dr * dr + dc * dc);
                        if (dist_sq < best_dist_sq) {
                            best_dist_sq = dist_sq;
                            best_idx = nidx;
                            found = true;
                        }
                    }
                }
                if (found) {
                    local_height = ground_model.height[best_idx];
                    local_roughness = ground_model.roughness[best_idx];
                    lookup_distance_m = std::sqrt(best_dist_sq) * ground_model.cell_size_m;
                    lookup_found = true;
                    break;
                }
            }
        }

        if (lookup_found) {
            double local_tol =
                ground_model_local_tolerance(range, local_roughness, lookup_distance_m);
            local_tol = std::min(local_tol, XY_GROUND_MAX_ABOVE_LOCAL_M);
            if (ground_model.indoor_scene) {
                local_tol = std::min(local_tol, XY_GROUND_INDOOR_MAX_Z_ABOVE_ESTIMATE_M);
            }
            const double residual = point.z() - local_height;
            if (normal != nullptr && residual > XY_GROUND_WALL_ABOVE_LOCAL_M &&
                point_normal_is_wall_like(*normal)) {
                return false;
            }
            if (residual > 0.35 && point_normal_is_clearly_non_ground(normal)) {
                return false;
            }
            return residual <= local_tol && residual >= -XY_GROUND_MAX_BELOW_LOCAL_M;
        }
    }

    // If this exact area has no nearby model support, allow only a narrow
    // global low-ground fallback. This recovers sparse true-ground holes
    // while still rejecting elevated container/trailer tops.
    if (ground_model.valid) {
        if (!std::isfinite(effective_ground_z)) {
            return false;
        }
        const double residual_to_global = point.z() - effective_ground_z;
        if (residual_to_global > 0.35 && point_normal_is_clearly_non_ground(normal)) {
            return false;
        }
        const double fallback_cap = ground_model.indoor_scene
                                        ? XY_GROUND_INDOOR_MAX_Z_ABOVE_ESTIMATE_M
                                        : XY_GROUND_UNSUPPORTED_FALLBACK_ABOVE_M;
        return residual_to_global <= fallback_cap &&
               residual_to_global >= -XY_GROUND_MAX_BELOW_LOCAL_M;
    }

    if (!std::isfinite(effective_ground_z)) {
        return false;
    }
    const double max_z_above = ground_model.indoor_scene ? XY_GROUND_INDOOR_MAX_Z_ABOVE_ESTIMATE_M
                                                         : XY_GROUND_OUTDOOR_MAX_Z_ABOVE_ESTIMATE_M;
    return point.z() <= effective_ground_z + max_z_above;
}

/// Try to load a precomputed normals field from the frame, returning the data
/// as a double matrix (H*W, 3).  Returns true on success.
bool try_load_normals_field(const ouster::sdk::core::LidarFrame& frame,
                            const std::string& field_name, MatrixX3dR& out) {
    if (!frame.has_field(field_name)) {
        return false;
    }
    const auto& field = frame.field(field_name);
    if (field.tag() != ouster::sdk::core::ChanFieldType::FLOAT32) {
        return false;
    }
    const auto& sh = field.shape();
    if (sh.size() != 3 || static_cast<int>(sh[0]) != static_cast<int>(frame.h) ||
        static_cast<int>(sh[1]) != static_cast<int>(frame.w) || sh[2] != 3) {
        return false;
    }
    const auto view = static_cast<ouster::sdk::core::ConstArrayView3<float>>(field);
    if (view.sparse()) {
        return false;
    }
    const Eigen::Index num_pts =
        static_cast<Eigen::Index>(frame.h) * static_cast<Eigen::Index>(frame.w);
    out = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(view.data(),
                                                                                     num_pts, 3)
              .cast<double>();
    return true;
}

}  // namespace

// Pipeline:
//   1. Coordinate frame selection – use non-identity LidarFrame poses directly
//      when SLAM has populated them; otherwise use sensor extrinsics and
//      frame-local column poses.
//   2. Lower-envelope 2.5-D grid – builds a height map from all valid returns
//      using the 15th-percentile Z in each cell.
//   3. Hole filling / smoothing – fills small gaps using median filtering
//      while preserving edges at height discontinuities.
//   4. BFS pruning – removes elevated platforms (e.g., truck/container roofs)
//      that are disconnected from the low ground layer.
//   5. Per-point classification – each frame point is tested against the
//      height map (or a global fallback Z) to produce the final ground mask.
//
// Fill one ground mask per return (index 0 = 1st return, etc.).
// Each output mask must have shape frame.h x frame.w.
using GroundMaskView = Eigen::Ref<ouster::sdk::core::img_t<uint8_t>>;

size_t get_ground_mask_into(const ouster::sdk::core::LidarFrame& frame, double grid_size,
                            const ouster::sdk::core::XYZLut& xyz_lut,
                            std::vector<GroundMaskView>& ground_masks) {
    if (!frame.sensor_info) {
        throw std::invalid_argument("frame.sensor_info is required for get_ground_mask");
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::RANGE)) {
        throw std::invalid_argument("frame must contain RANGE field for get_ground_mask");
    }

    const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    const int frame_height = static_cast<int>(range.rows());
    const int frame_width = static_cast<int>(range.cols());

    const int max_returns = frame.sensor_info->num_returns();
    int num_returns = 1;  // always have at least the 1st return
    std::vector<ouster::sdk::core::img_t<uint32_t>> extra_ranges;
    for (int ret = 1; ret < max_returns; ++ret) {
        const std::string field_name = ouster::sdk::core::ChanField::return_field_name(
            ouster::sdk::core::ChanField::RANGE, ret);
        if (frame.has_field(field_name)) {
            extra_ranges.push_back(frame.field<uint32_t>(field_name));
            ++num_returns;
        } else {
            break;
        }
    }

    if (ground_masks.size() < static_cast<size_t>(num_returns)) {
        throw std::invalid_argument("not enough output masks provided for get_ground_mask_into");
    }
    for (int ret = 0; ret < num_returns; ++ret) {
        if (ground_masks[ret].rows() != frame_height || ground_masks[ret].cols() != frame_width) {
            throw std::invalid_argument("output mask shape does not match frame shape");
        }
        ground_masks[ret].setZero();
    }

    int first_col = 0;
    int last_col = 0;
    try {
        first_col = frame.get_first_valid_column();
        last_col = frame.get_last_valid_column();
    } catch (const std::exception&) {
        return static_cast<size_t>(num_returns);
    }
    if (last_col < first_col) {
        return static_cast<size_t>(num_returns);
    }

    const bool has_second_return = !extra_ranges.empty();

    const PointCloudXYZd points = xyz_lut(range);
    PointCloudXYZd points2;
    if (has_second_return) {
        points2 = xyz_lut(extra_ranges[0]);
    }

    ouster::sdk::core::MatrixX16dR rel_poses(frame_width, 16);
    MatrixX3dR sensor_origins(frame_width, 3);
    for (int col = 0; col < frame_width; ++col) {
        const PoseH pose_c(frame.get_column_pose(static_cast<uint32_t>(col)));
        Eigen::Map<core::Matrix4dR>(rel_poses.row(col).data()) = core::Matrix4dR(pose_c);
        sensor_origins.row(col) =
            (pose_c.matrix() * frame.sensor_info->sensor_to_body).block<3, 1>(0, 3).transpose();
    }

    PointCloudXYZd dewarped(points.rows(), points.cols());
    ouster::sdk::core::dewarp<double>(dewarped, points, rel_poses);
    PointCloudXYZd dewarped2;
    if (has_second_return) {
        dewarped2.resize(points2.rows(), points2.cols());
        ouster::sdk::core::dewarp<double>(dewarped2, points2, rel_poses);
    }

    MatrixX3dR normal_values;
    const MatrixX3dR* normal_values_ptr = nullptr;
    MatrixX3dR normal_values2;
    const MatrixX3dR* normal_values2_ptr = nullptr;

    // Prefer precomputed normals when available; otherwise estimate from
    // the dewarped geometry.
    // NOTE: normals estimation only supports up to 2 returns; extra returns
    // (3+) will be classified without normals.
    const bool have_precomputed_normals =
        try_load_normals_field(frame, ouster::sdk::core::ChanField::NORMALS, normal_values);
    if (have_precomputed_normals) {
        normal_values_ptr = &normal_values;
        if (has_second_return &&
            try_load_normals_field(frame, ouster::sdk::core::ChanField::NORMALS2, normal_values2)) {
            normal_values2_ptr = &normal_values2;
        }
    } else {
        const MatrixX3dR& sensor_origins_for_normals = sensor_origins;
        try {
            if (has_second_return) {
                const auto normal_pair = ouster::sdk::algorithm::normals(
                    dewarped, range, dewarped2, extra_ranges[0], sensor_origins_for_normals);
                normal_values = normal_pair.first;
                normal_values2 = normal_pair.second;
                normal_values_ptr = &normal_values;
                normal_values2_ptr = &normal_values2;
            } else {
                normal_values =
                    ouster::sdk::algorithm::normals(dewarped, range, sensor_origins_for_normals);
                normal_values_ptr = &normal_values;
            }
        } catch (const std::exception& error) {
            logger().warn(
                "Normals estimation failed for ground segmentation; using "
                "z-only ground model: {}",
                error.what());
        }
    }

    const auto status = frame.status();
    GroundModel ground_model = build_lower_envelope_ground_model(
        dewarped, range, status, first_col, last_col, grid_size, normal_values_ptr,
        has_second_return ? &dewarped2 : nullptr, has_second_return ? &extra_ranges[0] : nullptr,
        normal_values2_ptr);
    ground_model.indoor_scene = ground_model.footprint_bound <= XY_GROUND_INDOOR_SCENE_BOUND_M;

    const double fallback_ground_z =
        std::isfinite(ground_model.fallback_ground_z) ? ground_model.fallback_ground_z : 0.0;

    // Classify points directly in (row, col) order for sequential memory
    // access on dewarped (row-major layout: flat_idx = row*W + col).
    const auto classify_return_points = [&](const PointCloudXYZd& return_points,
                                            const ouster::sdk::core::img_t<uint32_t>& return_range,
                                            const MatrixX3dR* return_normals,
                                            GroundMaskView& target_mask) {
        for (int row = 0; row < frame_height; ++row) {
            for (int col = first_col; col <= last_col; ++col) {
                if (status[col] == 0u) {
                    continue;
                }
                if (return_range(row, col) == 0u) {
                    continue;
                }
                const Eigen::Index flat_idx = static_cast<Eigen::Index>(row) * frame_width + col;
                const Eigen::Vector3d point = return_points.row(flat_idx).matrix();
                if (!point.allFinite()) {
                    continue;
                }
                Eigen::Vector3d normal;
                const Eigen::Vector3d* normal_ptr = nullptr;
                if (return_normals != nullptr) {
                    normal = return_normals->row(flat_idx).matrix();
                    normal_ptr = &normal;
                }
                const double point_range = point.norm();
                if (xy_point_is_ground_like(point, point_range, ground_model, fallback_ground_z,
                                            normal_ptr)) {
                    target_mask(row, col) = 1u;
                }
            }
        }
    };

    // Classify 1st return (always has normals if available).
    classify_return_points(dewarped, range, normal_values_ptr, ground_masks[0]);

    // Classify 2nd return (has normals if available).
    if (has_second_return) {
        classify_return_points(dewarped2, extra_ranges[0], normal_values2_ptr, ground_masks[1]);
    }

    // Classify 3rd+ returns: dewarp and classify without normals.
    for (int ret = 2; ret < num_returns; ++ret) {
        const auto& ret_range = extra_ranges[ret - 1];
        PointCloudXYZd ret_points = xyz_lut(ret_range);
        PointCloudXYZd ret_dewarped(ret_points.rows(), ret_points.cols());
        ouster::sdk::core::dewarp<double>(ret_dewarped, ret_points, rel_poses);
        classify_return_points(ret_dewarped, ret_range, nullptr, ground_masks[ret]);
    }

    return static_cast<size_t>(num_returns);
}

// Returns one ground mask per return (index 0 = 1st return, etc.).
namespace impl {

std::vector<std::vector<uint8_t>> get_ground_mask(const ouster::sdk::core::LidarFrame& frame,
                                                  double grid_size,
                                                  const ouster::sdk::core::XYZLut& xyz_lut) {
    if (!frame.sensor_info) {
        throw std::invalid_argument("frame.sensor_info is required for get_ground_mask");
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::RANGE)) {
        throw std::invalid_argument("frame must contain RANGE field for get_ground_mask");
    }

    const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    const size_t n_pixels = static_cast<size_t>(range.rows()) * static_cast<size_t>(range.cols());
    std::vector<std::vector<uint8_t>> ground_masks(frame.sensor_info->num_returns(),
                                                   std::vector<uint8_t>(n_pixels, 0u));
    std::vector<GroundMaskView> output_masks;
    output_masks.reserve(ground_masks.size());
    for (auto& mask : ground_masks) {
        Eigen::Map<ouster::sdk::core::img_t<uint8_t>> mask_view{mask.data(), range.rows(),
                                                                range.cols()};
        output_masks.emplace_back(mask_view);
    }
    const size_t filled_returns = get_ground_mask_into(frame, grid_size, xyz_lut, output_masks);
    ground_masks.resize(filled_returns);
    return ground_masks;
}

}  // namespace impl

namespace {

class EnvelopeGridGroundSegEngine : public GroundSegEngine {
   public:
    explicit EnvelopeGridGroundSegEngine(const GroundSegConfig& config) : config_(config) {}

    void update(ouster::sdk::core::FrameSet& frames) override {
        for (size_t idx : frames.valid_indices()) {
            auto& frame_ptr = frames[idx];
            if (!frame_ptr) {
                continue;
            }
            auto& frame = *frame_ptr;
            const ouster::sdk::core::XYZLut& xyz_lut = get_or_create_xyzlut(*frame.sensor_info);
            const std::vector<size_t> shape{frame.h, frame.w};
            const int max_returns = frame.sensor_info->num_returns();
            std::vector<GroundMaskView> output_masks;
            output_masks.reserve(static_cast<size_t>(max_returns));

            // Create GROUND field buffers, then fill them in place.
            for (int ret = 0; ret < max_returns; ++ret) {
                const std::string fname = ouster::sdk::core::ChanField::return_field_name(
                    ouster::sdk::core::ChanField::GROUND, ret);
                if (frame.has_field(fname)) {
                    frame.del_field(fname);
                }
                frame.add_field(fname, ouster::sdk::core::FieldDescriptor::array<uint8_t>(shape),
                                ouster::sdk::core::FieldClass::PIXEL_FIELD);
                output_masks.emplace_back(frame.field<uint8_t>(fname));
            }

            const size_t filled_returns =
                get_ground_mask_into(frame, config_.grid_size, xyz_lut, output_masks);

            // Remove stale GROUND fields for returns beyond what we computed.
            for (int ret = static_cast<int>(filled_returns); ret < max_returns; ++ret) {
                const std::string fname = ouster::sdk::core::ChanField::return_field_name(
                    ouster::sdk::core::ChanField::GROUND, ret);
                if (frame.has_field(fname)) {
                    frame.del_field(fname);
                }
            }
        }
    }

   private:
    GroundSegConfig config_;
    std::unordered_map<uint64_t, ouster::sdk::core::XYZLut> xyzlut_cache_;

    const ouster::sdk::core::XYZLut& get_or_create_xyzlut(
        const ouster::sdk::core::SensorInfo& info) {
        auto it = xyzlut_cache_.find(info.sn);
        if (it == xyzlut_cache_.end()) {
            it = xyzlut_cache_
                     .emplace(info.sn, ouster::sdk::core::XYZLut(info, /*use_extrinsics=*/true))
                     .first;
        }
        return it->second;
    }
};

}  // namespace

std::unique_ptr<GroundSegEngine> GroundSegEngine::create(const GroundSegConfig& config) {
    if (!std::isfinite(config.grid_size) || config.grid_size <= 0.0) {
        throw std::invalid_argument("GroundSegConfig.grid_size must be > 0");
    }
    return std::make_unique<EnvelopeGridGroundSegEngine>(config);
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
