#include "ouster/algorithm/ground_seg.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <system_error>
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
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"

using ouster::sdk::core::DEG_TO_RAD;
using ouster::sdk::core::logger;
using ouster::sdk::core::MatrixX3dR;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::impl::PoseH;

namespace ouster {
namespace sdk {
namespace algorithm {

namespace {

// -----------------------------------------------------------------------
// Ground normals are limited to 40 degrees from the global gravity axis.
// Normal direction is ignored, so both +Z and -Z normals are accepted.
// -----------------------------------------------------------------------
constexpr double MAX_GROUND_SLOPE_DEG = 40.0;

// -----------------------------------------------------------------------
// Shared numeric guards and robust-statistics scale factors.
// -----------------------------------------------------------------------
constexpr double MIN_RANGE_M = 0.15;
constexpr double NORMAL_EPS = 1e-6;
constexpr double MIN_RANGE_M_SQ = MIN_RANGE_M * MIN_RANGE_M;
constexpr double NORMAL_EPS_SQ = NORMAL_EPS * NORMAL_EPS;
// Scales median absolute deviation to the standard-deviation equivalent for
// normally distributed samples.
constexpr double MEDIAN_ABSOLUTE_DEVIATION_TO_SIGMA = 1.4826;

// -----------------------------------------------------------------------
// Low-tail range used for the global fallback ground-height estimate.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_TAIL_LOW_PCT = 0.01;
constexpr double XY_GROUND_TAIL_HIGH_PCT = 0.20;

// -----------------------------------------------------------------------
// Per-cell lower-envelope sampling and roughness estimation.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_CELL_ROUGHNESS_BAND_M = 0.45;

// -----------------------------------------------------------------------
// Hole filling and smoothing keep sparse ground connected without crossing
// sharp height edges. The final fill uses a smaller radius to avoid
// over-expanding into object boundaries.
// -----------------------------------------------------------------------
constexpr int XY_GROUND_INITIAL_FILL_RADIUS_CELLS = 6;
constexpr int XY_GROUND_FINAL_FILL_RADIUS_CELLS = 3;
constexpr double XY_GROUND_FILL_MAX_NEIGHBOR_SPREAD_M = 0.65;
constexpr double XY_GROUND_SMOOTH_MAX_HEIGHT_DIFF_M = 0.55;

// -----------------------------------------------------------------------
// Reject locally elevated surfaces that have enough nearby lower support.
// Applying this before and after interpolation removes compact platforms
// without assuming that all ground is near one global height.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_MODEL_LOCAL_SUPPORT_MIN_ABOVE_FALLBACK_M = 0.30;
constexpr int XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS = 2;
constexpr double XY_GROUND_MODEL_LOCAL_SUPPORT_SLOPE_PER_M = 0.40;
constexpr size_t XY_GROUND_MODEL_LOCAL_SUPPORT_MIN_CELLS = 12u;
constexpr size_t XY_GROUND_MODEL_LOCAL_SUPPORT_NEIGHBOR_COUNT =
    static_cast<size_t>((2 * XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS + 1) *
                            (2 * XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS + 1) -
                        1);

// Keep terrain that is continuous with the main ground, even when it climbs
// several meters. Disconnected components substantially above the main
// surface are elevated floors, roofs, or platforms rather than main ground.
constexpr double XY_GROUND_MAIN_COMPONENT_BASE_STEP_M = 0.15;
constexpr double XY_GROUND_MAIN_COMPONENT_MAX_SLOPE_PER_M = 2.0;
constexpr double XY_GROUND_DISCONNECTED_COMPONENT_BASE_HEIGHT_M = 0.75;
constexpr double XY_GROUND_DISCONNECTED_COMPONENT_MAX_SLOPE_PER_M = 0.30;
constexpr double XY_GROUND_DISCONNECTED_COMPONENT_MAX_GAP_RISE_M = 1.50;
constexpr double XY_GROUND_DISCONNECTED_COMPONENT_NEAR_RANGE_M = 50.0;
constexpr double XY_GROUND_DISCONNECTED_COMPONENT_MIN_POINTS_PER_CELL = 1.50;
constexpr double XY_GROUND_FAR_SPARSE_MIN_ELEVATION_M = 2.00;

// Ground candidates need local image-space support from geometrically
// compatible neighbors to suppress isolated depth-boundary and foliage noise.
constexpr size_t MIN_GROUND_IMAGE_NEIGHBOR_SUPPORT = 2u;
constexpr size_t MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT = 2u;
constexpr double MIN_GROUND_NORMAL_ALIGNMENT = 0.80;
constexpr double MIN_GROUND_NORMAL_ALIGNMENT_SQ =
    MIN_GROUND_NORMAL_ALIGNMENT * MIN_GROUND_NORMAL_ALIGNMENT;
constexpr double GROUND_NORMAL_RECOVERY_MAX_HEIGHT_RESIDUAL_M = 0.15;

// In well-populated XY cells, require a small fraction of valid returns to
// agree on ground. Sparse cells are exempt so distant terrain is preserved.
constexpr size_t XY_GROUND_CELL_VOTE_MIN_VALID_POINTS = 8u;
constexpr double XY_GROUND_CELL_VOTE_MIN_GROUND_FRACTION = 0.10;

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
constexpr double XY_GROUND_MAX_ABOVE_LOCAL_M = 1.20;
constexpr double XY_GROUND_MAX_BELOW_LOCAL_M = 1.20;
constexpr double XY_GROUND_UNSUPPORTED_FALLBACK_ABOVE_M = 0.90;

// -----------------------------------------------------------------------
// Raw floor estimate retained per cell for the final height cap, which rejects
// returns too far above the bottom 5% level. The cap is applied once, to every
// surviving candidate, in filter_ground_mask_by_spatial_support().
// -----------------------------------------------------------------------
constexpr double XY_GROUND_CELL_FLOOR_PERCENTILE = 0.05;
constexpr double XY_GROUND_CELL_MAX_ABOVE_FLOOR_M = 0.30;

// -----------------------------------------------------------------------
// Obstacle-column detection: a cell is treated as an object when enough of its
// points sit more than 0.25 m above the local floor -- both an absolute count
// (so a lone outlier never trips it) and a fraction of the cell's points (so a
// couple of noise/edge pixels in a densely sampled cell do not either). The
// sole effect is to cap such a cell's roughness, so elevated returns cannot
// widen the ground tolerance; the height rejection itself comes from
// XY_GROUND_CELL_MAX_ABOVE_FLOOR_M above.
// -----------------------------------------------------------------------
constexpr double XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_FLOOR_M = 0.25;
constexpr size_t XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_POINTS = 2u;
constexpr double XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_FRACTION = 0.10;
constexpr double XY_GROUND_OBSTACLE_COLUMN_ROUGHNESS_CAP_M = 0.10;

// Intermediate 2.5-D ground height map built from one frame.
struct GroundModel {
    bool valid = false;
    double fallback_ground_z = std::numeric_limits<double>::quiet_NaN();
    double cell_size_m = 0.5;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double inv_cell_size = 2.0;  // 1.0 / cell_size_m (avoid per-point division)
    int rows = 0;
    int cols = 0;
    // cell_valid is authoritative: every valid cell has a finite height.
    std::vector<uint8_t> cell_valid;
    std::vector<double> cell_floor_z;
    std::vector<double> height;
    std::vector<double> roughness;
};

struct GroundComponent {
    // Flattened grid-cell indices: row * grid_column_count + column.
    std::vector<size_t> cells;
    // Total candidate points across all cells. Multiple points may occupy one cell.
    size_t point_count = 0u;
    double range_sum = 0.0;
    double min_height = std::numeric_limits<double>::infinity();
    double max_height = -std::numeric_limits<double>::infinity();
};

struct GroundComponentMap {
    // Per-grid-cell component ID. -1 means the cell has no ground
    // candidate; otherwise the value indexes components.
    std::vector<int> labels;
    // Connected ground-candidate components indexed by labels.
    std::vector<GroundComponent> components;
    // Index in components of the component containing the most grid cells.
    size_t main_component_id = 0u;
};

constexpr std::array<std::array<int, 2>, 8> GROUND_CELL_NEIGHBOR_DELTAS = {
    {{{-1, -1}}, {{-1, 0}}, {{-1, 1}}, {{0, -1}}, {{0, 1}}, {{1, -1}}, {{1, 0}}, {{1, 1}}}};

// It's a bucket store: for each cell of the 2.5-D ground grid, it holds the list of z-values
// (heights) of the points that landed in that cell. It use compressed sparse row storage format for
// sparse matrix. The ground model later takes each cell's z-samples and computes
// percentiles/median/roughness to estimate the local ground height.
class CellSampleStorage {
   public:
    explicit CellSampleStorage(size_t cell_count) : offsets_(cell_count + 1u, 0u) {}

    void count_sample(size_t cell_idx) {
        ++offsets_[cell_idx + 1u];
    }

    void allocate_samples() {
        for (size_t idx = 1; idx < offsets_.size(); ++idx) {
            offsets_[idx] += offsets_[idx - 1u];
        }
        samples_.resize(offsets_.back());
        if (!samples_.empty()) {
            write_offsets_ = offsets_;
        }
    }

    void add_sample(size_t cell_idx, double value) {
        samples_[write_offsets_[cell_idx]++] = value;
    }

    void finish_writing() {
        write_offsets_ = std::vector<size_t>{};
    }

    size_t cell_size(size_t cell_idx) const {
        return offsets_[cell_idx + 1u] - offsets_[cell_idx];
    }

    double* cell_data(size_t cell_idx) {
        return samples_.data() + offsets_[cell_idx];
    }

   private:
    // Size cell_count + 1. offsets_[i] is where cell i's samples start in
    // samples_; offsets_[i + 1] - offsets_[i] is how many it has. (The classic
    // CSR "row-pointer" array.)
    std::vector<size_t> offsets_;
    // Temporary per-cell write cursors, used only during filling.
    std::vector<size_t> write_offsets_;
    // One contiguous array with every cell's z-values packed back-to-back,
    // ordered by cell.
    std::vector<double> samples_;
};

MatrixX3dR destagger_matrix_x3(Eigen::Ref<const MatrixX3dR> values, int frame_height,
                               int frame_width, const std::vector<int>& pixel_shift_by_row) {
    const Eigen::Index expected_rows =
        static_cast<Eigen::Index>(frame_height) * static_cast<Eigen::Index>(frame_width);
    if (values.rows() != expected_rows || values.cols() != 3 ||
        pixel_shift_by_row.size() != static_cast<size_t>(frame_height)) {
        throw std::invalid_argument("matrix dimensions do not match lidar frame");
    }

    MatrixX3dR result(expected_rows, 3);
    Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> input(
        values.data(), frame_height, frame_width, 3);
    Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>> output(result.data(), frame_height,
                                                                       frame_width, 3);
    core::destagger_into<double, 3>(input, pixel_shift_by_row, false, output);
    return result;
}

// Floor-based index of the pct-th percentile into a num_vals-length sample
// array. Precondition: num_vals >= 1 (every caller guards the empty case).
size_t percentile_index(size_t num_vals, double pct) {
    return std::min(num_vals - 1,
                    static_cast<size_t>(std::floor(pct * static_cast<double>(num_vals - 1))));
}

// The grid post-processing reduces very short ranges: a fill window holds on the
// order of 7-18 values and a smoothing window at most 9. At those lengths
// std::nth_element's introselect setup costs more than an insertion sort, and
// sorting has the added benefit of leaving the range fully ordered, so a
// following order statistic over the same values needs no second selection.
constexpr std::ptrdiff_t SMALL_SORT_MAX = 40;

void insertion_sort_small(double* first, const double* last) {
    for (double* it = first + 1; it < last; ++it) {
        const double value = *it;
        double* hole = it;
        while (hole > first && *(hole - 1) > value) {
            *hole = *(hole - 1);
            --hole;
        }
        *hole = value;
    }
}

// Range-based median; reorders [first, last) in place. Selects the same
// (num_vals / 2)-th smallest element std::nth_element would, so the returned
// double is unchanged. Postcondition for num_vals <= SMALL_SORT_MAX: the range is
// left fully sorted ascending, which robust_spread_of_sorted() relies on.
double median_value(double* first, double* last) {
    const std::ptrdiff_t num_vals = last - first;
    if (num_vals <= 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::ptrdiff_t mid = num_vals / 2;
    if (num_vals <= SMALL_SORT_MAX) {
        insertion_sort_small(first, last);
    } else {
        std::nth_element(first, first + mid, last);
    }
    return first[mid];
}

// Range-based robust spread; reorders/overwrites [first, last) in place.
// Median absolute deviation is used instead of standard deviation because a
// few obstacle or vegetation returns should not inflate the estimated ground
// roughness.
double robust_spread(double* first, double* last, double center) {
    if ((last - first) < 2 || !std::isfinite(center)) {
        return 0.0;
    }
    for (double* ptr = first; ptr != last; ++ptr) {
        *ptr = std::abs(*ptr - center);
    }
    const double median_absolute_deviation = median_value(first, last);
    if (!std::isfinite(median_absolute_deviation)) {
        return 0.0;
    }
    return MEDIAN_ABSOLUTE_DEVIATION_TO_SIGMA * median_absolute_deviation;
}

// robust_spread() for a range already sorted ascending whose center is its own
// median (center == first[num_vals / 2]). Walking outward from the median, the
// deviations below it and above it each increase, so merging those two runs
// enumerates the deviations in ascending order: the median absolute deviation
// falls out in O(num_vals) with no second selection and without writing to the
// range.
double robust_spread_of_sorted(const double* first, std::ptrdiff_t num_vals, double center) {
    if (num_vals < 2 || !std::isfinite(center)) {
        return 0.0;
    }
    const std::ptrdiff_t mid = num_vals / 2;
    std::ptrdiff_t below = mid - 1;  // walks down from the median
    std::ptrdiff_t above = mid;      // walks up from the median
    double median_absolute_deviation = std::numeric_limits<double>::quiet_NaN();
    for (std::ptrdiff_t taken = 0; taken <= mid; ++taken) {
        const double below_dev =
            below >= 0 ? center - first[below] : std::numeric_limits<double>::infinity();
        const double above_dev =
            above < num_vals ? first[above] - center : std::numeric_limits<double>::infinity();
        if (above_dev <= below_dev) {
            median_absolute_deviation = above_dev;
            ++above;
        } else {
            median_absolute_deviation = below_dev;
            --below;
        }
    }
    if (!std::isfinite(median_absolute_deviation)) {
        return 0.0;
    }
    return MEDIAN_ABSOLUTE_DEVIATION_TO_SIGMA * median_absolute_deviation;
}

double estimate_xy_ground_z_from_samples(std::vector<double>& zs) {
    if (zs.empty()) {
        return 0.0;
    }

    const size_t num_vals = zs.size();
    const size_t low_idx = percentile_index(num_vals, XY_GROUND_TAIL_LOW_PCT);
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
    for (size_t i = low_idx; i <= high_idx; ++i) {
        sum += zs[i];
    }
    return sum / static_cast<double>(high_idx - low_idx + 1u);
}

constexpr size_t INVALID_GROUND_CELL = std::numeric_limits<size_t>::max();

size_t ground_model_cell_index_for_point(const GroundModel& model, const Eigen::Vector3d& point) {
    if (!model.valid) {
        return INVALID_GROUND_CELL;
    }
    const int col =
        static_cast<int>(std::floor((point.x() - model.origin_x) * model.inv_cell_size));
    const int row =
        static_cast<int>(std::floor((point.y() - model.origin_y) * model.inv_cell_size));
    if (row < 0 || row >= model.rows || col < 0 || col >= model.cols) {
        return INVALID_GROUND_CELL;
    }
    return static_cast<size_t>(row) * static_cast<size_t>(model.cols) + static_cast<size_t>(col);
}

double ground_model_local_tolerance(double range, double roughness, double lookup_distance_m) {
    // All inputs are finite and non-negative values produced by the ground
    // model and the validated lidar range.
    const double range_noise =
        std::min(XY_GROUND_RANGE_NOISE_MAX_M, XY_GROUND_RANGE_NOISE_K * range);
    const double lookup_extra = std::min(XY_GROUND_LOOKUP_SLOPE_TOL_MAX_M,
                                         XY_GROUND_LOOKUP_SLOPE_TOL_PER_M * lookup_distance_m);
    return XY_GROUND_LOCAL_BASE_TOL_M + XY_GROUND_LOCAL_ROUGHNESS_K * roughness + range_noise +
           lookup_extra;
}

size_t finalize_ground_model_cells(GroundModel& model, CellSampleStorage& cell_zs) {
    const size_t cell_count = static_cast<size_t>(model.rows) * static_cast<size_t>(model.cols);
    size_t valid_cells = 0u;
    for (size_t idx = 0; idx < cell_count; ++idx) {
        const size_t num_zs = cell_zs.cell_size(idx);
        if (num_zs == 0u) {
            continue;
        }
        double* const unfiltered_zs = cell_zs.cell_data(idx);

        // Reordering the cell samples is harmless after binning, so compute the
        // floor percentile in place instead of copying every occupied cell.
        // Every cell sample passed point.allFinite() before binning.
        const size_t floor_idx = percentile_index(num_zs, XY_GROUND_CELL_FLOOR_PERCENTILE);
        std::nth_element(unfiltered_zs, unfiltered_zs + floor_idx, unfiltered_zs + num_zs);
        const double cell_floor = unfiltered_zs[floor_idx];

        model.cell_floor_z[idx] = cell_floor;

        size_t above_floor_count = 0u;
        for (size_t sample_idx = 0; sample_idx < num_zs; ++sample_idx) {
            const double z = unfiltered_zs[sample_idx];
            if (z > cell_floor + XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_FLOOR_M) {
                ++above_floor_count;
            }
        }

        const bool obstacle_column =
            above_floor_count >= XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_POINTS &&
            static_cast<double>(above_floor_count) >=
                XY_GROUND_OBSTACLE_COLUMN_MIN_ABOVE_FRACTION * static_cast<double>(num_zs);

        // Ground height is the low percentile (cell floor) of the cell samples.
        const double h = cell_floor;

        // Estimate roughness from the raw cell samples within the lower band.
        double* const lower_band_end =
            std::remove_if(unfiltered_zs, unfiltered_zs + num_zs,
                           [h](double z) { return z > h + XY_GROUND_CELL_ROUGHNESS_BAND_M; });
        double cell_roughness = robust_spread(unfiltered_zs, lower_band_end, h);
        if (obstacle_column) {
            cell_roughness = std::min(cell_roughness, XY_GROUND_OBSTACLE_COLUMN_ROUGHNESS_CAP_M);
        }

        model.cell_valid[idx] = 1u;
        model.height[idx] = h;
        model.roughness[idx] = cell_roughness;
        ++valid_cells;
    }
    return valid_cells;
}

// Fill missing grid cells from the median of nearby valid cells. A robust
// spread gate prevents interpolation across inconsistent surface heights.
void fill_ground_model_holes(GroundModel& model,
                             int fill_radius = XY_GROUND_INITIAL_FILL_RADIUS_CELLS) {
    if (!model.valid) {
        return;
    }

    const int rows = model.rows;
    const int cols = model.cols;
    const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);

    // Pass 1: collect valid source cells and count the sources in each
    // invalid cell's square neighborhood. A summed-area table computes the
    // same counts as scattering every source over its neighborhood, but in
    // constant time per grid cell.
    std::vector<int> recv_count(cell_count, 0);
    std::vector<size_t> valid_cells;
    std::vector<size_t> touched;  // invalid cells receiving >= 1 contribution
    const size_t prefix_cols = static_cast<size_t>(cols) + 1u;
    std::vector<size_t> valid_prefix((static_cast<size_t>(rows) + 1u) * prefix_cols, 0u);
    for (int row = 0; row < rows; ++row) {
        const size_t row_base = static_cast<size_t>(row) * static_cast<size_t>(cols);
        const size_t prefix_row = (static_cast<size_t>(row) + 1u) * prefix_cols;
        const size_t previous_prefix_row = static_cast<size_t>(row) * prefix_cols;
        size_t row_valid_count = 0u;
        for (int col = 0; col < cols; ++col) {
            const size_t idx = row_base + static_cast<size_t>(col);
            if (model.cell_valid[idx] != 0u) {
                valid_cells.push_back(idx);
                ++row_valid_count;
            }
            valid_prefix[prefix_row + static_cast<size_t>(col) + 1u] =
                valid_prefix[previous_prefix_row + static_cast<size_t>(col) + 1u] + row_valid_count;
        }
    }

    for (int row = 0; row < rows; ++row) {
        const size_t row_base = static_cast<size_t>(row) * static_cast<size_t>(cols);
        for (int col = 0; col < cols; ++col) {
            const size_t idx = row_base + static_cast<size_t>(col);
            if (model.cell_valid[idx] != 0u) {
                continue;
            }
            const int row_lo = std::max(0, row - fill_radius);
            const int row_hi = std::min(rows - 1, row + fill_radius);
            const int col_lo = std::max(0, col - fill_radius);
            const int col_hi = std::min(cols - 1, col + fill_radius);
            const size_t top = static_cast<size_t>(row_lo) * prefix_cols;
            const size_t bottom = (static_cast<size_t>(row_hi) + 1u) * prefix_cols;
            const size_t left = static_cast<size_t>(col_lo);
            const size_t right = static_cast<size_t>(col_hi) + 1u;
            const size_t count = (valid_prefix[bottom + right] - valid_prefix[top + right]) -
                                 (valid_prefix[bottom + left] - valid_prefix[top + left]);
            if (count != 0u) {
                recv_count[idx] = static_cast<int>(count);
                touched.push_back(idx);
            }
        }
    }

    if (touched.empty()) {
        return;  // nothing to fill; model unchanged
    }

    // Lay out a contiguous slice per touched cell; wpos doubles as the write
    // cursor during scatter and, afterwards, points one past each slice (so the
    // slice start is recovered as wpos[cell] - recv_count[cell]).
    std::vector<size_t> wpos(cell_count);
    size_t acc = 0;
    for (const size_t cell : touched) {
        wpos[cell] = acc;
        acc += static_cast<size_t>(recv_count[cell]);
    }
    std::vector<double> flat_h(acc);
    std::vector<double> flat_r(acc);

    // Pass 2: scatter each valid cell's value to its invalid neighbors.
    for (const size_t vidx : valid_cells) {
        const int row = static_cast<int>(vidx / static_cast<size_t>(cols));
        const int col = static_cast<int>(vidx % static_cast<size_t>(cols));
        const double vh = model.height[vidx];
        const double vr = model.roughness[vidx];
        const int row_lo = std::max(0, row - fill_radius);
        const int row_hi = std::min(rows - 1, row + fill_radius);
        const int col_lo = std::max(0, col - fill_radius);
        const int col_hi = std::min(cols - 1, col + fill_radius);
        for (int rr = row_lo; rr <= row_hi; ++rr) {
            size_t nidx =
                static_cast<size_t>(rr) * static_cast<size_t>(cols) + static_cast<size_t>(col_lo);
            for (int ncol = col_lo; ncol <= col_hi; ++ncol, ++nidx) {
                if (model.cell_valid[nidx] != 0u) {
                    continue;
                }
                const size_t w = wpos[nidx]++;
                flat_h[w] = vh;
                flat_r[w] = vr;
            }
        }
    }

    // Pass 3: reduce each touched cell's collected values.  Each cell owns a
    // disjoint, contiguous slice of flat_h/flat_r that is used exactly once,
    // so the order statistics run in place (no per-cell copy).
    for (const size_t cell : touched) {
        const size_t num_vals = static_cast<size_t>(recv_count[cell]);
        const size_t start = wpos[cell] - num_vals;  // wpos now points past the slice
        double* hs = flat_h.data() + start;

        const double fill_h = median_value(hs, hs + num_vals);
        // median_value leaves short ranges fully sorted, so the spread reads off
        // that order instead of costing a second selection.
        const double spread =
            static_cast<std::ptrdiff_t>(num_vals) <= SMALL_SORT_MAX
                ? robust_spread_of_sorted(hs, static_cast<std::ptrdiff_t>(num_vals), fill_h)
                : robust_spread(hs, hs + num_vals, fill_h);
        if (spread > XY_GROUND_FILL_MAX_NEIGHBOR_SPREAD_M) {
            continue;
        }

        double* rs = flat_r.data() + start;
        model.height[cell] = fill_h;
        model.roughness[cell] = median_value(rs, rs + num_vals);
        model.cell_valid[cell] = 1u;
    }
}

// Collect the holes a refill pass should revisit: every invalid cell within
// @p radius of a pruned cell. Empty cells near a pruned platform can be selected
// by the point lookup even when the platform cell itself has been refilled, so
// the radius covers that local lookup footprint without revisiting unrelated
// holes across the grid.
std::vector<size_t> collect_refill_target_cells(const GroundModel& model,
                                                const std::vector<size_t>& pruned_cells,
                                                int radius) {
    const int rows = model.rows;
    const int cols = model.cols;
    std::vector<uint8_t> target_mask(model.cell_valid.size(), 0u);
    std::vector<size_t> target_cells;
    for (const size_t pruned_cell : pruned_cells) {
        const int row = static_cast<int>(pruned_cell / static_cast<size_t>(cols));
        const int col = static_cast<int>(pruned_cell % static_cast<size_t>(cols));
        const int row_lo = std::max(0, row - radius);
        const int row_hi = std::min(rows - 1, row + radius);
        const int col_lo = std::max(0, col - radius);
        const int col_hi = std::min(cols - 1, col + radius);
        for (int rr = row_lo; rr <= row_hi; ++rr) {
            size_t idx =
                static_cast<size_t>(rr) * static_cast<size_t>(cols) + static_cast<size_t>(col_lo);
            for (int cc = col_lo; cc <= col_hi; ++cc, ++idx) {
                if (model.cell_valid[idx] == 0u && target_mask[idx] == 0u) {
                    target_mask[idx] = 1u;
                    target_cells.push_back(idx);
                }
            }
        }
    }
    return target_cells;
}

// Refill a sparse set of cells from their valid neighbors. Unlike the initial
// whole-grid fill, this target-centric path avoids allocating and scattering
// contribution buffers for cells that the second prune did not remove.
void fill_pruned_ground_model_cells(GroundModel& model, const std::vector<size_t>& pruned_cells) {
    if (!model.valid || pruned_cells.empty()) {
        return;
    }

    struct PendingFill {
        size_t cell;
        double height;
        double roughness;
    };
    constexpr int fill_radius = XY_GROUND_FINAL_FILL_RADIUS_CELLS;
    // Widen before multiplying: the window side is computed in int, then the
    // area is accumulated in size_t.
    constexpr int fill_window_cells = 2 * fill_radius + 1;
    constexpr size_t max_neighbors =
        static_cast<size_t>(fill_window_cells) * static_cast<size_t>(fill_window_cells);
    std::array<double, max_neighbors> heights{};
    std::array<double, max_neighbors> roughnesses{};
    const int rows = model.rows;
    const int cols = model.cols;

    const std::vector<size_t> target_cells = collect_refill_target_cells(
        model, pruned_cells, XY_GROUND_LOOKUP_RADIUS_CELLS + fill_radius);

    std::vector<PendingFill> pending_fills;
    pending_fills.reserve(target_cells.size());
    for (const size_t cell : target_cells) {
        const int row = static_cast<int>(cell / static_cast<size_t>(cols));
        const int col = static_cast<int>(cell % static_cast<size_t>(cols));
        const int row_lo = std::max(0, row - fill_radius);
        const int row_hi = std::min(rows - 1, row + fill_radius);
        const int col_lo = std::max(0, col - fill_radius);
        const int col_hi = std::min(cols - 1, col + fill_radius);
        size_t count = 0u;
        for (int rr = row_lo; rr <= row_hi; ++rr) {
            size_t nidx =
                static_cast<size_t>(rr) * static_cast<size_t>(cols) + static_cast<size_t>(col_lo);
            for (int ncol = col_lo; ncol <= col_hi; ++ncol, ++nidx) {
                if (model.cell_valid[nidx] == 0u) {
                    continue;
                }
                heights[count] = model.height[nidx];
                roughnesses[count] = model.roughness[nidx];
                ++count;
            }
        }
        if (count == 0u) {
            continue;
        }

        const double fill_h = median_value(heights.data(), heights.data() + count);
        const double spread = count <= static_cast<size_t>(SMALL_SORT_MAX)
                                  ? robust_spread_of_sorted(
                                        heights.data(), static_cast<std::ptrdiff_t>(count), fill_h)
                                  : robust_spread(heights.data(), heights.data() + count, fill_h);
        if (spread > XY_GROUND_FILL_MAX_NEIGHBOR_SPREAD_M) {
            continue;
        }
        pending_fills.push_back(
            {cell, fill_h, median_value(roughnesses.data(), roughnesses.data() + count)});
    }

    // Apply after all reductions so cells filled in this pass never become
    // sources for another target in the same pass.
    for (const auto& fill : pending_fills) {
        model.height[fill.cell] = fill.height;
        model.roughness[fill.cell] = fill.roughness;
        model.cell_valid[fill.cell] = 1u;
    }
}

void smooth_ground_model_edge_preserving(GroundModel& model) {
    if (!model.valid) {
        return;
    }

    const int rows = model.rows;
    const int cols = model.cols;

    std::vector<double> smooth_height = model.height;
    std::vector<double> smooth_roughness = model.roughness;

    // A 3x3 window holds at most nine values, so fixed-size stack buffers
    // suffice.
    std::array<double, 9> heights{};
    std::array<double, 9> roughnesses{};
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const size_t idx =
                static_cast<size_t>(row) * static_cast<size_t>(cols) + static_cast<size_t>(col);
            if (model.cell_valid[idx] == 0u) {
                continue;
            }

            const double center_h = model.height[idx];
            size_t count = 0;
            const int row_lo = std::max(0, row - 1);
            const int row_hi = std::min(rows - 1, row + 1);
            const int col_lo = std::max(0, col - 1);
            const int col_hi = std::min(cols - 1, col + 1);
            for (int rr = row_lo; rr <= row_hi; ++rr) {
                size_t nidx = static_cast<size_t>(rr) * static_cast<size_t>(cols) +
                              static_cast<size_t>(col_lo);
                for (int ncol = col_lo; ncol <= col_hi; ++ncol, ++nidx) {
                    if (model.cell_valid[nidx] == 0u) {
                        continue;
                    }
                    if (std::abs(model.height[nidx] - center_h) >
                        XY_GROUND_SMOOTH_MAX_HEIGHT_DIFF_M) {
                        continue;
                    }
                    heights.at(count) = model.height[nidx];
                    roughnesses.at(count) = model.roughness[nidx];
                    ++count;
                }
            }
            if (count > 0u) {
                smooth_height[idx] = median_value(heights.data(), heights.data() + count);
                smooth_roughness[idx] =
                    median_value(roughnesses.data(), roughnesses.data() + count);
            }
        }
    }

    model.height = std::move(smooth_height);
    model.roughness = std::move(smooth_roughness);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<size_t> prune_cells_above_local_lower_surface(GroundModel& model) {
    if (!model.valid || model.rows <= 0 || model.cols <= 0 ||
        !std::isfinite(model.fallback_ground_z)) {
        return {};
    }

    const int rows = model.rows;
    const int cols = model.cols;
    std::vector<size_t> rejected_cells;

    const auto cell_index = [cols](int row, int col) {
        return static_cast<size_t>(row) * static_cast<size_t>(cols) + static_cast<size_t>(col);
    };

    struct LocalSupportOffset {
        int row;
        int col;
        double max_height_delta;
    };
    std::vector<LocalSupportOffset> offsets;
    offsets.reserve(XY_GROUND_MODEL_LOCAL_SUPPORT_NEIGHBOR_COUNT);
    for (int dr = -XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS;
         dr <= XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS; ++dr) {
        for (int dc = -XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS;
             dc <= XY_GROUND_MODEL_LOCAL_SUPPORT_RADIUS_CELLS; ++dc) {
            if (dr == 0 && dc == 0) {
                continue;
            }
            const double distance_m =
                model.cell_size_m * std::sqrt(static_cast<double>(dr * dr + dc * dc));
            offsets.push_back({dr, dc, XY_GROUND_MODEL_LOCAL_SUPPORT_SLOPE_PER_M * distance_m});
        }
    }

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const size_t idx = cell_index(row, col);
            if (model.cell_valid[idx] == 0u ||
                model.height[idx] <=
                    model.fallback_ground_z + XY_GROUND_MODEL_LOCAL_SUPPORT_MIN_ABOVE_FALLBACK_M) {
                continue;
            }

            size_t local_support_count = 0u;
            double max_height_from_local_low = std::numeric_limits<double>::infinity();
            for (const auto& offset : offsets) {
                const int neighbor_row = row + offset.row;
                const int neighbor_col = col + offset.col;
                if (neighbor_row < 0 || neighbor_row >= rows || neighbor_col < 0 ||
                    neighbor_col >= cols) {
                    continue;
                }
                const size_t neighbor_idx = cell_index(neighbor_row, neighbor_col);
                if (model.cell_valid[neighbor_idx] == 0u) {
                    continue;
                }
                ++local_support_count;
                max_height_from_local_low =
                    std::min(max_height_from_local_low,
                             model.height[neighbor_idx] + offset.max_height_delta);
            }

            if (local_support_count >= XY_GROUND_MODEL_LOCAL_SUPPORT_MIN_CELLS &&
                model.height[idx] > max_height_from_local_low) {
                rejected_cells.push_back(idx);
            }
        }
    }

    for (const size_t idx : rejected_cells) {
        model.cell_valid[idx] = 0u;
        model.height[idx] = std::numeric_limits<double>::quiet_NaN();
        model.roughness[idx] = 0.0;
    }
    return rejected_cells;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
GroundModel build_lower_envelope_ground_model(
    Eigen::Ref<const PointCloudXYZd> points,
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& range,
    Eigen::Ref<const Eigen::Array<uint32_t, Eigen::Dynamic, 1>> status, int first_col, int last_col,
    double grid_size, const PointCloudXYZd* points2,
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>* range2) {
    GroundModel model;

    const int frame_height = static_cast<int>(range.rows());
    const int frame_width = static_cast<int>(range.cols());
    const Eigen::Index frame_size =
        static_cast<Eigen::Index>(frame_height) * static_cast<Eigen::Index>(frame_width);
    const bool have_second_return =
        points2 != nullptr && range2 != nullptr && points2->rows() == frame_size &&
        points2->cols() == points.cols() && range2->rows() == range.rows() &&
        range2->cols() == range.cols();
    if (points.rows() != frame_size || last_col < first_col || last_col >= frame_width) {
        return model;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    std::vector<double> all_zs;
    all_zs.reserve(static_cast<size_t>(frame_size) * (have_second_return ? 2u : 1u));
    std::vector<int> valid_columns;
    const size_t valid_column_capacity =
        static_cast<size_t>(static_cast<int64_t>(last_col) - static_cast<int64_t>(first_col) + 1LL);
    valid_columns.reserve(valid_column_capacity);
    for (int col = first_col; col <= last_col; ++col) {
        if (status(col) != 0u) {
            valid_columns.push_back(col);
        }
    }

    const auto visit_valid_return_points =
        [&](const PointCloudXYZd& return_points,
            const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& return_range,
            const auto& visitor) {
            for (int row = 0; row < frame_height; ++row) {
                for (const int col : valid_columns) {
                    if (return_range(row, col) == 0u) {
                        continue;
                    }
                    const Eigen::Index idx = static_cast<Eigen::Index>(row) * frame_width + col;
                    const Eigen::Vector3d point = return_points.row(idx).matrix();
                    if (!point.allFinite() || point.squaredNorm() < MIN_RANGE_M_SQ) {
                        continue;
                    }
                    visitor(idx, row, col, point);
                }
            }
        };

    // Save points that pass the validity checks so the binning pass below
    // doesn't re-walk the frame and repeat the same range/status/norm tests.
    std::vector<Eigen::Index> valid_indices;
    valid_indices.reserve(static_cast<size_t>(frame_size) * (have_second_return ? 2u : 1u));

    const auto update_extents = [&](Eigen::Index idx, int /*row*/, int /*col*/,
                                    const Eigen::Vector3d& point) {
        min_x = std::min(min_x, point.x());
        min_y = std::min(min_y, point.y());
        max_x = std::max(max_x, point.x());
        max_y = std::max(max_y, point.y());
        all_zs.push_back(point.z());
        valid_indices.push_back(idx);
    };

    visit_valid_return_points(points, range, update_extents);
    const size_t first_return_count = valid_indices.size();
    if (have_second_return) {
        visit_valid_return_points(*points2, *range2, update_extents);
    }

    // all_zs contains only finite points, so non-empty also guarantees finite
    // XY extents.
    if (all_zs.empty()) {
        return model;
    }

    model.cell_size_m = grid_size;
    model.inv_cell_size = 1.0 / grid_size;
    model.origin_x = std::floor(min_x * model.inv_cell_size) * model.cell_size_m;
    model.origin_y = std::floor(min_y * model.inv_cell_size) * model.cell_size_m;
    model.cols =
        std::max(1, static_cast<int>(std::ceil((max_x - model.origin_x) / model.cell_size_m)) + 1);
    model.rows =
        std::max(1, static_cast<int>(std::ceil((max_y - model.origin_y) / model.cell_size_m)) + 1);

    const size_t cell_count = static_cast<size_t>(model.rows) * static_cast<size_t>(model.cols);
    CellSampleStorage cell_zs(cell_count);
    model.cell_valid.assign(cell_count, 0u);
    model.cell_floor_z.assign(cell_count, std::numeric_limits<double>::quiet_NaN());
    model.height.assign(cell_count, std::numeric_limits<double>::quiet_NaN());
    model.roughness.assign(cell_count, 0.0);

    {
        std::vector<size_t> sample_cell_indices(valid_indices.size(), INVALID_GROUND_CELL);

        const auto cell_index_for_point = [&](const Eigen::Vector3d& point, size_t& idx) {
            const int col =
                static_cast<int>(std::floor((point.x() - model.origin_x) * model.inv_cell_size));
            const int row =
                static_cast<int>(std::floor((point.y() - model.origin_y) * model.inv_cell_size));
            if (row < 0 || row >= model.rows || col < 0 || col >= model.cols) {
                return false;
            }
            idx = static_cast<size_t>(row) * static_cast<size_t>(model.cols) +
                  static_cast<size_t>(col);
            return true;
        };

        const auto count_model_sample = [&](size_t sample_idx, const Eigen::Vector3d& point) {
            size_t cell_idx = 0u;
            if (!cell_index_for_point(point, cell_idx)) {
                return;
            }
            sample_cell_indices[sample_idx] = cell_idx;
            cell_zs.count_sample(cell_idx);
        };

        for (size_t i = 0; i < first_return_count; ++i) {
            count_model_sample(i, points.row(valid_indices[i]).matrix());
        }
        for (size_t i = first_return_count; i < valid_indices.size(); ++i) {
            count_model_sample(i, points2->row(valid_indices[i]).matrix());
        }

        cell_zs.allocate_samples();
        for (size_t i = 0; i < valid_indices.size(); ++i) {
            const size_t cell_idx = sample_cell_indices[i];
            if (cell_idx == INVALID_GROUND_CELL) {
                continue;
            }
            cell_zs.add_sample(cell_idx, all_zs[i]);
        }
        cell_zs.finish_writing();
    }
    model.fallback_ground_z = estimate_xy_ground_z_from_samples(all_zs);

    const size_t valid_cells = finalize_ground_model_cells(model, cell_zs);

    if (valid_cells == 0u) {
        return model;
    }

    model.valid = true;
    // Reject compact elevated surfaces before interpolation can manufacture
    // a gradual bridge from the surrounding lower ground.
    prune_cells_above_local_lower_surface(model);

    // Fill sparse gaps from the retained local lower surface. The spread gate
    // avoids filling across object edges.
    fill_ground_model_holes(model);
    smooth_ground_model_edge_preserving(model);

    // Re-check after interpolation has replaced rejected platform edges with
    // the surrounding lower surface.
    const std::vector<size_t> pruned_cells = prune_cells_above_local_lower_surface(model);

    // Fill newly pruned platform cells from nearby low ground using the
    // smaller radius to avoid over-expanding into object boundaries.
    fill_pruned_ground_model_cells(model, pruned_cells);

    return model;
}

bool xy_point_is_ground_like(const Eigen::Vector3d& point, double range,
                             const GroundModel& ground_model, size_t cell_idx) {
    if (cell_idx != INVALID_GROUND_CELL) {
        // Ground model lookup starting from the already-computed cell.
        const auto cell_index = [&](int r, int col) {
            return static_cast<size_t>(r) * static_cast<size_t>(ground_model.cols) +
                   static_cast<size_t>(col);
        };

        double local_height = std::numeric_limits<double>::quiet_NaN();
        double local_roughness = 0.0;
        double lookup_distance_m = 0.0;
        bool lookup_found = false;

        if (ground_model.cell_valid[cell_idx] != 0u) {
            local_height = ground_model.height[cell_idx];
            local_roughness = ground_model.roughness[cell_idx];
            lookup_found = true;
        } else {
            const int cell_row =
                static_cast<int>(cell_idx / static_cast<size_t>(ground_model.cols));
            const int cell_col =
                static_cast<int>(cell_idx % static_cast<size_t>(ground_model.cols));
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
                        if (ground_model.cell_valid[nidx] == 0u) {
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
            const double residual = point.z() - local_height;
            return residual <= local_tol && residual >= -XY_GROUND_MAX_BELOW_LOCAL_M;
        }
    }

    // If this exact area has no nearby model support, allow only a narrow
    // global low-ground fallback. This recovers sparse true-ground holes
    // while still rejecting elevated container/trailer tops.
    if (!std::isfinite(ground_model.fallback_ground_z)) {
        return false;
    }
    if (ground_model.valid) {
        const double residual_to_global = point.z() - ground_model.fallback_ground_z;
        return residual_to_global <= XY_GROUND_UNSUPPORTED_FALLBACK_ABOVE_M &&
               residual_to_global >= -XY_GROUND_MAX_BELOW_LOCAL_M;
    }

    return point.z() <= ground_model.fallback_ground_z + XY_GROUND_UNSUPPORTED_FALLBACK_ABOVE_M;
}

GroundComponentMap build_ground_component_map(const std::vector<double>& candidate_height,
                                              const std::vector<size_t>& candidate_point_count,
                                              const std::vector<double>& candidate_range_sum,
                                              int rows, int cols, double cell_size_m) {
    const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    GroundComponentMap result;
    result.labels.assign(cell_count, -1);

    const double cardinal_step_limit = XY_GROUND_MAIN_COMPONENT_BASE_STEP_M +
                                       XY_GROUND_MAIN_COMPONENT_MAX_SLOPE_PER_M * cell_size_m;
    const double diagonal_step_distance = cell_size_m * std::sqrt(2.0);
    const double diagonal_step_limit =
        XY_GROUND_MAIN_COMPONENT_BASE_STEP_M +
        XY_GROUND_MAIN_COMPONENT_MAX_SLOPE_PER_M * diagonal_step_distance;

    const auto cell_index = [cols](int row, int col) {
        return static_cast<size_t>(row) * static_cast<size_t>(cols) + static_cast<size_t>(col);
    };
    for (size_t start = 0; start < cell_count; ++start) {
        if (candidate_point_count[start] == 0u || result.labels[start] >= 0) {
            continue;
        }

        const int component_id = static_cast<int>(result.components.size());
        result.components.emplace_back();
        auto& component = result.components.back();
        component.cells.push_back(start);
        result.labels[start] = component_id;

        for (size_t head = 0; head < component.cells.size(); ++head) {
            const size_t idx = component.cells[head];
            component.point_count += candidate_point_count[idx];
            component.range_sum += candidate_range_sum[idx];
            component.min_height = std::min(component.min_height, candidate_height[idx]);
            component.max_height = std::max(component.max_height, candidate_height[idx]);
            const int row = static_cast<int>(idx / static_cast<size_t>(cols));
            const int col = static_cast<int>(idx % static_cast<size_t>(cols));
            for (const auto& delta : GROUND_CELL_NEIGHBOR_DELTAS) {
                const int neighbor_row = row + delta[0];
                const int neighbor_col = col + delta[1];
                if (neighbor_row < 0 || neighbor_row >= rows || neighbor_col < 0 ||
                    neighbor_col >= cols) {
                    continue;
                }
                const size_t next = cell_index(neighbor_row, neighbor_col);
                const double step_limit =
                    delta[0] != 0 && delta[1] != 0 ? diagonal_step_limit : cardinal_step_limit;
                if (result.labels[next] >= 0 || candidate_point_count[next] == 0u ||
                    std::abs(candidate_height[next] - candidate_height[idx]) > step_limit) {
                    continue;
                }
                result.labels[next] = component_id;
                component.cells.push_back(next);
            }
        }
    }

    if (!result.components.empty()) {
        result.main_component_id = static_cast<size_t>(std::distance(
            result.components.begin(),
            std::max_element(result.components.begin(), result.components.end(),
                             [](const GroundComponent& lhs, const GroundComponent& rhs) {
                                 return lhs.cells.size() < rhs.cells.size();
                             })));
    }
    return result;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<uint8_t> find_rejected_ground_components(const std::vector<double>& candidate_height,
                                                     const GroundComponentMap& component_map,
                                                     int rows, int cols, double cell_size_m,
                                                     double fallback_ground_z) {
    std::vector<uint8_t> reject_component(component_map.components.size(), 0u);

    // Sparse far-range patches are retained because angular sampling can split
    // real terrain into tiny components. Only components with enough spatial
    // evidence need the more expensive main-terrain height comparison.
    std::vector<uint8_t> evaluate_component(component_map.components.size(), 0u);
    size_t remaining_evaluation_cells = 0u;
    for (size_t component_id = 0; component_id < component_map.components.size(); ++component_id) {
        if (component_id == component_map.main_component_id) {
            continue;
        }
        const auto& component = component_map.components[component_id];
        const double mean_range =
            component.point_count > 0u
                ? component.range_sum / static_cast<double>(component.point_count)
                : std::numeric_limits<double>::infinity();
        const double points_per_cell = static_cast<double>(component.point_count) /
                                       static_cast<double>(component.cells.size());
        // Sparse far-range terrain is normally retained, but a component
        // whose entire lower surface is elevated still needs comparison with
        // the main terrain. Otherwise isolated tree returns bypass the height
        // test solely because they are far away and sparsely sampled.
        const bool wholly_elevated =
            std::isfinite(fallback_ground_z) &&
            component.min_height > fallback_ground_z + XY_GROUND_FAR_SPARSE_MIN_ELEVATION_M;
        if (mean_range <= XY_GROUND_DISCONNECTED_COMPONENT_NEAR_RANGE_M ||
            points_per_cell >= XY_GROUND_DISCONNECTED_COMPONENT_MIN_POINTS_PER_CELL ||
            wholly_elevated) {
            evaluate_component[component_id] = 1u;
            remaining_evaluation_cells += component.cells.size();
        }
    }
    if (remaining_evaluation_cells == 0u) {
        return reject_component;
    }

    const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    const auto cell_index = [cols](int row, int col) {
        return static_cast<size_t>(row) * static_cast<size_t>(cols) + static_cast<size_t>(col);
    };
    std::vector<size_t> main_boundary_cells;
    main_boundary_cells.reserve(
        component_map.components[component_map.main_component_id].cells.size());
    double main_boundary_min_height = std::numeric_limits<double>::infinity();
    double main_boundary_max_height = -std::numeric_limits<double>::infinity();
    for (const size_t idx : component_map.components[component_map.main_component_id].cells) {
        const int row = static_cast<int>(idx / static_cast<size_t>(cols));
        const int col = static_cast<int>(idx % static_cast<size_t>(cols));
        bool is_boundary = false;
        for (const auto& delta : GROUND_CELL_NEIGHBOR_DELTAS) {
            const int neighbor_row = row + delta[0];
            const int neighbor_col = col + delta[1];
            if (neighbor_row < 0 || neighbor_row >= rows || neighbor_col < 0 ||
                neighbor_col >= cols ||
                component_map.labels[cell_index(neighbor_row, neighbor_col)] !=
                    static_cast<int>(component_map.main_component_id)) {
                is_boundary = true;
                break;
            }
        }
        if (is_boundary) {
            main_boundary_cells.push_back(idx);
            main_boundary_min_height = std::min(main_boundary_min_height, candidate_height[idx]);
            main_boundary_max_height = std::max(main_boundary_max_height, candidate_height[idx]);
        }
    }

    // Resolve components whose residuals are all on the same side of the
    // median threshold without running the distance propagation. For every
    // target cell, the source height lies within the main-boundary extrema and
    // the permitted gap rise lies in [0, MAX_GAP_RISE].
    for (size_t component_id = 0; component_id < component_map.components.size(); ++component_id) {
        if (evaluate_component[component_id] == 0u) {
            continue;
        }
        const auto& component = component_map.components[component_id];
        const double minimum_possible_residual = component.min_height - main_boundary_max_height -
                                                 XY_GROUND_DISCONNECTED_COMPONENT_MAX_GAP_RISE_M;
        const double maximum_possible_residual = component.max_height - main_boundary_min_height;
        if (minimum_possible_residual > XY_GROUND_DISCONNECTED_COMPONENT_BASE_HEIGHT_M) {
            reject_component[component_id] = 1u;
            evaluate_component[component_id] = 0u;
            remaining_evaluation_cells -= component.cells.size();
        } else if (maximum_possible_residual <= XY_GROUND_DISCONNECTED_COMPONENT_BASE_HEIGHT_M) {
            evaluate_component[component_id] = 0u;
            remaining_evaluation_cells -= component.cells.size();
        }
    }
    if (remaining_evaluation_cells == 0u) {
        return reject_component;
    }

    // Find the nearest main-terrain cell and its height. Keep the permitted
    // rise across empty space bounded so distant trees and ledges are not
    // accepted as terrain.
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> nearest_main_distance(cell_count, infinity);
    for (const size_t idx : component_map.components[component_map.main_component_id].cells) {
        nearest_main_distance[idx] = 0.0;
    }
    using DistanceCell = std::pair<std::pair<double, size_t>, double>;
    std::vector<DistanceCell> frontier_storage;
    frontier_storage.reserve(cell_count);
    std::priority_queue<DistanceCell, std::vector<DistanceCell>, std::greater<>> frontier(
        std::greater<>(), std::move(frontier_storage));
    for (const size_t idx : main_boundary_cells) {
        frontier.emplace(std::make_pair(0.0, idx), candidate_height[idx]);
    }

    std::vector<size_t> component_visited_count(component_map.components.size(), 0u);
    std::vector<size_t> component_above_threshold_count(component_map.components.size(), 0u);

    const double cardinal_step_distance = cell_size_m;
    const double diagonal_step_distance = cell_size_m * std::sqrt(2.0);

    while (!frontier.empty()) {
        const DistanceCell current = frontier.top();
        frontier.pop();
        const double distance = current.first.first;
        const size_t idx = current.first.second;
        const double main_height = current.second;
        if (distance > nearest_main_distance[idx]) {
            continue;
        }
        const int component_id = component_map.labels[idx];
        if (component_id >= 0 &&
            component_id != static_cast<int>(component_map.main_component_id) &&
            evaluate_component[static_cast<size_t>(component_id)] != 0u) {
            const size_t evaluated_component_id = static_cast<size_t>(component_id);
            const size_t component_size =
                component_map.components[evaluated_component_id].cells.size();
            const double gap_rise =
                std::min(XY_GROUND_DISCONNECTED_COMPONENT_MAX_GAP_RISE_M,
                         XY_GROUND_DISCONNECTED_COMPONENT_MAX_SLOPE_PER_M * distance);
            if (candidate_height[idx] - main_height - gap_rise >
                XY_GROUND_DISCONNECTED_COMPONENT_BASE_HEIGHT_M) {
                ++component_above_threshold_count[evaluated_component_id];
            }
            const size_t visited_count = ++component_visited_count[evaluated_component_id];
            --remaining_evaluation_cells;
            // median_value() selects the upper median at index size / 2. It
            // exceeds the rejection threshold exactly when at least ceil(size / 2)
            // residuals exceed that threshold, so track the count instead of
            // allocating and partitioning a residual vector per component.
            const size_t required_above_count = component_size / 2u + component_size % 2u;
            const size_t remaining_component_cells = component_size - visited_count;
            if (component_above_threshold_count[evaluated_component_id] >= required_above_count) {
                reject_component[evaluated_component_id] = 1u;
                evaluate_component[evaluated_component_id] = 0u;
                remaining_evaluation_cells -= remaining_component_cells;
            } else if (component_above_threshold_count[evaluated_component_id] +
                           remaining_component_cells <
                       required_above_count) {
                evaluate_component[evaluated_component_id] = 0u;
                remaining_evaluation_cells -= remaining_component_cells;
            }
            if (remaining_evaluation_cells == 0u) {
                break;
            }
        }

        const int row = static_cast<int>(idx / static_cast<size_t>(cols));
        const int col = static_cast<int>(idx % static_cast<size_t>(cols));
        for (const auto& delta : GROUND_CELL_NEIGHBOR_DELTAS) {
            const int neighbor_row = row + delta[0];
            const int neighbor_col = col + delta[1];
            if (neighbor_row < 0 || neighbor_row >= rows || neighbor_col < 0 ||
                neighbor_col >= cols) {
                continue;
            }
            const size_t next = cell_index(neighbor_row, neighbor_col);
            const double step_distance =
                delta[0] != 0 && delta[1] != 0 ? diagonal_step_distance : cardinal_step_distance;
            const double next_distance = distance + step_distance;
            if (next_distance < nearest_main_distance[next]) {
                nearest_main_distance[next] = next_distance;
                frontier.emplace(std::make_pair(next_distance, next), main_height);
            }
        }
    }

    return reject_component;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void filter_ground_mask_by_spatial_support(
    Eigen::Ref<const PointCloudXYZd> points,
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& range, int first_col, int last_col,
    const GroundModel& ground_model, const std::vector<size_t>& point_cell_indices,
    const std::vector<size_t>& valid_point_count, Eigen::Ref<core::img_t<uint8_t>> ground_mask) {
    if (!ground_model.valid || ground_model.rows <= 0 || ground_model.cols <= 0 ||
        points.rows() != ground_mask.size() || points.cols() != 3 ||
        range.rows() != ground_mask.rows() || range.cols() != ground_mask.cols() ||
        point_cell_indices.size() != static_cast<size_t>(ground_mask.size()) || first_col < 0 ||
        last_col < first_col || last_col >= range.cols()) {
        throw std::invalid_argument("spatial support filter inputs are invalid");
    }

    const int rows = ground_model.rows;
    const int cols = ground_model.cols;
    const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    if (valid_point_count.size() != cell_count) {
        throw std::logic_error("valid point count size does not match ground model");
    }
    std::vector<double> candidate_height(cell_count, std::numeric_limits<double>::quiet_NaN());
    std::vector<size_t> candidate_point_count(cell_count, 0u);
    std::vector<double> candidate_range_sum(cell_count, 0.0);
    std::vector<Eigen::Index> candidate_point_indices;
    candidate_point_indices.reserve(static_cast<size_t>(ground_mask.size()) / 4u);

    for (Eigen::Index row = 0; row < ground_mask.rows(); ++row) {
        for (int col = first_col; col <= last_col; ++col) {
            if (ground_mask(row, col) == 0u) {
                continue;
            }
            const Eigen::Index point_idx = row * ground_mask.cols() + col;
            const size_t idx = point_cell_indices[static_cast<size_t>(point_idx)];
            if (idx == INVALID_GROUND_CELL) {
                continue;
            }
            candidate_point_indices.push_back(point_idx);
            ++candidate_point_count[idx];
            candidate_range_sum[idx] += static_cast<double>(range(row, col)) * core::RANGE_UNIT;
            const double point_z = points(point_idx, 2);
            if (candidate_point_count[idx] == 1u || point_z < candidate_height[idx]) {
                candidate_height[idx] = point_z;
            }
        }
    }

    const GroundComponentMap component_map =
        build_ground_component_map(candidate_height, candidate_point_count, candidate_range_sum,
                                   rows, cols, ground_model.cell_size_m);
    if (component_map.components.empty()) {
        return;
    }
    const std::vector<uint8_t> reject_component =
        find_rejected_ground_components(candidate_height, component_map, rows, cols,
                                        ground_model.cell_size_m, ground_model.fallback_ground_z);

    std::vector<uint8_t> reject_cell(cell_count, 0u);
    for (size_t component_id = 0; component_id < component_map.components.size(); ++component_id) {
        const bool rejected_component = reject_component[component_id] != 0u;
        for (const size_t idx : component_map.components[component_id].cells) {
            const size_t surviving_ground_votes =
                rejected_component ? 0u : candidate_point_count[idx];
            if (valid_point_count[idx] >= XY_GROUND_CELL_VOTE_MIN_VALID_POINTS &&
                static_cast<double>(surviving_ground_votes) <
                    XY_GROUND_CELL_VOTE_MIN_GROUND_FRACTION *
                        static_cast<double>(valid_point_count[idx])) {
                reject_cell[idx] = 1u;
            }
        }
    }

    for (const Eigen::Index point_idx : candidate_point_indices) {
        const size_t idx = point_cell_indices[static_cast<size_t>(point_idx)];
        const int component_id = component_map.labels[idx];
        const bool rejected_component =
            component_id >= 0 && reject_component[static_cast<size_t>(component_id)] != 0u;
        // All grid vectors contain rows * cols elements.
        const bool above_cell_floor = std::isfinite(ground_model.cell_floor_z[idx]) &&
                                      points(point_idx, 2) > ground_model.cell_floor_z[idx] +
                                                                 XY_GROUND_CELL_MAX_ABOVE_FLOOR_M;
        if (rejected_component || reject_cell[idx] != 0u || above_cell_floor) {
            ground_mask.data()[point_idx] = 0u;
        }
    }
}

bool has_compatible_ground_slope(const Eigen::Vector3d& point,
                                 const Eigen::Vector3d& neighbor_point, double max_slope_sq) {
    // Both callers validate the center point once before scanning neighbors.
    if (!neighbor_point.allFinite()) {
        return false;
    }
    const Eigen::Vector3d delta = point - neighbor_point;
    const double horizontal_distance_sq = delta.x() * delta.x() + delta.y() * delta.y();
    const double vertical_distance_sq = delta.z() * delta.z();
    return (horizontal_distance_sq <= NORMAL_EPS_SQ && vertical_distance_sq <= NORMAL_EPS_SQ) ||
           (horizontal_distance_sq > NORMAL_EPS_SQ &&
            vertical_distance_sq <= max_slope_sq * horizontal_distance_sq);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void filter_destaggered_ground_mask_by_normals_and_slope(
    Eigen::Ref<const core::PointCloudXYZd> destaggered_points,
    const core::MatrixX3dR& destaggered_normals, const std::vector<int>& pixel_shift_by_row,
    double max_ground_slope_deg, const GroundModel& ground_model,
    Eigen::Ref<core::img_t<uint8_t>> ground_mask) {
    const int frame_height = static_cast<int>(ground_mask.rows());
    const int frame_width = static_cast<int>(ground_mask.cols());
    const Eigen::Index frame_size =
        static_cast<Eigen::Index>(frame_height) * static_cast<Eigen::Index>(frame_width);
    if (frame_height <= 0 || frame_width <= 0 || destaggered_points.rows() != frame_size ||
        destaggered_points.cols() != 3 || destaggered_normals.rows() != frame_size ||
        destaggered_normals.cols() != 3 ||
        pixel_shift_by_row.size() != static_cast<size_t>(frame_height)) {
        throw std::invalid_argument("normal filter inputs do not match the ground-mask shape");
    }
    if (!std::isfinite(max_ground_slope_deg) || max_ground_slope_deg <= 0.0 ||
        max_ground_slope_deg > 90.0) {
        throw std::invalid_argument("max_ground_slope_deg must be in (0, 90]");
    }

    core::img_t<uint8_t> filtered = core::destagger<uint8_t>(ground_mask, pixel_shift_by_row);

    // A noisy or missing normal is not enough by itself to permanently reject
    // a height-classified point. Save it for the constrained recovery below.
    std::vector<Eigen::Index> normal_recovery_candidates;
    normal_recovery_candidates.reserve(static_cast<size_t>(frame_size) / 16u);
    const bool unrestricted_angle = max_ground_slope_deg >= 90.0;
    const double max_angle_rad = max_ground_slope_deg * DEG_TO_RAD;
    const double min_abs_normal_z = unrestricted_angle ? 0.0 : std::cos(max_angle_rad);
    const double min_abs_normal_z_sq = min_abs_normal_z * min_abs_normal_z;
    for (int row = 0; row < frame_height; ++row) {
        for (int col = 0; col < frame_width; ++col) {
            if (filtered(row, col) == 0u) {
                continue;
            }
            const Eigen::Index idx = static_cast<Eigen::Index>(row) * frame_width + col;
            const Eigen::Vector3d normal = destaggered_normals.row(idx).matrix();
            const double normal_norm_sq = normal.squaredNorm();
            if (!normal.allFinite() || normal_norm_sq <= NORMAL_EPS_SQ) {
                normal_recovery_candidates.push_back(idx);
                filtered(row, col) = 0u;
                continue;
            }
            if (normal.z() * normal.z() < min_abs_normal_z_sq * normal_norm_sq) {
                normal_recovery_candidates.push_back(idx);
                filtered(row, col) = 0u;
                continue;
            }
        }
    }

    const double max_slope =
        unrestricted_angle ? std::numeric_limits<double>::infinity() : std::tan(max_angle_rad);
    const double max_slope_sq = max_slope * max_slope;
    std::vector<Eigen::Index> changed_indices;
    changed_indices.reserve(normal_recovery_candidates.size());
    for (const Eigen::Index idx : normal_recovery_candidates) {
        const int row = static_cast<int>(idx / frame_width);
        const int col = static_cast<int>(idx % frame_width);
        const Eigen::Vector3d point = destaggered_points.row(idx).transpose();
        if (!point.allFinite()) {
            continue;
        }

        // Recover only points close to the local 2.5-D ground height. This
        // prevents low foliage or object boundaries from borrowing support
        // from nearby ground pixels.
        const size_t cell_idx = ground_model_cell_index_for_point(ground_model, point);
        if (cell_idx == INVALID_GROUND_CELL || ground_model.cell_valid[cell_idx] == 0u ||
            std::abs(point.z() - ground_model.height[cell_idx]) >
                GROUND_NORMAL_RECOVERY_MAX_HEIGHT_RESIDUAL_M) {
            continue;
        }

        size_t support_count = 0u;
        for (int dr = -1; dr <= 1 && support_count < MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT; ++dr) {
            const int neighbor_row = row + dr;
            if (neighbor_row < 0 || neighbor_row >= frame_height) {
                continue;
            }
            for (int dc = -1; dc <= 1; ++dc) {
                if (dr == 0 && dc == 0) {
                    continue;
                }
                const int neighbor_col = (col + dc + frame_width) % frame_width;
                // filtered is unchanged until every candidate is evaluated, so
                // only original angle-approved points can provide support.
                // Recovered points therefore never propagate.
                if (filtered(neighbor_row, neighbor_col) == 0u) {
                    continue;
                }
                const Eigen::Index neighbor_idx =
                    static_cast<Eigen::Index>(neighbor_row) * frame_width + neighbor_col;
                const Eigen::Vector3d neighbor_point =
                    destaggered_points.row(neighbor_idx).transpose();
                if (has_compatible_ground_slope(point, neighbor_point, max_slope_sq)) {
                    ++support_count;
                }
            }
        }
        if (support_count >= MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT) {
            changed_indices.push_back(idx);
        }
    }
    for (const Eigen::Index idx : changed_indices) {
        filtered.data()[idx] = 1u;
    }

    // Remove isolated boundary and foliage outliers, including valid but
    // locally inconsistent normals. Use the complete filtered mask for this
    // single pass so endpoints are not eroded recursively.
    changed_indices.clear();
    for (int row = 0; row < frame_height; ++row) {
        for (int col = 0; col < frame_width; ++col) {
            if (filtered(row, col) == 0u) {
                continue;
            }
            const Eigen::Index idx = static_cast<Eigen::Index>(row) * frame_width + col;
            const Eigen::Vector3d point = destaggered_points.row(idx).transpose();
            if (!point.allFinite()) {
                changed_indices.push_back(idx);
                continue;
            }
            const Eigen::Vector3d normal = destaggered_normals.row(idx).transpose();
            const double normal_norm_sq = normal.squaredNorm();
            const bool passed_normal_angle =
                normal.allFinite() && normal_norm_sq > NORMAL_EPS_SQ &&
                normal.z() * normal.z() >= min_abs_normal_z_sq * normal_norm_sq;
            size_t coherent_normal_count =
                passed_normal_angle ? 0u : MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT;

            size_t support_count = 0u;
            for (int dr = -1;
                 dr <= 1 && (support_count < MIN_GROUND_IMAGE_NEIGHBOR_SUPPORT ||
                             coherent_normal_count < MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT);
                 ++dr) {
                const int neighbor_row = row + dr;
                if (neighbor_row < 0 || neighbor_row >= frame_height) {
                    continue;
                }
                for (int dc = -1;
                     dc <= 1 && (support_count < MIN_GROUND_IMAGE_NEIGHBOR_SUPPORT ||
                                 coherent_normal_count < MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT);
                     ++dc) {
                    if (dr == 0 && dc == 0) {
                        continue;
                    }
                    const int neighbor_col = (col + dc + frame_width) % frame_width;
                    if (filtered(neighbor_row, neighbor_col) == 0u) {
                        continue;
                    }
                    const Eigen::Index neighbor_idx =
                        static_cast<Eigen::Index>(neighbor_row) * frame_width + neighbor_col;
                    const Eigen::Vector3d neighbor_point =
                        destaggered_points.row(neighbor_idx).transpose();
                    if (has_compatible_ground_slope(point, neighbor_point, max_slope_sq)) {
                        ++support_count;
                    }
                    if (passed_normal_angle &&
                        coherent_normal_count < MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT) {
                        const Eigen::Vector3d neighbor_normal =
                            destaggered_normals.row(neighbor_idx).transpose();
                        const double neighbor_normal_norm_sq = neighbor_normal.squaredNorm();
                        if (neighbor_normal.allFinite() &&
                            neighbor_normal_norm_sq > NORMAL_EPS_SQ &&
                            neighbor_normal.z() * neighbor_normal.z() >=
                                min_abs_normal_z_sq * neighbor_normal_norm_sq) {
                            const double normal_dot = normal.dot(neighbor_normal);
                            if (normal_dot * normal_dot >= MIN_GROUND_NORMAL_ALIGNMENT_SQ *
                                                               normal_norm_sq *
                                                               neighbor_normal_norm_sq) {
                                ++coherent_normal_count;
                            }
                        }
                    }
                }
            }
            if (support_count < MIN_GROUND_IMAGE_NEIGHBOR_SUPPORT ||
                coherent_normal_count < MIN_GROUND_NORMAL_NEIGHBOR_SUPPORT) {
                changed_indices.push_back(idx);
            }
        }
    }
    for (const Eigen::Index idx : changed_indices) {
        filtered.data()[idx] = 0u;
    }

    core::destagger_into<uint8_t>(filtered, pixel_shift_by_row, true, ground_mask);
}

// Ground segmentation runs per-frame on any Frame that carries a RANGE field.
// Lidar frames expose one or more returns and a staggered pixel layout; a camera
// (or any other non-lidar) frame has a single return and an unstaggered image.
int frame_num_returns(const core::Frame& frame) {
    if (const auto* lidar = dynamic_cast<const core::LidarFrame*>(&frame)) {
        return lidar->sensor_info()->num_returns();
    }
    return 1;
}

std::vector<int> frame_pixel_shift_by_row(const core::Frame& frame) {
    if (const auto* lidar = dynamic_cast<const core::LidarFrame*>(&frame)) {
        return lidar->sensor_info()->format.pixel_shift_by_row;
    }
    // Non-lidar frames (e.g. cameras) are not staggered: a zero shift per row
    // makes every destagger/restagger step below a no-op.
    return std::vector<int>(frame.h, 0);
}

bool prepare_column_poses(const core::Frame& frame, int frame_width,
                          core::MatrixX16dR& relative_poses) {
    // A lidar frame carries one pose per column (rolling shutter); a single-pose
    // frame (e.g. a global-shutter CameraFrame) carries one rigid pose that
    // applies to every column. Broadcast that single pose across all columns.
    const int num_poses = static_cast<int>(frame.body_to_world().shape()[0]);

    bool poses_are_identity = true;
    for (int i = 0; i < num_poses; ++i) {
        if (!PoseH(frame.get_column_pose(i)).isIdentity(0.0)) {
            poses_are_identity = false;
            break;
        }
    }

    if (poses_are_identity) {
        return true;
    }

    relative_poses.resize(frame_width, 16);
    for (int col = 0; col < frame_width; ++col) {
        const int pose_idx = (num_poses == 1) ? 0 : col;
        const PoseH column_pose(frame.get_column_pose(pose_idx));
        Eigen::Map<core::Matrix4dR>(relative_poses.row(col).data()) = core::Matrix4dR(column_pose);
    }
    return false;
}

const PointCloudXYZd* dewarp_if_needed(const PointCloudXYZd& points, bool poses_are_identity,
                                       const core::MatrixX16dR& relative_poses,
                                       PointCloudXYZd& dewarped) {
    if (poses_are_identity) {
        return &points;
    }
    dewarped.resize(points.rows(), points.cols());
    core::dewarp<double>(dewarped, points, relative_poses);
    return &dewarped;
}

struct GroundClassificationContext {
    Eigen::Ref<const Eigen::Array<uint32_t, Eigen::Dynamic, 1>> status;
    int first_col;
    int last_col;
    std::reference_wrapper<const std::vector<int>> pixel_shift_by_row;
    std::reference_wrapper<const GroundModel> ground_model;
};

bool try_load_destaggered_normals(const core::Frame& frame, const std::string& field_name,
                                  int frame_height, int frame_width,
                                  const std::vector<int>& pixel_shift_by_row,
                                  MatrixX3dR& destaggered_normals) {
    if (!frame.has_field(field_name)) {
        return false;
    }

    const auto& field = frame.field(field_name);
    const auto& shape = field.shape();
    if (field.tag() != core::ChanFieldType::FLOAT32 || shape.size() != 3 ||
        shape[0] != static_cast<size_t>(frame_height) ||
        shape[1] != static_cast<size_t>(frame_width) || shape[2] != 3u ||
        pixel_shift_by_row.size() != static_cast<size_t>(frame_height)) {
        return false;
    }

    const auto values = static_cast<core::ConstArrayView3<float>>(field);
    if (values.sparse()) {
        return false;
    }

    const Eigen::Index frame_size =
        static_cast<Eigen::Index>(frame_height) * static_cast<Eigen::Index>(frame_width);

    core::MatrixX3fR shifted(frame_size, 3);
    const Eigen::TensorMap<const Eigen::Tensor<float, 3, Eigen::RowMajor>> input(
        values.data(), frame_height, frame_width, 3);
    Eigen::TensorMap<Eigen::Tensor<float, 3, Eigen::RowMajor>> output(shifted.data(), frame_height,
                                                                      frame_width, 3);
    core::destagger_into<float, 3>(input, pixel_shift_by_row, false, output);

    destaggered_normals = shifted.cast<double>();
    return true;
}

void rotate_destaggered_normals_to_global(MatrixX3dR& normals,
                                          const core::MatrixX16dR& relative_poses, int frame_height,
                                          int frame_width,
                                          const std::vector<int>& pixel_shift_by_row) {
    const Eigen::Index frame_size =
        static_cast<Eigen::Index>(frame_height) * static_cast<Eigen::Index>(frame_width);
    if (normals.rows() != frame_size || normals.cols() != 3 ||
        relative_poses.rows() != frame_width ||
        pixel_shift_by_row.size() != static_cast<size_t>(frame_height)) {
        throw std::invalid_argument("normal poses do not match frame");
    }

    for (int row = 0; row < frame_height; ++row) {
        const int offset =
            (frame_width + pixel_shift_by_row[static_cast<size_t>(row)] % frame_width) %
            frame_width;
        for (int col = 0; col < frame_width; ++col) {
            const int acquisition_col = (col - offset + frame_width) % frame_width;
            const Eigen::Index idx = static_cast<Eigen::Index>(row) * frame_width + col;
            const Eigen::Map<const core::Matrix4dR> pose(
                relative_poses.row(acquisition_col).data());
            const Eigen::Vector3d normal = normals.row(idx).transpose();
            normals.row(idx) = (pose.block<3, 3>(0, 0) * normal).transpose();
        }
    }
}

struct PreparedNormals {
    PointCloudXYZd destaggered_points;
    MatrixX3dR destaggered_normals;
    PointCloudXYZd destaggered_points2;
    MatrixX3dR destaggered_normals2;
    bool first_available = false;
    bool second_available = false;
};

PreparedNormals prepare_ground_normals(
    const core::Frame& frame, const PointCloudXYZd& points, const PointCloudXYZd& dewarped_points,
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& range,
    const PointCloudXYZd* points2, const PointCloudXYZd* dewarped_points2,
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>* range2,
    const core::MatrixX16dR& relative_poses, bool poses_are_identity) {
    PreparedNormals output;
    const int frame_height = static_cast<int>(range.rows());
    const int frame_width = static_cast<int>(range.cols());
    const bool has_second_return =
        points2 != nullptr && dewarped_points2 != nullptr && range2 != nullptr;
    const std::vector<int> pixel_shift_by_row = frame_pixel_shift_by_row(frame);

    try {
        output.destaggered_points =
            destagger_matrix_x3(dewarped_points, frame_height, frame_width, pixel_shift_by_row);
        if (has_second_return) {
            output.destaggered_points2 = destagger_matrix_x3(*dewarped_points2, frame_height,
                                                             frame_width, pixel_shift_by_row);
        }

        // Scan normal fields are already global-coordinate vectors. A partial
        // dual-return set is recomputed to keep both returns coherent.
        const bool loaded_first =
            try_load_destaggered_normals(frame, core::ChanField::NORMALS, frame_height, frame_width,
                                         pixel_shift_by_row, output.destaggered_normals);
        const bool loaded_second =
            !has_second_return || try_load_destaggered_normals(
                                      frame, core::ChanField::NORMALS2, frame_height, frame_width,
                                      pixel_shift_by_row, output.destaggered_normals2);
        if (loaded_first && loaded_second) {
            output.first_available = true;
            output.second_available = has_second_return;
            return output;
        }

        // The shared normals implementation stays coordinate-frame agnostic:
        // compute fallback normals in the body frame, then rotate each vector
        // by its acquisition-column pose into the global frame used below.
        MatrixX3dR body_sensor_origins(frame_width, 3);
        body_sensor_origins.rowwise() =
            frame.sensor_info()->sensor_to_body.block<3, 1>(0, 3).transpose();
        PointCloudXYZd destaggered_body_points;
        PointCloudXYZd destaggered_body_points2;
        const PointCloudXYZd* normal_points = &output.destaggered_points;
        const PointCloudXYZd* normal_points2 = &output.destaggered_points2;
        if (!poses_are_identity) {
            destaggered_body_points =
                destagger_matrix_x3(points, frame_height, frame_width, pixel_shift_by_row);
            normal_points = &destaggered_body_points;
            if (has_second_return) {
                destaggered_body_points2 =
                    destagger_matrix_x3(*points2, frame_height, frame_width, pixel_shift_by_row);
                normal_points2 = &destaggered_body_points2;
            }
        }
        const auto normal_range = core::destagger<uint32_t>(range, pixel_shift_by_row);
        if (has_second_return) {
            const auto normal_range2 = core::destagger<uint32_t>(*range2, pixel_shift_by_row);
            auto normal_pair = ouster::sdk::algorithm::normals(
                *normal_points, normal_range, *normal_points2, normal_range2, body_sensor_origins);
            output.destaggered_normals = std::move(normal_pair.first);
            output.destaggered_normals2 = std::move(normal_pair.second);
        } else {
            output.destaggered_normals =
                ouster::sdk::algorithm::normals(*normal_points, normal_range, body_sensor_origins);
        }
        if (!poses_are_identity) {
            rotate_destaggered_normals_to_global(output.destaggered_normals, relative_poses,
                                                 frame_height, frame_width, pixel_shift_by_row);
            if (has_second_return) {
                rotate_destaggered_normals_to_global(output.destaggered_normals2, relative_poses,
                                                     frame_height, frame_width, pixel_shift_by_row);
            }
        }
        output.first_available = true;
        output.second_available = has_second_return;
    } catch (const std::exception& error) {
        logger().warn(
            "Normal preparation failed for ground segmentation; using "
            "z-only ground model: {}",
            error.what());
        return {};
    }
    return output;
}

void classify_return_points(const GroundClassificationContext& context,
                            const PointCloudXYZd& points,
                            const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>& range,
                            const PointCloudXYZd* destaggered_points,
                            const MatrixX3dR* destaggered_normals,
                            Eigen::Ref<core::img_t<uint8_t>> ground_mask) {
    const std::vector<int>& pixel_shift_by_row = context.pixel_shift_by_row.get();
    const GroundModel& ground_model = context.ground_model.get();
    const int frame_height = static_cast<int>(ground_mask.rows());
    const int frame_width = static_cast<int>(ground_mask.cols());
    std::vector<size_t> point_cell_indices(static_cast<size_t>(ground_mask.size()),
                                           INVALID_GROUND_CELL);
    const size_t cell_count =
        static_cast<size_t>(ground_model.rows) * static_cast<size_t>(ground_model.cols);
    std::vector<size_t> valid_point_count(cell_count, 0u);
    for (int row = 0; row < frame_height; ++row) {
        for (int col = context.first_col; col <= context.last_col; ++col) {
            if (context.status[col] == 0u || range(row, col) == 0u) {
                continue;
            }
            const Eigen::Index flat_idx = static_cast<Eigen::Index>(row) * frame_width + col;
            const Eigen::Vector3d point = points.row(flat_idx).matrix();
            if (!point.allFinite()) {
                continue;
            }
            const size_t cell_idx = ground_model_cell_index_for_point(ground_model, point);
            point_cell_indices[static_cast<size_t>(flat_idx)] = cell_idx;
            if (cell_idx != INVALID_GROUND_CELL) {
                ++valid_point_count[cell_idx];
            }
            const double point_range = static_cast<double>(range(row, col)) * core::RANGE_UNIT;
            if (xy_point_is_ground_like(point, point_range, ground_model, cell_idx)) {
                ground_mask(row, col) = 1u;
            }
        }
    }
    if (destaggered_points != nullptr && destaggered_normals != nullptr) {
        filter_destaggered_ground_mask_by_normals_and_slope(
            *destaggered_points, *destaggered_normals, pixel_shift_by_row, MAX_GROUND_SLOPE_DEG,
            ground_model, ground_mask);
    }
    filter_ground_mask_by_spatial_support(points, range, context.first_col, context.last_col,
                                          ground_model, point_cell_indices, valid_point_count,
                                          ground_mask);
}

// Pipeline:
//   1. Coordinate frame selection – use non-identity frame poses directly
//      when SLAM has populated them; otherwise use sensor extrinsics and
//      frame-local column poses.
//   2. Lower-envelope 2.5-D grid – builds a height map from valid first- and
//      second-return samples using the 5th-percentile (floor) Z in each cell.
//   3. Hole filling / smoothing – fills small gaps using median filtering
//      while preserving edges at height discontinuities.
//   4. Local height pruning – removes compact elevated surfaces that have
//      sufficient nearby lower support, before and after interpolation.
//   5. Height classification – each frame point is tested against the height
//      map (or a global fallback Z) to produce ground candidates.
//   6. Normal filtering / supported recovery – reject candidates whose surface
//      normal is more than 40 degrees from gravity. Recover a rejected point
//      only when it stays near the grid height and has enough angle-approved,
//      slope-compatible neighbors, then remove image-space or normal outliers.
//   7. Spatial support filtering – reject elevated disconnected surfaces,
//      cells with too few ground votes, and points too far above the cell floor.
//
// Fill one ground mask per return (index 0 = 1st return, etc.).
// Each output mask must have shape frame.h x frame.w.
using GroundMaskView = Eigen::Ref<ouster::sdk::core::img_t<uint8_t>>;

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
size_t get_ground_mask_into(const ouster::sdk::core::Frame& frame, const GroundSegConfig& config,
                            const ouster::sdk::core::XYZLut& xyz_lut,
                            std::vector<GroundMaskView>& ground_masks) {
    if (!frame.sensor_info()) {
        throw std::invalid_argument("frame.sensor_info() is required for get_ground_mask");
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::RANGE)) {
        throw std::invalid_argument("frame must contain RANGE field for get_ground_mask");
    }

    const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    const int frame_height = static_cast<int>(range.rows());
    const int frame_width = static_cast<int>(range.cols());

    const int max_returns = frame_num_returns(frame);
    std::vector<Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>> ranges;
    ranges.reserve(static_cast<size_t>(max_returns));
    ranges.emplace_back(range);
    for (int ret = 1; ret < max_returns; ++ret) {
        const std::string field_name = ouster::sdk::core::ChanField::return_field_name(
            ouster::sdk::core::ChanField::RANGE, ret);
        if (!frame.has_field(field_name)) {
            break;
        }
        ranges.emplace_back(frame.field<uint32_t>(field_name));
    }
    const size_t num_returns = ranges.size();
    const Eigen::Ref<const ouster::sdk::core::img_t<uint32_t>>* range2 =
        num_returns > 1u ? &ranges[1] : nullptr;

    if (ground_masks.size() < num_returns) {
        throw std::invalid_argument("not enough output masks provided for get_ground_mask_into");
    }
    for (size_t ret = 0; ret < num_returns; ++ret) {
        if (ground_masks[ret].rows() != frame_height || ground_masks[ret].cols() != frame_width) {
            throw std::invalid_argument("output mask shape does not match frame shape");
        }
        ground_masks[ret].setZero();
    }

    // Build a per-image-column status array. A lidar frame carries one status
    // word per column; a camera (or any single-status) frame carries a single
    // frame-level status, which we broadcast across every image column so
    // classification covers the whole frame rather than just column 0.
    const auto frame_status = frame.status();
    Eigen::Array<uint32_t, Eigen::Dynamic, 1> column_status;
    if (frame_status.size() == static_cast<Eigen::Index>(frame_width)) {
        column_status = frame_status;
    } else {
        const uint32_t single_status =
            frame_status.size() > 0 ? static_cast<uint32_t>(frame_status(0)) : 1u;
        column_status =
            Eigen::Array<uint32_t, Eigen::Dynamic, 1>::Constant(frame_width, single_status);
    }

    int first_col = 0;
    int last_col = frame_width - 1;
    while (first_col <= last_col && (column_status[first_col] & 1u) == 0u) {
        ++first_col;
    }
    while (last_col >= first_col && (column_status[last_col] & 1u) == 0u) {
        --last_col;
    }
    if (last_col < first_col) {
        return num_returns;
    }

    const PointCloudXYZd points = xyz_lut(range);
    PointCloudXYZd points2;
    if (range2 != nullptr) {
        points2 = xyz_lut(*range2);
    }

    ouster::sdk::core::MatrixX16dR rel_poses;
    const bool poses_are_identity = prepare_column_poses(frame, frame_width, rel_poses);

    PointCloudXYZd dewarped;
    const PointCloudXYZd* dewarped_points =
        dewarp_if_needed(points, poses_are_identity, rel_poses, dewarped);
    PointCloudXYZd dewarped2;
    const PointCloudXYZd* dewarped_points2 = nullptr;
    if (range2 != nullptr) {
        dewarped_points2 = dewarp_if_needed(points2, poses_are_identity, rel_poses, dewarped2);
    }

    const std::vector<int> pixel_shift_by_row = frame_pixel_shift_by_row(frame);
    // Normal preparation and 2.5-D grid construction only read the point data and
    // do not depend on each other, so overlap their two CPU-heavy passes.
    auto normals_task = std::async(std::launch::async, [&]() {
        return prepare_ground_normals(frame, points, *dewarped_points, range,
                                      range2 != nullptr ? &points2 : nullptr, dewarped_points2,
                                      range2, rel_poses, poses_are_identity);
    });
    GroundModel ground_model =
        build_lower_envelope_ground_model(*dewarped_points, range, column_status, first_col,
                                          last_col, config.grid_size, dewarped_points2, range2);
    PreparedNormals prepared_normals = normals_task.get();
    const MatrixX3dR* first_normals =
        prepared_normals.first_available ? &prepared_normals.destaggered_normals : nullptr;
    const MatrixX3dR* second_normals =
        prepared_normals.second_available ? &prepared_normals.destaggered_normals2 : nullptr;

    const GroundClassificationContext classification_context{column_status, first_col, last_col,
                                                             pixel_shift_by_row, ground_model};

    // Classify 1st return (always has normals if available).
    classify_return_points(
        classification_context, *dewarped_points, range,
        prepared_normals.first_available ? &prepared_normals.destaggered_points : nullptr,
        first_normals, ground_masks[0]);

    // Classify 2nd return (has normals if available).
    if (range2 != nullptr) {
        classify_return_points(
            classification_context, *dewarped_points2, *range2,
            prepared_normals.second_available ? &prepared_normals.destaggered_points2 : nullptr,
            second_normals, ground_masks[1]);
    }

    // Normals currently support two returns. Classify future returns with the
    // shared height model and spatial filters.
    for (size_t ret = 2; ret < num_returns; ++ret) {
        const auto& ret_range = ranges[ret];
        const PointCloudXYZd ret_points = xyz_lut(ret_range);
        PointCloudXYZd ret_dewarped;
        const PointCloudXYZd* classification_points =
            dewarp_if_needed(ret_points, poses_are_identity, rel_poses, ret_dewarped);
        classify_return_points(classification_context, *classification_points, ret_range, nullptr,
                               nullptr, ground_masks[ret]);
    }

    return num_returns;
}

class EnvelopeGridGroundSegEngine : public GroundSegEngine {
   public:
    explicit EnvelopeGridGroundSegEngine(const GroundSegConfig& config) : config_(config) {}

    void update(ouster::sdk::core::FrameSet& frames) override {
        std::vector<size_t> valid_indices;
        valid_indices.reserve(frames.size());
        for (const size_t idx : frames.valid_indices()) {
            valid_indices.push_back(idx);
        }
        if (valid_indices.size() <= 1u) {
            for (const size_t idx : valid_indices) {
                process_frame(frames, idx);
            }
            return;
        }

        // Sensors in a FrameSet are independent. Run them concurrently so a
        // multi-sensor source pays approximately the cost of its slowest
        // sensor rather than the sum of all sensor runtimes.
        std::vector<std::future<void>> tasks;
        tasks.reserve(valid_indices.size());
        for (const size_t idx : valid_indices) {
            try {
                tasks.emplace_back(std::async(
                    std::launch::async, [this, &frames, idx]() { process_frame(frames, idx); }));
            } catch (const std::system_error& error) {
                throw std::runtime_error("Failed to launch ground-segmentation task: " +
                                         std::string(error.what()));
            }
        }
        for (auto& task : tasks) {
            task.get();
        }
    }

   private:
    void process_frame(ouster::sdk::core::FrameSet& frames, size_t idx) const {
        const auto& frame_ptr = frames[idx];
        // Ground segmentation needs a calibrated frame with a RANGE field; this
        // covers lidar and camera (or any other RANGE-carrying) frames.
        if (!frame_ptr || !frame_ptr->sensor_info() ||
            !frame_ptr->has_field(ouster::sdk::core::ChanField::RANGE)) {
            return;
        }
        auto& frame = *frame_ptr;

        // Cached on SensorInfo; bakes in sensor_to_body.
        const auto xyz_lut = frame.sensor_info()->xyzlut<double>();

        const std::vector<size_t> shape{frame.h, frame.w};
        const int max_returns = frame_num_returns(frame);
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

        const size_t filled_returns = get_ground_mask_into(frame, config_, *xyz_lut, output_masks);

        // Remove stale GROUND fields for returns beyond what we computed.
        // The creation loop above unconditionally added GROUND fields for
        // every ret in [0, max_returns), so each ret here already exists.
        for (int ret = static_cast<int>(filled_returns); ret < max_returns; ++ret) {
            const std::string fname = ouster::sdk::core::ChanField::return_field_name(
                ouster::sdk::core::ChanField::GROUND, ret);
            frame.del_field(fname);
        }
    }

    GroundSegConfig config_;
};

}  // namespace

// Returns one ground mask per return (index 0 = 1st return, etc.).
namespace impl {

std::vector<std::vector<uint8_t>> get_ground_mask(const ouster::sdk::core::Frame& frame,
                                                  double grid_size,
                                                  const ouster::sdk::core::XYZLut& xyz_lut) {
    if (grid_size <= 0.0) {
        throw std::invalid_argument("grid_size must be > 0 for get_ground_mask");
    }
    if (!frame.sensor_info()) {
        throw std::invalid_argument("frame.sensor_info() is required for get_ground_mask");
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::RANGE)) {
        throw std::invalid_argument("frame must contain RANGE field for get_ground_mask");
    }

    const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    const size_t n_pixels = static_cast<size_t>(range.rows()) * static_cast<size_t>(range.cols());
    std::vector<std::vector<uint8_t>> ground_masks(frame_num_returns(frame),
                                                   std::vector<uint8_t>(n_pixels, 0u));
    std::vector<GroundMaskView> output_masks;
    output_masks.reserve(ground_masks.size());
    for (auto& mask : ground_masks) {
        Eigen::Map<ouster::sdk::core::img_t<uint8_t>> mask_view{mask.data(), range.rows(),
                                                                range.cols()};
        output_masks.emplace_back(mask_view);
    }
    GroundSegConfig config;
    config.grid_size = grid_size;
    const size_t filled_returns = get_ground_mask_into(frame, config, xyz_lut, output_masks);
    ground_masks.resize(filled_returns);
    return ground_masks;
}

}  // namespace impl

std::unique_ptr<GroundSegEngine> GroundSegEngine::create(const GroundSegConfig& config) {
    if (!std::isfinite(config.grid_size) || config.grid_size <= 0.0) {
        throw std::invalid_argument("GroundSegConfig.grid_size must be > 0");
    }
    return std::make_unique<EnvelopeGridGroundSegEngine>(config);
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
