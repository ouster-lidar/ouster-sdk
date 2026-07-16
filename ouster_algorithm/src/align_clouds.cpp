#include "ouster/algorithm/align_clouds.h"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <future>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unsupported/Eigen/FFT>
#include <utility>
#include <vector>

#include "ouster/algorithm/ground_seg.h"
#include "ouster/algorithm/impl/ground_seg_detail.h"
#include "ouster/algorithm/impl/spatial_hash.h"
#include "ouster/algorithm/normals.h"
#include "ouster/algorithm/voxel_downsample.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/impl/transform_typedefs.h"
#include "ouster/core/impl/transformation.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/voxel_hash_map.h"

using ouster::sdk::core::ArrayX3dR;
using ouster::sdk::core::ArrayX3fR;
using ouster::sdk::core::logger;
using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::MatrixX3dR;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseV;
using ouster::sdk::core::impl::RotV;
namespace ouster {
namespace sdk {
namespace algorithm {

namespace {

/// Return true if all three components of @p vec are finite.
inline bool finite_vec3(const Eigen::Vector3d& vec) {
    return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z());
}

size_t next_power_of_two(size_t num) {
    size_t pow = 1u;
    while (pow < num) {
        pow <<= 1u;
    }
    return pow;
}

inline double clamp01(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}

size_t parallel_worker_count(size_t task_count) {
    const unsigned hardware_threads = std::thread::hardware_concurrency();
    const size_t max_workers = std::max<size_t>(1, static_cast<size_t>(hardware_threads));
    return std::min(task_count, max_workers);
}

/// Run `func(idx)` for each idx in [0, task_count). When @p allow_parallel
/// is true and hardware allows it, tasks execute concurrently via std::async.
template <typename Func>
void run_parallel_indexed(size_t task_count, bool allow_parallel, Func&& func) {
    if (task_count == 0) {
        return;
    }

    const size_t worker_count = allow_parallel ? parallel_worker_count(task_count) : 1;
    if (worker_count <= 1) {
        for (size_t idx = 0; idx < task_count; ++idx) {
            func(idx);
        }
        return;
    }

    std::atomic<size_t> next_idx{0};
    std::vector<std::future<void>> futures;
    futures.reserve(worker_count);
    for (size_t worker = 0; worker < worker_count; ++worker) {
        futures.push_back(std::async(std::launch::async, [&]() {
            while (true) {
                const size_t idx = next_idx.fetch_add(1);
                if (idx >= task_count) {
                    break;
                }
                func(idx);
            }
        }));
    }
    for (auto& future : futures) {
        future.get();
    }
}

// Per-thread flag toggled by the scoped guard below. Non-const by design; the
// clang-tidy global-variable check is suppressed for this intentional
// thread-local toggle.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
thread_local bool pairwise_inner_parallel_disabled = false;

class ScopedPairwiseInnerParallelDisable {
   public:
    ScopedPairwiseInnerParallelDisable() : previous_(pairwise_inner_parallel_disabled) {
        pairwise_inner_parallel_disabled = true;
    }

    ~ScopedPairwiseInnerParallelDisable() {
        pairwise_inner_parallel_disabled = previous_;
    }

    ScopedPairwiseInnerParallelDisable(const ScopedPairwiseInnerParallelDisable&) = delete;
    ScopedPairwiseInnerParallelDisable& operator=(const ScopedPairwiseInnerParallelDisable&) =
        delete;
    ScopedPairwiseInnerParallelDisable(ScopedPairwiseInnerParallelDisable&&) = delete;
    ScopedPairwiseInnerParallelDisable& operator=(ScopedPairwiseInnerParallelDisable&&) = delete;

   private:
    bool previous_;
};

double median_abs(std::vector<double> vals) {
    if (vals.empty()) {
        return 0.0;
    }
    for (double& val : vals) {
        val = std::abs(val);
    }
    const size_t mid = vals.size() / 2;
    std::nth_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid), vals.end());
    if ((vals.size() & 1u) == 1u) {
        return vals[mid];
    }
    const double hi = vals[mid];
    std::nth_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(mid - 1), vals.end());
    return 0.5 * (vals[mid - 1] + hi);
}

// ---------------------------------------------------------------------------
// Spatial-hash accelerated nearest-neighbour lookup.
// ---------------------------------------------------------------------------

using CellKey3D = algorithm::impl::SpatialHashCell3D;
using CellKey3DHash = algorithm::impl::SpatialHashCell3DHash;

constexpr double NORMAL_EPS = 1e-12;  // Epsilon for valid surface-normal magnitude.

/// Spatial hash grid for O(1)-average nearest-neighbour queries in 3D.
/// Build once from target points, then query per source point.
class SpatialHashGrid3D {
   public:
    explicit SpatialHashGrid3D(Eigen::Ref<const ArrayX3dR> points, double cell_size)
        : inv_cell_size_(1.0 / cell_size) {
        grid_.reserve(static_cast<size_t>(points.rows()));
        for (Eigen::Index j = 0; j < points.rows(); ++j) {
            const Eigen::Vector3d point = points.row(j).matrix();
            if (!finite_vec3(point)) {
                continue;
            }
            grid_[cell_of(point)].push_back(static_cast<int>(j));
        }
    }

    SpatialHashGrid3D(Eigen::Ref<const ArrayX3dR> points, Eigen::Ref<const ArrayX3dR> normals,
                      double cell_size)
        : inv_cell_size_(1.0 / cell_size) {
        grid_.reserve(static_cast<size_t>(points.rows()));
        for (Eigen::Index j = 0; j < points.rows(); ++j) {
            const Eigen::Vector3d point = points.row(j).matrix();
            const Eigen::Vector3d normal = normals.row(j).matrix();
            if (!finite_vec3(point) || !finite_vec3(normal) || normal.norm() <= NORMAL_EPS) {
                continue;
            }
            grid_[cell_of(point)].push_back(static_cast<int>(j));
        }
    }

    /// Returns the index of the nearest point within max_dist_sq, or -1.
    int nearest(Eigen::Ref<const ArrayX3dR> points, const Eigen::Vector3d& query,
                double max_dist_sq) const {
        if (!finite_vec3(query) || !std::isfinite(max_dist_sq) || max_dist_sq <= 0.0) {
            return -1;
        }
        const CellKey3D query_cell = cell_of(query);
        int best_idx = -1;
        double best_d2 = max_dist_sq;
        algorithm::impl::for_each_neighbor_cell(query_cell, [&](const CellKey3D& neighbor) {
            auto it = grid_.find(neighbor);
            if (it == grid_.end()) {
                return;
            }
            for (int j : it->second) {
                const Eigen::Vector3d point = points.row(j).matrix();
                const double dist_sq = (point - query).squaredNorm();
                if (dist_sq < best_d2) {
                    best_d2 = dist_sq;
                    best_idx = j;
                }
            }
        });
        return best_idx;
    }

   private:
    CellKey3D cell_of(const Eigen::Vector3d& point) const {
        return algorithm::impl::compute_cell_xyz_from_inverse_cell_size(point, inv_cell_size_);
    }

    double inv_cell_size_;
    std::unordered_map<CellKey3D, std::vector<int>, CellKey3DHash> grid_;
};

using CellKey2D = algorithm::impl::SpatialHashCell2D;
using CellKey2DHash = algorithm::impl::SpatialHashCell2DHash;

/// Spatial hash grid for O(1)-average nearest-neighbour queries in XY.
class SpatialHashGridXY {
   public:
    explicit SpatialHashGridXY(const ArrayX3dR& points, double cell_size)
        : inv_cell_size_(1.0 / cell_size) {
        const Eigen::Index num_pts = points.rows();
        grid_.reserve(static_cast<size_t>(num_pts));
        for (Eigen::Index j = 0; j < num_pts; ++j) {
            const Eigen::Vector3d point = points.row(j).matrix();
            if (!finite_vec3(point)) {
                continue;
            }
            grid_[cell_of(point)].push_back(static_cast<int>(j));
        }
    }

    /// Find the nearest point index by XY distance within @p max_xy_dist_sq.
    int nearest_xy(const ArrayX3dR& points, const Eigen::Vector3d& query,
                   double max_xy_dist_sq) const {
        if (!std::isfinite(query.x()) || !std::isfinite(query.y()) ||
            !std::isfinite(max_xy_dist_sq) || max_xy_dist_sq <= 0.0) {
            return -1;
        }
        const CellKey2D query_cell = cell_of(query);
        int best_idx = -1;
        double best_d2 = max_xy_dist_sq;
        algorithm::impl::for_each_neighbor_cell(query_cell, [&](const CellKey2D& neighbor) {
            auto it = grid_.find(neighbor);
            if (it == grid_.end()) {
                return;
            }
            for (int j : it->second) {
                const double ddx = points(j, 0) - query.x();
                const double ddy = points(j, 1) - query.y();
                const double dist_sq = ddx * ddx + ddy * ddy;
                if (dist_sq < best_d2) {
                    best_d2 = dist_sq;
                    best_idx = j;
                }
            }
        });
        return best_idx;
    }

   private:
    CellKey2D cell_of(const Eigen::Vector3d& point) const {
        return algorithm::impl::compute_cell_xy_from_inverse_cell_size(point, inv_cell_size_);
    }

    double inv_cell_size_;
    std::unordered_map<CellKey2D, std::vector<int>, CellKey2DHash> grid_;
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

constexpr double TRANS_PITCH = 0.2;        // Metres per 1-D translation histogram bin.
constexpr int NUM_TRANS = 1024;            // Number of bins in the 1-D translation histogram.
constexpr double VOXEL_SIZE_COARSE = 0.4;  // Voxel size (m) for downsampling frame features.
constexpr double MAX_Z_COARSE_CORRECTION_M =
    4.0;  // Maximum coarse Z shift (m) from 1-D histogram alignment.
constexpr double XY_OVERLAP_RADIUS_M =
    0.5;  // XY radius (m) for overlap-based confidence estimation.
constexpr double CONFIDENCE_MAX_NORMAL_ANGLE_DEG =
    5.0;  // Normal-angle gate used for confidence counting.
constexpr Eigen::Index CONFIDENCE_MAX_QUERY_SAMPLES =
    16000;  // Per-direction cap for sampled overlap-confidence queries.
constexpr size_t MIN_ICP_POINTS =
    20u;  // Minimum point count required for ICP / correspondence steps.
constexpr double MAD_TO_SIGMA =
    1.4826;                      // MAD-to-sigma scale factor (consistency factor for Gaussian).
constexpr double HUBER_K = 1.5;  // Huber loss transition threshold in units of sigma.
constexpr double ALIGN_CLOUDS_GROUND_SEG_GRID_SIZE_M =
    0.5;  // Ground segmentation grid size used only for XY FFT filtering.
constexpr double MIN_FINITE_INITIAL_MATCH_SCORE =
    1e-6;  // Positive fallback score for finite reciprocal initial matches.
constexpr double MULTI_EDGE_ROTATION_ERROR_SCALE_DEG =
    10.0;  // Rotation error scale for multi-sensor edge consistency ranking.

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

using TransHistogram = std::array<double, NUM_TRANS>;
using Complex = std::complex<double>;

/// Voxel-downsampled points + normals prepared for frame matching.
struct FrameFeatures {
    ArrayX3dR points;
    ArrayX3dR normals;
    Eigen::ArrayXd point_dist;
    ArrayX3dR xy_points;
    ArrayX3dR xy_normals;
    Eigen::ArrayXd xy_point_dist;
};

struct PointCloudInputNames {
    const char* points_name = "";
    const char* normals_name = "";
};

constexpr PointCloudInputNames SOURCE_POINT_CLOUD_INPUTS{"source_points", "source_normals"};
constexpr PointCloudInputNames TARGET_POINT_CLOUD_INPUTS{"target_points", "target_normals"};

/// Result of a 1-D translation histogram cross-correlation.
struct Align1DResult {
    double translation = 0.0;
    double score = std::numeric_limits<double>::lowest();
};

// XY 2-D FFT correlation constants.
constexpr double XY_GRID_PIXEL_SIZE_M = 0.25;  // metres per pixel (fine grid)
constexpr double XY_GRID_BOUND_M = 60.0;       // outdoor cap for [-B, +B] X/Y
constexpr double XY_GRID_MIN_ENERGY = 1e-10;   // avoid empty-grid degeneracy
constexpr double XY_INDOOR_FINE_PIXEL_SIZE_M =
    0.20;  // Finer BEV resolution for compact indoor scenes.
constexpr double XY_MID_FINE_PIXEL_SIZE_M =
    0.15;  // Intermediate BEV resolution for medium-size scenes.
constexpr double XY_ADAPTIVE_BOUND_MIN_M =
    10.0;  // Compact indoor frames use at least this half-width.
constexpr double XY_ADAPTIVE_BOUND_MARGIN_M =
    2.0;  // Extra half-width around the observed XY footprint.
constexpr double XY_COMPACT_SCENE_BOUND_M = 18.0;  // Compact scenes get the finest BEV parameters.
constexpr double XY_MID_SCENE_BOUND_M = 30.0;      // Medium scenes get intermediate BEV parameters.
constexpr double XY_BOUNDS_PERCENTILE =
    0.95;  // Robust XY footprint percentile for adaptive grid sizing.

/// Parameters for a birds-eye-view (BEV) XY grid used in 2-D FFT correlation.
struct XYGridSpec {
    double pixel_size_m = XY_GRID_PIXEL_SIZE_M;
    double bound_m = XY_GRID_BOUND_M;
    int base_n = 0;  // unpadded grid side length
    int fft_n = 0;   // zero-padded FFT side length (power of two)
};

/// Result of a 2-D XY FFT cross-correlation alignment.
struct Align2DResult {
    double dx_m = 0.0;
    double dy_m = 0.0;
    double score = std::numeric_limits<double>::lowest();
};

/// Precomputed FFT of the source frame's BEV grid (avoids recomputation
/// across multiple yaw candidates).
struct PrecomputedXY {
    XYGridSpec spec;
    Eigen::MatrixXcd source_fft;  // FFT of zero-padded source grid
};

struct XYMatcherParams {
    double pixel_size_m = XY_GRID_PIXEL_SIZE_M;
    double bound_m = XY_GRID_BOUND_M;
    double max_shift_m = XY_GRID_BOUND_M;
};

Eigen::ArrayXd compute_point_dist(const ArrayX3dR& points) {
    return points.matrix().rowwise().norm().array();
}

/// Reduce a full SE(3) pose to a pure yaw rotation + XYZ translation,
/// discarding pitch and roll. Used to initialise the XY/yaw sweep.
PoseH project_pose_to_yaw_translation(const PoseH& pose) {
    const Eigen::Matrix3d rotation = pose.r();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    Matrix4dR transform = Matrix4dR::Identity();
    transform(0, 0) = cos_yaw;
    transform(0, 1) = -sin_yaw;
    transform(1, 0) = sin_yaw;
    transform(1, 1) = cos_yaw;
    transform.block<3, 1>(0, 3) = pose.t();
    return PoseH(transform);
}

FrameFeatures make_frame_features(ArrayX3dR&& points, ArrayX3dR&& normals) {
    FrameFeatures out;
    out.points = std::move(points);
    out.normals = std::move(normals);
    out.point_dist = compute_point_dist(out.points);
    return out;
}

void set_xy_matching_features(FrameFeatures& features, ArrayX3dR&& points, ArrayX3dR&& normals) {
    if (points.rows() == 0) {
        return;
    }

    features.xy_points = std::move(points);
    if (normals.rows() == features.xy_points.rows()) {
        features.xy_normals = std::move(normals);
    } else {
        features.xy_normals = ArrayX3dR(0, 3);
    }
    features.xy_point_dist = compute_point_dist(features.xy_points);
}

bool frame_features_have_normals(Eigen::Ref<const ArrayX3dR> points,
                                 Eigen::Ref<const ArrayX3dR> normals) {
    return points.rows() > 0 && normals.rows() == points.rows();
}

const ArrayX3dR& xy_matching_points(const FrameFeatures& features) {
    return features.xy_points.rows() > 0 ? features.xy_points : features.points;
}

const ArrayX3dR& xy_matching_normals(const FrameFeatures& features) {
    return features.xy_points.rows() > 0 ? features.xy_normals : features.normals;
}

const Eigen::ArrayXd& xy_matching_point_dist(const FrameFeatures& features) {
    return features.xy_points.rows() > 0 ? features.xy_point_dist : features.point_dist;
}

/// Estimate the XY half-width of the point cloud using the
/// XY_BOUNDS_PERCENTILE robust percentile of max(|x|, |y|).
double estimate_xy_footprint_bound(Eigen::Ref<const ArrayX3dR> points) {
    std::vector<double> extents;
    extents.reserve(static_cast<size_t>(points.rows()));
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3d point = points.row(i).matrix();
        if (!finite_vec3(point)) {
            continue;
        }
        extents.push_back(std::max(std::abs(point.x()), std::abs(point.y())));
    }
    if (extents.empty()) {
        return XY_ADAPTIVE_BOUND_MIN_M;
    }

    const double percentile = std::max(0.0, std::min(XY_BOUNDS_PERCENTILE, 1.0));
    const size_t nth_idx =
        static_cast<size_t>(std::floor(percentile * static_cast<double>(extents.size() - 1)));
    std::nth_element(extents.begin(), extents.begin() + static_cast<std::ptrdiff_t>(nth_idx),
                     extents.end());
    return extents[nth_idx];
}

/// Select adaptive BEV grid parameters based on the observed XY footprint of
/// both frames and the magnitude of the initial guess translation.
XYMatcherParams choose_xy_matcher_params(const FrameFeatures& reference,
                                         const FrameFeatures& moving, const PoseH& initial_guess) {
    XYMatcherParams params;
    // Size the FFT grid from a robust scene footprint plus any initial XY
    // offset. This keeps compact scenes high-resolution without making outdoor
    // grids so large that every yaw candidate becomes expensive.
    const double footprint_bound =
        std::max(estimate_xy_footprint_bound(xy_matching_points(reference)),
                 estimate_xy_footprint_bound(xy_matching_points(moving)));
    const Eigen::Vector3d initial_translation = initial_guess.t();
    const double initial_xy_bound =
        std::max(std::abs(initial_translation.x()), std::abs(initial_translation.y()));
    params.bound_m =
        std::max(XY_ADAPTIVE_BOUND_MIN_M,
                 std::min(XY_GRID_BOUND_M, std::max(footprint_bound, initial_xy_bound) +
                                               XY_ADAPTIVE_BOUND_MARGIN_M));
    params.max_shift_m = std::max(4.0, params.bound_m);
    // This pixel size is the fine grid used by the local yaw refinement. The
    // full 360 deg yaw pass below always uses a fixed 0.5 m coarse grid.
    if (params.bound_m <= XY_COMPACT_SCENE_BOUND_M) {
        params.pixel_size_m = XY_INDOOR_FINE_PIXEL_SIZE_M;
    } else if (params.bound_m <= XY_MID_SCENE_BOUND_M) {
        params.pixel_size_m = XY_MID_FINE_PIXEL_SIZE_M;
    }

    return params;
}

struct FrameNormalsFieldView {
    const float* data = nullptr;
    Eigen::Index rows = 0;

    bool valid() const {
        return data != nullptr && rows > 0;
    }

    Eigen::Map<const ArrayX3fR> map() const {
        return Eigen::Map<const ArrayX3fR>(data, rows, 3);
    }
};

// ---------------------------------------------------------------------------
// Frame normals
// ---------------------------------------------------------------------------

/// Attempt to obtain a zero-copy view of the frame's precomputed NORMALS field.
/// Returns true and populates @p out_normals on success; false if the field is
/// absent, has the wrong type/shape, or is stored sparsely.
bool try_map_frame_normals_field(const ouster::sdk::core::LidarFrame& frame,
                                 FrameNormalsFieldView& out_normals) {
    if (!frame.has_field(ouster::sdk::core::ChanField::NORMALS)) {
        return false;
    }
    const auto& field = frame.field(ouster::sdk::core::ChanField::NORMALS);
    const auto& shape = field.shape();
    const int h = static_cast<int>(frame.h);
    const int w = static_cast<int>(frame.w);

    if (field.tag() != ouster::sdk::core::ChanFieldType::FLOAT32) {
        return false;
    }
    if (shape.size() != 3 || static_cast<int>(shape[0]) != h || static_cast<int>(shape[1]) != w ||
        shape[2] != 3) {
        return false;
    }

    const auto normals_view = static_cast<ouster::sdk::core::ConstArrayView3<float>>(field);
    if (normals_view.sparse()) {
        return false;
    }
    out_normals.data = normals_view.data();
    out_normals.rows = static_cast<Eigen::Index>(shape[0]) * static_cast<Eigen::Index>(shape[1]);
    return out_normals.valid();
}

// ---------------------------------------------------------------------------
// Frame preparation
// ---------------------------------------------------------------------------

/// Prepare features for frame matching from a raw LidarFrame.
///
/// Steps:
///  1. Compute or load surface normals.
///  2. Dewarp columns into a single local frame using per-column poses.
///  3. Apply the frame's sensor_to_body extrinsic while dewarping.
///  4. Filter to valid, finite-range returns with well-defined normals.
///  5. Voxel-downsample to normalise point density.
/// The returned feature clouds are the "prepared" point sets used by FFT
/// matching, ICP, and confidence scoring.
FrameFeatures prepare_frame_features(const ouster::sdk::core::LidarFrame& frame,
                                     const PoseH* reference_pose = nullptr) {
    if (!frame.sensor_info) {
        throw std::invalid_argument("frame.sensor_info is required for frame matching");
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::RANGE)) {
        throw std::invalid_argument("frame must contain RANGE field for frame matching");
    }

    const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    const int frame_height = static_cast<int>(range.rows());
    const int frame_width = static_cast<int>(range.cols());
    int first_col = 0;
    int last_col = 0;
    try {
        first_col = frame.get_first_valid_column();
        last_col = frame.get_last_valid_column();
    } catch (const std::exception&) {
        return {};
    }
    if (last_col < first_col) {
        return {};
    }

    // Standard alignment uses the full existing sensor_to_body as the initial
    // extrinsic prior. The rotation is expected to be gravity-aligned, and the
    // translation is preserved so the solver estimates a correction from the
    // prior instead of aligning all sensors from the origin.
    const Matrix4dR sensor_to_body = frame.sensor_info->sensor_to_body;
    const Eigen::Matrix3d sensor_to_body_rotation = sensor_to_body.block<3, 3>(0, 0);
    const bool has_sensor_to_body = !sensor_to_body.isIdentity(1e-10);

    const size_t n_pixels = static_cast<size_t>(frame_height) * static_cast<size_t>(frame_width);
    std::vector<uint8_t> ground_mask;
    // Reuse a pre-existing "GROUND" field (written by GroundSegEngine) when
    // available to avoid redundant computation.
    if (frame.has_field("GROUND")) {
        try {
            const auto gf = frame.field<uint8_t>("GROUND");
            ground_mask.resize(n_pixels);
            const uint8_t* data = gf.data();
            for (size_t i = 0; i < n_pixels; ++i) {
                ground_mask[i] = data[i] != 0u ? 1u : 0u;
            }
        } catch (...) {
            ground_mask.clear();
        }
    }
    if (ground_mask.empty()) {
        try {
            ground_mask = impl::get_ground_mask(frame, ALIGN_CLOUDS_GROUND_SEG_GRID_SIZE_M,
                                                frame.sensor_info->xyzlut<double>())
                              .front();
        } catch (const std::exception& e) {
            logger().warn(
                "Ground segmentation failed in align_clouds(frame, frame); "
                "using all pixels for XY FFT: {}",
                e.what());
            ground_mask.clear();
        }
    }
    const bool have_ground_mask = !ground_mask.empty();

    // Use xyzlut WITHOUT extrinsics so points start in sensor frame; the full
    // sensor_to_body pose is applied during dewarping below.
    const ouster::sdk::core::XYZLutT<double> lut(*frame.sensor_info, false);
    const PointCloudXYZd points = lut(frame);

    // Re-express each column pose relative to the first valid column for
    // single-frame matching.  Multi-sensor FrameSet matching passes sensor
    // 0's selected pose as reference so frames filled from nearby frame sets
    // keep their body-motion offset instead of being independently rebased.
    const PoseH first_pose(frame.get_column_pose(first_col));
    const PoseH pose_reference_inv =
        reference_pose != nullptr ? PoseH(reference_pose->inverse()) : PoseH(first_pose.inverse());
    const PoseH sensor_to_body_pose(sensor_to_body);
    ouster::sdk::core::MatrixX16dR rel_poses(frame_width, 16);
    MatrixX3dR sensor_origins(frame_width, 3);
    for (int col = 0; col < frame_width; ++col) {
        const PoseH pose_c(frame.get_column_pose(col));
        PoseH rel_pose(pose_reference_inv * pose_c);
        if (has_sensor_to_body) {
            rel_pose *= sensor_to_body_pose;
        }
        Eigen::Map<ouster::sdk::core::Matrix4dR>(rel_poses.row(col).data()) = rel_pose;
        sensor_origins.row(col) = rel_pose.t().transpose();
    }

    PointCloudXYZd dewarped(points.rows(), points.cols());
    ouster::sdk::core::dewarp<double>(dewarped, points, rel_poses);

    // Prefer precomputed normals when available; otherwise estimate normals
    // from the dewarped geometry.
    FrameNormalsFieldView frame_normals_field;
    ArrayX3dR computed_normals;
    const bool have_normals_field = try_map_frame_normals_field(frame, frame_normals_field) &&
                                    frame_normals_field.rows == dewarped.rows();
    bool have_normals = have_normals_field;
    if (!have_normals_field) {
        const auto& sensor_origins_for_normals = sensor_origins;
        try {
            computed_normals =
                ouster::sdk::algorithm::normals(dewarped, range, sensor_origins_for_normals)
                    .array();
            have_normals = computed_normals.rows() == dewarped.rows();
        } catch (const std::exception& e) {
            logger().warn("Normals estimation failed in align_clouds(frame, frame): {}", e.what());
            have_normals = false;
        }
    }

    // Keep only valid returns with finite points and well-defined normals.
    const auto status = frame.status();
    const Eigen::Index max_valid = static_cast<Eigen::Index>(frame_height) *
                                   static_cast<Eigen::Index>(last_col - first_col + 1);
    ArrayX3dR valid_points(max_valid, 3);
    ArrayX3dR valid_normals = have_normals ? ArrayX3dR(max_valid, 3) : ArrayX3dR(0, 3);
    ArrayX3dR valid_xy_points = have_ground_mask ? ArrayX3dR(max_valid, 3) : ArrayX3dR(0, 3);
    ArrayX3dR valid_xy_normals =
        (have_ground_mask && have_normals) ? ArrayX3dR(max_valid, 3) : ArrayX3dR(0, 3);
    Eigen::Index idx_out = 0;
    Eigen::Index xy_idx_out = 0;
    static const float empty_normals[3] = {0.0f, 0.0f, 0.0f};
    const auto frame_normals = have_normals_field
                                   ? frame_normals_field.map()
                                   : Eigen::Map<const ArrayX3fR>(empty_normals, 0, 3);
    for (int col = first_col; col <= last_col; ++col) {
        if (status[col] == 0u) {
            continue;
        }
        for (int row = 0; row < frame_height; ++row) {
            if (range(row, col) == 0u) {
                continue;
            }
            const Eigen::Index idx = static_cast<Eigen::Index>(row) * frame_width + col;
            Eigen::Vector3d point = dewarped.row(idx).matrix();
            if (!finite_vec3(point)) {
                continue;
            }
            Eigen::Vector3d normal = Eigen::Vector3d::Zero();
            if (have_normals) {
                if (have_normals_field) {
                    normal = frame_normals.row(idx).cast<double>().matrix();
                } else {
                    normal = computed_normals.row(idx).matrix();
                }
                if (have_normals_field && has_sensor_to_body) {
                    normal = sensor_to_body_rotation * normal;
                }
                const double norm_val = normal.norm();
                if (!finite_vec3(normal) || norm_val <= NORMAL_EPS) {
                    continue;
                }
                normal /= norm_val;
                valid_normals.row(idx_out) = normal.array();
            }
            valid_points.row(idx_out) = point.array();
            if (have_ground_mask && ground_mask[static_cast<size_t>(idx)] == 0) {
                valid_xy_points.row(xy_idx_out) = point.array();
                if (have_normals) {
                    valid_xy_normals.row(xy_idx_out) = normal.array();
                }
                ++xy_idx_out;
            }
            ++idx_out;
        }
    }

    valid_points.conservativeResize(idx_out, Eigen::NoChange);
    if (have_normals) {
        valid_normals.conservativeResize(idx_out, Eigen::NoChange);
    }
    if (have_ground_mask) {
        valid_xy_points.conservativeResize(xy_idx_out, Eigen::NoChange);
        if (have_normals) {
            valid_xy_normals.conservativeResize(xy_idx_out, Eigen::NoChange);
        }
    }
    if (idx_out == 0) {
        return {};
    }

    if (!have_normals) {
        ArrayX3dR downsampled_points = core::voxel_downsample_3d(
            valid_points, VOXEL_SIZE_COARSE, 1, 1, core::VoxelDownsampleStrategy::AVERAGE_POINT);
        if (downsampled_points.rows() == 0) {
            downsampled_points = std::move(valid_points);
        }
        FrameFeatures features =
            make_frame_features(std::move(downsampled_points), ArrayX3dR(0, 3));
        if (have_ground_mask && valid_xy_points.rows() > 0) {
            ArrayX3dR downsampled_xy_points =
                core::voxel_downsample_3d(valid_xy_points, VOXEL_SIZE_COARSE, 1, 1,
                                          core::VoxelDownsampleStrategy::AVERAGE_POINT);
            if (downsampled_xy_points.rows() == 0) {
                downsampled_xy_points = std::move(valid_xy_points);
            }
            set_xy_matching_features(features, std::move(downsampled_xy_points), ArrayX3dR(0, 3));
        }
        return features;
    }

    // Normalize density before matching so histograms/ICP are less biased by
    // local over-sampling.
    auto downsampled =
        algorithm::voxel_downsample_with_normals(valid_points, valid_normals, VOXEL_SIZE_COARSE);
    FrameFeatures features =
        make_frame_features(std::move(downsampled.first), std::move(downsampled.second));
    if (have_ground_mask && valid_xy_points.rows() > 0) {
        auto downsampled_xy = algorithm::voxel_downsample_with_normals(
            valid_xy_points, valid_xy_normals, VOXEL_SIZE_COARSE);
        set_xy_matching_features(features, std::move(downsampled_xy.first),
                                 std::move(downsampled_xy.second));
    }
    return features;
}

void validate_xyz_array(Eigen::Ref<const ArrayX3dR> points, const char* name) {
    if (points.cols() != 3) {
        throw std::invalid_argument(std::string(name) + " must have shape (N, 3)");
    }
}

void validate_points_and_normals(Eigen::Ref<const ArrayX3dR> points,
                                 Eigen::Ref<const ArrayX3dR> normals, const char* points_name,
                                 const char* normals_name) {
    validate_xyz_array(points, points_name);
    validate_xyz_array(normals, normals_name);
    if (points.rows() != normals.rows()) {
        throw std::invalid_argument(std::string(points_name) + " and " + normals_name +
                                    " must have the same number of rows");
    }
}

std::pair<ArrayX3dR, ArrayX3dR> filter_valid_points_and_normals(
    Eigen::Ref<const ArrayX3dR> points, Eigen::Ref<const ArrayX3dR> normals) {
    ArrayX3dR valid_points(points.rows(), 3);
    ArrayX3dR valid_normals(normals.rows(), 3);
    Eigen::Index idx_out = 0;

    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3d point = points.row(i).matrix();
        Eigen::Vector3d normal = normals.row(i).matrix();
        const double norm_val = normal.norm();
        if (!finite_vec3(point) || !finite_vec3(normal) || norm_val <= NORMAL_EPS) {
            continue;
        }

        valid_points.row(idx_out) = point.array();
        valid_normals.row(idx_out) = (normal / norm_val).array();
        ++idx_out;
    }

    valid_points.conservativeResize(idx_out, Eigen::NoChange);
    valid_normals.conservativeResize(idx_out, Eigen::NoChange);
    return {std::move(valid_points), std::move(valid_normals)};
}

ArrayX3dR filter_valid_points(Eigen::Ref<const ArrayX3dR> points) {
    ArrayX3dR valid_points(points.rows(), 3);
    Eigen::Index idx_out = 0;
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3d point = points.row(i).matrix();
        if (!finite_vec3(point)) {
            continue;
        }
        valid_points.row(idx_out) = point.array();
        ++idx_out;
    }
    valid_points.conservativeResize(idx_out, Eigen::NoChange);
    return valid_points;
}

FrameFeatures make_point_cloud_features(ArrayX3dR points, ArrayX3dR normals,
                                        bool run_voxel_downsample) {
    if (run_voxel_downsample) {
        auto downsampled =
            algorithm::voxel_downsample_with_normals(points, normals, VOXEL_SIZE_COARSE);
        points = downsampled.first;
        normals = downsampled.second;
    }

    return make_frame_features(std::move(points), std::move(normals));
}

FrameFeatures prepare_point_cloud_features(Eigen::Ref<const ArrayX3dR> points,
                                           Eigen::Ref<const ArrayX3dR> normals,
                                           const PointCloudInputNames& input_names) {
    validate_points_and_normals(points, normals, input_names.points_name, input_names.normals_name);
    auto valid_data = filter_valid_points_and_normals(points, normals);
    return make_point_cloud_features(std::move(valid_data.first), std::move(valid_data.second),
                                     /*run_voxel_downsample=*/true);
}

FrameFeatures prepare_point_cloud_features(Eigen::Ref<const ArrayX3dR> points,
                                           const PointCloudInputNames& input_names) {
    validate_xyz_array(points, input_names.points_name);

    ArrayX3dR valid_points = filter_valid_points(points);
    if (valid_points.rows() == 0) {
        return {};
    }

    ArrayX3dR downsampled_points = core::voxel_downsample_3d(
        valid_points, VOXEL_SIZE_COARSE, 1, 1, core::VoxelDownsampleStrategy::AVERAGE_POINT);
    if (downsampled_points.rows() == 0) {
        downsampled_points = std::move(valid_points);
    }

    ArrayX3dR empty_normals(0, 3);
    return make_frame_features(std::move(downsampled_points), std::move(empty_normals));
}

// Forward declarations (used by the XY 2-D FFT helpers below).
ArrayX3dR transform_points(const ArrayX3dR& points, const PoseH& pose);
ArrayX3dR rotate_vectors(const ArrayX3dR& vectors, const PoseH& pose);

// ---------------------------------------------------------------------------
// XY 2-D FFT cross-correlation helpers
// ---------------------------------------------------------------------------

inline XYGridSpec make_xy_grid_spec(double pixel_size_m, double bound_m) {
    XYGridSpec grid_spec;
    grid_spec.pixel_size_m = std::max(1e-6, pixel_size_m);
    grid_spec.bound_m = std::max(grid_spec.pixel_size_m, bound_m);

    const double span = 2.0 * grid_spec.bound_m;
    grid_spec.base_n = std::max(8, static_cast<int>(std::ceil(span / grid_spec.pixel_size_m)) + 1);

    grid_spec.fft_n =
        static_cast<int>(next_power_of_two(static_cast<size_t>(2 * grid_spec.base_n - 1)));
    grid_spec.fft_n = std::max(grid_spec.fft_n, 8);
    return grid_spec;
}

/// Subtract the mean and normalise @p mat to unit Frobenius norm in-place.
/// No-ops on empty or constant matrices.
inline void normalize_zero_mean_unit_norm(Eigen::MatrixXd& mat) {
    if (mat.size() == 0) {
        return;
    }
    const double mean = mat.mean();
    mat.array() -= mean;
    const double norm_sq = mat.array().square().sum();
    if (!std::isfinite(norm_sq) || norm_sq <= 1e-30) {
        return;
    }
    mat.array() /= std::sqrt(norm_sq);
}

/// Rasterise @p points into a birds-eye-view (BEV) grid weighted by point
/// distance and, when @p normals_arr is provided, by horizontal normal
/// strength. Floor and ceiling bands are filtered out when the Z span exceeds 1
/// m.
inline Eigen::MatrixXd build_xy_bev_grid(Eigen::Ref<const ArrayX3dR> points,
                                         Eigen::Ref<const Eigen::ArrayXd> point_dist,
                                         const XYGridSpec& spec,
                                         const ArrayX3dR* normals_arr = nullptr) {
    Eigen::MatrixXd grid = Eigen::MatrixXd::Zero(spec.base_n, spec.base_n);
    const double half = spec.bound_m;
    const double inv = 1.0 / spec.pixel_size_m;
    const bool use_normals =
        normals_arr != nullptr && frame_features_have_normals(points, *normals_arr);

    // Floor/ceiling Z-filter: remove floor band and ceiling band so
    // horizontal surfaces don't fill every BEV cell.
    // Uses 4th percentile for the floor reference to be robust against
    // noise outliers below the actual floor.
    constexpr double floor_strip_height_m = 0.2;
    constexpr double ceil_margin_m = 0.1;
    constexpr double min_z_range_for_filter = 1.0;
    constexpr double floor_percentile = 0.04;
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    std::vector<double> z_values;
    z_values.reserve(static_cast<size_t>(points.rows()));
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const double z = points(i, 2);
        if (std::isfinite(z)) {
            z_values.push_back(z);
            if (z < min_z) {
                min_z = z;
            }
            if (z > max_z) {
                max_z = z;
            }
        }
    }
    const double z_range = max_z - min_z;
    const bool filter_floor_ceil = std::isfinite(z_range) && z_range > min_z_range_for_filter;
    double floor_ref = min_z;
    if (filter_floor_ceil && !z_values.empty()) {
        const size_t p_idx =
            std::min(static_cast<size_t>(floor_percentile * static_cast<double>(z_values.size())),
                     z_values.size() - 1);
        std::nth_element(z_values.begin(), z_values.begin() + static_cast<std::ptrdiff_t>(p_idx),
                         z_values.end());
        floor_ref = z_values[p_idx];
    }
    const double floor_z_cutoff = floor_ref + floor_strip_height_m;
    const double ceil_z_cutoff = max_z - ceil_margin_m;

    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const double point_dist_i = point_dist(i);
        if (point_dist_i <= 0.0) {
            continue;
        }

        const Eigen::Vector3d point = points.row(i).matrix();
        if (!finite_vec3(point)) {
            continue;
        }

        // Exclude floor band (bottom 20 cm) and ceiling band (top 10 cm).
        if (filter_floor_ceil && (point.z() <= floor_z_cutoff || point.z() >= ceil_z_cutoff)) {
            continue;
        }

        const double x = point.x();
        const double y = point.y();
        if (std::abs(x) > half || std::abs(y) > half) {
            continue;
        }

        double weight = point_dist_i;
        if (use_normals) {
            const Eigen::Vector3d normal = normals_arr->row(i).matrix();
            if (!finite_vec3(normal)) {
                continue;
            }

            const double strength = normal.x() * normal.x() + normal.y() * normal.y();
            if (strength < 0.5) {
                continue;
            }
            weight *= strength;
        }

        const int ix = static_cast<int>(std::floor((x + half) * inv));
        const int iy = static_cast<int>(std::floor((y + half) * inv));
        if (ix < 0 || iy < 0 || ix >= spec.base_n || iy >= spec.base_n) {
            continue;
        }

        grid(iy, ix) += weight;
    }

    return grid;
}

inline Eigen::MatrixXcd pad_real_to_complex(const Eigen::MatrixXd& input_mat, int fft_n) {
    Eigen::MatrixXcd out = Eigen::MatrixXcd::Zero(fft_n, fft_n);
    out.block(0, 0, input_mat.rows(), input_mat.cols()) = input_mat.cast<Complex>();
    return out;
}

/// Compute a separable 2-D DFT (or its inverse) on @p data in-place,
/// processing rows then columns via Eigen::FFT.
inline void fft2d_inplace(Eigen::MatrixXcd& data, bool inverse) {
    const int num_rows = static_cast<int>(data.rows());
    const int num_cols = static_cast<int>(data.cols());
    Eigen::FFT<double> fft;

    // rows
    {
        std::vector<Complex> row_in(static_cast<size_t>(num_cols));
        std::vector<Complex> row_out;
        row_out.reserve(static_cast<size_t>(num_cols));
        for (int row = 0; row < num_rows; ++row) {
            for (int col = 0; col < num_cols; ++col) {
                row_in[static_cast<size_t>(col)] = data(row, col);
            }
            if (!inverse) {
                fft.fwd(row_out, row_in);
            } else {
                fft.inv(row_out, row_in);
            }
            for (int col = 0; col < num_cols; ++col) {
                data(row, col) = row_out[static_cast<size_t>(col)];
            }
        }
    }

    // cols
    {
        std::vector<Complex> col_in(static_cast<size_t>(num_rows));
        std::vector<Complex> col_out;
        col_out.reserve(static_cast<size_t>(num_rows));
        for (int col = 0; col < num_cols; ++col) {
            for (int row = 0; row < num_rows; ++row) {
                col_in[static_cast<size_t>(row)] = data(row, col);
            }
            if (!inverse) {
                fft.fwd(col_out, col_in);
            } else {
                fft.inv(col_out, col_in);
            }
            for (int row = 0; row < num_rows; ++row) {
                data(row, col) = col_out[static_cast<size_t>(row)];
            }
        }
    }
}

/// Build a BEV grid from the source frame and compute its 2-D FFT.
/// The result is reused for every yaw candidate in `initial_pairwise_alignment`.
inline PrecomputedXY precompute_source_xy_fft(const FrameFeatures& source, double pixel_size_m,
                                              double bound_m) {
    PrecomputedXY out;
    out.spec = make_xy_grid_spec(pixel_size_m, bound_m);

    const ArrayX3dR& source_points = xy_matching_points(source);
    const ArrayX3dR& source_normals_for_xy = xy_matching_normals(source);
    const Eigen::ArrayXd& source_point_dist = xy_matching_point_dist(source);
    const ArrayX3dR* source_normals =
        frame_features_have_normals(source_points, source_normals_for_xy) ? &source_normals_for_xy
                                                                          : nullptr;
    Eigen::MatrixXd src =
        build_xy_bev_grid(source_points, source_point_dist, out.spec, source_normals);

    normalize_zero_mean_unit_norm(src);

    if (src.array().square().sum() <= XY_GRID_MIN_ENERGY) {
        out.source_fft = Eigen::MatrixXcd::Zero(out.spec.fft_n, out.spec.fft_n);
        return out;
    }

    out.source_fft = pad_real_to_complex(src, out.spec.fft_n);
    fft2d_inplace(out.source_fft, /*inverse=*/false);
    return out;
}

/// Align the moving frame to the source in XY via 2-D FFT cross-correlation.
///
/// Transforms @p moving by @p current_pose, builds its BEV grid, cross-
/// correlates against the precomputed source FFT, and returns the optimal
/// (dx, dy) shift within ±@p max_shift_m together with the correlation score.
inline Align2DResult align_xy_2d_fft(const FrameFeatures& moving, const PrecomputedXY& src_pre,
                                     const PoseH& current_pose, double max_shift_m) {
    Align2DResult best;

    const XYGridSpec& spec = src_pre.spec;
    const int fft_size = spec.fft_n;

    if (src_pre.source_fft.size() == 0 ||
        src_pre.source_fft.cwiseAbs2().sum() <= XY_GRID_MIN_ENERGY) {
        best.dx_m = 0.0;
        best.dy_m = 0.0;
        best.score = 0.0;
        return best;
    }

    const ArrayX3dR& moving_xy_points = xy_matching_points(moving);
    const ArrayX3dR& moving_xy_normals = xy_matching_normals(moving);
    const Eigen::ArrayXd& moving_xy_point_dist = xy_matching_point_dist(moving);
    const ArrayX3dR moving_points = transform_points(moving_xy_points, current_pose);
    const bool use_normals = frame_features_have_normals(moving_xy_points, moving_xy_normals);
    const ArrayX3dR moving_normals =
        use_normals ? rotate_vectors(moving_xy_normals, current_pose) : ArrayX3dR(0, 3);

    Eigen::MatrixXd mov = build_xy_bev_grid(moving_points, moving_xy_point_dist, spec,
                                            use_normals ? &moving_normals : nullptr);

    normalize_zero_mean_unit_norm(mov);

    if (mov.array().square().sum() <= XY_GRID_MIN_ENERGY) {
        best.dx_m = 0.0;
        best.dy_m = 0.0;
        best.score = 0.0;
        return best;
    }

    Eigen::MatrixXcd mov_fft = pad_real_to_complex(mov, fft_size);
    fft2d_inplace(mov_fft, /*inverse=*/false);

    // Cross-correlation done in place: conj(FFT(mov)) .* FFT(src). Both factors
    // are coefficient-wise, so writing back into mov_fft is alias-safe and
    // avoids a second (fft_size x fft_size) allocation.
    mov_fft = mov_fft.conjugate().cwiseProduct(src_pre.source_fft);
    fft2d_inplace(mov_fft, /*inverse=*/true);
    const Eigen::MatrixXcd corr_fft = std::move(mov_fft);

    // The search region is square: dx and dy independently span
    // [-max_shift_m, +max_shift_m], capped to the current grid half-width.
    const int max_shift_bins =
        std::max(1, static_cast<int>(std::llround(std::max(0.1, max_shift_m) / spec.pixel_size_m)));

    const int max_shift = std::min(max_shift_bins, spec.base_n / 2);

    int best_dx = 0;
    int best_dy = 0;
    double best_score = std::numeric_limits<double>::lowest();

    for (int dy = -max_shift; dy <= max_shift; ++dy) {
        const int iy = (dy >= 0) ? dy : (fft_size + dy);
        for (int dx = -max_shift; dx <= max_shift; ++dx) {
            const int ix = (dx >= 0) ? dx : (fft_size + dx);
            const double corr_val = corr_fft(iy, ix).real();
            if (corr_val > best_score) {
                best_score = corr_val;
                best_dx = dx;
                best_dy = dy;
            }
        }
    }

    best.dx_m = spec.pixel_size_m * static_cast<double>(best_dx);
    best.dy_m = spec.pixel_size_m * static_cast<double>(best_dy);
    best.score = best_score;
    return best;
}

// ---------------------------------------------------------------------------
// Point / normal transformation
// ---------------------------------------------------------------------------

/// Apply a rigid-body transform (rotation + translation) to a point cloud.
ArrayX3dR transform_points(const ArrayX3dR& points, const PoseH& pose) {
    return ((pose.r() * points.matrix().transpose()).colwise() + pose.t()).transpose().array();
}

/// Rotate direction vectors (e.g. normals) by the rotation part of @p pose.
ArrayX3dR rotate_vectors(const ArrayX3dR& vectors, const PoseH& pose) {
    return (pose.r() * vectors.matrix().transpose()).transpose().array();
}

// -------------------------------------------------------------------
// 1-D translation histogram cross-correlation (used for Z alignment)
// -------------------------------------------------------------------

/// Project points onto @p direction, weighted by point distance and optional
/// normal-direction alignment, and accumulate into a 1-D histogram.
TransHistogram compute_translation_histogram(const ArrayX3dR& points,
                                             const Eigen::ArrayXd& point_dist,
                                             const Eigen::Vector3d& direction,
                                             const ArrayX3dR* normals = nullptr) {
    TransHistogram hist{};
    hist.fill(0.0);
    const bool use_normals = normals != nullptr && frame_features_have_normals(points, *normals);
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const double point_dist_i = point_dist(i);
        if (point_dist_i <= 0.0) {
            continue;
        }
        const Eigen::Vector3d point = points.row(i).matrix();
        double weight = 1.0;
        if (use_normals) {
            const Eigen::Vector3d normal = normals->row(i).matrix();
            weight = std::abs(normal.dot(direction));
            if (weight <= 0.5) {
                continue;
            }
        }
        const int pos =
            static_cast<int>(std::llround(point.dot(direction) / TRANS_PITCH)) + (NUM_TRANS / 2);
        if (pos < 0 || pos >= NUM_TRANS) {
            continue;
        }
        hist[static_cast<size_t>(pos)] += weight * point_dist_i;
    }
    return hist;
}

/// Find the integer-bin shift that maximises the cross-correlation
/// between two 1-D translation histograms using FFT.  Returns
/// (best_shift_bins, best_score).
std::pair<int, double> best_translation_shift(const TransHistogram& moving,
                                              const TransHistogram& source, int max_shift_bins) {
    constexpr size_t trans_size = static_cast<size_t>(NUM_TRANS);
    const size_t fft_size = next_power_of_two(2u * trans_size - 1u);

    Eigen::FFT<double> fft;
    std::vector<Complex> moving_time(fft_size, Complex{0.0, 0.0});
    std::vector<Complex> source_time(fft_size, Complex{0.0, 0.0});
    for (size_t i = 0; i < trans_size; ++i) {
        moving_time[i] = Complex{moving[i], 0.0};
        source_time[i] = Complex{source[i], 0.0};
    }

    std::vector<Complex> moving_freq;
    std::vector<Complex> source_freq;
    fft.fwd(moving_freq, moving_time);
    fft.fwd(source_freq, source_time);

    std::vector<Complex> corr_freq(fft_size, Complex{0.0, 0.0});
    for (size_t i = 0; i < fft_size; ++i) {
        // Linear (zero-padded) cross-correlation:
        // score(shift) = sum_i moving[i] * source[i + shift]
        corr_freq[i] = std::conj(moving_freq[i]) * source_freq[i];
    }

    std::vector<Complex> corr_time;
    fft.inv(corr_time, corr_freq);

    int best_shift = 0;
    double best_score = std::numeric_limits<double>::lowest();
    const int max_shift = std::min(std::max(1, max_shift_bins), NUM_TRANS / 2);
    for (int shift = -max_shift; shift <= max_shift; ++shift) {
        const size_t corr_idx =
            (shift >= 0) ? static_cast<size_t>(shift) : (fft_size - static_cast<size_t>(-shift));
        const double score = corr_time[corr_idx].real();
        if (score > best_score) {
            best_score = score;
            best_shift = shift;
        }
    }
    return {best_shift, best_score};
}

/// Align the moving frame to the source along a single axis via 1-D
/// histogram cross-correlation.  Returns the optimal translation (m)
/// and its correlation score.
Align1DResult align_translation_1d(const FrameFeatures& moving, const TransHistogram& source_hist,
                                   const PoseH& current_pose, const Eigen::Vector3d& direction,
                                   double max_shift_m) {
    const ArrayX3dR moving_points = transform_points(moving.points, current_pose);
    const bool use_normals = frame_features_have_normals(moving.points, moving.normals);
    const ArrayX3dR moving_normals =
        use_normals ? rotate_vectors(moving.normals, current_pose) : ArrayX3dR(0, 3);
    const TransHistogram moving_hist = compute_translation_histogram(
        moving_points, moving.point_dist, direction, use_normals ? &moving_normals : nullptr);
    const int max_shift_bins =
        static_cast<int>(std::llround(std::max(0.1, max_shift_m) / TRANS_PITCH));
    const auto best = best_translation_shift(moving_hist, source_hist, max_shift_bins);
    const double unclipped = TRANS_PITCH * static_cast<double>(best.first);
    const double clipped = std::max(-max_shift_m, std::min(unclipped, max_shift_m));
    return {clipped, best.second};
}

// -------------------------------------------------------------------
// Confidence estimation
// -------------------------------------------------------------------

std::vector<uint8_t> make_confidence_sample_mask(Eigen::Ref<const ArrayX3dR> points) {
    if (points.rows() <= CONFIDENCE_MAX_QUERY_SAMPLES) {
        return {};
    }

    std::vector<Eigen::Vector3d> sample_points;
    std::vector<Eigen::Index> sample_rows;
    sample_points.reserve(static_cast<size_t>(points.rows()));
    sample_rows.reserve(static_cast<size_t>(points.rows()));
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3d point = points.row(i).matrix();
        sample_points.emplace_back(point);
        sample_rows.push_back(i);
    }
    if (sample_points.size() <= static_cast<size_t>(CONFIDENCE_MAX_QUERY_SAMPLES)) {
        return {};
    }

    // Reuse the voxel-map downsample shuffle selection function
    // TODO [hao] Once Ussama has the final shuffle select API, edit below
    // TODO [hao] averaging downsample & not cap may be better. Need to experiment with it.
    const auto downsampled = core::voxel_downsample(sample_points, VOXEL_SIZE_COARSE);
    const auto& downsampled_indices = downsampled.second;
    const size_t sample_count =
        std::min(static_cast<size_t>(CONFIDENCE_MAX_QUERY_SAMPLES), downsampled_indices.size());

    std::vector<uint8_t> keep(static_cast<size_t>(points.rows()), 0u);
    for (size_t i = 0; i < sample_count; ++i) {
        keep[static_cast<size_t>(sample_rows[downsampled_indices[i]])] = 1u;
    }
    return keep;
}

struct ConfidenceSampleMasks {
    std::vector<uint8_t> source;
    std::vector<uint8_t> target;
};

ConfidenceSampleMasks make_confidence_sample_masks(Eigen::Ref<const ArrayX3dR> source_points,
                                                   Eigen::Ref<const ArrayX3dR> target_points) {
    // Store selected rows as source/target masks. The masks depend only on each
    // cloud, not on the tested pose, so they are reused for both initial-pose and
    // refined-pose confidence checks.
    return {make_confidence_sample_mask(source_points), make_confidence_sample_mask(target_points)};
}

struct ConfidenceQueryData {
    ConfidenceQueryData(const ArrayX3dR& source_points, const ArrayX3dR& target_points)
        : source_grid(source_points, XY_OVERLAP_RADIUS_M),
          target_grid(target_points, XY_OVERLAP_RADIUS_M),
          sample_masks(make_confidence_sample_masks(source_points, target_points)) {}

    SpatialHashGridXY source_grid;
    SpatialHashGridXY target_grid;
    ConfidenceSampleMasks sample_masks;
};

/// Estimate the quality of a pairwise alignment by measuring the fraction of
/// source points that have a nearby neighbour in the target. When normals are
/// available on both clouds, the neighbour must also satisfy a normal-angle
/// gate. Large clouds use a deterministic voxel-map downsampled subset capped
/// per direction; smaller clouds scan every valid point. Returns a value in
/// [0, 1].
double xy_matching_confidence(Eigen::Ref<const ArrayX3dR> source_points,
                              Eigen::Ref<const ArrayX3dR> target_points,
                              Eigen::Ref<const ArrayX3dR> source_normals,
                              Eigen::Ref<const ArrayX3dR> target_normals, const PoseH& pose,
                              const ConfidenceQueryData& query_data) {
    if (source_points.rows() == 0 || target_points.rows() == 0) {
        return 0.0;
    }

    const Eigen::Matrix3d rotation = pose.r();
    const Eigen::Vector3d translation = pose.t();
    const double max_xy_dist_sq = XY_OVERLAP_RADIUS_M * XY_OVERLAP_RADIUS_M;
    const bool use_normals = frame_features_have_normals(source_points, source_normals) &&
                             frame_features_have_normals(target_points, target_normals);
    const double cos_normal_gate = std::cos(CONFIDENCE_MAX_NORMAL_ANGLE_DEG * M_PI / 180.0);

    // Count one directed overlap score: transform query_xyz by (rot, trans),
    // then ask whether each query point has a nearby XY neighbour in grid_xyz.
    // The caller runs this twice, source->target and target->source, so the final
    // confidence is symmetric.
    const auto count_direction =
        [&](Eigen::Ref<const ArrayX3dR> query_xyz, Eigen::Ref<const ArrayX3dR> query_normals,
            Eigen::Ref<const ArrayX3dR> grid_xyz, Eigen::Ref<const ArrayX3dR> grid_normals,
            const SpatialHashGridXY& grid, const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans,
            const std::vector<uint8_t>& sample_mask) -> std::pair<size_t, size_t> {
        const Eigen::Index num_rows = query_xyz.rows();
        if (num_rows == 0) {
            return {0u, 0u};
        }
        const size_t worker_count = pairwise_inner_parallel_disabled
                                        ? 1u
                                        : parallel_worker_count(static_cast<size_t>(num_rows));
        std::vector<size_t> local_total(worker_count, 0u);
        std::vector<size_t> local_matched(worker_count, 0u);
        const Eigen::Index chunk = (num_rows + static_cast<Eigen::Index>(worker_count) - 1) /
                                   static_cast<Eigen::Index>(worker_count);
        // Split rows into contiguous chunks. Each worker writes only its own
        // counters, and the final reduction is an integer sum, so the result is
        // deterministic regardless of scheduling.
        run_parallel_indexed(worker_count, worker_count > 1, [&](size_t w) {
            const Eigen::Index begin = static_cast<Eigen::Index>(w) * chunk;
            const Eigen::Index end = std::min(num_rows, begin + chunk);
            size_t total = 0u;
            size_t matched = 0u;
            for (Eigen::Index i = begin; i < end; ++i) {
                // Empty mask means scan every valid row. Non-empty mask means
                // large-cloud confidence sampling selected this query row.
                if (!sample_mask.empty() && sample_mask[static_cast<size_t>(i)] == 0u) {
                    continue;
                }
                const Eigen::Vector3d point = query_xyz.row(i).matrix();
                if (!finite_vec3(point)) {
                    continue;
                }
                // total is the denominator for this direction: valid sampled
                // query points after finite filtering.
                ++total;
                const Eigen::Vector3d x = rot * point + trans;
                const int nn_idx = grid.nearest_xy(grid_xyz, x, max_xy_dist_sq);
                if (nn_idx < 0) {
                    continue;
                }
                if (!use_normals) {
                    ++matched;
                    continue;
                }
                // With normals, a nearby XY point counts only when the rotated
                // query normal agrees with the neighbour normal within the gate.
                Eigen::Vector3d n_query = query_normals.row(i).matrix();
                Eigen::Vector3d n_grid = grid_normals.row(nn_idx).matrix();
                const double n_query_norm = n_query.norm();
                const double n_grid_norm = n_grid.norm();
                if (finite_vec3(n_query) && finite_vec3(n_grid) && n_query_norm > NORMAL_EPS &&
                    n_grid_norm > NORMAL_EPS) {
                    n_query /= n_query_norm;
                    n_grid /= n_grid_norm;
                    const Eigen::Vector3d n_query_world = (rot * n_query).normalized();
                    if (std::abs(n_query_world.dot(n_grid)) >= cos_normal_gate) {
                        ++matched;
                    }
                }
            }
            local_total[w] = total;
            local_matched[w] = matched;
        });
        size_t total = 0u;
        size_t matched = 0u;
        for (size_t w = 0; w < worker_count; ++w) {
            total += local_total[w];
            matched += local_matched[w];
        }
        return {total, matched};
    };

    const std::pair<size_t, size_t> source_dir = count_direction(
        source_points, source_normals, target_points, target_normals, query_data.target_grid,
        rotation, translation, query_data.sample_masks.source);

    const Eigen::Matrix3d rotation_inv = rotation.transpose();
    const Eigen::Vector3d t_inv = -rotation_inv * translation;
    const std::pair<size_t, size_t> target_dir = count_direction(
        target_points, target_normals, source_points, source_normals, query_data.source_grid,
        rotation_inv, t_inv, query_data.sample_masks.target);

    if (source_dir.first == 0u || target_dir.first == 0u) {
        return 0.0;
    }
    const double conf = static_cast<double>(source_dir.second + target_dir.second) /
                        static_cast<double>(source_dir.first + target_dir.first);
    return clamp01(conf);
}

// -------------------------------------------------------------------
// Exhaustive yaw search with 2-D FFT XY alignment
// -------------------------------------------------------------------

struct YawSearchCandidate {
    PoseH pose;
    double score = std::numeric_limits<double>::lowest();
};

/// Find the best SE(3) initial alignment between two plumb-aligned frames using
/// 2-D FFT cross-correlation for XY and 1-D histogram correlation for Z.
///
/// Pass 1 selects the yaw basin by sweeping the full 360 deg in 2 deg steps on a
/// coarse (0.5 m) BEV grid; the coarse grid captures overall structure and is
/// robust against fine-detail spurious peaks (e.g. a 180-deg-flipped basin on a
/// near-symmetric indoor scene). Pass 2 refines +/-3 deg around the chosen basin
/// on the adaptive fine XY grid and applies the XY/Z translations.
PoseH initial_pairwise_alignment(const FrameFeatures& reference, const FrameFeatures& moving,
                                 const PoseH& initial_guess, double* score_out = nullptr) {
    const XYMatcherParams xy_params = choose_xy_matcher_params(reference, moving, initial_guess);

    // Precompute the reference Z histogram (1-D, reused for every yaw candidate).
    const ArrayX3dR* reference_normals =
        frame_features_have_normals(reference.points, reference.normals) ? &reference.normals
                                                                         : nullptr;
    const TransHistogram reference_hist_z = compute_translation_histogram(
        reference.points, reference.point_dist, Eigen::Vector3d::UnitZ(), reference_normals);

    // Precompute the reference XY FFT at two resolutions:
    //  - a coarse (0.5 m) grid used to pick the yaw basin. It captures the
    //    scene's overall structure and is robust against fine-detail spurious
    //    correlation peaks, which on near-symmetric scenes (e.g. indoor rooms)
    //    can otherwise lock onto a 180-degree-flipped basin on the fine grid.
    //  - the adaptive fine grid used to refine XY precisely within the basin.
    constexpr double coarse_yaw_pixel_m = 0.5;
    const PrecomputedXY src_xy_pre =
        precompute_source_xy_fft(reference, xy_params.pixel_size_m, xy_params.bound_m);
    const PrecomputedXY src_xy_coarse =
        precompute_source_xy_fft(reference, coarse_yaw_pixel_m, xy_params.bound_m);

    // --- Pass 1: coarse-grid yaw sweep (2 deg steps) selects the basin. ---
    // Use a fine 2 deg angular step so a sharp correlation peak is not straddled.
    constexpr int coarse_steps = 180;  // 360 deg / 2 deg = 180
    constexpr double coarse_step_rad = 2.0 * M_PI / static_cast<double>(coarse_steps);
    std::vector<double> coarse_scores(coarse_steps, std::numeric_limits<double>::lowest());
    run_parallel_indexed(coarse_steps, !pairwise_inner_parallel_disabled, [&](size_t yi) {
        const double yaw = static_cast<double>(yi) * coarse_step_rad;
        PoseH yaw_delta;
        yaw_delta.set_rot(RotV(0.0, 0.0, yaw).exp());
        PoseH pose = yaw_delta * initial_guess;
        const auto z_aligned = align_translation_1d(
            moving, reference_hist_z, pose, Eigen::Vector3d::UnitZ(), MAX_Z_COARSE_CORRECTION_M);
        pose(2, 3) += z_aligned.translation;
        const auto xy = align_xy_2d_fft(moving, src_xy_coarse, pose, xy_params.max_shift_m);
        coarse_scores[yi] = xy.score;
    });

    double best_yaw = 0.0;
    double best_coarse_score = std::numeric_limits<double>::lowest();
    for (int yi = 0; yi < coarse_steps; ++yi) {
        if (coarse_scores[yi] > best_coarse_score) {
            best_coarse_score = coarse_scores[yi];
            best_yaw = static_cast<double>(yi) * coarse_step_rad;
        }
    }

    // --- Pass 2: fine-grid refinement around the chosen basin (+/-3 deg). ---
    // A 1 deg local sweep covers the selected 2 deg coarse bin and its adjacent
    // neighborhood without paying for a full-resolution 360 deg sweep.
    // Both Z and XY translations are applied to produce the output pose.
    constexpr double refine_half_range_rad = 3.0 * M_PI / 180.0;
    constexpr double refine_step_rad = 1.0 * M_PI / 180.0;
    constexpr int refine_steps = 7;  // offsets [-3, +3] deg in 1 deg steps
    std::vector<YawSearchCandidate> fine_candidates(refine_steps);
    run_parallel_indexed(refine_steps, !pairwise_inner_parallel_disabled, [&](size_t step_idx) {
        const double yaw =
            best_yaw - refine_half_range_rad + static_cast<double>(step_idx) * refine_step_rad;
        PoseH yaw_delta;
        yaw_delta.set_rot(RotV(0.0, 0.0, yaw).exp());
        PoseH pose = yaw_delta * initial_guess;
        const auto z_aligned = align_translation_1d(
            moving, reference_hist_z, pose, Eigen::Vector3d::UnitZ(), MAX_Z_COARSE_CORRECTION_M);
        pose(2, 3) += z_aligned.translation;
        const auto xy = align_xy_2d_fft(moving, src_xy_pre, pose, xy_params.max_shift_m);
        pose(0, 3) += xy.dx_m;
        pose(1, 3) += xy.dy_m;
        fine_candidates[step_idx] = YawSearchCandidate{pose, xy.score};
    });

    PoseH best_pose = initial_guess;
    double best_score = std::numeric_limits<double>::lowest();
    for (const auto& candidate : fine_candidates) {
        if (candidate.score > best_score) {
            best_score = candidate.score;
            best_pose = candidate.pose;
        }
    }

    if (score_out != nullptr) {
        *score_out = std::isfinite(best_score) ? std::max(0.0, best_score) : 0.0;
    }
    return best_pose;
}

// -------------------------------------------------------------------
// Point-to-plane ICP refinement
// -------------------------------------------------------------------

}  // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
Matrix4dR point_to_point_align(Eigen::Ref<const ArrayX3dR> source_points,
                               Eigen::Ref<const ArrayX3dR> target_points,
                               const Matrix4dR& initial_guess, double max_corr_dist) {
    if (!std::isfinite(max_corr_dist) || max_corr_dist <= 0.0) {
        throw std::invalid_argument("max_corr_dist must be finite and greater than zero");
    }

    PoseH current_pose(initial_guess);
    bool solved_any_level = false;

    constexpr int icp_max_iterations = 10;

    if (static_cast<size_t>(source_points.rows()) < MIN_ICP_POINTS ||
        static_cast<size_t>(target_points.rows()) < MIN_ICP_POINTS) {
        return initial_guess;
    }

    const double corr_dist_sq = max_corr_dist * max_corr_dist;

    // Build hash once — target is constant across ICP iterations.
    const SpatialHashGrid3D tgt_grid(target_points, max_corr_dist);

    for (int iter = 0; iter < icp_max_iterations; ++iter) {
        struct Corr {
            Eigen::Vector3d x;
            Eigen::Vector3d q;
            double r = 0.0;
        };

        std::vector<Corr> corrs;
        std::vector<double> residuals;
        corrs.reserve(static_cast<size_t>(source_points.rows()));
        residuals.reserve(static_cast<size_t>(source_points.rows()));

        const Eigen::Matrix3d rotation = current_pose.r();
        const Eigen::Vector3d translation = current_pose.t();
        for (Eigen::Index i = 0; i < source_points.rows(); ++i) {
            const Eigen::Vector3d point = source_points.row(i).matrix();
            const Eigen::Vector3d x = rotation * point + translation;
            const int nn_idx = tgt_grid.nearest(target_points, x, corr_dist_sq);
            if (nn_idx < 0) {
                continue;
            }

            const Eigen::Vector3d target_pt = target_points.row(nn_idx).matrix();
            const double r = (x - target_pt).norm();
            if (!finite_vec3(x) || !finite_vec3(target_pt) || !std::isfinite(r)) {
                continue;
            }

            corrs.push_back({x, target_pt, r});
            residuals.push_back(r);
        }

        if (corrs.size() < MIN_ICP_POINTS) {
            break;
        }

        solved_any_level = true;

        const double mad = median_abs(residuals);
        double sigma = MAD_TO_SIGMA * mad;
        if (!std::isfinite(sigma) || sigma < 1e-4) {
            sigma = std::max(1e-3, 0.25 * max_corr_dist);
        }
        const double huber_delta = HUBER_K * sigma;

        std::vector<double> weights;
        weights.reserve(corrs.size());
        double weight_sum = 0.0;
        Eigen::Vector3d centroid_x = Eigen::Vector3d::Zero();
        Eigen::Vector3d centroid_q = Eigen::Vector3d::Zero();
        for (const auto& corr : corrs) {
            const double abs_r = std::abs(corr.r);
            const double w =
                (abs_r <= huber_delta || huber_delta <= 0.0) ? 1.0 : (huber_delta / abs_r);
            weights.push_back(w);
            weight_sum += w;
            centroid_x += w * corr.x;
            centroid_q += w * corr.q;
        }

        if (!std::isfinite(weight_sum) || weight_sum <= 1e-12) {
            break;
        }

        centroid_x /= weight_sum;
        centroid_q /= weight_sum;

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < corrs.size(); ++i) {
            covariance.noalias() +=
                weights[i] * (corrs[i].x - centroid_x) * (corrs[i].q - centroid_q).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d svd_u = svd.matrixU();
        Eigen::Matrix3d svd_v = svd.matrixV();
        if (!svd_u.allFinite() || !svd_v.allFinite()) {
            break;
        }

        Eigen::Matrix3d delta_rotation = svd_v * svd_u.transpose();
        if (delta_rotation.determinant() < 0.0) {
            svd_v.col(2) *= -1.0;
            delta_rotation = svd_v * svd_u.transpose();
        }

        const Eigen::Vector3d delta_translation = centroid_q - delta_rotation * centroid_x;
        if (!delta_rotation.allFinite() || !finite_vec3(delta_translation)) {
            break;
        }

        Matrix4dR delta = Matrix4dR::Identity();
        delta.block<3, 3>(0, 0) = delta_rotation;
        delta.block<3, 1>(0, 3) = delta_translation;
        current_pose = PoseH(delta) * current_pose;

        const double trace_term =
            std::max(-1.0, std::min(0.5 * (delta_rotation.trace() - 1.0), 1.0));
        const double d_rot = std::acos(trace_term);
        const double d_trans = delta_translation.norm();
        if (d_rot < 1e-4 && d_trans < 1e-3) {
            break;
        }
    }

    if (!solved_any_level) {
        return initial_guess;
    }
    return current_pose;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
Matrix4dR point_to_plane_align(Eigen::Ref<const ArrayX3dR> source_points,
                               Eigen::Ref<const ArrayX3dR> target_points,
                               Eigen::Ref<const ArrayX3dR> source_normals,
                               Eigen::Ref<const ArrayX3dR> target_normals,
                               const Matrix4dR& initial_guess, double max_corr_dist,
                               double max_normal_angle_deg) {
    if (!std::isfinite(max_corr_dist) || max_corr_dist <= 0.0) {
        throw std::invalid_argument("max_corr_dist must be finite and greater than zero");
    }
    if (!std::isfinite(max_normal_angle_deg) || max_normal_angle_deg < 0.0 ||
        max_normal_angle_deg > 180.0) {
        throw std::invalid_argument("max_normal_angle_deg must be finite and in [0, 180]");
    }
    if (source_points.rows() != source_normals.rows()) {
        throw std::invalid_argument(
            "source_points and source_normals must have the same number of "
            "rows");
    }
    if (target_points.rows() != target_normals.rows()) {
        throw std::invalid_argument(
            "target_points and target_normals must have the same number of "
            "rows");
    }

    PoseH current_pose(initial_guess);
    bool solved_any_level = false;

    constexpr int icp_max_iterations = 10;

    if (static_cast<size_t>(source_points.rows()) < MIN_ICP_POINTS ||
        static_cast<size_t>(target_points.rows()) < MIN_ICP_POINTS) {
        return initial_guess;
    }

    const double corr_dist_sq = max_corr_dist * max_corr_dist;
    const double cos_angle_gate = std::cos(max_normal_angle_deg * M_PI / 180.0);

    // Build hash once — target is constant across ICP iterations.
    const SpatialHashGrid3D tgt_grid(target_points, target_normals, max_corr_dist);

    for (int iter = 0; iter < icp_max_iterations; ++iter) {
        struct Corr {
            Eigen::Vector3d x;
            Eigen::Vector3d n;
            double r = 0.0;
        };

        std::vector<Corr> corrs;
        std::vector<double> residuals;
        corrs.reserve(static_cast<size_t>(source_points.rows()));
        residuals.reserve(static_cast<size_t>(source_points.rows()));

        const Eigen::Matrix3d rotation = current_pose.r();
        const Eigen::Vector3d translation = current_pose.t();

        // Build correspondence set with distance and normal-angle gating.
        for (Eigen::Index i = 0; i < source_points.rows(); ++i) {
            const Eigen::Vector3d point = source_points.row(i).matrix();
            Eigen::Vector3d n_src = source_normals.row(i).matrix();
            const double n_src_norm = n_src.norm();
            if (!finite_vec3(n_src) || n_src_norm <= NORMAL_EPS) {
                continue;
            }
            n_src /= n_src_norm;
            const Eigen::Vector3d x = rotation * point + translation;
            const Eigen::Vector3d n_src_world = rotation * n_src;
            const int nn_idx = tgt_grid.nearest(target_points, x, corr_dist_sq);
            if (nn_idx < 0) {
                continue;
            }

            const Eigen::Vector3d target_pt = target_points.row(nn_idx).matrix();
            Eigen::Vector3d n_tgt = target_normals.row(nn_idx).matrix();
            const double n_tgt_norm = n_tgt.norm();
            if (!finite_vec3(n_tgt) || n_tgt_norm <= NORMAL_EPS) {
                continue;
            }
            n_tgt /= n_tgt_norm;
            const double n_align = std::abs(n_tgt.dot(n_src_world));
            if (!std::isfinite(n_align) || n_align < cos_angle_gate) {
                continue;
            }

            const double r = n_tgt.dot(x - target_pt);
            if (!std::isfinite(r)) {
                continue;
            }
            corrs.push_back({x, n_tgt, r});
            residuals.push_back(r);
        }

        if (corrs.size() < MIN_ICP_POINTS) {
            break;
        }

        solved_any_level = true;

        // Robust weighting via MAD-scaled Huber to reduce outlier influence.
        const double mad = median_abs(residuals);
        double sigma = MAD_TO_SIGMA * mad;
        if (!std::isfinite(sigma) || sigma < 1e-4) {
            sigma = std::max(1e-3, 0.25 * max_corr_dist);
        }
        const double huber_delta = HUBER_K * sigma;

        // Solve one Gauss-Newton normal-equation step on SE(3).
        Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

        for (const auto& corr : corrs) {
            const double abs_r = std::abs(corr.r);
            const double w =
                (abs_r <= huber_delta || huber_delta <= 0.0) ? 1.0 : (huber_delta / abs_r);

            Eigen::Matrix<double, 6, 1> jacobian;
            jacobian.head<3>() = corr.x.cross(corr.n);
            jacobian.tail<3>() = corr.n;

            hessian.noalias() += w * (jacobian * jacobian.transpose());
            b.noalias() += -w * (jacobian * corr.r);
        }

        hessian.diagonal().array() += 1e-10;
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(hessian);
        if (ldlt.info() != Eigen::Success) {
            break;
        }
        const Eigen::Matrix<double, 6, 1> dx = ldlt.solve(b);
        if (ldlt.info() != Eigen::Success || !dx.allFinite()) {
            break;
        }

        PoseV delta;
        delta.set_rot(dx.head<3>());
        delta.set_trans(dx.tail<3>());
        current_pose = delta.exp() * current_pose;

        // Stop early once the incremental pose update becomes negligible.
        const double d_rot = dx.head<3>().norm();
        const double d_trans = dx.tail<3>().norm();
        if (d_rot < 1e-4 && d_trans < 1e-3) {
            break;
        }
    }

    if (!solved_any_level) {
        return initial_guess;
    }
    return current_pose;
}

namespace {

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// Internal implementation of the full alignment pipeline:
///  1. Consume pre-extracted, downsampled frame features in the
///     extrinsic-applied frame.
///  2. Run `initial_pairwise_alignment` (360 deg yaw sweep on a 0.5 m grid,
///     then +/-3 deg refinement on the adaptive fine grid) to obtain an
///     initial SE(3) alignment.
///  3. Refine with multi-pass ICP at decreasing correspondence distances
///     using point-to-plane when normals are available, otherwise
///     point-to-point.
///  4. Compute an overlap-based confidence score for the initial-vs-refined
///     guard and optionally return it to callers.
std::pair<PoseH, double> align_clouds_from_features_impl(const FrameFeatures& source,
                                                         const FrameFeatures& target,
                                                         const PoseH& initial_guess_pose,
                                                         bool compute_confidence) {
    if (static_cast<size_t>(source.points.rows()) < MIN_ICP_POINTS ||
        static_cast<size_t>(target.points.rows()) < MIN_ICP_POINTS) {
        logger().warn("Skip frame matching: src={} tgt={} (<{} points).", source.points.rows(),
                      target.points.rows(), MIN_ICP_POINTS);
        return {initial_guess_pose, 0.0};
    }

    // The features' points/normals are already in the plumbed (gravity-aligned)
    // frame, so alignment operates directly in that frame.
    const PoseH initial_guess_yaw_translation = project_pose_to_yaw_translation(initial_guess_pose);

    // initial_pairwise_alignment transforms its second argument into its first.
    // Reverse the feature arguments so the public pipeline carries
    // source_to_target_transform throughout.
    const PoseH initial_pose =
        initial_pairwise_alignment(target, source, initial_guess_yaw_translation);
    const bool use_normals = frame_features_have_normals(source.points, source.normals) &&
                             frame_features_have_normals(target.points, target.normals);
    const XYMatcherParams xy_params =
        choose_xy_matcher_params(target, source, initial_guess_yaw_translation);
    const bool compact_scene = xy_params.bound_m <= XY_COMPACT_SCENE_BOUND_M;
    const double icp_max_normal_angle_deg = compact_scene ? 10.0 : 20.0;

    // ICP always refines against the full prepared feature point sets
    // (source.points/target.points), not the sampled confidence subset.
    PoseH best_pose_plumb = initial_pose;
    constexpr double icp_distances[] = {2.0, 0.6, 0.25};
    for (double dist : icp_distances) {
        best_pose_plumb =
            use_normals
                ? point_to_plane_align(source.points, target.points, source.normals, target.normals,
                                       best_pose_plumb, dist, icp_max_normal_angle_deg)
                : point_to_point_align(source.points, target.points, best_pose_plumb, dist);
    }

    // All confidence paths use deterministic voxel-map downsampled sampling for large clouds.
    const ConfidenceQueryData confidence_query_data(source.points, target.points);
    const double initial_overlap_conf =
        xy_matching_confidence(source.points, target.points, source.normals, target.normals,
                               initial_pose, confidence_query_data);
    double refined_overlap_conf =
        xy_matching_confidence(source.points, target.points, source.normals, target.normals,
                               best_pose_plumb, confidence_query_data);
    if (refined_overlap_conf + 1e-6 < initial_overlap_conf) {
        best_pose_plumb = initial_pose;
        refined_overlap_conf = initial_overlap_conf;
    }

    double confidence = 0.0;
    if (compute_confidence) {
        confidence = refined_overlap_conf;
    }
    return {best_pose_plumb, confidence};
}

std::pair<Matrix4dR, double> align_clouds_from_frames_impl(
    const ouster::sdk::core::LidarFrame& source_frame,
    const ouster::sdk::core::LidarFrame& target_frame, const PoseH& initial_guess_pose,
    bool compute_confidence) {
    auto source_features_future = std::async(
        std::launch::async, [&source_frame]() { return prepare_frame_features(source_frame); });
    const FrameFeatures target = prepare_frame_features(target_frame);
    const FrameFeatures source = source_features_future.get();
    const auto result =
        align_clouds_from_features_impl(source, target, initial_guess_pose, compute_confidence);
    return {result.first.matrix(), result.second};
}

std::pair<Matrix4dR, double> align_clouds_from_point_clouds_impl(const FrameFeatures& source,
                                                                 const FrameFeatures& target,
                                                                 const PoseH& initial_guess_pose,
                                                                 bool compute_confidence) {
    const auto result =
        align_clouds_from_features_impl(source, target, initial_guess_pose, compute_confidence);
    return {result.first.matrix(), result.second};
}

std::pair<Matrix4dR, double> align_clouds_from_point_clouds_impl(
    Eigen::Ref<const ArrayX3dR> source_points, Eigen::Ref<const ArrayX3dR> target_points,
    const Matrix4dR& initial_guess, bool compute_confidence) {
    const FrameFeatures source =
        prepare_point_cloud_features(source_points, SOURCE_POINT_CLOUD_INPUTS);
    const FrameFeatures target =
        prepare_point_cloud_features(target_points, TARGET_POINT_CLOUD_INPUTS);
    return align_clouds_from_point_clouds_impl(source, target, PoseH(initial_guess),
                                               compute_confidence);
}

std::pair<Matrix4dR, double> align_clouds_from_point_clouds_impl(
    Eigen::Ref<const ArrayX3dR> source_points, Eigen::Ref<const ArrayX3dR> source_normals,
    Eigen::Ref<const ArrayX3dR> target_points, Eigen::Ref<const ArrayX3dR> target_normals,
    const Matrix4dR& initial_guess, bool compute_confidence) {
    const FrameFeatures source =
        prepare_point_cloud_features(source_points, source_normals, SOURCE_POINT_CLOUD_INPUTS);
    const FrameFeatures target =
        prepare_point_cloud_features(target_points, target_normals, TARGET_POINT_CLOUD_INPUTS);
    return align_clouds_from_point_clouds_impl(source, target, PoseH(initial_guess),
                                               compute_confidence);
}

Matrix4dR finalize_align_clouds_result(const std::pair<Matrix4dR, double>& result,
                                       double* confidence = nullptr) {
    if (confidence != nullptr) {
        *confidence = clamp01(result.second);
    }
    return result.first;
}

enum class MultiProjectionMode { XY_YAW, Z_ONLY, FULL };

struct MultiValidFrame {
    size_t frame_index = 0;
    FrameFeatures features;
};

struct MultiPairwiseMatch {
    size_t src = 0;
    size_t tgt = 0;
    Matrix4dR transform = Matrix4dR::Identity();
    double score = 0.0;
};

bool multi_frame_has_enough_features(const MultiValidFrame& multi_frame) {
    const size_t point_count = static_cast<size_t>(multi_frame.features.points.rows());
    if (point_count >= MIN_ICP_POINTS) {
        return true;
    }
    logger().warn(
        "align_clouds(FrameSet): sensor {} has only {} usable "
        "points; need at least {}. Returning its input extrinsic "
        "and skipping this sensor.",
        multi_frame.frame_index, point_count, MIN_ICP_POINTS);
    return false;
}

class MultiUnionFind {
   public:
    explicit MultiUnionFind(size_t num_elements) : parent_(num_elements), rank_(num_elements, 0) {
        for (size_t i = 0; i < num_elements; ++i) {
            parent_[i] = i;
        }
    }

    size_t find(size_t x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }

    bool unite(size_t a, size_t b) {
        size_t ra = find(a);
        size_t rb = find(b);
        if (ra == rb) {
            return false;
        }
        if (rank_[ra] < rank_[rb]) {
            std::swap(ra, rb);
        }
        parent_[rb] = ra;
        if (rank_[ra] == rank_[rb]) {
            ++rank_[ra];
        }
        return true;
    }

   private:
    std::vector<size_t> parent_;
    std::vector<int> rank_;
};

Matrix4dR pose_inverse_matrix(const Matrix4dR& pose) {
    Matrix4dR inverse = Matrix4dR::Identity();
    const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
    inverse.block<3, 3>(0, 0) = rotation.transpose();
    inverse.block<3, 1>(0, 3) = -(rotation.transpose() * translation);
    return inverse;
}

Matrix4dR project_pose_to_xy_yaw_matrix(const Matrix4dR& pose) {
    const double yaw = std::atan2(pose(1, 0), pose(0, 0));
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    Matrix4dR out = Matrix4dR::Identity();
    out(0, 0) = cos_yaw;
    out(0, 1) = -sin_yaw;
    out(1, 0) = sin_yaw;
    out(1, 1) = cos_yaw;
    out(0, 3) = pose(0, 3);
    out(1, 3) = pose(1, 3);
    return out;
}

Matrix4dR project_pose_for_multi(const Matrix4dR& pose, const Matrix4dR& initial_guess,
                                 MultiProjectionMode mode) {
    switch (mode) {
        case MultiProjectionMode::XY_YAW:
            return project_pose_to_xy_yaw_matrix(pose);
        case MultiProjectionMode::Z_ONLY: {
            Matrix4dR out = initial_guess;
            out(2, 3) = pose(2, 3);
            return out;
        }
        case MultiProjectionMode::FULL:
            return pose;
    }
    return pose;
}

double rotation_angle_deg_matrix(const Matrix4dR& pose) {
    const double trace = pose.block<3, 3>(0, 0).trace();
    const double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
    return std::acos(cos_theta) * 180.0 / M_PI;
}

/// Score the geometric consistency of a forward/reverse transform pair.
/// Returns 1 / (1 + trans_error + rot_error / scale), i.e. closer to 1
/// means forward * reverse ≈ Identity.
double reciprocal_consistency_penalty(const Matrix4dR& forward, const Matrix4dR& reverse) {
    const Matrix4dR consistency = forward * reverse;
    const double trans_error = consistency.block<3, 1>(0, 3).norm();
    const double rot_error_deg = rotation_angle_deg_matrix(consistency);
    return 1.0 / (1.0 + trans_error + rot_error_deg / MULTI_EDGE_ROTATION_ERROR_SCALE_DEG);
}

/// Score how closely @p candidate matches an @p implied transform derived
/// from triangulating through a third sensor. Returns 1 when identical.
double transform_consistency_penalty(const Matrix4dR& candidate, const Matrix4dR& implied) {
    const Matrix4dR delta = candidate * pose_inverse_matrix(implied);
    const double trans_error = delta.block<3, 1>(0, 3).norm();
    const double rot_error_deg = rotation_angle_deg_matrix(delta);
    return 1.0 / (1.0 + trans_error + rot_error_deg / MULTI_EDGE_ROTATION_ERROR_SCALE_DEG);
}

/// Run a pairwise alignment for a multi-sensor frame set.
/// Full mode runs the complete alignment pipeline;
/// XY_YAW mode runs only initial yaw+XY alignment;
/// Z_ONLY mode runs only 1-D Z histogram alignment.
std::pair<Matrix4dR, double> align_clouds_for_multi_mode(const FrameFeatures& source,
                                                         const FrameFeatures& target,
                                                         const PoseH& initial_guess_pose,
                                                         MultiProjectionMode projection_mode,
                                                         bool compute_score) {
    if (projection_mode == MultiProjectionMode::FULL) {
        const auto result =
            align_clouds_from_features_impl(source, target, initial_guess_pose, compute_score);
        return {result.first.matrix(), compute_score ? result.second : 1.0};
    }

    if (static_cast<size_t>(source.points.rows()) < MIN_ICP_POINTS ||
        static_cast<size_t>(target.points.rows()) < MIN_ICP_POINTS) {
        throw std::runtime_error(
            "align_clouds(FrameSet): pairwise match has too few usable "
            "points.");
    }

    PoseH pose_plumb = initial_guess_pose;
    double match_score = 1.0;
    if (projection_mode == MultiProjectionMode::XY_YAW) {
        double initial_score = 0.0;
        pose_plumb = initial_pairwise_alignment(
            target, source, project_pose_to_yaw_translation(initial_guess_pose), &initial_score);
        // Outdoor multi-sensor pairs can have zero overlap-confidence and a
        // flat raw correlation value even when the finite forward/reverse
        // initial transforms are geometrically consistent. Give finite initial
        // matches a small positive base score so graph selection can still
        // reject actual zero-score failures while preferring any positive
        // overlap-confidence edge.
        match_score = (std::isfinite(initial_score) && initial_score > 0.0)
                          ? initial_score
                          : MIN_FINITE_INITIAL_MATCH_SCORE;
    } else {
        const ArrayX3dR* target_normals =
            frame_features_have_normals(target.points, target.normals) ? &target.normals : nullptr;
        const TransHistogram target_hist_z = compute_translation_histogram(
            target.points, target.point_dist, Eigen::Vector3d::UnitZ(), target_normals);
        const auto z_aligned = align_translation_1d(
            source, target_hist_z, pose_plumb, Eigen::Vector3d::UnitZ(), MAX_Z_COARSE_CORRECTION_M);
        pose_plumb(2, 3) += z_aligned.translation;
    }

    double score = match_score;
    if (compute_score) {
        const ConfidenceQueryData confidence_query_data(source.points, target.points);
        const double overlap_score =
            xy_matching_confidence(source.points, target.points, source.normals, target.normals,
                                   pose_plumb, confidence_query_data);
        if (overlap_score > 0.0 || projection_mode != MultiProjectionMode::XY_YAW) {
            score = overlap_score;
        }
    }
    return {
        project_pose_for_multi(pose_plumb.matrix(), initial_guess_pose.matrix(), projection_mode),
        score};
}

std::pair<Matrix4dR, double> run_cached_pairwise_match(const std::vector<MultiValidFrame>& frames,
                                                       size_t local_src, size_t local_tgt,
                                                       const Matrix4dR& initial_guess,
                                                       MultiProjectionMode projection_mode,
                                                       bool compute_score) {
    // The graph stores source-frame-from-target-frame edges. Pass the graph
    // target as the matcher source so its source-to-target result has exactly
    // the direction required by the graph.
    const auto result =
        align_clouds_for_multi_mode(frames[local_tgt].features, frames[local_src].features,
                                    PoseH(initial_guess), projection_mode, compute_score);
    return {result.first, clamp01(result.second)};
}

/// Run all pairwise matches for @p edge_pairs in parallel.
/// When @p reciprocal_check is true, each edge also runs the reverse match
/// and the effective score is sqrt(forward * reverse) * consistency penalty.
std::vector<MultiPairwiseMatch> run_cached_pairwise_matches(
    const std::vector<MultiValidFrame>& frames,
    const std::vector<std::pair<size_t, size_t>>& edge_pairs,
    const std::vector<Matrix4dR>& current_global_poses, MultiProjectionMode projection_mode,
    bool reciprocal_check) {
    if (edge_pairs.empty()) {
        throw std::runtime_error("No pairwise matches were produced.");
    }

    std::vector<MultiPairwiseMatch> matches(edge_pairs.size());
    const bool compute_score =
        reciprocal_check || edge_pairs.size() > (frames.empty() ? 0 : frames.size() - 1);
    const auto compute_edge = [&](size_t edge_idx) {
        ScopedPairwiseInnerParallelDisable disable_nested_parallel;
        const auto& edge = edge_pairs[edge_idx];
        const size_t local_src = edge.first;
        const size_t local_tgt = edge.second;
        const Matrix4dR initial_guess =
            pose_inverse_matrix(current_global_poses[local_src]) * current_global_poses[local_tgt];
        const auto forward = run_cached_pairwise_match(frames, local_src, local_tgt, initial_guess,
                                                       projection_mode, compute_score);
        double effective_score = forward.second;
        if (reciprocal_check) {
            const Matrix4dR reverse_initial_guess =
                pose_inverse_matrix(current_global_poses[local_tgt]) *
                current_global_poses[local_src];
            // Intentionally reversed: src - tgt for the reciprocal check.
            const size_t rev_src = local_tgt;
            const size_t rev_tgt = local_src;
            const auto reverse = run_cached_pairwise_match(frames, rev_src, rev_tgt,
                                                           reverse_initial_guess, projection_mode,
                                                           /*compute_score=*/true);
            effective_score = std::sqrt(forward.second * reverse.second) *
                              reciprocal_consistency_penalty(forward.first, reverse.first);
        }
        effective_score = clamp01(effective_score);
        matches[edge_idx] =
            MultiPairwiseMatch{local_src, local_tgt, forward.first, effective_score};
    };

    run_parallel_indexed(edge_pairs.size(), /*allow_parallel=*/true, compute_edge);
    return matches;
}

/// Select a maximum-spanning-tree of pairwise matches, reweight each edge's
/// score by its triangle-consistency with all available third sensors, and
/// BFS-propagate the tree into a set of global poses anchored at
/// @p anchor_local_idx. Returns the global poses and the selected tree edges.
std::pair<std::vector<Matrix4dR>, std::vector<std::pair<size_t, size_t>>>
solve_consistent_multi_poses(const std::vector<MultiPairwiseMatch>& matches, size_t num_nodes,
                             size_t anchor_local_idx) {
    if (num_nodes == 0) {
        return {{}, {}};
    }

    std::vector<MultiPairwiseMatch> sorted_matches = matches;
    if (num_nodes >= 3 && sorted_matches.size() > num_nodes - 1) {
        std::vector<std::vector<int>> match_index(num_nodes, std::vector<int>(num_nodes, -1));
        for (size_t idx = 0; idx < matches.size(); ++idx) {
            if (matches[idx].src >= num_nodes || matches[idx].tgt >= num_nodes) {
                continue;
            }
            match_index[matches[idx].src][matches[idx].tgt] = static_cast<int>(idx);
            match_index[matches[idx].tgt][matches[idx].src] = static_cast<int>(idx);
        }

        const auto match_is_usable = [](const MultiPairwiseMatch& match) {
            return match.transform.allFinite() && std::isfinite(match.score) && match.score > 0.0;
        };
        const auto transform_between = [&](size_t src, size_t tgt) -> Matrix4dR {
            if (src == tgt) {
                return Matrix4dR::Identity();
            }
            const int idx = match_index[src][tgt];
            const MultiPairwiseMatch& match = matches[static_cast<size_t>(idx)];
            if (match.src == src && match.tgt == tgt) {
                return match.transform;
            }
            return pose_inverse_matrix(match.transform);
        };

        for (size_t idx = 0; idx < matches.size(); ++idx) {
            const MultiPairwiseMatch& match = matches[idx];
            if (!match_is_usable(match)) {
                continue;
            }

            std::vector<double> consistency_scores;
            consistency_scores.reserve(num_nodes > 2 ? num_nodes - 2 : 0);
            for (size_t via = 0; via < num_nodes; ++via) {
                if (via == match.src || via == match.tgt) {
                    continue;
                }
                const int src_via_idx = match_index[match.src][via];
                const int via_tgt_idx = match_index[via][match.tgt];
                if (src_via_idx < 0 || via_tgt_idx < 0) {
                    continue;
                }
                const MultiPairwiseMatch& src_via = matches[static_cast<size_t>(src_via_idx)];
                const MultiPairwiseMatch& via_tgt = matches[static_cast<size_t>(via_tgt_idx)];
                if (!match_is_usable(src_via) || !match_is_usable(via_tgt)) {
                    continue;
                }

                const Matrix4dR implied =
                    transform_between(match.src, via) * transform_between(via, match.tgt);
                consistency_scores.push_back(
                    clamp01(transform_consistency_penalty(match.transform, implied)));
            }

            if (consistency_scores.empty()) {
                continue;
            }
            // Median-only consistency reduces outlier influence when multiple via paths exist.
            std::sort(consistency_scores.begin(), consistency_scores.end());
            const size_t mid = consistency_scores.size() / 2;
            const double median_consistency =
                (consistency_scores.size() % 2 == 0)
                    ? 0.5 * (consistency_scores[mid - 1] + consistency_scores[mid])
                    : consistency_scores[mid];
            const double consistency = clamp01(median_consistency);
            sorted_matches[idx].score = clamp01(match.score) * consistency;
        }
    }
    // Tie-break by edge key for reproducible spanning-tree selection.
    std::sort(sorted_matches.begin(), sorted_matches.end(),
              [](const MultiPairwiseMatch& a, const MultiPairwiseMatch& b) {
                  if (a.score != b.score) {
                      return a.score > b.score;
                  }
                  if (a.src != b.src) {
                      return a.src < b.src;
                  }
                  return a.tgt < b.tgt;
              });

    std::vector<std::vector<std::pair<size_t, Matrix4dR>>> adjacency(num_nodes);
    MultiUnionFind union_find(num_nodes);
    std::vector<std::pair<size_t, size_t>> tree_edges;
    tree_edges.reserve(num_nodes > 0 ? num_nodes - 1 : 0);
    size_t finite_edge_count = 0;
    size_t positive_edge_count = 0;

    for (const auto& match : sorted_matches) {
        if (!match.transform.allFinite() || !std::isfinite(match.score)) {
            continue;
        }
        ++finite_edge_count;
        if (match.score <= 0.0) {
            continue;
        }
        ++positive_edge_count;
        if (!union_find.unite(match.src, match.tgt)) {
            continue;
        }
        adjacency[match.src].emplace_back(match.tgt, match.transform);
        adjacency[match.tgt].emplace_back(match.src, pose_inverse_matrix(match.transform));
        tree_edges.emplace_back(match.src, match.tgt);
        if (tree_edges.size() == num_nodes - 1) {
            break;
        }
    }

    if (tree_edges.size() != num_nodes - 1) {
        throw std::runtime_error(
            "Failed to connect all sensors from pairwise "
            "align_clouds matches. finite_edges=" +
            std::to_string(finite_edge_count) +
            ", positive_edges=" + std::to_string(positive_edge_count) + ", selected_edges=" +
            std::to_string(tree_edges.size()) + "/" + std::to_string(num_nodes - 1) + ".");
    }

    std::vector<Matrix4dR> global_poses(num_nodes, Matrix4dR::Identity());
    std::vector<bool> visited(num_nodes, false);
    std::deque<size_t> queue;
    visited[anchor_local_idx] = true;
    queue.push_back(anchor_local_idx);
    size_t visited_count = 1;
    while (!queue.empty()) {
        const size_t curr = queue.front();
        queue.pop_front();
        for (const auto& edge : adjacency[curr]) {
            const size_t neighbor = edge.first;
            if (visited[neighbor]) {
                continue;
            }
            global_poses[neighbor] = global_poses[curr] * edge.second;
            visited[neighbor] = true;
            ++visited_count;
            queue.push_back(neighbor);
        }
    }

    if (visited_count != num_nodes) {
        throw std::runtime_error("Spanning-tree propagation did not reach every sensor.");
    }
    return {global_poses, tree_edges};
}

void set_multi_aligned_extrinsics(std::vector<Matrix4dR>& extrinsics,
                                  const std::vector<Matrix4dR>& base_extrinsics,
                                  const std::vector<MultiValidFrame>& valid_frames,
                                  const std::vector<Matrix4dR>& global_poses) {
    for (size_t local_idx = 0; local_idx < valid_frames.size(); ++local_idx) {
        const size_t frame_idx = valid_frames[local_idx].frame_index;
        extrinsics[frame_idx] = global_poses[local_idx] * base_extrinsics[frame_idx];
    }
}

std::vector<std::pair<size_t, size_t>> all_multi_pairs(size_t num_nodes) {
    std::vector<std::pair<size_t, size_t>> pairs;
    pairs.reserve(num_nodes > 1 ? (num_nodes * (num_nodes - 1)) / 2 : 0);
    for (size_t src = 0; src < num_nodes; ++src) {
        for (size_t tgt = src + 1; tgt < num_nodes; ++tgt) {
            pairs.emplace_back(src, tgt);
        }
    }
    return pairs;
}

}  // namespace

Matrix4dR align_clouds(const ouster::sdk::core::LidarFrame& source_frame,
                       const ouster::sdk::core::LidarFrame& target_frame,
                       const Matrix4dR& initial_guess) {
    return finalize_align_clouds_result(
        align_clouds_from_frames_impl(source_frame, target_frame, PoseH(initial_guess),
                                      /*compute_confidence=*/false));
}

Matrix4dR align_clouds(const ouster::sdk::core::LidarFrame& source_frame,
                       const ouster::sdk::core::LidarFrame& target_frame, double& confidence) {
    return align_clouds(source_frame, target_frame, Matrix4dR::Identity(), confidence);
}

Matrix4dR align_clouds(const ouster::sdk::core::LidarFrame& source_frame,
                       const ouster::sdk::core::LidarFrame& target_frame,
                       const Matrix4dR& initial_guess, double& confidence) {
    return finalize_align_clouds_result(
        align_clouds_from_frames_impl(source_frame, target_frame, PoseH(initial_guess),
                                      /*compute_confidence=*/true),
        &confidence);
}

std::vector<Matrix4dR> align_clouds(const ouster::sdk::core::FrameSet& frames) {
    std::vector<Matrix4dR> aligned_extrinsics(frames.size(), Matrix4dR::Identity());
    if (frames.size() == 0) {
        return aligned_extrinsics;
    }
    if (!frames[0]) {
        throw std::invalid_argument(
            "align_clouds(FrameSet) requires frames[0] as the anchor frame.");
    }
    for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
        const auto& lidar_frame = frames[frame_idx];
        if (!lidar_frame) {
            logger().warn(
                "align_clouds(FrameSet): frame {} is missing; leaving "
                "output extrinsic as identity.",
                frame_idx);
            continue;
        }
        if (!lidar_frame->sensor_info) {
            logger().warn(
                "align_clouds(FrameSet): frame {} has null sensor_info; "
                "skipping.",
                frame_idx);
            continue;
        }
        aligned_extrinsics[frame_idx] = lidar_frame->sensor_info->sensor_to_body;
    }
    int anchor_first_col = 0;
    try {
        anchor_first_col = frames[0]->get_first_valid_column();
    } catch (const std::exception&) {
        logger().warn(
            "align_clouds(FrameSet): anchor sensor 0 has no valid "
            "columns; returning input extrinsics.");
        return aligned_extrinsics;
    }
    const PoseH anchor_reference_pose(frames[0]->get_column_pose(anchor_first_col));

    // Prepare per-frame features with bounded concurrency. Each frame's ground
    // segmentation, normals, dewarp and downsample are independent, but the work
    // can be CPU/RAM-heavy, so use the shared worker cap instead of launching one
    // async task per sensor.
    std::vector<FrameFeatures> prepared_features(frames.size());
    run_parallel_indexed(frames.size(), /*allow_parallel=*/true, [&](size_t frame_idx) {
        if (!frames[frame_idx]) {
            return;
        }
        prepared_features[frame_idx] =
            prepare_frame_features(*frames[frame_idx], &anchor_reference_pose);
    });

    std::vector<MultiValidFrame> valid_frames;
    valid_frames.reserve(frames.size());
    for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
        if (!frames[frame_idx]) {
            continue;
        }
        MultiValidFrame valid;
        valid.frame_index = frame_idx;
        valid.features = std::move(prepared_features[frame_idx]);
        if (!multi_frame_has_enough_features(valid)) {
            continue;
        }
        valid_frames.push_back(std::move(valid));
    }

    if (valid_frames.empty()) {
        logger().warn(
            "align_clouds(FrameSet): no frame has enough usable "
            "points; returning input extrinsics.");
        return aligned_extrinsics;
    }
    if (valid_frames.front().frame_index != 0) {
        logger().warn(
            "align_clouds(FrameSet): anchor sensor 0 has too few "
            "usable points; returning input extrinsics.");
        return aligned_extrinsics;
    }
    if (valid_frames.size() < 2) {
        return aligned_extrinsics;
    }

    const std::vector<Matrix4dR> base_extrinsics = aligned_extrinsics;
    if (valid_frames.size() == 2) {
        const auto result = align_clouds_from_features_impl(
            valid_frames[1].features, valid_frames[0].features, PoseH(Matrix4dR::Identity()),
            /*compute_confidence=*/true);
        if (!std::isfinite(result.second) || result.second <= 0.0) {
            logger().warn(
                "align_clouds(FrameSet): pairwise alignment confidence is "
                "zero; returning input extrinsics.");
            return aligned_extrinsics;
        }
        std::vector<Matrix4dR> global_poses{Matrix4dR::Identity(), result.first.matrix()};
        set_multi_aligned_extrinsics(aligned_extrinsics, base_extrinsics, valid_frames,
                                     global_poses);
        return aligned_extrinsics;
    }

    std::vector<Matrix4dR> current_global_poses(valid_frames.size(), Matrix4dR::Identity());
    const size_t anchor_local_idx = 0;
    const auto all_pairs = all_multi_pairs(valid_frames.size());
    const auto initial_matches = run_cached_pairwise_matches(
        valid_frames, all_pairs, current_global_poses, MultiProjectionMode::XY_YAW,
        /*reciprocal_check=*/true);
    auto initial_solution =
        solve_consistent_multi_poses(initial_matches, valid_frames.size(), anchor_local_idx);
    current_global_poses = std::move(initial_solution.first);
    set_multi_aligned_extrinsics(aligned_extrinsics, base_extrinsics, valid_frames,
                                 current_global_poses);

    std::vector<std::pair<size_t, size_t>> tree_edges = std::move(initial_solution.second);
    for (int pass = 0; pass < 2; ++pass) {
        const auto refine_matches = run_cached_pairwise_matches(
            valid_frames, tree_edges, current_global_poses, MultiProjectionMode::XY_YAW,
            /*reciprocal_check=*/false);
        auto refine_solution =
            solve_consistent_multi_poses(refine_matches, valid_frames.size(), anchor_local_idx);
        current_global_poses = std::move(refine_solution.first);
        set_multi_aligned_extrinsics(aligned_extrinsics, base_extrinsics, valid_frames,
                                     current_global_poses);
        tree_edges = std::move(refine_solution.second);
    }

    const auto z_matches = run_cached_pairwise_matches(
        valid_frames, tree_edges, current_global_poses, MultiProjectionMode::Z_ONLY,
        /*reciprocal_check=*/false);
    auto z_solution =
        solve_consistent_multi_poses(z_matches, valid_frames.size(), anchor_local_idx);
    current_global_poses = std::move(z_solution.first);
    set_multi_aligned_extrinsics(aligned_extrinsics, base_extrinsics, valid_frames,
                                 current_global_poses);

    const auto fine_matches = run_cached_pairwise_matches(
        valid_frames, tree_edges, current_global_poses, MultiProjectionMode::FULL,
        /*reciprocal_check=*/false);
    const auto fine_solution =
        solve_consistent_multi_poses(fine_matches, valid_frames.size(), anchor_local_idx);
    set_multi_aligned_extrinsics(aligned_extrinsics, base_extrinsics, valid_frames,
                                 fine_solution.first);

    return aligned_extrinsics;
}

Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                       Eigen::Ref<const core::ArrayX3dR> target_points,
                       const Matrix4dR& initial_guess) {
    return finalize_align_clouds_result(
        align_clouds_from_point_clouds_impl(source_points, target_points, initial_guess,
                                            /*compute_confidence=*/false));
}

Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                       Eigen::Ref<const core::ArrayX3dR> target_points,
                       const Matrix4dR& initial_guess, double& confidence) {
    return finalize_align_clouds_result(
        align_clouds_from_point_clouds_impl(source_points, target_points, initial_guess,
                                            /*compute_confidence=*/true),
        &confidence);
}

Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                       Eigen::Ref<const core::ArrayX3dR> target_points, double& confidence) {
    return align_clouds(source_points, target_points, Matrix4dR::Identity(), confidence);
}

Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                       Eigen::Ref<const core::ArrayX3dR> source_normals,
                       Eigen::Ref<const core::ArrayX3dR> target_points,
                       Eigen::Ref<const core::ArrayX3dR> target_normals,
                       const Matrix4dR& initial_guess) {
    return finalize_align_clouds_result(align_clouds_from_point_clouds_impl(
        source_points, source_normals, target_points, target_normals, initial_guess,
        /*compute_confidence=*/false));
}

Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                       Eigen::Ref<const core::ArrayX3dR> source_normals,
                       Eigen::Ref<const core::ArrayX3dR> target_points,
                       Eigen::Ref<const core::ArrayX3dR> target_normals, double& confidence) {
    return align_clouds(source_points, source_normals, target_points, target_normals,
                        Matrix4dR::Identity(), confidence);
}

Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                       Eigen::Ref<const core::ArrayX3dR> source_normals,
                       Eigen::Ref<const core::ArrayX3dR> target_points,
                       Eigen::Ref<const core::ArrayX3dR> target_normals,
                       const Matrix4dR& initial_guess, double& confidence) {
    return finalize_align_clouds_result(
        align_clouds_from_point_clouds_impl(source_points, source_normals, target_points,
                                            target_normals, initial_guess,
                                            /*compute_confidence=*/true),
        &confidence);
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
