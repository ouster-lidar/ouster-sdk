#pragma once

#include <ouster/core/eigen_hash.h>
#include <ouster/core/typedefs.h>
#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "visibility.h"

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

// ---------------------------------------------------------------------------
// PointDefaultValue: constructs the "not-found" sentinel returned by
// get_closest_neighbor when no neighbor is within max_distance_sq. For Eigen
// types the sentinel is the zero vector; for other types (e.g. IndexedPoint)
// default construction is used.
// ---------------------------------------------------------------------------

template <typename T>
struct PointDefaultValue {
    static T make(Eigen::Index /*n*/) {
        return T{};
    }
};

template <>
struct PointDefaultValue<Eigen::Vector3d> {
    static Eigen::Vector3d make(Eigen::Index) {
        return Eigen::Vector3d::Zero();
    }
};
template <>
struct PointDefaultValue<Eigen::VectorXd> {
    static Eigen::VectorXd make(Eigen::Index n) {
        return Eigen::VectorXd::Zero(n);
    }
};

}  // namespace impl

/// Default per-voxel bucket. Stores only a collection of points with no
/// additional metadata. sizeof(DefaultVoxelBucket<T>) == sizeof(std::vector<T>).
template <typename T>
struct OUSTER_API_CLASS DefaultVoxelBucket {
    /// Concrete container type used to store the points in this bucket.
    using points_type = std::vector<T>;
    /// Points admitted into this voxel, up to max_points_per_voxel.
    std::vector<T> points;

    /// Reserve storage for the maximum expected number of points per voxel.
    /// @param[in] map The owning VoxelHashMap; queried for max_points_per_voxel.
    /// @param[out] b  The bucket to initialise.
    template <typename MapType>
    static void init_voxel(const MapType& map, DefaultVoxelBucket<T>& b) noexcept {
        b.points.reserve(map.max_points_per_voxel());
    }

    /// Invoke @p func for every point stored in the bucket.
    /// @param[in] b    Bucket whose points are visited.
    /// @param[in] func Callable invoked with each stored point.
    template <typename MapType, typename Func>
    static void visit_voxel(const MapType& /*map*/, const DefaultVoxelBucket<T>& b, Func&& func) {
        for (const auto& p : b.points) func(p);
    }

    /// Return the number of points currently stored in the bucket.
    /// @param[in] b The bucket to query.
    /// @return Number of points in the bucket.
    template <typename MapType>
    static std::size_t point_count(const MapType& /*map*/,
                                   const DefaultVoxelBucket<T>& b) noexcept {
        return b.points.size();
    }
};

/// Per-voxel bucket that accumulates all inserted points into a single running
/// sum. Pair with accumulate_strategy so that every inserted point contributes
/// to the sum, and with pointcloud() / pointcloud_vector() to retrieve the
/// per-voxel average (sum / count).
template <typename T>
struct OUSTER_API_CLASS AveragePointBucket {
    /// Underlying point type stored in this bucket.
    using point_type = T;
    /// Running sum of all points inserted into this voxel.
    T point;
    /// Number of points accumulated so far; 0 means the bucket is empty.
    std::size_t count;

    /// Zero-initialise the running sum to the correct size and reset the count.
    /// @param[in] map The owning VoxelHashMap; queried for point_cols.
    /// @param[out] b  The bucket to initialise.
    template <typename MapType>
    static void init_voxel(const MapType& map, AveragePointBucket<T>& b) noexcept {
        // TODO[UN]: Note that for VectorXd, the point cols isn't known at compile time
        // This is why we invoke map.point_cols() here, but maybe there is a better way
        // to do this? The idea here is to initialize the point directly to zero here such
        // so that we don't need to check for bucket.count == 0 during accumulation
        b.point = impl::PointDefaultValue<T>::make(map.point_cols());
        b.count = 0;
    }

    /// Invoke @p func with the per-voxel average if the bucket meets the minimum
    /// points threshold required by the map.
    /// @param[in] map  The owning VoxelHashMap; queried for min_pts_threshold.
    /// @param[in] b    Bucket holding the running sum and count.
    /// @param[in] func Callable invoked with the per-voxel average point.
    template <typename MapType, typename Func>
    static void visit_voxel(const MapType& map, const AveragePointBucket<T>& b, Func&& func) {
        if (b.count >= map.min_pts_threshold()) func(b.point / static_cast<double>(b.count));
    }

    /// Return 1 if the bucket meets the minimum points threshold, 0 otherwise.
    /// @param[in] map The owning VoxelHashMap; queried for min_pts_threshold.
    /// @param[in] b   The bucket to query.
    /// @return 1 if count >= min_pts_threshold, 0 otherwise.
    template <typename MapType>
    static std::size_t point_count(const MapType& map, const AveragePointBucket<T>& b) noexcept {
        return b.count >= map.min_pts_threshold() ? 1u : 0u;
    }
};

/// Per-voxel bucket that jointly accumulates 3-D positions and pre-normalized
/// normals. Pair with accumulate_point_normal_strategy for insertion (expects
/// a 6-D packed input [x,y,z,nx,ny,nz]) and use pointcloud() /
/// pointcloud_vector() to retrieve the averaged position and re-normalized
/// average normal direction packed into a 6-D output vector.
struct OUSTER_API_CLASS PointNormalBucket {
    /// Running sum of positions.
    Eigen::Vector3d point_sum;
    /// Running sum of pre-normalized normals.
    Eigen::Vector3d normal_sum;
    /// Number of points accumulated so far; 0 means the bucket is empty.
    std::size_t count;

    /// Zero-initialise both running sums and reset the count.
    /// @param[out] b The bucket to initialise.
    template <typename MapType>
    static void init_voxel(const MapType& /*map*/, PointNormalBucket& b) noexcept {
        b.point_sum = Eigen::Vector3d::Zero();
        b.normal_sum = Eigen::Vector3d::Zero();
        b.count = 0;
    }

    /// Invoke @p func with a packed [avg_position | renormalized_avg_normal] vector
    /// if the bucket meets the minimum points threshold.
    /// @param[in] map  The owning VoxelHashMap; queried for min_pts_threshold.
    /// @param[in] b    Bucket holding the accumulated position and normal sums.
    /// @param[in] func Callable invoked with the packed 6-D output vector.
    template <typename MapType, typename Func>
    static void visit_voxel(const MapType& map, const PointNormalBucket& b, Func&& func) {
        if (b.count >= map.min_pts_threshold()) {
            const double n_len = b.normal_sum.norm();
            if (n_len <= 1e-12) return;
            const Eigen::Index half = b.point_sum.size();
            Eigen::VectorXd out(half + b.normal_sum.size());
            out.head(half) = b.point_sum / static_cast<double>(b.count);
            out.tail(b.normal_sum.size()) = b.normal_sum / n_len;
            func(out);
        }
    }

    /// Return 1 if the bucket meets the minimum points threshold, 0 otherwise.
    /// @param[in] map The owning VoxelHashMap; queried for min_pts_threshold.
    /// @param[in] b   The bucket to query.
    /// @return 1 if count >= min_pts_threshold, 0 otherwise.
    template <typename MapType>
    static std::size_t point_count(const MapType& map, const PointNormalBucket& b) noexcept {
        return b.count >= map.min_pts_threshold() ? 1u : 0u;
    }
};

/// A point that bundles an Eigen position with a 32-bit source index. Use as
/// the PointType of a VoxelHashMap when the original position of the surviving
/// voxel representative must be recoverable after downsampling.
///
/// SizeAtCompileTime mirrors the inner Eigen type so that VoxelHashMap::
/// point_cols() and the num_attributes constructor check work uniformly for
/// IndexedPoint without special-casing.
///
/// @tparam T  Underlying Eigen point type (e.g., Eigen::Vector3d).
template <typename T>
struct IndexedPoint {
    /// Underlying Eigen point type (e.g., Eigen::Vector3d).
    using point_type = T;
    /// Mirrors T::SizeAtCompileTime so VoxelHashMap works uniformly for IndexedPoint.
    static const int SizeAtCompileTime = T::SizeAtCompileTime;
    /// Spatial position.
    T point;
    /// Zero-based index of this point in the original source array.
    std::uint32_t index = 0;
};

namespace impl {

// ---------------------------------------------------------------------------
// is_eigen_vector: true for plain Eigen vector types (Vector3d, VectorXd),
// false for structured types like IndexedPoint. Used to SFINAE the
// add_points(matrix) overload out of existence for non-Eigen PointTypes.
// Detection is via operator[] — Eigen vectors support it; IndexedPoint does not.
// ---------------------------------------------------------------------------

template <typename T, typename = void>
struct is_eigen_vector : std::false_type {};
template <typename T>
struct is_eigen_vector<T, decltype(void(std::declval<const T&>()[0]))> : std::true_type {};

// ---------------------------------------------------------------------------
// spatial_view: extracts the 3-D spatial position from any supported point
// type. Used by point_to_voxel, first_n_point, and get_closest_neighbor
// so that all three work uniformly on plain Eigen vectors and on IndexedPoint.
// ---------------------------------------------------------------------------

/// For a fixed-size 3-D Eigen vector the spatial view IS the vector.
inline const Eigen::Vector3d& spatial_view(const Eigen::Vector3d& p) {
    return p;
}
/// For a dynamic-size Eigen vector the spatial view is the first 3 elements.
inline auto spatial_view(const Eigen::VectorXd& p) -> decltype(p.head<3>()) {
    return p.head<3>();
}
/// For IndexedPoint delegate to the spatial view of the inner point.
template <typename T>
inline auto spatial_view(const IndexedPoint<T>& p) -> decltype(spatial_view(p.point)) {
    return spatial_view(p.point);
}

// ---------------------------------------------------------------------------
// point_view: returns the Eigen part of a point that is written into
// the ArrayXXdR produced by pointcloud(). For plain Eigen types this is the
// whole point; for IndexedPoint the 32-bit index is dropped.
// ---------------------------------------------------------------------------

inline const Eigen::Vector3d& point_view(const Eigen::Vector3d& p) {
    return p;
}
inline const Eigen::VectorXd& point_view(const Eigen::VectorXd& p) {
    return p;
}
template <typename T>
inline const T& point_view(const IndexedPoint<T>& p) {
    return p.point;
}

// Selects the best hash for a voxel index type. Specialized for
// Eigen::Vector3i; everything else falls through to std::hash.
template <typename T>
struct DefaultVoxelHash {
    using type = std::hash<T>;
};
template <>
struct DefaultVoxelHash<Eigen::Vector3i> {
    using type = ::ouster::sdk::core::Vector3iHash;
};

// ---------------------------------------------------------------------------
// bucket_dispatch: tags used by random_selection_strategy to select between
// DefaultVoxelBucket-style (has ::points_type) and AveragePointBucket-style
// (has ::point_type but no ::points_type) buckets. C++14 SFINAE; no if constexpr.
// ---------------------------------------------------------------------------
struct has_points_tag {};
struct no_points_tag {};

template <typename VB, typename = void>
struct bucket_dispatch {
    using tag = no_points_tag;
};
template <typename VB>
struct bucket_dispatch<VB, decltype(void(std::declval<typename VB::points_type>()))> {
    using tag = has_points_tag;
};

/// Built-in insertion strategy: admits a point if the voxel is below the
/// capacity cap and no existing point lies within map_resolution_sq of it.
/// Distance is computed on the spatial part only (via impl::spatial_view) so
/// the strategy works uniformly for plain Eigen vectors and for IndexedPoint.
struct OUSTER_API_CLASS first_n_point {
    template <typename VoxelBucket, typename PointType>
    static void apply(VoxelBucket& bucket, const PointType& point, double map_resolution_sq,
                      std::size_t max_points_per_voxel) {
        auto& voxel_points = bucket.points;
        const auto pos = impl::spatial_view(point);
        if (voxel_points.size() == max_points_per_voxel ||
            std::any_of(voxel_points.cbegin(), voxel_points.cend(), [&](const PointType& vp) {
                return (impl::spatial_view(vp) - pos).squaredNorm() < map_resolution_sq;
            })) {
            return;
        }
        voxel_points.emplace_back(point);
    }
};

/// Insertion strategy for AveragePointBucket: accumulates every inserted point
/// into the bucket's running sum and increments its count. When
/// pointcloud() or pointcloud_vector() is called the map divides the sum by
/// the count to produce the per-voxel average.
struct OUSTER_API_CLASS accumulate_strategy {
    template <typename VoxelBucket, typename PointType>
    static void apply(VoxelBucket& bucket, const PointType& point, double /*map_resolution_sq*/,
                      std::size_t /*max_points_per_voxel*/) {
        bucket.point += point;
        ++bucket.count;
    }
};

/// Insertion strategy for PointNormalBucket: splits the input point (a packed
/// [position | normal] vector) into equal halves, accumulating positions into
/// point_sum and pre-normalized normals into normal_sum. When
/// pointcloud() / pointcloud_vector() is called, visit_voxel averages the
/// position and re-normalizes the summed normal direction.
struct OUSTER_API_CLASS accumulate_point_normal_strategy {
    template <typename VoxelBucket, typename PointType>
    static void apply(VoxelBucket& bucket, const PointType& point, double /*map_resolution_sq*/,
                      std::size_t /*max_points_per_voxel*/) {
        const Eigen::Index half = point.size() / 2;
        bucket.point_sum += point.head(half);
        bucket.normal_sum += point.tail(half);
        ++bucket.count;
    }
};

}  // namespace impl

/// Spatial hash map for organizing 3D points into voxels.
///
/// Template-based voxel hash map supporting optional per-voxel metadata
/// (e.g., observation counts, timestamps). Uses efficient robin_map for
/// fast O(1) average-case lookups and insertions.
///
/// @tparam VoxelIndex      Type for voxel coordinates, e.g., Eigen::Vector3i
/// @tparam PointType       Type for 3D points, e.g., Eigen::Vector3d or
///                         Eigen::VectorXd
/// @tparam VoxelBucketType Per-voxel bucket type; DefaultVoxelBucket<PointType>
///                         (default) for minimal memory footprint, or
///                         use a custom bucket type with other metadata such as
///                         observation counts and timestamps. Custom bucket types
///                          are supported as long as they expose a `points_type`.
template <typename VoxelIndex = Eigen::Vector3i, typename PointType = Eigen::Vector3d,
          typename VoxelBucketType = DefaultVoxelBucket<PointType>,
          typename InsertionStrategy = impl::first_n_point>
struct OUSTER_API_CLASS VoxelHashMap {
    /// Bucket type alias – the type stored at each occupied voxel.
    using VoxelBucket = VoxelBucketType;

    /// Point type alias for template consistency.
    using point_type = PointType;

    /// Constructor for the voxel hash map.
    ///
    /// @param[in] voxel_size            Size of each voxel in coordinate units.
    /// @param[in] max_distance          Maximum distance to retain points from
    /// a reference.
    /// @param[in] max_points_per_voxel  Maximum number of points allowed per
    /// voxel.
    /// @param[in] min_pts_threshold     Minimum number of points required in a voxel to be
    /// considered valid.
    /// @param[in] num_attributes        Number of additional attributes for
    /// dynamic-size points.
    VoxelHashMap(double voxel_size, double max_distance = 100.0,
                 std::size_t max_points_per_voxel = 20, std::size_t min_pts_threshold = 1,
                 std::size_t num_attributes = 0);

    /// Check if the hash map is empty.
    /// @return True if no voxels are present, false otherwise.
    inline bool empty() const {
        return map_.empty();
    }
    /// Clear all voxels from the map.
    inline void clear() {
        map_.clear();
    }

    /// Insert points and trim voxels that are far from @p position.
    /// @param[in] points            Vector of points to insert.
    /// @param[in] position   Reference position for distance trimming.
    void update(const std::vector<PointType>& points, const PointType& position);

    /// Insert points into the map.
    ///
    /// @param[in] points    Vector of points to insert.
    void add_points(const std::vector<PointType>& points);

    /// Insert points from an Eigen matrix into the map.
    ///
    /// Only participates in overload resolution when PointType is a plain
    /// Eigen vector (e.g., Vector3d, VectorXd). Not available for structured
    /// point types such as IndexedPoint; use add_point() or add_points(vector)
    /// for those.
    ///
    /// @param[in] points    Row-major matrix where each row is a point.
    template <typename PT = PointType,
              typename std::enable_if<impl::is_eigen_vector<PT>::value, int>::type = 0>
    void add_points(const Eigen::Ref<const ArrayXXdR>& points) {
        if (points.cols() != point_cols()) {
            throw std::invalid_argument(
                "VoxelHashMap::add_points received unexpected point dimension");
        }
        for (Eigen::Index row = 0; row < points.rows(); ++row) {
            const PointType point = points.matrix().row(row).transpose();
            add_point(point);
        }
    }

    /// Remove voxels far from a specified location.
    /// @param[in] origin Location to measure distance from.
    void remove_voxels_far_from_location(const PointType& origin);
    /// Extract voxels far from a specified location.
    /// @param[in] origin Location to measure distance from.
    /// @return Matrix of points from extracted voxels.
    ArrayXXdR extract_voxels_far_from_location(const PointType& origin);

    /// Get a complete pointcloud as a vector.
    /// @return Vector containing all points from all voxels.
    std::vector<PointType> pointcloud_vector() const;
    /// Get a complete pointcloud as an Eigen matrix.
    /// @return Matrix where each column is a point (3 rows + attributes).
    ArrayXXdR pointcloud() const;

    /// Find the closest point in the map to a query point.
    /// @param[in] query             The query point.
    /// @param[in] max_distance_sq   Maximum squared distance to search.
    /// @return Tuple of (closest_point, squared_distance). If no point is
    ///         found within max_distance_sq, squared_distance will be >=
    ///         max_distance_sq.
    std::tuple<PointType, double> get_closest_neighbor(
        const PointType& query, double max_distance_sq = std::numeric_limits<double>::max()) const;

    /// Return the bucket for the voxel containing @p point, or a default
    /// (zero-initialized) bucket if no such voxel exists.
    /// @param[in] point The query point.
    /// @return Const reference to the voxel bucket, or a default bucket.
    const VoxelBucketType& get_voxel_bucket(const PointType& point) const;

    /// Convert a point to its voxel index.
    /// Voxelization is based on the spatial (3-D position) part only; any
    /// extra attributes or index fields (as in IndexedPoint) are ignored.
    /// @param[in] point           The query point.
    /// @param[in] inv_voxel_size  Inverse of voxel size (1.0 / voxel_size).
    /// @return Voxel index containing the point.
    static inline VoxelIndex point_to_voxel(const PointType& point, const double inv_voxel_size) {
        const auto pos = impl::spatial_view(point);
        return VoxelIndex(
            static_cast<typename VoxelIndex::Scalar>(std::floor(pos[0] * inv_voxel_size)),
            static_cast<typename VoxelIndex::Scalar>(std::floor(pos[1] * inv_voxel_size)),
            static_cast<typename VoxelIndex::Scalar>(std::floor(pos[2] * inv_voxel_size)));
    }

    /// Returns the row width of a serialized point.
    /// @return the row width of a serialized.
    Eigen::Index point_cols() const {
        return PointType::SizeAtCompileTime == Eigen::Dynamic
                   ? static_cast<Eigen::Index>(3 + num_attributes_)
                   : static_cast<Eigen::Index>(PointType::SizeAtCompileTime);
    }

    /// Insert a single point into the map.
    /// @param[in] point The point to insert.
    void add_point(const PointType& point);

    /// Per-map insertion strategy instance (holds any mutable state such as RNG).
    InsertionStrategy strategy_;

   private:
    /// Convert a point to its voxel index using the cached inverse voxel size.
    /// @param[in] point The query point.
    /// @return Voxel index containing the point.
    VoxelIndex point_to_voxel(const PointType& point) const {
        return point_to_voxel(point, inv_voxel_size_);
    }

    double voxel_size_;
    double max_distance_;
    std::size_t max_points_per_voxel_;
    std::size_t min_pts_threshold_;
    std::size_t num_attributes_;
    tsl::robin_map<VoxelIndex, VoxelBucketType, typename impl::DefaultVoxelHash<VoxelIndex>::type>
        map_;
    // cached values
    double map_resolution_sq_;
    double inv_voxel_size_;

    /* TODO[UN]: AD HOC PUBLIC MEMBERS for now..
    in the future let the insertion and summarization strategies define these values */
   public:
    /// Return the maximum number of points allowed per voxel.
    /// @return Maximum points per voxel as set at construction.
    std::size_t max_points_per_voxel() const {
        return max_points_per_voxel_;
    }

    /// Return the minimum number of raw insertions required for a voxel to appear in the output.
    /// @return Minimum points threshold as set at construction.
    std::size_t min_pts_threshold() const {
        return min_pts_threshold_;
    }
};

// The "plain" aliases (VoxelHashMap3d/Xd) keep the original 24-byte bucket
// footprint and are what SLAM and the CLI map exporter use today.
using VoxelHashMap3d =
    VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>>;
using VoxelHashMapXd =
    VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>>;
using AverageVoxelHashMap3d =
    VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, AveragePointBucket<Eigen::Vector3d>,
                 impl::accumulate_strategy>;
using AverageVoxelHashMapXd =
    VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, AveragePointBucket<Eigen::VectorXd>,
                 impl::accumulate_strategy>;

using AccumulateAttribVoxelHashMap =
    VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, AveragePointBucket<Eigen::VectorXd>,
                 impl::accumulate_strategy>;

/// VoxelHashMap for joint position+normal averaging. Expects 6-D packed input
/// [x,y,z,nx,ny,nz] from accumulate_point_normal_strategy; pointcloud() returns
/// an Nx6 matrix [avg_position | renormalized_avg_normal].
using PointNormalVoxelHashMap3d = VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, PointNormalBucket,
                                               impl::accumulate_point_normal_strategy>;

namespace impl {

/// Insertion strategy for IndexedPoint buckets. On every apply() call a
/// per-strategy monotonic counter is incremented and written into the inserted
/// point's index field so that each point carries its global insertion-order
/// rank (0-based across all add_point / add_points calls on this map instance).
///
/// When a voxel bucket is not yet full the point is simply appended. When
/// the bucket is full, an XOR-shift fast_rand picks a uniform random slot in
/// [0, max_points_per_voxel) (Lemire range reduction, same as voxel_downsample)
/// and replaces the point there.
///
/// The strategy stores two public fields so callers can reseed the RNG or
/// inspect how many points have been offered to the map:
///   rng_state  — XOR-shift PRNG seed (default 42).
///   call_count — total insertions attempted so far (default 0).
struct OUSTER_API_CLASS indexed_reservoir_strategy {
    std::uint32_t rng_state = 42u;
    std::uint64_t call_count = 0u;

    template <typename VoxelBucket, typename PointType>
    void apply(VoxelBucket& bucket, const PointType& point, double /*map_resolution_sq*/,
               std::size_t max_points_per_voxel) {
        PointType stamped = point;
        stamped.index = static_cast<std::uint32_t>(call_count++);

        auto& pts = bucket.points;
        if (pts.size() < max_points_per_voxel) {
            pts.emplace_back(std::move(stamped));
        } else {
            const std::size_t j = static_cast<std::size_t>(
                (static_cast<std::uint64_t>(fast_rand()) * max_points_per_voxel) >> 32);
            pts[j] = std::move(stamped);
        }
    }

   private:
    std::uint32_t fast_rand() {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 17;
        rng_state ^= rng_state << 5;
        return rng_state;
    }
};

/// Reservoir-sampling insertion strategy. Compatible with both
/// DefaultVoxelBucket<T> (where it performs proper k-slot reservoir sampling)
/// and AveragePointBucket<T> (where it performs single-slot Algorithm R,
/// maintaining the visit_voxel invariant.
///
/// Unlike indexed_reservoir_strategy, no IndexedPoint wrapper is required —
/// works directly with plain Eigen vector point types.
struct OUSTER_API_CLASS random_selection_strategy {
    std::uint32_t rng_state = 42u;

    template <typename VoxelBucket, typename PointType>
    void apply(VoxelBucket& bucket, const PointType& point, double /*map_resolution_sq*/,
               std::size_t max_points_per_voxel) {
        apply_impl(bucket, point, max_points_per_voxel,
                   typename impl::bucket_dispatch<VoxelBucket>::tag{});
    }

   private:
    std::uint32_t fast_rand() noexcept {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 17;
        rng_state ^= rng_state << 5;
        return rng_state;
    }

    // DefaultVoxelBucket path: reservoir sampling of up to max_points_per_voxel points.
    template <typename VoxelBucket, typename PointType>
    void apply_impl(VoxelBucket& bucket, const PointType& point, std::size_t max_points_per_voxel,
                    impl::has_points_tag) {
        auto& pts = bucket.points;
        if (pts.size() < max_points_per_voxel) {
            pts.emplace_back(point);
        } else {
            // j here represents the index of the point to be replaced (victim)
            const std::size_t j = static_cast<std::size_t>(
                (static_cast<std::uint64_t>(fast_rand()) * max_points_per_voxel) >> 32);
            pts[j] = point;
        }
    }

    // AveragePointBucket path: Algorithm R (k=1). Maintains b.point == selected * b.count
    // so that visit_voxel (which divides b.point / b.count) returns the selected point.
    // max_points_per_voxel is ignored; the bucket holds exactly one representative.
    template <typename VoxelBucket, typename PointType>
    void apply_impl(VoxelBucket& bucket, const PointType& point,
                    std::size_t /*max_points_per_voxel*/, impl::no_points_tag) {
        ++bucket.count;
        // j in [0, count); probability of replace == 1/count (Algorithm R).
        // When count==1 fast_rand()*1>>32 == 0 always, so first point is always selected.
        const std::size_t j = static_cast<std::size_t>(
            (static_cast<std::uint64_t>(fast_rand()) * bucket.count) >> 32);
        if (j == 0) {
            bucket.point = point;
        }
    }
};

}  // namespace impl

/// VoxelHashMap that stores IndexedPoint per voxel. First-in wins within each
/// voxel (spatial part only for the min-distance check). pointcloud_vector()
/// returns the surviving IndexedPoint objects so the original source index is
/// recoverable. pointcloud() serialises only the 3-D position (index dropped).
using IndexedVoxelHashMap3d =
    VoxelHashMap<Eigen::Vector3i, IndexedPoint<Eigen::Vector3d>,
                 DefaultVoxelBucket<IndexedPoint<Eigen::Vector3d>>, impl::first_n_point>;
/// VoxelHashMap that stores IndexedPoint per voxel with random replacement.
/// Uses indexed_reservoir_strategy: the index field records global insertion
/// order; when a voxel is full, a random existing point is replaced. The
/// strategy_ member is public so callers can reseed rng_state or read
/// call_count after construction.
using IndexedReservoirHashMap3d = VoxelHashMap<Eigen::Vector3i, IndexedPoint<Eigen::Vector3d>,
                                               DefaultVoxelBucket<IndexedPoint<Eigen::Vector3d>>,
                                               impl::indexed_reservoir_strategy>;

std::pair<std::vector<Eigen::Vector3d>, std::vector<std::uint32_t>> voxel_downsample(
    const std::vector<Eigen::Vector3d>& frame, const double voxel_size);

enum class VoxelDownsampleStrategy {
    FIRST_N_POINT,  /// First n points in a voxel are kept (where n is max_points_per_voxel)
    AVERAGE_POINT,  /// Compute the average of all points in a voxel
    RANDOM,         /// Reservoir-sample one random point per voxel (uniform, unbiased)
};

OUSTER_API_FUNCTION ArrayX3dR voxel_downsample_3d(
    const Eigen::Ref<const ArrayX3dR>& frame, const double voxel_size,
    const std::size_t max_points_per_voxel = 1, const std::size_t min_pts_threshold = 1,
    const VoxelDownsampleStrategy strategy = VoxelDownsampleStrategy::RANDOM);

OUSTER_API_FUNCTION ArrayXXdR voxel_downsample_xd(
    const Eigen::Ref<const ArrayXXdR>& frame, const double voxel_size,
    const std::size_t max_points_per_voxel = 1, const std::size_t min_pts_threshold = 1,
    const VoxelDownsampleStrategy strategy = VoxelDownsampleStrategy::RANDOM);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
