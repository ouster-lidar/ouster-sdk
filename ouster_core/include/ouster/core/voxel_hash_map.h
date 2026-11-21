#pragma once

#include <ouster/core/attribute_layout.h>
#include <ouster/core/typedefs.h>
#include <ouster/core/visibility.h>

#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

/// @brief Select the internal storage representation to use for storing points in voxels.
enum class VHMInternalStorageMode : std::uint8_t {
    AUTO,  ///< Selects the best (smallest) internal storage representation for the voxel size
    ABS,   ///< Directly store the absolute point representation
    FX16,  ///< Store points as a 16-bit offset from the voxel origin internally
    FX8    ///< Store points as an 8-bit offset from the voxel origin internally
};

enum class VoxelDownsampleStrategy {
    FIRST_N_POINT,  /// First n points in a voxel are kept (where n is max_points_per_voxel)
    /// Last n points in a voxel are kept (where n is max_points_per_voxel):
    /// once a voxel is full, an arriving point evicts whichever kept point
    /// has been there longest instead of being dropped.
    LAST_N_POINT,
    AVERAGE_POINT,  /// Compute the average of all points in a voxel
    RANDOM,         /// Reservoir-sample one random point per voxel (uniform, unbiased)
    /// Keep the up to max_points_per_voxel points with the smallest value of
    /// one scalar attribute, named by index (see VoxelHashMap::create()'s
    /// attribute_index). Needs a non-empty attribute_layout; there is nothing
    /// to rank positions-only points by.
    SMALLEST,
    /// Like SMALLEST, but keeps the points with the largest value instead.
    LARGEST,
};

/// Sentinel meaning "no attribute selected": out of range for any
/// AttributeLayout, so passing it to impl::extremum_strategy's (layout,
/// attribute_index) constructor always throws std::invalid_argument.
constexpr std::size_t NO_ATTRIBUTE_INDEX = std::numeric_limits<std::size_t>::max();

/// A spatial hash map organizing 3D points, and optionally per-point
/// attributes, into cubic voxels.
///
/// Each VoxelDownsampleStrategy is implemented by its own internal map type:
/// a different per-voxel bucket and insertion strategy, specialized again for
/// positions-only points or points carrying attributes. create() picks the
/// right one and returns this proxy, which forwards every call to it. Callers
/// choose a strategy at runtime and never name the implementation types.
///
/// - FIRST_N_POINT keeps the first max_points_per_voxel points to reach a voxel,
///   skipping any that land too close to one already kept.
/// - LAST_N_POINT is the same, but keeps the most recently admitted
///   max_points_per_voxel points instead: a full voxel evicts its oldest.
/// - AVERAGE_POINT keeps each voxel's average position and attributes.
/// - RANDOM reservoir-samples max_points_per_voxel points per voxel.
/// - SMALLEST / LARGEST keep the points with the smallest / largest value of
///   one scalar attribute (see attribute_index), needing a non-empty
///   attribute_layout.
///
/// With an empty attribute layout the positions-only implementation is used,
/// which stores fixed-size xyz points; otherwise the attributed one.
///
/// Points are exchanged as Eigen::VectorXd of point_cols() values: xyz followed
/// by the attributes as doubles. Reference positions (a location to trim
/// around, a nearest-neighbor query) only need their first three values. The
/// packed-record methods (add_points(xyz, records), pointcloud(), ...) keep attributes in their own
/// types instead, so e.g. a uint32 RANGE is never widened to double.
///
/// Each call is one virtual dispatch into the underlying map, and the
/// per-point work stays inside it, so bulk operations cost the same as on the
/// implementation type. get_closest_neighbor_position() is the allocation-free
/// query for hot loops such as ICP correspondence search.
class OUSTER_API_CLASS VoxelHashMap {
   public:
    /// Point type every method speaks: xyz plus the attributes as doubles.
    using point_type = Eigen::VectorXd;

    // The defaults below are the only definition of each: create(),
    // voxel_downsample(), voxel_downsample_3d(), voxel_downsample_xd(), the
    // Python bindings and the SDK-internal map templates all take theirs from here.

    /// Default max_points_per_voxel configuration
    static constexpr std::size_t DEFAULT_MAX_POINTS_PER_VOXEL = 1;

    /// Default min_points_per_voxel configuration
    static constexpr std::size_t DEFAULT_MIN_POINTS_PER_VOXEL = 1;

    /// Default strategy configuration
    static constexpr VoxelDownsampleStrategy DEFAULT_STRATEGY = VoxelDownsampleStrategy::RANDOM;

    /// Default max_distance configuration
    static constexpr double DEFAULT_MAX_DISTANCE = 100.0;

    /// Default internal_storage_mode configuration
    static constexpr VHMInternalStorageMode DEFAULT_INTERNAL_STORAGE_MODE =
        VHMInternalStorageMode::AUTO;

    /// Build the map implementing @p strategy.
    ///
    /// The parameters match voxel_downsample_xd(), in the same order and with
    /// the same defaults, except that create() takes a max_distance where
    /// voxel_downsample_xd() takes the attribute records.
    ///
    /// @param[in] voxel_size            Edge length of each cubic voxel.
    /// @param[in] max_points_per_voxel  Maximum points kept per voxel (FIRST_N_POINT,
    ///                                  LAST_N_POINT, RANDOM, SMALLEST, LARGEST).
    /// @param[in] min_points_per_voxel  Minimum insertions before a voxel contributes.
    /// @param[in] strategy              The VoxelDownsampleStrategy selecting the per-voxel
    ///                                  reduction.
    /// @param[in] attribute_layout      Types and widths of the per-point attributes; empty for
    ///                                  positions only. SMALLEST and LARGEST need a non-empty
    ///                                  layout -- there is nothing to rank positions-only points
    ///                                  by.
    /// @param[in] max_distance          Maximum distance to retain points from a reference.
    /// @param[in] internal_storage_mode How points are stored inside the voxels. AUTO picks fixed
    ///                                  point for small voxels, which returns kept positions
    ///                                  within a small fraction of the voxel size of the input;
    ///                                  ABS returns them exactly.
    /// @param[in] attribute_index       Index into @p attribute_layout of the scalar attribute to
    ///                                  rank by; ignored unless @p strategy is SMALLEST or LARGEST,
    ///                                  where it must name a scalar attribute.
    /// @return The proxy, owning the selected map.
    /// @throw std::invalid_argument for an unknown @p strategy, a non-positive
    ///        @p voxel_size or @p max_distance, a zero @p max_points_per_voxel, an empty
    ///        @p attribute_layout with @p strategy SMALLEST or LARGEST, or (same two strategies)
    ///        an @p attribute_index that does not name a scalar attribute of @p attribute_layout.
    OUSTER_API_FUNCTION
    static std::unique_ptr<VoxelHashMap> create(
        double voxel_size, std::size_t max_points_per_voxel = DEFAULT_MAX_POINTS_PER_VOXEL,
        std::size_t min_points_per_voxel = DEFAULT_MIN_POINTS_PER_VOXEL,
        VoxelDownsampleStrategy strategy = DEFAULT_STRATEGY,
        AttributeLayout attribute_layout = AttributeLayout(),
        double max_distance = DEFAULT_MAX_DISTANCE,
        VHMInternalStorageMode internal_storage_mode = DEFAULT_INTERNAL_STORAGE_MODE,
        std::size_t attribute_index = 0);

    OUSTER_API_FUNCTION
    virtual ~VoxelHashMap() = default;

    /// @brief The strategy create() selected the underlying map for.
    /// @return The per-voxel reduction strategy.
    OUSTER_API_FUNCTION
    virtual VoxelDownsampleStrategy strategy() const = 0;

    /// Check if the hash map is empty.
    /// @return True if no voxels are present, false otherwise.
    OUSTER_API_FUNCTION
    virtual bool empty() const = 0;

    /// Clear all voxels from the map.
    OUSTER_API_FUNCTION
    virtual void clear() = 0;

    /// Insert points and trim voxels that are far from @p position.
    /// @param[in] points   Points of point_cols() values each.
    /// @param[in] position Reference position for distance trimming (xyz first).
    /// @throw std::invalid_argument if any point's size differs from
    ///        point_cols(); no point is inserted in that case.
    OUSTER_API_FUNCTION
    virtual void update(const std::vector<point_type>& points, const point_type& position) = 0;

    /// Insert points from an Eigen matrix and trim voxels far from @p position.
    ///
    /// The same as the std::vector overload, but zero-copy: @p points binds
    /// directly to whatever row-major buffer backs it (e.g. a numpy array),
    /// with no intermediate per-point copy.
    /// @param[in] points   Row-major matrix of point_cols() columns, one row per point.
    /// @param[in] position Reference position for distance trimming (xyz first).
    /// @throw std::invalid_argument if points.cols() differs from point_cols().
    OUSTER_API_FUNCTION
    virtual void update(const Eigen::Ref<const ArrayXXdR>& points, const point_type& position) = 0;

    /// Insert a single point into the map.
    /// @param[in] point The point to insert, point_cols() values.
    /// @throw std::invalid_argument if point.size() differs from point_cols().
    OUSTER_API_FUNCTION
    virtual void add_point(const point_type& point) = 0;

    /// Insert a single point with an already-packed attribute record.
    /// @param[in] xyz    The point's position.
    /// @param[in] record `attribute_layout().stride()` bytes, or empty when
    ///                   the layout has no attributes.
    /// @throw std::invalid_argument if record.size() does not equal
    ///        attribute_layout().stride().
    OUSTER_API_FUNCTION
    virtual void add_point(const Eigen::Ref<const Eigen::Vector3d>& xyz,
                           const Eigen::Ref<const AttributeRecord>& record) = 0;

    /// Insert points into the map.
    /// @param[in] points Points of point_cols() values each.
    /// @throw std::invalid_argument if any point's size differs from
    ///        point_cols(); every point is checked before any is inserted.
    OUSTER_API_FUNCTION
    virtual void add_points(const std::vector<point_type>& points) = 0;

    /// Insert points from an Eigen matrix into the map.
    /// @param[in] points Row-major matrix of point_cols() columns, one row per point.
    /// @throw std::invalid_argument if points.cols() differs from point_cols().
    OUSTER_API_FUNCTION
    virtual void add_points(const Eigen::Ref<const ArrayXXdR>& points) = 0;

    /// Insert points whose attributes are already packed in the map's layout.
    ///
    /// Nothing is converted: the `attribute_layout().stride()` bytes belonging
    /// to each row are copied into the voxel as-is, so integer attributes keep
    /// their exact values and their original width.
    ///
    /// @param[in] xyz     Row-major Nx3 array of point positions.
    /// @param[in] records `xyz.rows()` x `attribute_layout().stride()` packed
    ///                    attribute records, one row per row of @p xyz. May be
    ///                    empty when the layout has no attributes.
    /// @throw std::invalid_argument if @p records is not
    ///        `xyz.rows()` x `attribute_layout().stride()`.
    OUSTER_API_FUNCTION
    virtual void add_points(const Eigen::Ref<const ArrayX3dR>& xyz,
                            const Eigen::Ref<const AttributeRecords>& records) = 0;

    /// Convenience overload for callers already holding a positions-only
    /// std::vector<Eigen::Vector3d>, e.g. from ICPRegistration. Zero-copy:
    /// forwards through an Eigen::Map to add_points(ArrayXXdR), no
    /// intermediate copy of @p points is made.
    /// @param[in] points The points to insert.
    /// @throw std::invalid_argument if points.cols() (always 3 here) differs
    ///        from point_cols(), i.e. if this map carries attributes.
    OUSTER_API_FUNCTION
    void add_points(const std::vector<Eigen::Vector3d>& points) {
        if (points.empty()) return;
        const Eigen::Map<const ArrayX3dR> view(points.front().data(),
                                               static_cast<Eigen::Index>(points.size()), 3);
        add_points(view);
    }

    /// Insert a pointcloud read back from a map, e.g. by pointcloud().
    /// @param[in] cloud Positions, packed records and the layout describing them.
    /// @throw std::invalid_argument if `cloud.layout` differs from
    ///        attribute_layout(), or if the records are not
    ///        `cloud.xyz.rows()` x `attribute_layout().stride()`.
    OUSTER_API_FUNCTION
    virtual void add_points(const AttributedPointCloud& cloud) = 0;

    /// Get the complete pointcloud, with attributes in their original types.
    ///
    /// Use `.xyz` for just the positions, and AttributedPointCloud's typed
    /// accessors (e.g. `attribute<std::uint32_t>(0)`) for the attributes. A map
    /// without an attribute layout returns zero-width records.
    /// @return Positions plus packed attribute records; see AttributedPointCloud.
    OUSTER_API_FUNCTION
    virtual AttributedPointCloud pointcloud() const = 0;

    /// Remove voxels far from a specified location and return their points.
    ///
    /// The same as remove_voxels_far_from_location(), but returns what it
    /// removed in the form pointcloud() uses, attributes in their original types.
    /// @param[in] origin Location to measure distance from (xyz first).
    /// @return Positions plus packed attribute records of the extracted voxels.
    OUSTER_API_FUNCTION
    virtual AttributedPointCloud extract_voxels_far_from_location(const point_type& origin) = 0;

    /// Remove voxels far from a specified location.
    /// @param[in] origin Location to measure distance from (xyz first).
    OUSTER_API_FUNCTION
    virtual void remove_voxels_far_from_location(const point_type& origin) = 0;

    /// Get a complete pointcloud as a vector.
    /// @return Every point of every voxel, point_cols() values each.
    OUSTER_API_FUNCTION
    virtual std::vector<point_type> pointcloud_vector() const = 0;

    /// Find the closest point in the map to a query point.
    ///
    /// Only the query's voxel and its 26 neighbors are searched, and only
    /// within `8 * voxel_size^2` (squared): the effective radius is
    /// `min(max_distance_sq, 8 * voxel_size^2)`. A point outside that
    /// neighborhood is never returned, even if it is the closest in the map.
    ///
    /// @param[in] query           The query point (xyz first).
    /// @param[in] max_distance_sq Maximum squared distance to search.
    /// @return Tuple of (closest_point, squared_distance). If no point is
    ///         found within the effective radius, squared_distance will be
    ///         >= max_distance_sq.
    OUSTER_API_FUNCTION
    virtual std::tuple<point_type, double> get_closest_neighbor(const point_type& query,
                                                                double max_distance_sq) const = 0;

    /// Find the closest point in the map to a query point, with no limit
    /// beyond the neighborhood bound described above (the 27 voxels around
    /// @p query, within `8 * voxel_size^2` squared).
    /// @param[in] query The query point (xyz first).
    /// @return Tuple of (closest_point, squared_distance). If nothing lies in
    ///         that neighborhood, squared_distance is
    ///         `std::numeric_limits<double>::max()`.
    OUSTER_API_FUNCTION
    std::tuple<point_type, double> get_closest_neighbor(const point_type& query) const {
        return get_closest_neighbor(query, std::numeric_limits<double>::max());
    }

    /// Find the position of the closest point in the map to @p query.
    ///
    /// Same search as get_closest_neighbor(), but returns only the position,
    /// decodes no attributes and allocates nothing, whatever the strategy or
    /// attribute layout, so it suits being called per point from a hot (e.g.
    /// parallel ICP) loop.
    ///
    /// @param[in] query           The query position.
    /// @param[in] max_distance_sq Maximum squared distance to search, bounded
    ///            as for get_closest_neighbor().
    /// @return (closest_position, squared_distance). If no point is found
    ///         within the effective radius, squared_distance will be >= max_distance_sq.
    OUSTER_API_FUNCTION
    virtual std::pair<Eigen::Vector3d, double> get_closest_neighbor_position(
        const Eigen::Vector3d& query, double max_distance_sq) const = 0;

    /// Returns the row width of a serialized point.
    /// @return 3 plus the attribute layout's element count.
    OUSTER_API_FUNCTION
    virtual Eigen::Index point_cols() const = 0;

    /// @brief Types and widths of the per-point attributes stored in this map.
    /// @return The attribute layout; empty when the map stores positions only.
    OUSTER_API_FUNCTION
    virtual const AttributeLayout& attribute_layout() const = 0;

    /// @brief Returns the internal storage representation of points stored in the voxels
    /// @return Internal storage mode (never AUTO: the mode AUTO resolved to)
    OUSTER_API_FUNCTION
    virtual VHMInternalStorageMode storage_mode() const = 0;

    /// Return the maximum number of points allowed per voxel.
    /// @return Maximum points per voxel as set at construction.
    OUSTER_API_FUNCTION
    virtual std::size_t max_points_per_voxel() const = 0;

    /// Return the minimum number of raw insertions required for a voxel to appear in the output.
    /// @return Minimum points threshold as set at construction.
    OUSTER_API_FUNCTION
    virtual std::size_t min_points_per_voxel() const = 0;

    /// @brief The voxel size
    /// @return The voxel size
    OUSTER_API_FUNCTION
    virtual double voxel_size() const = 0;

    /// @brief Maximum distance of points to retain
    /// @return The maximum distance
    OUSTER_API_FUNCTION
    virtual double max_distance() const = 0;
};

// The parameters shared by voxel_downsample(), voxel_downsample_3d(),
// voxel_downsample_xd() and VoxelHashMap::create() come in the same order, with
// the same defaults and the same descriptions; keep them in sync.

/// Downsample a positions-only pointcloud.
///
/// Takes positions only; use voxel_downsample_xd() for points with attributes,
/// or voxel_downsample() to have the choice made from the attribute layout.
///
/// @param[in] frame                 Row-major Nx3 array of point positions.
/// @param[in] voxel_size            Edge length of each cubic voxel.
/// @param[in] max_points_per_voxel  Maximum points kept per voxel (FIRST_N_POINT, LAST_N_POINT,
///                                  RANDOM).
/// @param[in] min_points_per_voxel  Minimum insertions before a voxel contributes.
/// @param[in] strategy              The VoxelDownsampleStrategy selecting the per-voxel
///                                  reduction.
/// @param[in] internal_storage_mode How points are stored inside the voxels. AUTO picks fixed
///                                  point for small voxels, which returns kept positions within
///                                  a small fraction of the voxel size of the input; ABS
///                                  returns them exactly.
/// @return The downsampled positions.
/// @throw std::invalid_argument for a non-positive @p voxel_size, a zero
///        @p max_points_per_voxel, an unknown @p strategy, or @p strategy SMALLEST or
///        LARGEST (which rank points by an attribute; use voxel_downsample_xd()), even
///        when @p frame is empty.
OUSTER_API_FUNCTION ArrayX3dR voxel_downsample_3d(
    const Eigen::Ref<const ArrayX3dR>& frame, double voxel_size,
    std::size_t max_points_per_voxel = VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL,
    std::size_t min_points_per_voxel = VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL,
    VoxelDownsampleStrategy strategy = VoxelHashMap::DEFAULT_STRATEGY,
    VHMInternalStorageMode internal_storage_mode = VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE);

/// Downsample a pointcloud, optionally with attributes that keep their own types.
///
/// Points go in as positions plus packed attribute records described by
/// @p attribute_layout, and come back the same way, so e.g. a uint32 RANGE or
/// uint8 REFLECTIVITY is never widened to double. With VoxelDownsampleStrategy::AVERAGE_POINT
/// the per-voxel mean of an integer attribute is rounded to the nearest
/// representable value. With SMALLEST or LARGEST, @p attribute_layout must not be empty,
/// and the points kept per voxel are the ones with the smallest, respectively largest,
/// value of the scalar attribute named by @p attribute_index. Leave @p attribute_layout
/// and @p records out (or empty)
/// to downsample positions only. Such input goes through the same path as
/// attributed input; use voxel_downsample() to have it sent to
/// voxel_downsample_3d() instead.
///
/// The parameters follow VoxelHashMap::create(), with the same defaults. The
/// records ride next to the attribute_layout describing them; create() has no
/// records, and voxel_downsample_xd() has no max_distance, since downsampling
/// never trims.
///
/// @param[in] xyz                   Row-major Nx3 array of point positions.
/// @param[in] voxel_size            Edge length of each cubic voxel.
/// @param[in] max_points_per_voxel  Maximum points kept per voxel (FIRST_N_POINT, LAST_N_POINT,
///                                  RANDOM).
/// @param[in] min_points_per_voxel  Minimum insertions before a voxel contributes.
/// @param[in] strategy              The VoxelDownsampleStrategy selecting the per-voxel
///                                  reduction.
/// @param[in] attribute_layout      Types and widths of the per-point attributes; empty for
///                                  positions only.
/// @param[in] records               `xyz.rows()` x `attribute_layout.stride()` packed
///                                  attribute records, one row per point; empty when
///                                  @p attribute_layout is.
/// @param[in] internal_storage_mode How points are stored inside the voxels. AUTO picks fixed
///                                  point for small voxels, which returns kept positions within
///                                  a small fraction of the voxel size of the input; ABS
///                                  returns them exactly.
/// @param[in] attribute_index       Index into @p attribute_layout of the scalar attribute to
///                                  rank by; ignored unless @p strategy is SMALLEST or LARGEST,
///                                  where it must name a scalar attribute.
/// @return The downsampled points, their records, and @p attribute_layout.
/// @throw std::invalid_argument if @p records is not
///        `xyz.rows()` x `attribute_layout.stride()` (any records given without a
///        layout included), for a non-positive @p voxel_size, a zero @p max_points_per_voxel,
///        or an unknown @p strategy, or (SMALLEST/LARGEST only) an empty @p attribute_layout
///        or an @p attribute_index that does not name a scalar attribute of it.
OUSTER_API_FUNCTION AttributedPointCloud voxel_downsample_xd(
    const Eigen::Ref<const ArrayX3dR>& xyz, double voxel_size,
    std::size_t max_points_per_voxel = VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL,
    std::size_t min_points_per_voxel = VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL,
    VoxelDownsampleStrategy strategy = VoxelHashMap::DEFAULT_STRATEGY,
    const AttributeLayout& attribute_layout = AttributeLayout(),
    const Eigen::Ref<const AttributeRecords>& records = AttributeRecords(),
    VHMInternalStorageMode internal_storage_mode = VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE,
    std::size_t attribute_index = 0);

/// Downsample a pointcloud, with or without attributes; the recommended entry point.
///
/// Takes the parameters of voxel_downsample_xd() and returns the same result,
/// choosing the implementation from @p attribute_layout: positions-only input
/// (an empty layout) goes to voxel_downsample_3d(), anything else to
/// voxel_downsample_xd(). This is the downsampler the Python API exposes.
///
/// @param[in] xyz                   Row-major Nx3 array of point positions.
/// @param[in] voxel_size            Edge length of each cubic voxel.
/// @param[in] max_points_per_voxel  Maximum points kept per voxel (FIRST_N_POINT, LAST_N_POINT,
///                                  RANDOM).
/// @param[in] min_points_per_voxel  Minimum insertions before a voxel contributes.
/// @param[in] strategy              The VoxelDownsampleStrategy selecting the per-voxel
///                                  reduction.
/// @param[in] attribute_layout      Types and widths of the per-point attributes; empty for
///                                  positions only.
/// @param[in] records               `xyz.rows()` x `attribute_layout.stride()` packed
///                                  attribute records, one row per point; empty when
///                                  @p attribute_layout is.
/// @param[in] internal_storage_mode How points are stored inside the voxels. AUTO picks fixed
///                                  point for small voxels, which returns kept positions within
///                                  a small fraction of the voxel size of the input; ABS
///                                  returns them exactly.
/// @param[in] attribute_index       Index into @p attribute_layout of the scalar attribute to
///                                  rank by; ignored unless @p strategy is SMALLEST or LARGEST,
///                                  where it must name a scalar attribute, and @p attribute_layout
///                                  must not be empty.
/// @return The downsampled points, their records, and @p attribute_layout; with an
///         empty layout the records have zero columns.
/// @throw std::invalid_argument if @p records is not
///        `xyz.rows()` x `attribute_layout.stride()` (any records given without a
///        layout included), for a non-positive @p voxel_size, a zero @p max_points_per_voxel,
///        or an unknown @p strategy, or (SMALLEST/LARGEST only) an empty @p attribute_layout
///        or an @p attribute_index that does not name a scalar attribute of it.
OUSTER_API_FUNCTION AttributedPointCloud voxel_downsample(
    const Eigen::Ref<const ArrayX3dR>& xyz, double voxel_size,
    std::size_t max_points_per_voxel = VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL,
    std::size_t min_points_per_voxel = VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL,
    VoxelDownsampleStrategy strategy = VoxelHashMap::DEFAULT_STRATEGY,
    const AttributeLayout& attribute_layout = AttributeLayout(),
    const Eigen::Ref<const AttributeRecords>& records = AttributeRecords(),
    VHMInternalStorageMode internal_storage_mode = VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE,
    std::size_t attribute_index = 0);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
