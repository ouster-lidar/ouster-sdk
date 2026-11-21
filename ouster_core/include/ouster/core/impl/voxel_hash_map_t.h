#pragma once

// VoxelHashMapT and its strategy/bucket building blocks. Internal to the SDK,
// like the rest of ouster/core/impl: the installed <ouster/core/voxel_hash_map.h>
// exposes only VoxelHashMap, which create()s one of the specializations defined
// here. Not supported for external use and may change or disappear without
// notice. SDK sources that need the concrete types (ouster_core itself,
// ouster_algorithm, internal tests and benchmarks) include this header directly
// -- it lives under the public include tree so no private include path into
// another target's src/ is needed, but requires tsl::robin_map (see below) to
// compile, so any such consumer must link tsl::robin_map itself.

#include <ouster/core/attribute_layout.h>
#include <ouster/core/eigen_hash.h>
#include <ouster/core/typedefs.h>
#include <ouster/core/visibility.h>
#include <ouster/core/voxel_hash_map.h>
#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

extern const std::uint8_t BLUE_NOISE_32X32X32X3[32][32][32][3];

namespace impl {

// ---------------------------------------------------------------------------
// PointDefaultValue: constructs the "not-found" sentinel returned by
// get_closest_neighbor when no neighbor is within max_distance_sq. For Eigen
// types the sentinel is the zero vector; for other types default construction
// is used.
// ---------------------------------------------------------------------------

template <typename T>
struct PointDefaultValue {
    static T make(Eigen::Index /*count*/) {
        return T::Zero();
    }
};

template <>
struct PointDefaultValue<Eigen::VectorXd> {
    static Eigen::VectorXd make(Eigen::Index count) {
        return Eigen::VectorXd::Zero(count);
    }
};

// ---------------------------------------------------------------------------
// spatial_view: extracts the 3-D spatial position from any supported point
// type. Used by point_to_voxel, first_n_point, and get_closest_neighbor
// ---------------------------------------------------------------------------
template <typename T>
inline auto spatial_view(const T& vec) -> decltype(vec.template head<3>()) {
    return vec.template head<3>();
}
template <typename T>
inline auto spatial_view(T& vec) -> decltype(vec.template head<3>()) {
    return vec.template head<3>();
}

// The voxel index containing position `xyz`. The one formula behind both
// VoxelHashMapT::point_to_voxel() and the position-only neighbor search.
template <typename VoxelIndex, typename Derived>
inline VoxelIndex xyz_to_voxel(const Eigen::MatrixBase<Derived>& xyz, double inv_voxel_size) {
    return VoxelIndex(
        static_cast<typename VoxelIndex::Scalar>(std::floor(xyz[0] * inv_voxel_size)),
        static_cast<typename VoxelIndex::Scalar>(std::floor(xyz[1] * inv_voxel_size)),
        static_cast<typename VoxelIndex::Scalar>(std::floor(xyz[2] * inv_voxel_size)));
}

// Whether PointType carries its attributes inside the point vector itself.
// Fixed-size points (e.g. Vector4d) do; dynamic-size points (VectorXd) keep
// only xyz in the bucket and store their attributes as packed records sized by
// the map's AttributeLayout, so that a uint32 attribute costs 4 bytes per point
// rather than 8 plus a per-point heap allocation.
template <typename PointType>
using PointPacksAttribs =
    std::integral_constant<bool, PointType::SizeAtCompileTime != Eigen::Dynamic>;

template <typename PointType>
struct VHMStorageAbs {
    using DistType = double;
    using V3AbsType = Eigen::Matrix<typename PointType::Scalar, 3, 1>;
    using StorageType =
        std::conditional_t<PointPacksAttribs<PointType>::value, PointType, V3AbsType>;

    // Templated on the stored type rather than pinned to StorageType: the
    // averaging buckets keep the whole (position + attribute sums) point as
    // their storage while still using this ABS implementation.
    template <typename S>
    static auto get_xyz_component(const S& point) -> decltype(point.template head<3>()) {
        return point.template head<3>();
    }

    // Narrow to xyz for dynamic-size points; keep the whole point otherwise.
    static StorageType to_storage(const PointType& point, std::true_type /*packs_attribs*/) {
        return point;
    }
    static StorageType to_storage(const PointType& point, std::false_type /*packs_attribs*/) {
        return point.template head<3>();
    }

    static StorageType point_to_internal_storage_nodither(const V3AbsType& /*voxel_origin*/,
                                                          const PointType& point,
                                                          double /*voxel_size*/) {
        return to_storage(point, PointPacksAttribs<PointType>{});
    }
    template <typename S>
    static PointType internal_storage_to_point(const V3AbsType& /*voxel_origin*/, const S& offset,
                                               double /*voxel_size*/) {
        return offset;
    }
    static StorageType point_to_internal_storage(const V3AbsType& /*voxel_origin*/,
                                                 const PointType& point, double /*voxel_size*/) {
        return to_storage(point, PointPacksAttribs<PointType>{});
    }
    // A stored point's position alone, as bucket visit_voxel_positions()
    // hands it out and the position-only neighbor search compares it.
    using XyzType = V3AbsType;
    static XyzType xyz_to_internal(const V3AbsType& /*voxel_origin*/, const Eigen::Vector3d& xyz,
                                   double /*voxel_size*/) {
        return xyz;
    }
    static V3AbsType internal_xyz_to_abs(const V3AbsType& /*voxel_origin*/, const XyzType& xyz,
                                         double /*voxel_size*/) {
        return xyz;
    }

    static DistType dist_sq_to_dist_type(double dist_sq, double /*voxel_size*/) {
        return dist_sq;
    }
    static DistType voxel_size_as_dist(double voxel_size) {
        return voxel_size;
    }
    static double dist_sq_from_dist_type(DistType dist_sq, double /*voxel_size*/) {
        return dist_sq;
    }

    template <typename VoxelIndex>
    static V3AbsType calc_voxel_origin_from_icoords(const VoxelIndex& voxel_icoord,
                                                    double voxel_size) {
        using DType = typename std::decay_t<PointType>::Scalar;
        V3AbsType result = voxel_icoord.template cast<DType>().eval() * voxel_size;
        return result;
    }

    struct VoxelShift {
        Eigen::Vector3i offset;
        VoxelShift(int x, int y, int z) : offset{x, y, z} {}
        V3AbsType get_origin(const Eigen::Vector3i& other, double voxel_size) const {
            V3AbsType origin =
                calc_voxel_origin_from_icoords<Eigen::Vector3i>(other + this->offset, voxel_size);
            return origin;
        }
        // The origin offset needed in fixed-point for computing point differences doesn't apply to
        // abs points (just zero this offset)
        V3AbsType get_offset_for_diff() const {
            // NOTE[BW]: For dynamic vectors don't care about the total size of the PointType used
            // for internal storage - just need a spatial point here.
            return V3AbsType::Zero(3);
        }
    };
};

template <typename PointType, typename ITypeArg>
struct VHMStorageRelFx {
    using IType = ITypeArg;
    static constexpr int FX_SHIFT = 8 * sizeof(IType);
    static constexpr IType FX_MASK = std::numeric_limits<IType>::max();
    using V3FxOffType = Eigen::Vector<IType, 3>;
    using V3AbsType = Eigen::Matrix<typename PointType::Scalar, 3, 1>;

    template <typename Derived>
    static V3FxOffType abs_to_fx(const V3AbsType& voxel_origin,
                                 const Eigen::MatrixBase<Derived>& point, double voxel_size) {
        V3FxOffType fx = ((point - voxel_origin) / voxel_size * static_cast<double>(1 << FX_SHIFT))
                             .template cast<IType>()
                             .eval();
        return fx;
    }
    static V3AbsType fx_to_abs(const V3AbsType& voxel_origin, const V3FxOffType& offset,
                               double voxel_size) {
        V3AbsType result = offset.template cast<double>().eval() /
                               static_cast<double>(1 << FX_SHIFT) * voxel_size +
                           voxel_origin;
        return result;
    }

    // Number of fixed-size scalars trailing the xyz block for a fixed-size
    // PointType (e.g. Vector4d packs 1 trailing attribute alongside xyz).
    // Meaningless (and never instantiated) when PointType is Dynamic-sized.
    static constexpr int FixedAttribSize =
        PointType::SizeAtCompileTime == Eigen::Dynamic ? 0 : PointType::SizeAtCompileTime - 3;

    struct StorageTypeXYZOnly {
        V3FxOffType xyz;

        static StorageTypeXYZOnly init(const V3FxOffType& fx, const PointType& /*pt*/) {
            return StorageTypeXYZOnly{fx};
        }
        PointType to_point(const V3AbsType& voxel_origin, double voxel_size) const {
            const V3AbsType xyz_abs = fx_to_abs(voxel_origin, this->xyz, voxel_size);
            return xyz_abs;
        }
    };

    // Storage for a fixed-size PointType that DOES carry trailing attributes
    // past xyz (e.g. Vector4d, which packs one attribute alongside xyz).
    template <typename AttribsType>
    struct StorageTypeXYZAttribs {
        V3FxOffType xyz;
        AttribsType attribs;

        static StorageTypeXYZAttribs init(const V3FxOffType& fx, const PointType& pt) {
            StorageTypeXYZAttribs result;
            result.xyz = fx;
            result.attribs = pt.tail(pt.size() - 3);
            return result;
        }

        PointType to_point(const V3AbsType& voxel_origin, double voxel_size) const {
            const V3AbsType xyz_abs = fx_to_abs(voxel_origin, this->xyz, voxel_size);
            PointType result = PointDefaultValue<PointType>::make(3 + this->attribs.size());
            result.template head<3>() = xyz_abs;
            result.tail(this->attribs.size()) = this->attribs;
            return result;
        }
    };

    // Dynamic-size points have FixedAttribSize == 0 and so land on
    // StorageTypeXYZOnly: their attributes live in the bucket's packed
    // attribute records instead of inside every stored point.
    using StorageType =
        std::conditional_t<FixedAttribSize == 0, StorageTypeXYZOnly,
                           StorageTypeXYZAttribs<Eigen::Matrix<double, FixedAttribSize, 1>>>;

    // Type used for distance calculations -- we can just use int64_t for everything here to avoid
    // tricky shenanigans with avoiding possible int overflows in int32_t types for FX16. For scalar
    // integer operations I would not expect any speed difference between register-sized integers
    // and smaller.
    using DistType = std::int64_t;
    // NOTE[BW]: The integer math ops will not work as-is with FX32 internal representation (not
    // that I would expect us to ever add that?)
    static_assert(sizeof(IType) == 1 || sizeof(IType) == 2, "Invalid integer type size");

    static V3FxOffType get_xyz_component(const StorageType& point) {
        return point.xyz;
    }

    // A stored point's position alone; see VHMStorageAbs::XyzType.
    using XyzType = V3FxOffType;
    static XyzType xyz_to_internal(const V3AbsType& voxel_origin, const Eigen::Vector3d& xyz,
                                   double voxel_size) {
        return abs_to_fx(voxel_origin, xyz, voxel_size);
    }
    static V3AbsType internal_xyz_to_abs(const V3AbsType& voxel_origin, const XyzType& xyz,
                                         double voxel_size) {
        return fx_to_abs(voxel_origin, xyz, voxel_size);
    }

    static StorageType point_to_internal_storage_nodither(const V3AbsType& voxel_origin,
                                                          const PointType& point,
                                                          double voxel_size) {
        V3FxOffType fx = abs_to_fx(voxel_origin, spatial_view(point), voxel_size);
        return StorageType::init(fx, point);
    }

    static StorageType point_to_internal_storage(const V3AbsType& voxel_origin,
                                                 const PointType& point, double voxel_size) {
        V3FxOffType offset_fx0 = abs_to_fx(voxel_origin, spatial_view(point), voxel_size);
        const uint8_t* noise_ptr =
            &BLUE_NOISE_32X32X32X3[offset_fx0(0, 0) & 0x1f][offset_fx0(1, 0) & 0x1f]
                                  [offset_fx0(2, 0) & 0x1f][0];
        V3AbsType noise = {static_cast<double>(noise_ptr[0]), static_cast<double>(noise_ptr[1]),
                           static_cast<double>(noise_ptr[2])};
        noise.array() =
            (1.0 / 255.0) * noise.array() * voxel_size / static_cast<double>(1 << FX_SHIFT);

        // NOTE[BW]: Since we're adding one-sided (positive) noise, if we're at the final
        // quantization cell just clamp the noise before adding it to the point
        for (int idx = 0; idx < 3; ++idx) {
            if (offset_fx0(idx, 0) == std::numeric_limits<IType>::max()) {
                noise(idx, 0) = 0;
            }
        }

        PointType point_with_noise = point;
        spatial_view(point_with_noise) += noise;
        StorageType result =
            point_to_internal_storage_nodither(voxel_origin, point_with_noise, voxel_size);
        return result;
    }

    static PointType internal_storage_to_point(const V3AbsType& voxel_origin,
                                               const StorageType& offset, double voxel_size) {
        return offset.to_point(voxel_origin, voxel_size);
    }

    static DistType dist_sq_to_dist_type(double dist_sq, double voxel_size) {
        DistType result = dist_sq / (voxel_size * voxel_size) * ((DistType)1 << 2 * FX_SHIFT);
        return result;
    }

    static DistType voxel_size_as_dist(double /*voxel_size*/) {
        return (DistType)(1 << FX_SHIFT);
    }

    static double dist_sq_from_dist_type(DistType dist_sq, double voxel_size) {
        double result = (double)dist_sq * voxel_size * voxel_size * (1.0 / (1LL << 2 * FX_SHIFT));
        return result;
    }

    template <typename VoxelIndex>
    static V3AbsType calc_voxel_origin_from_icoords(const VoxelIndex& voxel_icoord,
                                                    double voxel_size) {
        using DType = typename std::decay_t<PointType>::Scalar;
        V3AbsType result = voxel_icoord.template cast<DType>().eval() * voxel_size;
        return result;
    }

    struct VoxelShift {
        Eigen::Vector3i offset;
        Eigen::Vector<DistType, 3>
            origin_q;  // Origin stored as a fixed-point offset relative to the queried voxel
        VoxelShift(int x, int y, int z)
            : offset{x, y, z},
              origin_q(offset.array().template cast<DistType>() *
                       static_cast<DistType>((1 << FX_SHIFT))) {}

        Eigen::Vector<DistType, 3> get_origin(Eigen::Vector3i& /*other*/,
                                              double /*voxel_size*/) const {
            return origin_q;
        }
        Eigen::Vector<DistType, 3> get_offset_for_diff() const {
            return origin_q;
        }
    };
};

template <typename PointType, VHMInternalStorageMode StorageMode>
struct VHMStorageTraits;
template <typename PointType>
struct VHMStorageTraits<PointType, VHMInternalStorageMode::ABS> {
    using type = VHMStorageAbs<PointType>;
};
template <typename PointType>
struct VHMStorageTraits<PointType, VHMInternalStorageMode::FX16> {
    using type = VHMStorageRelFx<PointType, std::uint16_t>;
};
template <typename PointType>
struct VHMStorageTraits<PointType, VHMInternalStorageMode::FX8> {
    using type = VHMStorageRelFx<PointType, std::uint8_t>;
};

// Per-voxel storage for packed attribute records, one run of `stride` bytes per
// point in the bucket. Only dynamic-size point types can carry an
// AttributeLayout (the map rejects one for fixed-size points), so the fixed-size
// specialization below holds nothing at all and, as an empty base, costs a
// VoxelHashMap3d bucket nothing.
template <bool PointPacksAttribsValue>
struct VoxelAttributeRecords {
    std::vector<std::uint8_t> attribs_arr;

    void append_record(const std::uint8_t* record, std::size_t stride) {
        if (stride != 0) {
            attribs_arr.insert(attribs_arr.end(), record, record + stride);
        }
    }
    void replace_record(std::size_t index, const std::uint8_t* record, std::size_t stride) {
        if (stride != 0) {
            std::memcpy(attribs_arr.data() + index * stride, record, stride);
        }
    }
    void pop_front_record(std::size_t stride) {
        if (stride != 0) {
            attribs_arr.erase(attribs_arr.begin(),
                              attribs_arr.begin() + static_cast<std::ptrdiff_t>(stride));
        }
    }
    const std::uint8_t* record_at(std::size_t index, std::size_t stride) const {
        return stride == 0 ? nullptr : attribs_arr.data() + index * stride;
    }
};

template <>
struct VoxelAttributeRecords<true> {
    void append_record(const std::uint8_t* /*record*/, std::size_t /*stride*/) {}
    void replace_record(std::size_t /*index*/, const std::uint8_t* /*record*/,
                        std::size_t /*stride*/) {}
    void pop_front_record(std::size_t /*stride*/) {}
    const std::uint8_t* record_at(std::size_t /*index*/, std::size_t /*stride*/) const {
        return nullptr;
    }
};

}  // namespace impl

/// Default per-voxel bucket. Stores a collection of point positions plus, when
/// the map declares an AttributeLayout, one packed attribute record per point.
///
/// Positions and attribute records are kept in two parallel arrays rather than
/// interleaved in a single point vector: the record is a flat run of
/// `layout.stride()` bytes per point, so a uint32 attribute costs 4 bytes
/// instead of the 8 it would cost as a double (and with no per-point heap
/// allocation, which a per-point Eigen::VectorXd would need). Point types that
/// cannot carry attributes inherit an empty record store, so their buckets keep
/// their original footprint of sizeof(std::vector<StorageType>) + sizeof(size_t).
template <typename PointType, VHMInternalStorageMode StorageMode>
struct OUSTER_API_CLASS DefaultVoxelBucketTarget
    : impl::VoxelAttributeRecords<impl::PointPacksAttribs<PointType>::value> {
    /// @cond
    using StorageImpl = typename impl::VHMStorageTraits<PointType, StorageMode>::type;
    using StorageType = typename StorageImpl::StorageType;
    /// @endcond

    /// Concrete container type used to store the points in this bucket.
    using PointsArrayType = std::vector<StorageType>;
    /// Points admitted into this voxel, up to max_points_per_voxel.
    PointsArrayType points_arr;
    /// Number of points accumulated so far; 0 means the bucket is empty.
    std::size_t count{0};

    /// Reserve storage for expected number of points per voxel.
    /// @param[in] map The owning VoxelHashMapT; queried for max_points_per_voxel.
    template <typename MapType>
    void init_voxel(const MapType& map) noexcept {
        this->points_arr.reserve(map.max_points_per_voxel() / 2);
        // The attribute records are deliberately left to grow on demand: most
        // voxels hold far fewer than max_points_per_voxel points, and reserving
        // a full bucket's worth of records up front costs more than the
        // doubling does.
        count = 0;
    }

    /// Append a point and its attribute record to the bucket.
    /// @param[in] point  Point in the map's internal storage representation.
    /// @param[in] record Packed attribute record, or nullptr when @p stride is 0.
    /// @param[in] stride Bytes per attribute record.
    void append(const StorageType& point, const std::uint8_t* record, std::size_t stride) {
        this->points_arr.emplace_back(point);
        this->append_record(record, stride);
    }

    /// Overwrite the point at @p index and its attribute record.
    /// @param[in] index  Position in points_arr to overwrite.
    /// @param[in] point  Point in the map's internal storage representation.
    /// @param[in] record Packed attribute record, or nullptr when @p stride is 0.
    /// @param[in] stride Bytes per attribute record.
    void replace(std::size_t index, const StorageType& point, const std::uint8_t* record,
                 std::size_t stride) {
        this->points_arr[index] = point;
        this->replace_record(index, record, stride);
    }

    /// Drop the first point (and its attribute record) and append a new one at the
    /// end. With append() this makes points_arr an insertion-ordered queue, oldest
    /// first. O(size), the same order as the near-duplicate scan.
    /// @param[in] point  Point in the map's internal storage representation.
    /// @param[in] record Packed attribute record, or nullptr when @p stride is 0.
    /// @param[in] stride Bytes per attribute record.
    void pop_front_and_append(const StorageType& point, const std::uint8_t* record,
                              std::size_t stride) {
        this->points_arr.erase(this->points_arr.begin());
        this->pop_front_record(stride);
        this->append(point, record, stride);
    }

    /// Invoke @p func for every point stored in the bucket.
    /// @param[in] map The VoxelHashMapT
    /// @param[in] voxel_origin The origin of this voxel
    /// @param[in] func Callable invoked with each stored point and its attribute
    ///                 record (nullptr when the map has no attributes).
    template <typename MapType, typename Func>
    void visit_voxels_as_point_type(const MapType& map,
                                    const typename StorageImpl::V3AbsType& voxel_origin,
                                    Func&& func) const {
        if (count < map.min_points_per_voxel()) {
            return;
        }
        const std::size_t stride = map.attribute_layout().stride();
        for (std::size_t i = 0; i < this->points_arr.size(); ++i) {
            PointType point = StorageImpl::internal_storage_to_point(
                voxel_origin, this->points_arr[i], map.voxel_size());
            func(point, this->record_at(i, stride));
        }
    }

    /// Invoke @p func for every point stored in the bucket.
    /// @param[in] map The VoxelHashMapT
    /// @param[in] func Callable invoked with each stored point and its attribute
    ///                 record (nullptr when the map has no attributes).
    template <typename MapType, typename Func>
    void visit_voxels_as_storage_type(const MapType& map, Func&& func) const {
        if (count < map.min_points_per_voxel()) {
            return;
        }
        const std::size_t stride = map.attribute_layout().stride();
        for (std::size_t i = 0; i < this->points_arr.size(); ++i) {
            func(this->points_arr[i], this->record_at(i, stride));
        }
    }

    /// Invoke @p func with the position of every point stored in the bucket.
    /// @param[in] map  The VoxelHashMapT
    /// @param[in] func Callable invoked with each stored point's
    ///                 StorageImpl::XyzType position.
    template <typename MapType, typename Func>
    void visit_voxel_positions(const MapType& map, Func&& func) const {
        if (count < map.min_points_per_voxel()) {
            return;
        }
        for (const StorageType& point : this->points_arr) {
            func(StorageImpl::get_xyz_component(point));
        }
    }

    /// Return the number of points currently stored in the bucket.
    /// @param[in] map The VoxelHashMapT
    /// @return Number of points in the bucket.
    template <typename MapType>
    std::size_t point_count(const MapType& map) const noexcept {
        return count >= map.min_points_per_voxel() ? points_arr.size() : 0;
    }
};

/// Per-voxel bucket that accumulates all inserted points into a single running
/// sum. Pair with accumulate_strategy so that every inserted point contributes
/// to the sum, and with pointcloud_double() / pointcloud_vector() to retrieve the
/// per-voxel average (sum / count).
template <typename T>
struct OUSTER_API_CLASS AveragePointBucketTarget {
    /// @cond
    using StorageImpl = typename impl::VHMStorageTraits<T, VHMInternalStorageMode::ABS>::type;
    using StorageType = T;
    /// @endcond

    /// Underlying point type stored in this bucket.
    using PointType = T;
    /// Running sum of all points inserted into this voxel. Attributes are
    /// summed as doubles regardless of the type they are stored in elsewhere;
    /// the per-voxel average is converted back to the layout's types on output.
    T point;
    /// Number of points accumulated so far; 0 means the bucket is empty.
    std::size_t count{0};

    /// Add a point to the running sum.
    /// @param[in] value  Point to accumulate; carries its attributes as doubles
    ///                   when @p record is nullptr.
    /// @param[in] record Packed attribute record, or nullptr.
    /// @param[in] layout Layout describing @p record.
    void accumulate(const T& value, const std::uint8_t* record, const AttributeLayout& layout) {
        if (record == nullptr) {
            this->point += value;
        } else {
            this->point.template head<3>() += value.template head<3>();
            layout.decode_add(record, this->point.data() + 3);
        }
        ++this->count;
    }

    /// Replace the running sum with a single point (reservoir sampling, k = 1).
    /// @param[in] value  Point to keep; carries its attributes as doubles when
    ///                   @p record is nullptr.
    /// @param[in] record Packed attribute record, or nullptr.
    /// @param[in] layout Layout describing @p record.
    void assign(const T& value, const std::uint8_t* record, const AttributeLayout& layout) {
        if (record == nullptr) {
            this->point = value;
        } else {
            this->point.template head<3>() = value.template head<3>();
            layout.decode(record, this->point.data() + 3);
        }
    }

    /// Zero-initialise the running sum to the correct size and reset the count.
    /// @param[in] map The owning VoxelHashMapT; queried for point_cols.
    template <typename MapType>
    void init_voxel(const MapType& map) noexcept {
        // TODO[UN]: Note that for VectorXd, the point cols isn't known at compile time
        // This is why we invoke map.point_cols() here, but maybe there is a better way
        // to do this? The idea here is to initialize the point directly to zero here such
        // so that we don't need to check for bucket.count == 0 during accumulation
        this->point = impl::PointDefaultValue<T>::make(map.point_cols());
        this->count = 0;
    }

    /// Invoke @p func with the per-voxel average if the bucket meets the minimum
    /// points threshold required by the map.
    /// @param[in] map  The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @param[in] func Callable invoked with the per-voxel average point.
    template <typename MapType, typename Func>
    void visit_voxels_as_point_type(const MapType& map,
                                    const typename StorageImpl::V3AbsType& /*voxel_origin*/,
                                    Func&& func) const {
        // The averaged point already carries its attributes as doubles, so it
        // needs no separate attribute record.
        if (this->count >= map.min_points_per_voxel())
            std::forward<Func>(func)(T(this->point / static_cast<double>(this->count)), nullptr);
    }

    /// Invoke @p func with the per-voxel average if the bucket meets the minimum
    /// points threshold required by the map.
    /// @param[in] map  The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @param[in] func Callable invoked with the per-voxel average point.
    template <typename MapType, typename Func>
    void visit_voxels_as_storage_type(const MapType& map, Func&& func) const {
        if (this->count >= map.min_points_per_voxel())
            std::forward<Func>(func)(T(this->point / static_cast<double>(this->count)), nullptr);
    }

    /// Invoke @p func with the average position if the bucket meets the
    /// minimum points threshold. Unlike visit_voxels_as_storage_type(), only
    /// the position is averaged, so a dynamic-size T allocates nothing.
    /// @param[in] map  The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @param[in] func Callable invoked with the StorageImpl::XyzType average position.
    template <typename MapType, typename Func>
    void visit_voxel_positions(const MapType& map, Func&& func) const {
        if (this->count >= map.min_points_per_voxel()) {
            const typename StorageImpl::XyzType average =
                this->point.template head<3>() / static_cast<double>(this->count);
            std::forward<Func>(func)(average);
        }
    }

    /// Return 1 if the bucket meets the minimum points threshold, 0 otherwise.
    /// @param[in] map The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @return 1 if count >= min_points_per_voxel, 0 otherwise.
    template <typename MapType>
    std::size_t point_count(const MapType& map) const noexcept {
        return this->count >= map.min_points_per_voxel() ? 1u : 0u;
    }
};

/// Per-voxel bucket that jointly accumulates 3-D positions and pre-normalized
/// normals. Pair with accumulate_point_normal_strategy for insertion (expects
/// a 6-D packed input [x,y,z,nx,ny,nz]) and use pointcloud_double() /
/// pointcloud_vector() to retrieve the averaged position and re-normalized
/// average normal direction packed into a 6-D output vector.
struct OUSTER_API_CLASS PointNormalBucketTarget {
    /// Packed [x,y,z,nx,ny,nz] point this bucket accepts and produces.
    using PointType = Eigen::Matrix<double, 6, 1>;
    /// @cond
    using StorageImpl =
        typename impl::VHMStorageTraits<PointType, VHMInternalStorageMode::ABS>::type;
    using StorageType = PointType;
    /// @endcond

    /// This bucket spends the second half of its input on the normal, so it has
    /// nowhere to keep per-point attribute records. VoxelHashMapT rejects a
    /// non-empty AttributeLayout at construction rather than let the records be
    /// dropped on every insert.
    static constexpr bool stores_attributes = false;

    /// Running sum of positions.
    Eigen::Vector3d point_sum;
    /// Running sum of pre-normalized normals.
    Eigen::Vector3d normal_sum;
    /// Number of points accumulated or seen so far; 0 means the bucket is empty.
    std::size_t count{0};

    /// Zero-initialise both running sums and reset the count.
    template <typename MapType>
    void init_voxel(const MapType& /*map*/) noexcept {
        this->point_sum = Eigen::Vector3d::Zero();
        this->normal_sum = Eigen::Vector3d::Zero();
        this->count = 0;
    }

    /// Invoke @p func with a packed [avg_position | renormalized_avg_normal] vector
    /// if the bucket meets the minimum points threshold.
    /// @param[in] map  The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @param[in] func Callable invoked with the packed 6-D output vector.
    template <typename MapType, typename Func>
    void visit_voxels_as_point_type(const MapType& map,
                                    const typename StorageImpl::V3AbsType& /*voxel_origin*/,
                                    Func&& func) const {
        if (emits_point(map)) {
            PointType out;
            out.head<3>() = this->point_sum / static_cast<double>(this->count);
            out.tail<3>() = this->normal_sum / this->normal_sum.norm();
            std::forward<Func>(func)(out, nullptr);
        }
    }

    /// Invoke @p func with a packed [avg_position | renormalized_avg_normal] vector
    /// if the bucket meets the minimum points threshold.
    /// @param[in] map  The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @param[in] func Callable invoked with the packed 6-D output vector.
    template <typename MapType, typename Func>
    void visit_voxels_as_storage_type(const MapType& map, Func&& func) const {
        const auto origin = StorageImpl::V3AbsType::Zero();
        this->visit_voxels_as_point_type(map, origin, func);
    }

    /// Invoke @p func with the average position under the same conditions
    /// visit_voxels_as_point_type() emits a point.
    /// @param[in] map  The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @param[in] func Callable invoked with the StorageImpl::XyzType average position.
    template <typename MapType, typename Func>
    void visit_voxel_positions(const MapType& map, Func&& func) const {
        if (emits_point(map)) {
            const typename StorageImpl::XyzType average =
                this->point_sum / static_cast<double>(this->count);
            std::forward<Func>(func)(average);
        }
    }

    /// Return 1 if the visitors emit a point for this bucket, 0 otherwise.
    /// @param[in] map The owning VoxelHashMapT; queried for min_points_per_voxel.
    /// @return 1 if count >= min_points_per_voxel and the summed normal is not
    ///         degenerate, 0 otherwise.
    template <typename MapType>
    std::size_t point_count(const MapType& map) const noexcept {
        return emits_point(map) ? 1u : 0u;
    }

   private:
    // The one condition for this bucket producing a point. Callers size their
    // output from point_count() and then fill it through a visitor, so both
    // must agree or rows are left uninitialised.
    template <typename MapType>
    bool emits_point(const MapType& map) const noexcept {
        return this->count >= map.min_points_per_voxel() && this->normal_sum.norm() > 1e-12;
    }
};

/// @brief See @ref ouster::sdk::core::DefaultVoxelBucketTarget
template <typename PointType>
struct OUSTER_API_CLASS DefaultVoxelBucket {
    /// @cond
    template <VHMInternalStorageMode StorageMode>
    using BucketType = DefaultVoxelBucketTarget<PointType, StorageMode>;
    /// @endcond
};

/// @brief See @ref ouster::sdk::core::AveragePointBucketTarget
template <typename PointType>
struct OUSTER_API_CLASS AveragePointBucket {
    /// @cond
    template <VHMInternalStorageMode StorageMode>
    using BucketType = AveragePointBucketTarget<PointType>;
    /// @endcond
};

/// @brief See @ref ouster::sdk::core::PointNormalBucketTarget
struct OUSTER_API_CLASS PointNormalBucket {
    /// @cond
    template <VHMInternalStorageMode StorageMode>
    using BucketType = PointNormalBucketTarget;
    /// @endcond
};

namespace impl {
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
// DefaultVoxelBucket-style (has ::PointsArrayType) and AveragePointBucket-style
// (has ::PointType but no ::PointsArrayType) buckets. C++14 SFINAE; no if constexpr.
// ---------------------------------------------------------------------------
struct has_points_tag {};
struct no_points_tag {};

template <typename VB, typename = void>
struct bucket_dispatch {
    using tag = no_points_tag;
};
template <typename VB>
struct bucket_dispatch<VB, decltype(void(std::declval<typename VB::PointsArrayType>()))> {
    using tag = has_points_tag;
};

// ---------------------------------------------------------------------------
// BucketStoresAttributes: true unless a bucket opts out by declaring
// `static constexpr bool stores_attributes = false;`. A bucket that opts out
// would silently discard the packed attribute records handed to its insertion
// strategy, so the map refuses a non-empty AttributeLayout for it up front.
// Buckets that say nothing keep the default, which leaves third-party bucket
// types compiling unchanged.
// ---------------------------------------------------------------------------
template <typename Bucket, typename = void>
struct BucketStoresAttributes : std::true_type {};
template <typename Bucket>
struct BucketStoresAttributes<Bucket, typename std::enable_if<!Bucket::stores_attributes>::type>
    : std::false_type {};

/// Everything about the owning map an insertion strategy needs, without handing
/// it the map itself.
struct OUSTER_API_CLASS InsertContext {
    /// Describes the packed attribute record accompanying each inserted point.
    const AttributeLayout& layout;
    /// Size of each voxel in coordinate units.
    double voxel_size;
    /// Maximum number of points a voxel may hold.
    std::size_t max_points_per_voxel;
    /// Squared rejection radius for near-duplicate points.
    double map_resolution_sq;
};

/// Which end of insertion order impl::n_point_strategy keeps once a voxel
/// reaches max_points_per_voxel.
enum class NPointSelection {
    FIRST,  ///< Keep the first points admitted; later arrivals are dropped.
    LAST,   ///< Keep the last points admitted; a full voxel evicts its oldest.
};

/// Built-in insertion strategy: admits a point if the voxel is below the
/// capacity cap and no existing point lies within map_resolution_sq of it.
/// Once the voxel is full, FIRST (first_n_point) drops the arriving point,
/// so the points kept are the first ones admitted; LAST (last_n_point)
/// instead evicts the voxel's oldest kept point, so the points kept are
/// always exactly the most recently admitted ones. The near-duplicate check
/// applies the same way for both -- a point that is essentially already
/// represented is dropped rather than displacing an existing one -- so
/// LAST's "most recent" is in terms of admitted (not near-duplicate), not
/// necessarily every point physically inserted. The bucket's points are kept
/// in admission order (oldest first), so eviction is dropping the front
/// element; this costs O(max_points_per_voxel) per admitted point once the
/// voxel is full and needs no extra per-bucket state. bucket.count still
/// counts every point offered, duplicates included, but plays no part in
/// eviction.
template <NPointSelection Selection>
struct OUSTER_API_CLASS n_point_strategy {
    template <typename PointType, typename BucketType>
    static void apply(const typename BucketType::StorageImpl::V3AbsType& voxel_origin,
                      BucketType& bucket, const PointType& point, const std::uint8_t* record,
                      const InsertContext& ctx) {
        using StorageImpl = typename BucketType::StorageImpl;
        using StorageType = typename BucketType::StorageType;
        using DistType = typename StorageImpl::DistType;

        ++bucket.count;

        auto& bucket_arr = bucket.points_arr;
        StorageType p0_internal =
            StorageImpl::point_to_internal_storage(voxel_origin, point, ctx.voxel_size);

        auto p0_xyz = StorageImpl::get_xyz_component(p0_internal);
        DistType thresh_sq =
            StorageImpl::dist_sq_to_dist_type(ctx.map_resolution_sq, ctx.voxel_size);

        const bool near_duplicate = std::any_of(
            bucket_arr.cbegin(), bucket_arr.cend(), [&](const StorageType& p1_internal) {
                const auto p1_xyz = StorageImpl::get_xyz_component(p1_internal);
                DistType norm_sq = 0;
                for (int idx = 0; idx < 3; ++idx) {
                    DistType elem = static_cast<DistType>(p1_xyz.array()[idx]) -
                                    static_cast<DistType>(p0_xyz.array()[idx]);
                    norm_sq += elem * elem;
                }
                return norm_sq < thresh_sq;
            });

        if (near_duplicate) {
            return;
        }

        if (bucket_arr.size() < ctx.max_points_per_voxel) {
            bucket.append(p0_internal, record, ctx.layout.stride());
            return;
        }

        // The voxel is full and the arriving point is not a near-duplicate of
        // anything already kept.
        if (Selection == NPointSelection::FIRST) {
            return;  // Keep the existing points; drop the newcomer.
        }

        // LAST: points_arr is kept in admission order (append() above only ever
        // adds at the back), so the oldest kept point is always at the front.
        // Drop it and append the newcomer, so the voxel holds exactly the last
        // max_points_per_voxel admitted points regardless of how many
        // near-duplicates were rejected in between.
        bucket.pop_front_and_append(p0_internal, record, ctx.layout.stride());
    }
};

using first_n_point = n_point_strategy<NPointSelection::FIRST>;
using last_n_point = n_point_strategy<NPointSelection::LAST>;

/// Insertion strategy for AveragePointBucket: accumulates every inserted point
/// into the bucket's running sum and increments its count. When
/// pointcloud_double() or pointcloud_vector() is called the map divides the sum by
/// the count to produce the per-voxel average.
struct OUSTER_API_CLASS accumulate_strategy {
    template <typename PointType, typename BucketType>
    static void apply(const typename BucketType::StorageImpl::V3AbsType& /*voxel_origin*/,
                      BucketType& bucket, const PointType& point, const std::uint8_t* record,
                      const InsertContext& ctx) {
        bucket.accumulate(point, record, ctx.layout);
    }
};

/// Insertion strategy for PointNormalBucket: splits the input point (a packed
/// [position | normal] 6-vector) into its two halves, accumulating positions into
/// point_sum and pre-normalized normals into normal_sum. When
/// pointcloud_double() / pointcloud_vector() is called, visit_voxel averages the
/// position and re-normalizes the summed normal direction.
///
/// The attribute record is ignored: PointNormalBucket has nowhere to keep it.
/// Maps paired with this strategy declare
/// PointNormalBucketTarget::stores_attributes == false, so a non-empty
/// AttributeLayout is rejected at construction and no record ever reaches here.
struct OUSTER_API_CLASS accumulate_point_normal_strategy {
    template <typename PointType, typename BucketType>
    static void apply(const typename BucketType::StorageImpl::V3AbsType& /*voxel_origin*/,
                      BucketType& bucket, const PointType& point, const std::uint8_t* /*record*/,
                      const InsertContext& /*ctx*/) {
        static_assert(PointType::SizeAtCompileTime == 6,
                      "accumulate_point_normal_strategy expects a packed [xyz | normal] 6-vector");
        bucket.point_sum += point.template head<3>();
        bucket.normal_sum += point.template tail<3>();
        ++bucket.count;
    }
};

template <typename VoxelIndex, typename VoxelBucketType>
struct VoxelHashMapInternal {
    using VoxelType = VoxelIndex;
    using BucketType = VoxelBucketType;
    tsl::robin_map<VoxelIndex, BucketType, typename impl::DefaultVoxelHash<VoxelIndex>::type,
                   std::equal_to<>>
        map;
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
///                          are supported as long as they expose a `PointsArrayType`.
template <typename VoxelIndex = Eigen::Vector3i, typename PointType = Eigen::Vector3d,
          typename VoxelBucketType = DefaultVoxelBucket<PointType>,
          typename InsertionStrategy = impl::first_n_point>
struct OUSTER_API_CLASS VoxelHashMapT {
    /// Point type alias for template consistency.
    using point_type = PointType;

    /// Insertion strategy type alias; names which VoxelDownsampleStrategy the
    /// map implements.
    using insertion_strategy_type = InsertionStrategy;

    // Defaults come from the public VoxelHashMap, their only definition.

    /// Default max_distance configuration
    static constexpr double DEFAULT_MAX_DISTANCE = VoxelHashMap::DEFAULT_MAX_DISTANCE;

    /// Default max_points_per_voxel configuration
    static constexpr std::size_t DEFAULT_MAX_POINTS_PER_VOXEL =
        VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL;

    /// Default min_points_per_voxel configuration
    static constexpr std::size_t DEFAULT_MIN_POINTS_PER_VOXEL =
        VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL;

    /// Default internal_storage_mode configuration
    static constexpr VHMInternalStorageMode DEFAULT_INTERNAL_STORAGE_MODE =
        VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE;

    /// Constructor for the voxel hash map.
    ///
    /// Each point carries `attribute_layout.element_count()` attributes, stored in the types the
    /// layout declares rather than widened to double. The Eigen-based API (add_points(),
    /// pointcloud_double(), get_closest_neighbor()) continues to speak in doubles and converts on
    /// the way in and out; use add_points()/pointcloud_attributed() with packed records to keep the
    /// original types end to end. Pass an empty layout (the default) for a map without attributes.
    ///
    /// @param[in] voxel_size            Size of each voxel in coordinate units.
    /// @param[in] max_distance          Maximum distance to retain points from a reference.
    /// @param[in] max_points_per_voxel  Maximum number of points allowed per voxel.
    /// @param[in] min_points_per_voxel  Minimum number of points required in a voxel to be
    /// considered valid.
    /// @param[in] attribute_layout      Types and widths of the per-point attributes; empty for
    /// none. Must be empty for fixed-size point types and buckets that cannot store attributes.
    /// @param[in] internal_storage_mode The internal storage representation for points stored with
    /// the voxels
    /// @throw std::invalid_argument if a parameter is out of range or @p attribute_layout is
    /// not supported by this map type.
    explicit VoxelHashMapT(
        double voxel_size, double max_distance = DEFAULT_MAX_DISTANCE,
        std::size_t max_points_per_voxel = DEFAULT_MAX_POINTS_PER_VOXEL,
        std::size_t min_points_per_voxel = DEFAULT_MIN_POINTS_PER_VOXEL,
        AttributeLayout attribute_layout = AttributeLayout(),
        VHMInternalStorageMode internal_storage_mode = DEFAULT_INTERNAL_STORAGE_MODE);

    /// Check if the hash map is empty.
    /// @return True if no voxels are present, false otherwise.
    bool empty() const {
        return dispatch_storage([](const auto& map) { return map.map.empty(); });
    }

    /// Clear all voxels from the map.
    void clear() {
        dispatch_storage([](auto& map) { map.map.clear(); });
    }

    /// Insert points and trim voxels that are far from @p position.
    /// @param[in] points            Vector of points to insert.
    /// @param[in] position   Reference position for distance trimming.
    /// @throw std::invalid_argument if any point's size differs from
    ///        point_cols(); no point is inserted in that case.
    void update(const std::vector<PointType>& points, const PointType& position);

    /// Insert points into the map.
    ///
    /// @param[in] points    Vector of points to insert.
    /// @throw std::invalid_argument if any point's size differs from
    ///        point_cols(); every point is checked before any is inserted.
    void add_points(const std::vector<PointType>& points);

    /// Insert points from an Eigen matrix into the map.
    ///
    /// Only participates in overload resolution when PointType is a plain
    /// Eigen vector (e.g., Vector3d, VectorXd). Not available for structured
    /// point types such as IndexedPoint; use add_point() or add_points(vector)
    /// for those.
    ///
    /// @param[in] points    Row-major matrix where each row is a point.
    void add_points(const Eigen::Ref<const ArrayXXdR>& points) {
        if (points.cols() != point_cols()) {
            throw std::invalid_argument(
                "VoxelHashMap::add_points received unexpected point dimension");
        }
        // Every row is exactly point_cols() wide by construction, so add_point()'s
        // per-point size check would only ever repeat the check above.
        for (Eigen::Index row = 0; row < points.rows(); ++row) {
            const PointType point = points.matrix().row(row).transpose();
            add_point_unchecked(point);
        }
    }

    /// Insert points whose attributes are already packed in the map's layout.
    ///
    /// Nothing is converted: the `attribute_layout().stride()` bytes belonging
    /// to each row are copied into the voxel as-is, so integer attributes keep
    /// their exact values and their original width.
    ///
    /// The records are read in place: any row-major buffer with contiguous
    /// rows binds to @p records without a copy.
    ///
    /// @param[in] xyz     Row-major Nx3 array of point positions.
    /// @param[in] records `xyz.rows()` x `attribute_layout().stride()` packed
    ///                    attribute records, one row per row of @p xyz. May be
    ///                    empty when the layout has no attributes.
    /// @throw std::invalid_argument if @p records is not
    ///        `xyz.rows()` x `attribute_layout().stride()`.
    void add_points(const Eigen::Ref<const ArrayX3dR>& xyz,
                    const Eigen::Ref<const AttributeRecords>& records);

    /// Insert a pointcloud read back from a map, e.g. by pointcloud_attributed()
    /// or extract_voxels_far_from_location_attributed().
    ///
    /// Unlike the (xyz, records) overload, this also checks the attribute
    /// types rather than just the record width, so records laid out for a
    /// different layout of the same stride are rejected instead of being
    /// silently reinterpreted.
    ///
    /// @param[in] cloud Positions, packed records and the layout describing them.
    /// @throw std::invalid_argument if `cloud.layout` differs from
    ///        attribute_layout(), or if the records are not
    ///        `cloud.xyz.rows()` x `attribute_layout().stride()`.
    void add_points(const AttributedPointCloud& cloud);

    /// Insert a single point with an already-packed attribute record.
    /// @param[in] xyz    The point's position.
    /// @param[in] record `attribute_layout().stride()` bytes, or empty when
    ///                   the layout has no attributes.
    /// @throw std::invalid_argument if record.size() does not equal
    ///        attribute_layout().stride().
    void add_point(const Eigen::Ref<const Eigen::Vector3d>& xyz,
                   const Eigen::Ref<const AttributeRecord>& record);

    /// Get the complete pointcloud with attributes in their original types.
    /// @return Positions plus packed attribute records; see AttributedPointCloud.
    AttributedPointCloud pointcloud_attributed() const;

    /// Extract voxels far from a specified location, keeping attribute types.
    /// @param[in] origin Location to measure distance from.
    /// @return Positions plus packed attribute records of the extracted voxels.
    AttributedPointCloud extract_voxels_far_from_location_attributed(const PointType& origin);

    /// Remove voxels far from a specified location.
    /// @param[in] origin Location to measure distance from.
    void remove_voxels_far_from_location(const PointType& origin);
    /// extract_voxels_far_from_location_attributed(), with attributes widened
    /// to double; see pointcloud_double() for why.
    /// @param[in] origin Location to measure distance from.
    /// @return Matrix of points from extracted voxels.
    ArrayXXdR extract_voxels_far_from_location_double(const PointType& origin);

    /// Get a complete pointcloud as a vector.
    /// @return Vector containing all points from all voxels.
    std::vector<PointType> pointcloud_vector() const;
    /// pointcloud_vector(), converting each point to @p OutPoint as it is
    /// collected rather than in a second pass. Defined, and only usable, in
    /// voxel_hash_map.cpp.
    /// @tparam OutPoint Element type of the result; constructible from PointType.
    /// @return Vector containing all points from all voxels.
    template <typename OutPoint>
    std::vector<OutPoint> pointcloud_vector_as() const;
    /// Get a complete pointcloud as one Eigen matrix of doubles.
    ///
    /// Every value of a point is widened to double: its position, its
    /// attributes (whatever type the layout stores them in), and any extra
    /// fixed-size coordinates PointType carries outside the layout -- e.g. the
    /// row index of a Vector4d point, or the normal of a point-normal map.
    /// pointcloud_attributed() keeps attribute types but returns only the
    /// position and the layout's records, so it drops such extra coordinates.
    /// @return Matrix with one row of point_cols() doubles per point.
    ArrayXXdR pointcloud_double() const;

    /// Find the closest point in the map to a query point.
    /// @param[in] query             The query point.
    /// @param[in] max_distance_sq   Maximum squared distance to search.
    /// @return Tuple of (closest_point, squared_distance). If no point is
    ///         found within max_distance_sq, squared_distance will be >=
    ///         max_distance_sq.
    std::tuple<PointType, double> get_closest_neighbor(
        const PointType& query, double max_distance_sq = std::numeric_limits<double>::max()) const;

    /// Find the position of the closest point in the map to a query position.
    ///
    /// The same search as get_closest_neighbor(), but only positions are
    /// visited and returned: no attribute is decoded and, whatever the
    /// PointType or bucket, nothing is allocated.
    ///
    /// @param[in] query             The query position.
    /// @param[in] max_distance_sq   Maximum squared distance to search.
    /// @return (closest_position, squared_distance). If no point is found
    ///         within max_distance_sq, squared_distance will be >=
    ///         max_distance_sq (and the position is zero).
    std::pair<Eigen::Vector3d, double> get_closest_neighbor_position(
        const Eigen::Vector3d& query,
        double max_distance_sq = std::numeric_limits<double>::max()) const;

    /// Convert a point to its voxel index.
    /// @param[in] point           The query point.
    /// @param[in] inv_voxel_size  Inverse of voxel size (1.0 / voxel_size).
    /// @return Voxel index containing the point.
    static VoxelIndex point_to_voxel(const PointType& point, const double inv_voxel_size) {
        return impl::xyz_to_voxel<VoxelIndex>(impl::spatial_view(point), inv_voxel_size);
    }
    /// Convert a point to its voxel index using the cached inverse voxel size.
    /// @param[in] point The query point.
    /// @return Voxel index containing the point.
    VoxelIndex point_to_voxel(const PointType& point) const {
        return point_to_voxel(point, inv_voxel_size_);
    }

    /// Returns the row width of a serialized point.
    /// @return the row width of a serialized.
    Eigen::Index point_cols() const {
        return PointType::SizeAtCompileTime == Eigen::Dynamic
                   ? static_cast<Eigen::Index>(3 + attribute_layout_.element_count())
                   : static_cast<Eigen::Index>(PointType::SizeAtCompileTime);
    }

    /// @brief Types and widths of the per-point attributes stored in this map.
    /// @return The attribute layout; empty when the map stores positions only.
    const AttributeLayout& attribute_layout() const {
        return attribute_layout_;
    }

    /// Rebuild the full double-valued point a bucket visit produced.
    ///
    /// Buckets hand back a position plus, for maps with an attribute layout, the
    /// packed attribute record that goes with it. A null @p record means the
    /// point is already complete (fixed-size point types, and the averaging
    /// buckets, which accumulate their attributes as doubles).
    ///
    /// @param[in] point  Position, or complete point when @p record is nullptr.
    /// @param[in] record Packed attribute record, or nullptr.
    /// @return The point with its attributes appended as doubles.
    PointType assemble_point(const PointType& point, const std::uint8_t* record) const {
        if (record == nullptr) {
            return point;
        }
        PointType result = impl::PointDefaultValue<PointType>::make(point_cols());
        result.template head<3>() = point.template head<3>();
        attribute_layout_.decode(record, result.data() + 3);
        return result;
    }

    /// Insert a single point into the map.
    /// @param[in] point The point to insert.
    /// @throw std::invalid_argument if point.size() differs from point_cols().
    void add_point(const PointType& point);

    /// @brief Returns the internal storage representation of points stored in the voxels
    /// @return Internal storage mode
    VHMInternalStorageMode storage_mode() const {
        return storage_mode_;
    }

    /* TODO[UN]: AD HOC PUBLIC MEMBERS for now..
    in the future let the insertion and summarization strategies define these values */
    /// Return the maximum number of points allowed per voxel.
    /// @return Maximum points per voxel as set at construction.
    std::size_t max_points_per_voxel() const {
        return max_points_per_voxel_;
    }

    /// Return the minimum number of raw insertions required for a voxel to appear in the output.
    /// @return Minimum points threshold as set at construction.
    std::size_t min_points_per_voxel() const {
        return min_points_per_voxel_;
    }

    /// @brief The voxel size
    /// @return The voxel size
    double voxel_size() const {
        return voxel_size_;
    }

    /// @brief Maximum distance of points to retain
    /// @return The maximum distance
    double max_distance() const {
        return max_distance_;
    }

    /// @brief Squared size of internal voxel cells for rejecting overlapping points
    /// @return The resolution squared
    double map_resolution_sq() const {
        return map_resolution_sq_;
    }

    /// @brief Inverse of the voxel size
    /// @return Inverse of the voxel size
    double inv_voxel_size() const {
        return inv_voxel_size_;
    }

    /// Per-map insertion strategy instance
    InsertionStrategy strategy_;

   private:
    double voxel_size_;
    double max_distance_;
    std::size_t max_points_per_voxel_;
    std::size_t min_points_per_voxel_;
    AttributeLayout attribute_layout_;

    VHMInternalStorageMode storage_mode_;
    impl::VoxelHashMapInternal<
        VoxelIndex, typename VoxelBucketType::template BucketType<VHMInternalStorageMode::ABS>>
        map_abs_;
    impl::VoxelHashMapInternal<
        VoxelIndex, typename VoxelBucketType::template BucketType<VHMInternalStorageMode::FX16>>
        map_fx16_;
    impl::VoxelHashMapInternal<
        VoxelIndex, typename VoxelBucketType::template BucketType<VHMInternalStorageMode::FX8>>
        map_fx8_;

    // cached values
    double map_resolution_sq_;
    double inv_voxel_size_;
    // Staging buffer used to pack a point's attributes when they arrive as
    // doubles and the layout stores them narrower. Sized once at construction.
    std::vector<std::uint8_t> encode_scratch_;
    // Reused by the packed-record insertion path so that inserting a point
    // never allocates, even when PointType is dynamically sized.
    PointType xyz_scratch_;

    /// @cond
    // Invokes `func` with the internal map selected by storage_mode_ -- one of
    // map_abs_, map_fx16_ or map_fx8_, each a different type -- and returns
    // whatever it returns, so every member dispatches over the storage
    // representations through this one switch. `func` must be a generic
    // callable (e.g. `[&](auto& map) {...}`) returning the same type for all
    // three. `Self` lets the const and non-const overloads below share it.
    template <typename Self, typename Func>
    static decltype(auto) dispatch_storage_impl(Self& self, Func&& func) {
        switch (self.storage_mode_) {
            case VHMInternalStorageMode::ABS:
                return func(self.map_abs_);
            case VHMInternalStorageMode::FX16:
                return func(self.map_fx16_);
            case VHMInternalStorageMode::FX8:
                return func(self.map_fx8_);
            default:
                throw std::logic_error("Invalid VHM storage_mode_");
        }
    }
    template <typename Func>
    decltype(auto) dispatch_storage(Func&& func) {
        return dispatch_storage_impl(*this, std::forward<Func>(func));
    }
    template <typename Func>
    decltype(auto) dispatch_storage(Func&& func) const {
        return dispatch_storage_impl(*this, std::forward<Func>(func));
    }

    // Packs `point`'s attribute tail into the map's layout, returning a pointer
    // valid until the next call. Null when the map has no attributes. When
    // every attribute is already a double the point's own storage is handed
    // back directly, so nothing is copied.
    const std::uint8_t* pack_attributes(const PointType& point) {
        // add_point() has already checked that `point` is exactly
        // point_cols() wide, so the attribute tail is complete.
        if (attribute_layout_.stride() == 0) {
            return nullptr;
        }
        if (attribute_layout_.all_double()) {
            return reinterpret_cast<const std::uint8_t*>(point.data() + 3);
        }
        attribute_layout_.encode(point.data() + 3, encode_scratch_.data());
        return encode_scratch_.data();
    }

    // Throws unless `point` is exactly point_cols() wide. Buckets size their
    // storage from point_cols(), so any other width would reach e.g. the
    // averaging bucket's `point += value` with mismatched sizes. Free for
    // fixed-size PointTypes, where both sides are compile-time constants.
    void check_point_size(const PointType& point) const {
        if (point.size() != point_cols()) {
            throw std::invalid_argument("VoxelHashMap received a point of " +
                                        std::to_string(point.size()) + " values, expected " +
                                        std::to_string(point_cols()) +
                                        " (3 + the attribute layout's element count)");
        }
    }

    // Inserts `xyz` with an already-packed attribute record (or nullptr when
    // the layout has no attributes), reusing xyz_scratch_ so that a single
    // insertion never allocates. `record`'s size against attribute_layout_ is
    // the caller's responsibility: add_point() and add_points() below check it
    // once, up front, so this inner-loop helper doesn't re-check per point.
    void add_point_record(const Eigen::Ref<const Eigen::Vector3d>& xyz, const std::uint8_t* record);

    // add_point() without the point_cols() size check. `point`'s size is the
    // caller's responsibility: add_points() below checks every point once, up
    // front, then calls this per point instead of add_point() so the same
    // size is never checked twice.
    void add_point_unchecked(const PointType& point);
    /// @endcond
};

// Out-of-line definitions for the static constexpr data members declared above.
// In C++14 these members are not implicitly inline, so they need a namespace-
// scope definition whenever they are ODR-used. They are ODR-used through perfect
// forwarding, e.g. std::make_unique<VoxelHashMap3d>(..., DEFAULT_MIN_POINTS_PER_VOXEL, ...) and
// nanobind's py::arg(...) = DEFAULT_..., both of which bind a reference to the member. Without
// these definitions the link fails with undefined references.
// (Redundant but valid under C++17, where they are implicitly inline.)
template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
constexpr double
    VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::DEFAULT_MAX_DISTANCE;

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
constexpr std::size_t VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                                    InsertionStrategy>::DEFAULT_MAX_POINTS_PER_VOXEL;

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
constexpr std::size_t VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                                    InsertionStrategy>::DEFAULT_MIN_POINTS_PER_VOXEL;

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
constexpr VHMInternalStorageMode VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                                               InsertionStrategy>::DEFAULT_INTERNAL_STORAGE_MODE;

// The "plain" aliases (VoxelHashMap3d/Xd) keep the original 24-byte bucket
// footprint and are what SLAM and the CLI map exporter use today.
using VoxelHashMap3d =
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>>;
using VoxelHashMapXd =
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>>;

/// Keeps, per voxel, the up to max_points_per_voxel most recently admitted
/// points (see impl::last_n_point): once a voxel is full, an arriving point
/// evicts whichever kept point has been there longest.
using LastNPointVoxelHashMap3d =
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>,
                  impl::last_n_point>;
using LastNPointVoxelHashMapXd =
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                  impl::last_n_point>;

using AverageVoxelHashMap3d =
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d, AveragePointBucket<Eigen::Vector3d>,
                  impl::accumulate_strategy>;
using AverageVoxelHashMapXd =
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, AveragePointBucket<Eigen::VectorXd>,
                  impl::accumulate_strategy>;

/// VoxelHashMapT for joint position+normal averaging. Expects 6-D packed input
/// [x,y,z,nx,ny,nz] from accumulate_point_normal_strategy; pointcloud_double() returns
/// an Nx6 matrix [avg_position | renormalized_avg_normal].
using PointNormalVoxelHashMap3d =
    VoxelHashMapT<Eigen::Vector3i, Eigen::Matrix<double, 6, 1>, PointNormalBucket,
                  impl::accumulate_point_normal_strategy>;

namespace impl {

/// True if @p v is NaN. Always false for integer types; handles Eigen::half,
/// which std::isnan does not.
template <typename T>
inline bool is_nan(T v) {
    return Eigen::numext::isnan(v);
}

struct OUSTER_API_CLASS random_selection_strategy {
    std::uint32_t rng_state = 42u;

    template <typename PointType, typename BucketType>
    void apply(const typename BucketType::StorageImpl::V3AbsType& voxel_origin, BucketType& bucket,
               const PointType& point, const std::uint8_t* record, const InsertContext& ctx) {
        apply_impl(voxel_origin, bucket, point, record, ctx,
                   typename impl::bucket_dispatch<BucketType>::tag{});
    }

   private:
    std::uint32_t fast_rand() noexcept {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 17;
        rng_state ^= rng_state << 5;
        return rng_state;
    }

    // DefaultVoxelBucket path: Algorithm R reservoir sampling of up to
    // max_points_per_voxel points. Each arriving point is kept with
    // probability k/n (k = max_points_per_voxel, n = total seen).
    template <typename PointType, typename BucketType>
    void apply_impl(const typename BucketType::StorageImpl::V3AbsType& voxel_origin,
                    BucketType& bucket, const PointType& point, const std::uint8_t* record,
                    const InsertContext& ctx, impl::has_points_tag /*tag*/) {
        using StorageImpl = typename BucketType::StorageImpl;
        using StorageType = typename BucketType::StorageType;

        StorageType pt_internal =
            StorageImpl::point_to_internal_storage(voxel_origin, point, ctx.voxel_size);
        auto& bucket_arr = bucket.points_arr;
        const std::size_t stride = ctx.layout.stride();
        ++bucket.count;
        if (bucket_arr.size() < ctx.max_points_per_voxel) {
            bucket.append(pt_internal, record, stride);
        } else {
            // Pick j in [0, count). Replace only when j < k (Algorithm R).
            const std::size_t j = static_cast<std::size_t>(
                (static_cast<std::uint64_t>(fast_rand()) * bucket.count) >> 32);
            if (j < ctx.max_points_per_voxel) {
                bucket.replace(j, pt_internal, record, stride);
            }
        }
    }

    // AveragePointBucket path: Algorithm R (k=1). Maintains b.point == selected * b.count
    // so that visit_voxel (which divides b.point / b.count) returns the selected point.
    // max_points_per_voxel is ignored; the bucket holds exactly one representative.
    template <typename PointType, typename BucketType>
    void apply_impl(const typename BucketType::StorageImpl::V3AbsType& /*voxel_origin*/,
                    BucketType& bucket, const PointType& point, const std::uint8_t* record,
                    const InsertContext& ctx, impl::no_points_tag /*tag*/) {
        ++bucket.count;
        // j in [0, count); probability of replace == 1/count (Algorithm R).
        // When count==1 fast_rand()*1>>32 == 0 always, so first point is always selected.
        const std::size_t j = static_cast<std::size_t>(
            (static_cast<std::uint64_t>(fast_rand()) * bucket.count) >> 32);
        if (j == 0) {
            bucket.assign(point, record, ctx.layout);
        }
    }
};

/// Which end of an attribute's value range impl::extremum_strategy keeps.
enum class ExtremumDirection {
    SMALLEST,  ///< Keep the points with the smallest attribute value.
    LARGEST,   ///< Keep the points with the largest attribute value.
};

/// Insertion strategy that keeps, per voxel, the up to max_points_per_voxel
/// points with the smallest (SMALLEST) or largest (LARGEST) value of one
/// scalar attribute of any AttributeType, named by index rather than by
/// meaning -- e.g. RANGE to keep the points closest to (SMALLEST) or farthest
/// from (LARGEST) the sensor, but any scalar attribute works. Once a voxel is
/// full, an arriving point replaces whichever kept point currently has the
/// least favorable value, if the arriving point's value is more favorable.
///
/// Used by VoxelHashMap::create()'s SMALLEST and LARGEST strategies. Default
/// construction leaves the strategy unconfigured; apply() throws
/// std::invalid_argument if used that way. Construct with (layout,
/// attribute_index) instead to validate attribute_index once -- it must name
/// a scalar attribute of layout, or the constructor throws
/// std::invalid_argument -- and cache its offset and type, so apply() does no
/// per-point layout lookups.
template <ExtremumDirection Direction>
struct OUSTER_API_CLASS extremum_strategy {
    extremum_strategy() = default;

    extremum_strategy(const AttributeLayout& layout, std::size_t attribute_index) {
        // NO_ATTRIBUTE_INDEX (SIZE_MAX) is always out of range for any real
        // layout, so this one check also rejects it -- no separate case needed.
        if (attribute_index >= layout.specs().size()) {
            throw std::invalid_argument(
                "extremum_strategy requires attribute_index to name a scalar attribute");
        }
        const AttributeSpec& spec = layout.specs()[attribute_index];
        if (spec.count != 1) {
            throw std::invalid_argument(
                "extremum_strategy's attribute_index must name a scalar attribute");
        }
        offset_ = layout.byte_offset(attribute_index);
        type_ = spec.type;
        configured_ = true;
    }

    template <typename PointType, typename BucketType>
    void apply(const typename BucketType::StorageImpl::V3AbsType& voxel_origin, BucketType& bucket,
               const PointType& point, const std::uint8_t* record, const InsertContext& ctx) const {
        if (!configured_) {
            throw std::invalid_argument(
                "extremum_strategy requires attribute_index to name a scalar attribute");
        }

        using StorageImpl = typename BucketType::StorageImpl;
        using StorageType = typename BucketType::StorageType;
        StorageType pnt_internal =
            StorageImpl::point_to_internal_storage(voxel_origin, point, ctx.voxel_size);
        const std::size_t stride = ctx.layout.stride();

        auto& bucket_arr = bucket.points_arr;
        ++bucket.count;
        if (bucket_arr.size() < ctx.max_points_per_voxel) {
            bucket.append(pnt_internal, record, stride);
            return;
        }

        // Dispatch on type_ once per insertion, then run the whole eviction
        // scan with that concrete type -- one dispatch rather than one per
        // comparison. O(bucket size), same as first_n_point's near-duplicate
        // scan.
        const std::size_t offset = offset_;
        visit_attribute_type(type_, [&](auto zero) {
            using T = decltype(zero);
            // True when `a` is more favorable to keep than `b`. A NaN value
            // (possible for a float/double/half attribute) is never more
            // favorable than anything, in either direction: without this, a
            // NaN that ends up tracked as the bucket's "worst" value could
            // never be displaced -- every comparison against it is false --
            // permanently freezing that slot regardless of what arrives later.
            auto more_favorable = [](T a, T b) {
                if (is_nan(a)) return false;
                if (is_nan(b)) return true;
                return Direction == ExtremumDirection::SMALLEST ? a < b : a > b;
            };

            T incoming_value;
            std::memcpy(&incoming_value, record + offset, sizeof(T));

            std::size_t worst_index = 0;
            T worst_value;
            std::memcpy(&worst_value, bucket.record_at(0, stride) + offset, sizeof(T));
            for (std::size_t i = 1; i < bucket_arr.size(); ++i) {
                T candidate_value;
                std::memcpy(&candidate_value, bucket.record_at(i, stride) + offset, sizeof(T));
                if (more_favorable(worst_value, candidate_value)) {
                    worst_value = candidate_value;
                    worst_index = i;
                }
            }
            if (more_favorable(incoming_value, worst_value)) {
                bucket.replace(worst_index, pnt_internal, record, stride);
            }
        });
    }

   private:
    std::size_t offset_ = 0;
    AttributeType type_ = AttributeType::U8;
    bool configured_ = false;
};

using smallest_strategy = extremum_strategy<ExtremumDirection::SMALLEST>;
using largest_strategy = extremum_strategy<ExtremumDirection::LARGEST>;

}  // namespace impl

// Reservoir-sampling aliases: keep up to max_points_per_voxel uniformly-sampled
// points per voxel. Defined here (not next to the plain aliases) because
// random_selection_strategy lives in the impl namespace block above.
using RandomVoxelHashMap3d =
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>,
                  impl::random_selection_strategy>;
using RandomVoxelHashMapXd =
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                  impl::random_selection_strategy>;

/// Keeps, per voxel, the up to max_points_per_voxel points with the smallest
/// (SmallestVoxelHashMapXd) or largest (LargestVoxelHashMapXd) value of a
/// designated attribute (see impl::extremum_strategy). Only meaningful for
/// attributed (dynamic-size) points, since the ranking attribute rides in the
/// packed record; used by VoxelHashMap::create()'s SMALLEST and LARGEST
/// strategies.
using SmallestVoxelHashMapXd =
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                  impl::smallest_strategy>;
using LargestVoxelHashMapXd =
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                  impl::largest_strategy>;

}  // namespace core
}  // namespace sdk
}  // namespace ouster
