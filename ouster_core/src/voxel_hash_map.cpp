#include <ouster/core/attribute_layout.h>
#include <ouster/core/eigen_hash.h>
#include <ouster/core/impl/voxel_hash_map_t.h>
#include <ouster/core/typedefs.h>
#include <ouster/core/visibility.h>
#include <ouster/core/voxel_hash_map.h>

#include <algorithm>
#include <array>
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

#include "bluenoise32x32x32x3.inc"

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::VoxelHashMapT(
    double voxel_size, double max_distance, std::size_t max_points_per_voxel,
    std::size_t min_points_per_voxel, AttributeLayout attribute_layout,
    VHMInternalStorageMode internal_storage_mode)
    : voxel_size_(voxel_size),
      max_distance_(max_distance),
      max_points_per_voxel_(max_points_per_voxel),
      min_points_per_voxel_(min_points_per_voxel),
      attribute_layout_(std::move(attribute_layout)),
      storage_mode_(VHMInternalStorageMode::ABS) {
    if (max_points_per_voxel_ == 0) {
        throw std::invalid_argument("max_points_per_voxel must be greater than 0");
    }
    if (voxel_size_ <= 0) {
        throw std::invalid_argument("voxel_size must be greater than 0");
    }
    if (max_distance_ <= 0) {
        throw std::invalid_argument("max_distance must be greater than 0");
    }
    // Attributes only make sense for dynamic-size point types. For fixed
    // sizes (e.g. Vector3d) the layout must be empty or get_closest_neighbor
    // would try to allocate the wrong-sized Eigen vector at runtime.
    if (PointType::SizeAtCompileTime != Eigen::Dynamic && !attribute_layout_.empty()) {
        throw std::invalid_argument("attribute_layout must be empty for a fixed-size PointType");
    }
    // Some buckets (PointNormalBucket) have nowhere to put an attribute record
    // and their insertion strategy drops it. Refuse the layout here instead of
    // advertising attributes the map would never store.
    if (!impl::BucketStoresAttributes<
            typename VoxelBucketType::template BucketType<VHMInternalStorageMode::ABS>>::value &&
        !attribute_layout_.empty()) {
        throw std::invalid_argument("this VoxelHashMap's bucket type cannot store attributes");
    }
    encode_scratch_.resize(attribute_layout_.stride());
    xyz_scratch_ = impl::PointDefaultValue<PointType>::make(3);

    map_resolution_sq_ = voxel_size_ * voxel_size_ / static_cast<double>(max_points_per_voxel_);
    inv_voxel_size_ = 1.0 / voxel_size_;

    switch (internal_storage_mode) {
        case VHMInternalStorageMode::ABS:
        case VHMInternalStorageMode::FX16:
        case VHMInternalStorageMode::FX8: {
            storage_mode_ = internal_storage_mode;
        } break;
        case VHMInternalStorageMode::AUTO: {
            if (voxel_size_ <= 0.256) {
                storage_mode_ = VHMInternalStorageMode::FX8;
            } else if (voxel_size_ <= 32.768) {
                storage_mode_ = VHMInternalStorageMode::FX16;
            } else {
                storage_mode_ = VHMInternalStorageMode::ABS;
            }
        } break;
        default: {
            throw std::invalid_argument("Unknown internal storage mode");
        } break;
    }
}

namespace {

// Retain every inserted point: downsampling has no reference position to trim
// against.
constexpr double DOWNSAMPLE_MAX_DISTANCE = std::numeric_limits<double>::max() / 2.0;
// ---------------------------------------------------------------------------
// Collectors: what each point a bucket visit hands back becomes in the output.
// Every collector is constructed with the owning map and the number of points
// it will receive, is invoked once per point with (point, record) -- record
// being the packed attribute record, or nullptr when the point is already
// complete -- and exposes its output as `result`. collect_pointcloud_impl and
// extract_far_impl below drive them, so the traversal and eviction logic is
// written once for every output representation.
// ---------------------------------------------------------------------------

// One complete double-valued point per entry, converted to OutPoint as it is
// collected; backs pointcloud_vector() and pointcloud_vector_as().
template <typename VHMType, typename OutPoint = typename VHMType::point_type>
struct VectorCollector {
    using PointType = typename VHMType::point_type;
    using Result = std::vector<OutPoint>;

    const VHMType* vhm;
    Result result;

    VectorCollector(const VHMType& vhm_in, std::size_t total) : vhm(&vhm_in) {
        result.reserve(total);
    }

    void operator()(const PointType& point, const std::uint8_t* record) {
        result.emplace_back(vhm->assemble_point(point, record));
    }
};

// One point_cols()-wide row of doubles per point; backs pointcloud_double() and
// extract_voxels_far_from_location_double().
template <typename VHMType>
struct DoubleCollector {
    using PointType = typename VHMType::point_type;
    using Result = ArrayXXdR;

    const AttributeLayout* layout;
    Result result;
    Eigen::Index row{0};

    DoubleCollector(const VHMType& vhm, std::size_t total)
        : layout(&vhm.attribute_layout()),
          result(static_cast<Eigen::Index>(total), vhm.point_cols()) {}

    void operator()(const PointType& point, const std::uint8_t* record) {
        // ArrayXXdR is row-major, so a row is contiguous and the attributes can
        // be decoded straight into it.
        double* dst = result.row(row++).data();
        for (int idx = 0; idx < 3; ++idx) {
            dst[idx] = point[idx];
        }
        if (record != nullptr) {
            layout->decode(record, dst + 3);
        } else {
            for (Eigen::Index idx = 3; idx < point.size(); ++idx) {
                dst[idx] = point[idx];
            }
        }
    }
};

// Positions plus the attribute records exactly as they are stored, so integer
// attributes reach the caller in their original type and width; backs
// pointcloud_attributed() and extract_voxels_far_from_location_attributed().
template <typename VHMType>
struct AttributedCollector {
    using PointType = typename VHMType::point_type;
    using Result = AttributedPointCloud;

    Result result;
    Eigen::Index row{0};

    AttributedCollector(const VHMType& vhm, std::size_t total) {
        result.layout = vhm.attribute_layout();
        result.xyz.resize(static_cast<Eigen::Index>(total), 3);
        result.attributes.resize(static_cast<Eigen::Index>(total),
                                 static_cast<Eigen::Index>(result.layout.stride()));
    }

    void operator()(const PointType& point, const std::uint8_t* record) {
        double* dst = result.xyz.row(row).data();
        for (int idx = 0; idx < 3; ++idx) {
            dst[idx] = point[idx];
        }
        const AttributeLayout& layout = result.layout;
        const std::size_t stride = layout.stride();
        if (stride != 0) {
            std::uint8_t* attrs = result.attributes.row(row).data();
            if (record != nullptr) {
                std::memcpy(attrs, record, stride);
            } else {
                // Averaging buckets sum in double; convert the per-voxel
                // average back into the layout's types.
                layout.encode(point.data() + 3, attrs);
            }
        }
        ++row;
    }
};

// Feeds every point of every voxel in `map` to a Collector.
template <typename Collector, typename VHMType, typename InternalMapType>
typename Collector::Result collect_pointcloud_impl(const VHMType& vhm, InternalMapType& map) {
    using StorageImpl = typename InternalMapType::BucketType::StorageImpl;

    std::size_t total = 0;
    for (const auto& kv : map.map) {
        total += kv.second.point_count(vhm);
    }

    Collector collector(vhm, total);
    for (const auto& kv : map.map) {
        auto voxel_origin = StorageImpl::calc_voxel_origin_from_icoords(kv.first, vhm.voxel_size());
        kv.second.visit_voxels_as_point_type(vhm, voxel_origin, std::ref(collector));
    }
    return std::move(collector.result);
}

// Predicate selecting the voxels that lie farther than max_distance() from
// `origin`; shared by every remove/extract-far operation.
template <typename VHMType>
auto far_from(const VHMType& vhm, const typename VHMType::point_type& origin) {
    const auto origin_voxel = vhm.point_to_voxel(origin);
    const int max_voxel_dist =
        static_cast<int>(std::ceil(vhm.max_distance() * vhm.inv_voxel_size())) + 1;
    const int max_voxel_dist_sq = max_voxel_dist * max_voxel_dist;
    return [origin_voxel, max_voxel_dist_sq](const decltype(origin_voxel)& voxel) {
        return (voxel - origin_voxel).squaredNorm() >= max_voxel_dist_sq;
    };
}

template <typename VHMType, typename InternalMapType>
void add_point_impl(VHMType& vhm, InternalMapType& map, const typename VHMType::point_type& point,
                    const std::uint8_t* record) {
    using BucketType = typename InternalMapType::BucketType;
    using StorageImpl = typename BucketType::StorageImpl;

    const double voxel_size = vhm.voxel_size();
    const impl::InsertContext ctx{vhm.attribute_layout(), voxel_size, vhm.max_points_per_voxel(),
                                  vhm.map_resolution_sq()};

    const auto voxel_idx = vhm.point_to_voxel(point);
    auto voxel_origin = StorageImpl::calc_voxel_origin_from_icoords(voxel_idx, voxel_size);
    auto search = map.map.find(voxel_idx);
    if (search != map.map.end()) {
        BucketType& bucket = search.value();
        vhm.strategy_.apply(voxel_origin, bucket, point, record, ctx);
    } else {
        BucketType bucket;
        bucket.init_voxel(vhm);
        vhm.strategy_.apply(voxel_origin, bucket, point, record, ctx);
        map.map.insert({voxel_idx, std::move(bucket)});
    }
}

template <typename VHMType, typename InternalMapType>
void remove_voxels_far_from_location_impl(VHMType& vhm, InternalMapType& map,
                                          const typename VHMType::point_type& origin) {
    const auto is_far = far_from(vhm, origin);
    for (auto it = map.map.begin(); it != map.map.end();) {
        if (is_far(it->first)) {
            it = map.map.erase(it);
        } else {
            ++it;
        }
    }
}

// Removes the voxels far from `origin`, feeding their points to a Collector
// first. Two passes: sizing the output up front lets it land in one allocation.
template <typename Collector, typename VHMType, typename InternalMapType>
typename Collector::Result extract_far_impl(VHMType& vhm, InternalMapType& map,
                                            const typename VHMType::point_type& origin) {
    using StorageImpl = typename InternalMapType::BucketType::StorageImpl;

    const auto is_far = far_from(vhm, origin);
    std::size_t total = 0;
    for (const auto& kv : map.map) {
        if (is_far(kv.first)) {
            total += kv.second.point_count(vhm);
        }
    }

    Collector collector(vhm, total);
    for (auto it = map.map.begin(); it != map.map.end();) {
        if (is_far(it->first)) {
            it->second.visit_voxels_as_point_type(
                vhm, StorageImpl::calc_voxel_origin_from_icoords(it->first, vhm.voxel_size()),
                std::ref(collector));
            it = map.map.erase(it);
        } else {
            ++it;
        }
    }
    return std::move(collector.result);
}

// The squared search radius for a nearest-neighbor query, in the storage's
// DistType, capped at sqrt(8) * voxel_size so it fits that type. Note this is
// shorter than the farthest a point in the 27 searched voxels can be (opposite
// corners of two corner-adjacent voxels, sqrt(12) * voxel_size), so such
// points beyond sqrt(8) * voxel_size are not reported.
template <typename StorageImpl>
typename StorageImpl::DistType neighbor_search_threshold(double voxel_size,
                                                         double max_distance_sq) {
    return StorageImpl::dist_sq_to_dist_type(std::min(8 * voxel_size * voxel_size, max_distance_sq),
                                             voxel_size);
}

// The 27-voxel nearest-neighbor search behind get_closest_neighbor() and
// get_closest_neighbor_position(). For every voxel around `query_voxel` that
// could still hold a closer point than the best so far, calls
// `visit_bucket(bucket, voxel, consider)`; the visitor passes each candidate's
// internal xyz to `consider`, which returns true when it is the closest yet,
// and then records whatever it needs about that candidate. `p0_xyz` is the
// query in the same internal representation. Returns the best squared
// distance found, or `dist_thresh_sq` when nothing is closer than that.
template <typename VHMType, typename InternalMapType, typename P0Xyz, typename VisitBucket>
typename InternalMapType::BucketType::StorageImpl::DistType search_closest(
    const VHMType& vhm, const InternalMapType& map, typename InternalMapType::VoxelType query_voxel,
    const P0Xyz& p0_xyz, typename InternalMapType::BucketType::StorageImpl::DistType dist_thresh_sq,
    VisitBucket&& visit_bucket) {
    using VoxelType = typename InternalMapType::VoxelType;
    using StorageImpl = typename InternalMapType::BucketType::StorageImpl;
    using DistType = typename StorageImpl::DistType;
    using VoxelShift = typename StorageImpl::VoxelShift;
    const double voxel_size = vhm.voxel_size();

    static const std::array<VoxelShift, 27> voxel_shifts_arr{{
        VoxelShift{0, 0, 0},   VoxelShift{1, 0, 0},   VoxelShift{-1, 0, 0},   VoxelShift{0, 1, 0},
        VoxelShift{0, -1, 0},  VoxelShift{0, 0, 1},   VoxelShift{0, 0, -1},   VoxelShift{1, 1, 0},
        VoxelShift{1, -1, 0},  VoxelShift{-1, 1, 0},  VoxelShift{-1, -1, 0},  VoxelShift{1, 0, 1},
        VoxelShift{1, 0, -1},  VoxelShift{-1, 0, 1},  VoxelShift{-1, 0, -1},  VoxelShift{0, 1, 1},
        VoxelShift{0, 1, -1},  VoxelShift{0, -1, 1},  VoxelShift{0, -1, -1},  VoxelShift{1, 1, 1},
        VoxelShift{1, 1, -1},  VoxelShift{1, -1, 1},  VoxelShift{1, -1, -1},  VoxelShift{-1, 1, 1},
        VoxelShift{-1, 1, -1}, VoxelShift{-1, -1, 1}, VoxelShift{-1, -1, -1},
    }};

    // Seed with the caller's upper bound so voxels beyond it are pruned
    // immediately.
    DistType closest_distance_sq = dist_thresh_sq;

    for (const VoxelShift& voxel_shift : voxel_shifts_arr) {
        VoxelType other_voxel = query_voxel + voxel_shift.offset;
        auto other_origin = voxel_shift.get_origin(query_voxel, voxel_size);

        // Compute the minimum possible squared distance from the query point
        // to this voxel's axis-aligned bounding box. If it already exceeds
        // the best distance found so far, no point inside the voxel can
        // improve the result, so skip it.
        DistType lb_sq = 0;
        for (int dim = 0; dim < 3; ++dim) {
            DistType low = other_origin[dim];
            DistType high = low + StorageImpl::voxel_size_as_dist(voxel_size);
            if (p0_xyz[dim] < low) {
                DistType delta = low - p0_xyz[dim];
                lb_sq += delta * delta;
            } else if (p0_xyz[dim] > high) {
                DistType delta = p0_xyz[dim] - high;
                lb_sq += delta * delta;
            }
        }
        if (lb_sq >= closest_distance_sq) {
            continue;
        }

        auto diff_offset = voxel_shift.get_offset_for_diff();

        auto search = map.map.find(other_voxel);
        if (search != map.map.end()) {
            // Update the global bound per-point so later voxels are pruned
            // as aggressively as possible.
            auto consider = [&](const auto& p1_xyz) {
                DistType norm_sq = 0;
                for (int idx = 0; idx < 3; ++idx) {
                    DistType elem = static_cast<DistType>(p1_xyz.array()[idx]) -
                                    static_cast<DistType>(p0_xyz.array()[idx]) + diff_offset[idx];
                    norm_sq += elem * elem;
                }
                if (norm_sq < closest_distance_sq) {
                    closest_distance_sq = norm_sq;
                    return true;
                }
                return false;
            };
            visit_bucket(search.value(), other_voxel, consider);
        }
    }
    return closest_distance_sq;
}

template <typename VHMType, typename InternalMapType>
std::tuple<typename VHMType::point_type, double> get_closest_neighbor_impl(
    const VHMType& vhm, const InternalMapType& map, const typename VHMType::point_type& query,
    double max_distance_sq) {
    using VoxelType = typename InternalMapType::VoxelType;
    using PointType = typename VHMType::point_type;
    using BucketType = typename InternalMapType::BucketType;
    using StorageImpl = typename BucketType::StorageImpl;
    using StorageType = typename BucketType::StorageType;
    using DistType = typename StorageImpl::DistType;
    const double voxel_size = vhm.voxel_size();
    const DistType dist_thresh_sq =
        neighbor_search_threshold<StorageImpl>(voxel_size, max_distance_sq);

    const VoxelType query_voxel = vhm.point_to_voxel(query);
    const auto query_origin = StorageImpl::calc_voxel_origin_from_icoords(query_voxel, voxel_size);
    const auto p0_internal =
        StorageImpl::point_to_internal_storage_nodither(query_origin, query, voxel_size);

    // Seed the "best so far" trackers with valid, fully-initialized values.
    VoxelType closest_neighbor_voxel = query_voxel;
    StorageType closest_neighbor_p1 = p0_internal;
    // Points into the winning bucket's attribute records; the map is not
    // modified during the search, so it stays valid until we assemble below.
    const std::uint8_t* closest_neighbor_record = nullptr;
    const DistType closest_distance_sq = search_closest(
        vhm, map, query_voxel, StorageImpl::get_xyz_component(p0_internal), dist_thresh_sq,
        [&](const BucketType& bucket, const VoxelType& voxel, auto& consider) {
            bucket.visit_voxels_as_storage_type(
                vhm, [&](const StorageType& p1_internal, const std::uint8_t* record) {
                    if (consider(StorageImpl::get_xyz_component(p1_internal))) {
                        closest_neighbor_voxel = voxel;
                        closest_neighbor_p1 = p1_internal;
                        closest_neighbor_record = record;
                    }
                });
        });

    PointType closest_neighbor = impl::PointDefaultValue<PointType>::make(vhm.point_cols());
    double closest_distance_sq_abs = max_distance_sq;
    if (closest_distance_sq < dist_thresh_sq) {
        auto other_origin =
            StorageImpl::calc_voxel_origin_from_icoords(closest_neighbor_voxel, voxel_size);
        closest_neighbor = vhm.assemble_point(
            StorageImpl::internal_storage_to_point(other_origin, closest_neighbor_p1, voxel_size),
            closest_neighbor_record);
        closest_distance_sq_abs =
            StorageImpl::dist_sq_from_dist_type(closest_distance_sq, voxel_size);
    }

    return std::make_tuple(closest_neighbor, closest_distance_sq_abs);
}

// get_closest_neighbor_impl() for positions only: the query and the best match
// stay fixed-size xyz throughout, and buckets are visited through
// visit_voxel_positions(), so no PointType is built and nothing is allocated.
template <typename VHMType, typename InternalMapType>
std::pair<Eigen::Vector3d, double> get_closest_neighbor_position_impl(const VHMType& vhm,
                                                                      const InternalMapType& map,
                                                                      const Eigen::Vector3d& query,
                                                                      double max_distance_sq) {
    using VoxelType = typename InternalMapType::VoxelType;
    using BucketType = typename InternalMapType::BucketType;
    using StorageImpl = typename BucketType::StorageImpl;
    using XyzType = typename StorageImpl::XyzType;
    using DistType = typename StorageImpl::DistType;
    const double voxel_size = vhm.voxel_size();
    const DistType dist_thresh_sq =
        neighbor_search_threshold<StorageImpl>(voxel_size, max_distance_sq);

    const VoxelType query_voxel = impl::xyz_to_voxel<VoxelType>(query, vhm.inv_voxel_size());
    const auto query_origin = StorageImpl::calc_voxel_origin_from_icoords(query_voxel, voxel_size);
    const XyzType p0_xyz = StorageImpl::xyz_to_internal(query_origin, query, voxel_size);

    VoxelType closest_voxel = query_voxel;
    XyzType closest_xyz = p0_xyz;
    const DistType closest_distance_sq =
        search_closest(vhm, map, query_voxel, p0_xyz, dist_thresh_sq,
                       [&](const BucketType& bucket, const VoxelType& voxel, auto& consider) {
                           bucket.visit_voxel_positions(vhm, [&](const XyzType& p1_xyz) {
                               if (consider(p1_xyz)) {
                                   closest_voxel = voxel;
                                   closest_xyz = p1_xyz;
                               }
                           });
                       });

    if (closest_distance_sq >= dist_thresh_sq) {
        return {Eigen::Vector3d::Zero(), max_distance_sq};
    }
    const auto closest_origin =
        StorageImpl::calc_voxel_origin_from_icoords(closest_voxel, voxel_size);
    return {StorageImpl::internal_xyz_to_abs(closest_origin, closest_xyz, voxel_size),
            StorageImpl::dist_sq_from_dist_type(closest_distance_sq, voxel_size)};
}

}  // namespace

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
template <typename OutPoint>
std::vector<OutPoint> VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                                    InsertionStrategy>::pointcloud_vector_as() const {
    return dispatch_storage([&](auto& map) {
        return collect_pointcloud_impl<VectorCollector<VoxelHashMapT, OutPoint>>(*this, map);
    });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
std::vector<PointType> VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                                     InsertionStrategy>::pointcloud_vector() const {
    return pointcloud_vector_as<PointType>();
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
ArrayXXdR VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                        InsertionStrategy>::pointcloud_double() const {
    return dispatch_storage([&](auto& map) {
        return collect_pointcloud_impl<DoubleCollector<VoxelHashMapT>>(*this, map);
    });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::update(
    const std::vector<PointType>& points, const PointType& position) {
    add_points(points);
    remove_voxels_far_from_location(position);
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_point(
    const PointType& point) {
    check_point_size(point);
    add_point_unchecked(point);
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_point_unchecked(
    const PointType& point) {
    const std::uint8_t* record = pack_attributes(point);
    dispatch_storage([&](auto& map) { add_point_impl(*this, map, point, record); });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_point_record(
    const Eigen::Ref<const Eigen::Vector3d>& xyz, const std::uint8_t* record) {
    // The bucket only needs xyz from the point; the attributes ride along in
    // the record, already in the types the layout declares. Reusing the scratch
    // point keeps a dynamic-size PointType from heap-allocating per insertion.
    PointType& point = xyz_scratch_;
    point.template head<3>() = xyz;
    dispatch_storage([&](auto& map) { add_point_impl(*this, map, point, record); });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_point(
    const Eigen::Ref<const Eigen::Vector3d>& xyz, const Eigen::Ref<const AttributeRecord>& record) {
    const std::size_t stride = attribute_layout_.stride();
    if (static_cast<std::size_t>(record.size()) != stride) {
        throw std::invalid_argument{"VoxelHashMap::add_point record size (" +
                                    std::to_string(record.size()) +
                                    " bytes) does not match the attribute layout stride (" +
                                    std::to_string(stride) + " bytes)"};
    }
    add_point_record(xyz, stride == 0 ? nullptr : record.data());
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_points(
    const Eigen::Ref<const ArrayX3dR>& xyz, const Eigen::Ref<const AttributeRecords>& records) {
    const std::size_t stride = attribute_layout_.stride();
    if (stride == 0) {
        // Nothing to read; accept any empty records, including a default 0x0.
        if (records.size() != 0) {
            throw std::invalid_argument(
                "VoxelHashMap::add_points got attribute records for a map without attributes");
        }
    } else if (records.rows() != xyz.rows() || static_cast<std::size_t>(records.cols()) != stride) {
        throw std::invalid_argument{
            "VoxelHashMap::add_points records are " + std::to_string(records.rows()) + "x" +
            std::to_string(records.cols()) + ", expected " + std::to_string(xyz.rows()) + "x" +
            std::to_string(stride) + " (xyz.rows() x attribute layout stride)"};
    }
    for (Eigen::Index row = 0; row < xyz.rows(); ++row) {
        const Eigen::Vector3d point = xyz.row(row).matrix().transpose();
        // Each row is contiguous, but a Ref may view a wider buffer, so step by
        // the row's own address rather than by `stride`.
        add_point_record(point, stride == 0 ? nullptr : records.row(row).data());
    }
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_points(
    const AttributedPointCloud& cloud) {
    if (cloud.layout != attribute_layout_) {
        throw std::invalid_argument(
            "VoxelHashMap::add_points pointcloud attribute layout does not match the map's");
    }
    add_points(cloud.xyz, cloud.attributes);
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
AttributedPointCloud VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                                   InsertionStrategy>::pointcloud_attributed() const {
    return dispatch_storage([&](auto& map) {
        return collect_pointcloud_impl<AttributedCollector<VoxelHashMapT>>(*this, map);
    });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
AttributedPointCloud VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::
    extract_voxels_far_from_location_attributed(const PointType& origin) {
    return dispatch_storage([&](auto& map) {
        return extract_far_impl<AttributedCollector<VoxelHashMapT>>(*this, map, origin);
    });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_points(
    const std::vector<PointType>& points) {
    // Validate every point first so a bad one never leaves the map with only
    // some of the points inserted, then insert without re-checking each one.
    for (const auto& point : points) {
        check_point_size(point);
    }
    for (const auto& point : points) {
        add_point_unchecked(point);
    }
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
                   InsertionStrategy>::remove_voxels_far_from_location(const PointType& origin) {
    dispatch_storage([&](auto& map) { remove_voxels_far_from_location_impl(*this, map, origin); });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
ArrayXXdR
VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
              InsertionStrategy>::extract_voxels_far_from_location_double(const PointType& origin) {
    return dispatch_storage([&](auto& map) {
        return extract_far_impl<DoubleCollector<VoxelHashMapT>>(*this, map, origin);
    });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
std::tuple<PointType, double>
VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::get_closest_neighbor(
    const PointType& query, double max_distance_sq) const {
    return dispatch_storage(
        [&](auto& map) { return get_closest_neighbor_impl(*this, map, query, max_distance_sq); });
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
std::pair<Eigen::Vector3d, double>
VoxelHashMapT<VoxelIndex, PointType, VoxelBucketType,
              InsertionStrategy>::get_closest_neighbor_position(const Eigen::Vector3d& query,
                                                                double max_distance_sq) const {
    return dispatch_storage([&](auto& map) {
        return get_closest_neighbor_position_impl(*this, map, query, max_distance_sq);
    });
}

// Explicit instantiation for the four canonical specializations exposed by
// the convenience aliases. Non-template members are emitted here once,
// while the SFINAE'd remove_voxels_older_than lives inline in the header.
//
template struct OUSTER_API_FUNCTION VoxelHashMapT<
    Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>, impl::first_n_point>;
template struct OUSTER_API_FUNCTION VoxelHashMapT<
    Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>, impl::first_n_point>;
template struct OUSTER_API_FUNCTION VoxelHashMapT<
    Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>, impl::last_n_point>;
template struct OUSTER_API_FUNCTION VoxelHashMapT<
    Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>, impl::last_n_point>;
template struct OUSTER_API_FUNCTION
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d, AveragePointBucket<Eigen::Vector3d>,
                  impl::accumulate_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, AveragePointBucket<Eigen::VectorXd>,
                  impl::accumulate_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMapT<Eigen::Vector3i, Eigen::Matrix<double, 6, 1>, PointNormalBucket,
                  impl::accumulate_point_normal_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>,
                  impl::random_selection_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                  impl::random_selection_strategy>;
// Used by algorithm::voxel_downsample_with_index(): a fixed-size Vector4d (xyz + row index)
// keeps bucket points stack-allocated.
template struct OUSTER_API_FUNCTION
    VoxelHashMapT<Eigen::Vector3i, Eigen::Vector4d, DefaultVoxelBucket<Eigen::Vector4d>,
                  impl::random_selection_strategy>;

namespace {
// The VoxelDownsampleStrategy a map implements, read off its insertion
// strategy: the one place a map type and its strategy are tied together, so
// VoxelHashMap::strategy() cannot disagree with the map actually built.
template <typename InsertionStrategy>
struct StrategyOf;
template <>
struct StrategyOf<impl::first_n_point>
    : std::integral_constant<VoxelDownsampleStrategy, VoxelDownsampleStrategy::FIRST_N_POINT> {};
template <>
struct StrategyOf<impl::last_n_point>
    : std::integral_constant<VoxelDownsampleStrategy, VoxelDownsampleStrategy::LAST_N_POINT> {};
template <>
struct StrategyOf<impl::accumulate_strategy>
    : std::integral_constant<VoxelDownsampleStrategy, VoxelDownsampleStrategy::AVERAGE_POINT> {};
template <>
struct StrategyOf<impl::random_selection_strategy>
    : std::integral_constant<VoxelDownsampleStrategy, VoxelDownsampleStrategy::RANDOM> {};
template <>
struct StrategyOf<impl::smallest_strategy>
    : std::integral_constant<VoxelDownsampleStrategy, VoxelDownsampleStrategy::SMALLEST> {};
template <>
struct StrategyOf<impl::largest_strategy>
    : std::integral_constant<VoxelDownsampleStrategy, VoxelDownsampleStrategy::LARGEST> {};

template <typename MapT>
constexpr VoxelDownsampleStrategy strategy_of() {
    return StrategyOf<typename MapT::insertion_strategy_type>::value;
}

// Converts between the proxy's VectorXd points and a map's own point type.
// The attributed maps already speak VectorXd, so everything passes straight
// through and the map validates point sizes itself.
template <typename PointType>
struct ProxyPoints;

template <>
struct ProxyPoints<Eigen::VectorXd> {
    static const Eigen::VectorXd& point(const Eigen::VectorXd& point) {
        return point;
    }
    static const Eigen::VectorXd& position(const Eigen::VectorXd& position) {
        check_position(position);
        return position;
    }
    // The map checks every point's size before inserting any.
    template <typename MapT>
    static void add_points(MapT& map, const std::vector<Eigen::VectorXd>& points) {
        map.add_points(points);
    }
    static Eigen::VectorXd from_map(Eigen::VectorXd point) {
        return point;
    }
    static void check_position(const Eigen::VectorXd& position) {
        if (position.size() < 3) {
            throw std::invalid_argument(
                "VoxelHashMap expects a position of at least 3 values (xyz first), got " +
                std::to_string(position.size()));
        }
    }
};

// The positions-only maps take Vector3d, so a point must be exactly xyz.
template <>
struct ProxyPoints<Eigen::Vector3d> {
    static Eigen::Vector3d point(const Eigen::VectorXd& point) {
        if (point.size() != 3) {
            throw std::invalid_argument("VoxelHashMap received a point of " +
                                        std::to_string(point.size()) +
                                        " values, expected 3 (the map has no attributes)");
        }
        return point;
    }
    static Eigen::Vector3d position(const Eigen::VectorXd& position) {
        ProxyPoints<Eigen::VectorXd>::check_position(position);
        return position.head<3>();
    }
    // Checks every point before inserting any, so a bad point never leaves
    // the map holding only some of them, then inserts each in place: no copy
    // of the batch is made.
    template <typename MapT>
    static void add_points(MapT& map, const std::vector<Eigen::VectorXd>& points) {
        for (const auto& pnt : points) {
            point(pnt);
        }
        for (const auto& pnt : points) {
            map.add_point(Eigen::Vector3d(pnt.head<3>()));
        }
    }
    static Eigen::VectorXd from_map(const Eigen::Vector3d& point) {
        return point;
    }
};

// The proxy's one implementation: owns a MapT and forwards every call to it,
// converting points between VectorXd and MapT::point_type on the way.
template <typename MapT>
class VoxelHashMapImpl final : public VoxelHashMap {
    using Points = ProxyPoints<typename MapT::point_type>;

   public:
    VoxelHashMapImpl(double voxel_size, double max_distance, std::size_t max_points_per_voxel,
                     std::size_t min_points_per_voxel, AttributeLayout attribute_layout,
                     VHMInternalStorageMode internal_storage_mode)
        : map_(voxel_size, max_distance, max_points_per_voxel, min_points_per_voxel,
               std::move(attribute_layout), internal_storage_mode) {}

    // Only used for the SMALLEST/LARGEST maps, whose insertion_strategy_type
    // needs to be validated against attribute_index and layout once, up
    // front, rather than default-constructed. attribute_layout is taken by
    // value (not moved into map_) so it is still available afterward to
    // build strategy_; never instantiated for a MapT whose
    // insertion_strategy_type has no such constructor.
    VoxelHashMapImpl(double voxel_size, double max_distance, std::size_t max_points_per_voxel,
                     std::size_t min_points_per_voxel, AttributeLayout attribute_layout,
                     VHMInternalStorageMode internal_storage_mode, std::size_t attribute_index)
        : map_(voxel_size, max_distance, max_points_per_voxel, min_points_per_voxel,
               attribute_layout, internal_storage_mode) {
        map_.strategy_ = typename MapT::insertion_strategy_type(attribute_layout, attribute_index);
    }

    VoxelDownsampleStrategy strategy() const override {
        return strategy_of<MapT>();
    }
    bool empty() const override {
        return map_.empty();
    }
    void clear() override {
        map_.clear();
    }
    void update(const std::vector<point_type>& points, const point_type& position) override {
        // Validate the position before any point goes in, as for the points.
        const auto& trim_position = Points::position(position);
        Points::add_points(map_, points);
        map_.remove_voxels_far_from_location(trim_position);
    }
    void update(const Eigen::Ref<const ArrayXXdR>& points, const point_type& position) override {
        // add_points(ArrayXXdR) checks points.cols() against point_cols() up
        // front and inserts straight from the matrix, so this never makes a
        // per-point copy of the batch the way the std::vector overload does.
        const auto& trim_position = Points::position(position);
        map_.add_points(points);
        map_.remove_voxels_far_from_location(trim_position);
    }
    void add_point(const point_type& point) override {
        map_.add_point(Points::point(point));
    }
    void add_point(const Eigen::Ref<const Eigen::Vector3d>& xyz,
                   const Eigen::Ref<const AttributeRecord>& record) override {
        map_.add_point(xyz, record);
    }
    void add_points(const std::vector<point_type>& points) override {
        Points::add_points(map_, points);
    }
    void add_points(const Eigen::Ref<const ArrayXXdR>& points) override {
        map_.add_points(points);
    }
    void add_points(const Eigen::Ref<const ArrayX3dR>& xyz,
                    const Eigen::Ref<const AttributeRecords>& records) override {
        map_.add_points(xyz, records);
    }
    void add_points(const AttributedPointCloud& cloud) override {
        map_.add_points(cloud);
    }
    AttributedPointCloud pointcloud() const override {
        return map_.pointcloud_attributed();
    }
    AttributedPointCloud extract_voxels_far_from_location(const point_type& origin) override {
        return map_.extract_voxels_far_from_location_attributed(Points::position(origin));
    }
    void remove_voxels_far_from_location(const point_type& origin) override {
        map_.remove_voxels_far_from_location(Points::position(origin));
    }
    std::vector<point_type> pointcloud_vector() const override {
        return map_.template pointcloud_vector_as<point_type>();
    }
    std::tuple<point_type, double> get_closest_neighbor(const point_type& query,
                                                        double max_distance_sq) const override {
        auto result = map_.get_closest_neighbor(Points::position(query), max_distance_sq);
        return std::make_tuple(Points::from_map(std::move(std::get<0>(result))),
                               std::get<1>(result));
    }
    std::pair<Eigen::Vector3d, double> get_closest_neighbor_position(
        const Eigen::Vector3d& query, double max_distance_sq) const override {
        return map_.get_closest_neighbor_position(query, max_distance_sq);
    }
    Eigen::Index point_cols() const override {
        return map_.point_cols();
    }
    const AttributeLayout& attribute_layout() const override {
        return map_.attribute_layout();
    }
    VHMInternalStorageMode storage_mode() const override {
        return map_.storage_mode();
    }
    std::size_t max_points_per_voxel() const override {
        return map_.max_points_per_voxel();
    }
    std::size_t min_points_per_voxel() const override {
        return map_.min_points_per_voxel();
    }
    double voxel_size() const override {
        return map_.voxel_size();
    }
    double max_distance() const override {
        return map_.max_distance();
    }

   private:
    MapT map_;
};

// Builds the proxy for strategy S: PlainMap for positions only, AttributedMap
// when there are attributes. Both must implement S, which the compiler checks,
// so a create() case can't be wired to the wrong maps.
template <VoxelDownsampleStrategy S, typename PlainMap, typename AttributedMap>
std::unique_ptr<VoxelHashMap> make_map(double voxel_size, std::size_t max_points_per_voxel,
                                       std::size_t min_points_per_voxel,
                                       AttributeLayout attribute_layout, double max_distance,
                                       VHMInternalStorageMode internal_storage_mode) {
    static_assert(strategy_of<PlainMap>() == S && strategy_of<AttributedMap>() == S,
                  "make_map: both maps must implement the requested strategy");
    static_assert(PlainMap::point_type::SizeAtCompileTime == 3 &&
                      AttributedMap::point_type::SizeAtCompileTime == Eigen::Dynamic,
                  "make_map: expects a positions-only map and an attributed map");
    // Positions only need the fixed-size map; attributes need the
    // dynamic-size one to carry them.
    if (attribute_layout.empty()) {
        return std::make_unique<VoxelHashMapImpl<PlainMap>>(
            voxel_size, max_distance, max_points_per_voxel, min_points_per_voxel,
            std::move(attribute_layout), internal_storage_mode);
    }
    return std::make_unique<VoxelHashMapImpl<AttributedMap>>(
        voxel_size, max_distance, max_points_per_voxel, min_points_per_voxel,
        std::move(attribute_layout), internal_storage_mode);
}

// Builds the proxy for strategy S (SMALLEST or LARGEST), both of which only
// have an attributed MapT: there is no positions-only variant to pair it
// with, unlike make_map()'s strategies, since there is no attribute to rank
// positions-only points by.
template <VoxelDownsampleStrategy S, typename MapT>
std::unique_ptr<VoxelHashMap> make_extremum_map(double voxel_size, std::size_t max_points_per_voxel,
                                                std::size_t min_points_per_voxel,
                                                AttributeLayout attribute_layout,
                                                double max_distance,
                                                VHMInternalStorageMode internal_storage_mode,
                                                std::size_t attribute_index) {
    static_assert(strategy_of<MapT>() == S, "make_extremum_map: MapT must implement strategy S");
    if (attribute_layout.empty()) {
        throw std::invalid_argument(
            "VoxelHashMap::create: SMALLEST/LARGEST rank points by an attribute, so "
            "attribute_layout must not be empty");
    }
    return std::make_unique<VoxelHashMapImpl<MapT>>(
        voxel_size, max_distance, max_points_per_voxel, min_points_per_voxel,
        std::move(attribute_layout), internal_storage_mode, attribute_index);
}
}  // namespace

// Out-of-line definitions for the static constexpr members, which are
// ODR-used when bound to a reference (e.g. nanobind's py::arg(...) = ...).
constexpr std::size_t VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL;
constexpr std::size_t VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL;
constexpr VoxelDownsampleStrategy VoxelHashMap::DEFAULT_STRATEGY;
constexpr double VoxelHashMap::DEFAULT_MAX_DISTANCE;
constexpr VHMInternalStorageMode VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE;

std::unique_ptr<VoxelHashMap> VoxelHashMap::create(
    double voxel_size, std::size_t max_points_per_voxel, std::size_t min_points_per_voxel,
    VoxelDownsampleStrategy strategy, AttributeLayout attribute_layout, double max_distance,
    VHMInternalStorageMode internal_storage_mode, std::size_t attribute_index) {
    using Strategy = VoxelDownsampleStrategy;
    switch (strategy) {
        case Strategy::FIRST_N_POINT:
            return make_map<Strategy::FIRST_N_POINT, VoxelHashMap3d, VoxelHashMapXd>(
                voxel_size, max_points_per_voxel, min_points_per_voxel, std::move(attribute_layout),
                max_distance, internal_storage_mode);
        case Strategy::LAST_N_POINT:
            return make_map<Strategy::LAST_N_POINT, LastNPointVoxelHashMap3d,
                            LastNPointVoxelHashMapXd>(
                voxel_size, max_points_per_voxel, min_points_per_voxel, std::move(attribute_layout),
                max_distance, internal_storage_mode);
        case Strategy::AVERAGE_POINT:
            return make_map<Strategy::AVERAGE_POINT, AverageVoxelHashMap3d, AverageVoxelHashMapXd>(
                voxel_size, max_points_per_voxel, min_points_per_voxel, std::move(attribute_layout),
                max_distance, internal_storage_mode);
        case Strategy::RANDOM:
            return make_map<Strategy::RANDOM, RandomVoxelHashMap3d, RandomVoxelHashMapXd>(
                voxel_size, max_points_per_voxel, min_points_per_voxel, std::move(attribute_layout),
                max_distance, internal_storage_mode);
        case Strategy::SMALLEST:
            return make_extremum_map<Strategy::SMALLEST, SmallestVoxelHashMapXd>(
                voxel_size, max_points_per_voxel, min_points_per_voxel, std::move(attribute_layout),
                max_distance, internal_storage_mode, attribute_index);
        case Strategy::LARGEST:
            return make_extremum_map<Strategy::LARGEST, LargestVoxelHashMapXd>(
                voxel_size, max_points_per_voxel, min_points_per_voxel, std::move(attribute_layout),
                max_distance, internal_storage_mode, attribute_index);
        default:
            throw std::invalid_argument("VoxelHashMap::create: unknown strategy");
    }
}

OUSTER_API_FUNCTION ArrayX3dR voxel_downsample_3d(
    const Eigen::Ref<const ArrayX3dR>& frame, const double voxel_size,
    const std::size_t max_points_per_voxel, const std::size_t min_points_per_voxel,
    const VoxelDownsampleStrategy strategy, const VHMInternalStorageMode internal_storage_mode) {
    if (strategy == VoxelDownsampleStrategy::SMALLEST ||
        strategy == VoxelDownsampleStrategy::LARGEST) {
        throw std::invalid_argument(
            "voxel_downsample_3d: SMALLEST/LARGEST rank points by an attribute, which "
            "voxel_downsample_3d has none of; use voxel_downsample_xd instead");
    }
    // No early return for an empty frame: create() rejects bad arguments
    // either way, and an empty map yields an empty 0x3 cloud.
    auto map =
        VoxelHashMap::create(voxel_size, max_points_per_voxel, min_points_per_voxel, strategy,
                             AttributeLayout(), DOWNSAMPLE_MAX_DISTANCE, internal_storage_mode);
    map->add_points(frame);
    return map->pointcloud().xyz;
}

OUSTER_API_FUNCTION AttributedPointCloud voxel_downsample_xd(
    const Eigen::Ref<const ArrayX3dR>& xyz, const double voxel_size,
    const std::size_t max_points_per_voxel, const std::size_t min_points_per_voxel,
    const VoxelDownsampleStrategy strategy, const AttributeLayout& attribute_layout,
    const Eigen::Ref<const AttributeRecords>& records,
    const VHMInternalStorageMode internal_storage_mode, const std::size_t attribute_index) {
    // An empty layout is not special-cased: add_points() rejects records given
    // without a layout, and pointcloud() returns zero-width records. create()
    // itself rejects an empty layout for SMALLEST/LARGEST.
    auto map = VoxelHashMap::create(voxel_size, max_points_per_voxel, min_points_per_voxel,
                                    strategy, attribute_layout, DOWNSAMPLE_MAX_DISTANCE,
                                    internal_storage_mode, attribute_index);
    map->add_points(xyz, records);
    return map->pointcloud();
}

OUSTER_API_FUNCTION AttributedPointCloud voxel_downsample(
    const Eigen::Ref<const ArrayX3dR>& xyz, const double voxel_size,
    const std::size_t max_points_per_voxel, const std::size_t min_points_per_voxel,
    const VoxelDownsampleStrategy strategy, const AttributeLayout& attribute_layout,
    const Eigen::Ref<const AttributeRecords>& records,
    const VHMInternalStorageMode internal_storage_mode, const std::size_t attribute_index) {
    if (!attribute_layout.empty()) {
        return voxel_downsample_xd(xyz, voxel_size, max_points_per_voxel, min_points_per_voxel,
                                   strategy, attribute_layout, records, internal_storage_mode,
                                   attribute_index);
    }
    if (records.size() != 0) {
        throw std::invalid_argument(
            "voxel_downsample got attribute records without an attribute layout");
    }
    AttributedPointCloud cloud;
    cloud.xyz = voxel_downsample_3d(xyz, voxel_size, max_points_per_voxel, min_points_per_voxel,
                                    strategy, internal_storage_mode);
    cloud.attributes.resize(cloud.xyz.rows(), 0);
    return cloud;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
