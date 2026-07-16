#include <ouster/core/voxel_hash_map.h>
#include <tsl/robin_set.h>

#include <algorithm>
#include <array>
#include <numeric>
#include <stdexcept>

namespace ouster {
namespace sdk {
namespace core {

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::VoxelHashMap(
    double voxel_size, double max_distance, std::size_t max_points_per_voxel,
    std::size_t min_pts_threshold, std::size_t num_attributes)
    : voxel_size_(voxel_size),
      max_distance_(max_distance),
      max_points_per_voxel_(max_points_per_voxel),
      min_pts_threshold_(min_pts_threshold),
      num_attributes_(num_attributes) {
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
    // sizes (e.g. Vector3d) num_attributes_ must be 0 or get_closest_neighbor
    // would try to allocate the wrong-sized Eigen vector at runtime.
    if (PointType::SizeAtCompileTime != Eigen::Dynamic && num_attributes_ != 0) {
        throw std::invalid_argument("num_attributes must be 0 for a fixed-size PointType");
    }

    map_resolution_sq_ = voxel_size_ * voxel_size_ / static_cast<double>(max_points_per_voxel_);
    inv_voxel_size_ = 1.0 / voxel_size_;
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
std::vector<PointType>
VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::pointcloud_vector() const {
    std::size_t total = 0;
    for (const auto& kv : map_) total += VoxelBucketType::point_count(*this, kv.second);

    std::vector<PointType> points;
    points.reserve(total);
    for (const auto& kv : map_) {
        VoxelBucketType::visit_voxel(*this, kv.second,
                                     [&points](const PointType& p) { points.push_back(p); });
    }
    return points;
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
ArrayXXdR VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::pointcloud()
    const {
    std::size_t total = 0;
    for (const auto& kv : map_) total += VoxelBucketType::point_count(*this, kv.second);

    ArrayXXdR points(static_cast<Eigen::Index>(total), point_cols());
    Eigen::Index row = 0;
    for (const auto& kv : map_) {
        VoxelBucketType::visit_voxel(*this, kv.second, [&](const PointType& point) {
            points.row(row++) = impl::point_view(point).array();
        });
    }
    return points;
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::update(
    const std::vector<PointType>& points, const PointType& position) {
    add_points(points);
    remove_voxels_far_from_location(position);
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_point(
    const PointType& point) {
    const auto voxel_idx = point_to_voxel(point);
    auto search = map_.find(voxel_idx);
    if (search != map_.end()) {
        auto& bucket = search.value();
        strategy_.apply(bucket, point, map_resolution_sq_, max_points_per_voxel_);
    } else {
        VoxelBucket bucket;
        VoxelBucketType::init_voxel(*this, bucket);
        strategy_.apply(bucket, point, map_resolution_sq_, max_points_per_voxel_);
        map_.insert({voxel_idx, std::move(bucket)});
    }
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::add_points(
    const std::vector<PointType>& points) {
    for (const auto& point : points) {
        add_point(point);
    }
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
void VoxelHashMap<VoxelIndex, PointType, VoxelBucketType,
                  InsertionStrategy>::remove_voxels_far_from_location(const PointType& origin) {
    const VoxelIndex origin_voxel = point_to_voxel(origin);
    const int max_voxel_dist = static_cast<int>(std::ceil(max_distance_ * inv_voxel_size_)) + 1;
    const int max_voxel_dist_sq = max_voxel_dist * max_voxel_dist;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto& voxel = it->first;
        if ((voxel - origin_voxel).squaredNorm() >= max_voxel_dist_sq) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
ArrayXXdR
VoxelHashMap<VoxelIndex, PointType, VoxelBucketType,
             InsertionStrategy>::extract_voxels_far_from_location(const PointType& origin) {
    const VoxelIndex origin_voxel = point_to_voxel(origin);
    const int max_voxel_dist = static_cast<int>(std::ceil(max_distance_ * inv_voxel_size_)) + 1;
    const int max_voxel_dist_sq = max_voxel_dist * max_voxel_dist;

    std::vector<PointType> evicted;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto& voxel = it->first;
        if ((voxel - origin_voxel).squaredNorm() >= max_voxel_dist_sq) {
            VoxelBucketType::visit_voxel(*this, it->second,
                                         [&evicted](const PointType& p) { evicted.push_back(p); });
            it = map_.erase(it);
        } else {
            ++it;
        }
    }

    ArrayXXdR out(static_cast<Eigen::Index>(evicted.size()), point_cols());
    for (std::size_t i = 0; i < evicted.size(); ++i) {
        out.row(static_cast<Eigen::Index>(i)) = impl::point_view(evicted[i]).array();
    }
    return out;
}

namespace {

using VoxelIndexT = Eigen::Vector3i;
static const std::array<VoxelIndexT, 27> VOXEL_SHIFTS{{
    VoxelIndexT{0, 0, 0},   VoxelIndexT{1, 0, 0},   VoxelIndexT{-1, 0, 0},   VoxelIndexT{0, 1, 0},
    VoxelIndexT{0, -1, 0},  VoxelIndexT{0, 0, 1},   VoxelIndexT{0, 0, -1},   VoxelIndexT{1, 1, 0},
    VoxelIndexT{1, -1, 0},  VoxelIndexT{-1, 1, 0},  VoxelIndexT{-1, -1, 0},  VoxelIndexT{1, 0, 1},
    VoxelIndexT{1, 0, -1},  VoxelIndexT{-1, 0, 1},  VoxelIndexT{-1, 0, -1},  VoxelIndexT{0, 1, 1},
    VoxelIndexT{0, 1, -1},  VoxelIndexT{0, -1, 1},  VoxelIndexT{0, -1, -1},  VoxelIndexT{1, 1, 1},
    VoxelIndexT{1, 1, -1},  VoxelIndexT{1, -1, 1},  VoxelIndexT{1, -1, -1},  VoxelIndexT{-1, 1, 1},
    VoxelIndexT{-1, 1, -1}, VoxelIndexT{-1, -1, 1}, VoxelIndexT{-1, -1, -1},
}};

}  // namespace

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
std::tuple<PointType, double>
VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::get_closest_neighbor(
    const PointType& query, const double max_distance_sq) const {
    const VoxelIndex voxel = point_to_voxel(query);
    PointType closest_neighbor = impl::PointDefaultValue<PointType>::make(point_cols());
    // Seed with the caller's upper bound so voxels beyond it are pruned
    // immediately.
    double closest_distance_sq = max_distance_sq;
    const auto query_pos = impl::spatial_view(query);
    for (const auto& voxel_shift : VOXEL_SHIFTS) {
        const VoxelIndex query_voxel = voxel + voxel_shift;
        // Compute the minimum possible squared distance from the query point
        // to this voxel's axis-aligned bounding box. If it already exceeds
        // the best distance found so far, no point inside the voxel can
        // improve the result, so skip it.
        double lb_sq = 0.0;
        for (int d = 0; d < 3; ++d) {
            const double lo = query_voxel[d] * voxel_size_;
            const double hi = lo + voxel_size_;
            if (query_pos[d] < lo) {
                const double delta = lo - query_pos[d];
                lb_sq += delta * delta;
            } else if (query_pos[d] > hi) {
                const double delta = query_pos[d] - hi;
                lb_sq += delta * delta;
            }
        }
        if (lb_sq >= closest_distance_sq) continue;
        auto search = map_.find(query_voxel);
        if (search != map_.end()) {
            // Update the global bound per-point so later voxels are pruned
            // as aggressively as possible.
            VoxelBucketType::visit_voxel(*this, search.value(), [&](const PointType& point) {
                const double distance_sq = (impl::spatial_view(point) - query_pos).squaredNorm();
                if (distance_sq < closest_distance_sq) {
                    closest_neighbor = point;
                    closest_distance_sq = distance_sq;
                }
            });
        }
    }

    return std::make_tuple(closest_neighbor, closest_distance_sq);
}

template <typename VoxelIndex, typename PointType, typename VoxelBucketType,
          typename InsertionStrategy>
const VoxelBucketType&
VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>::get_voxel_bucket(
    const PointType& point) const {
    static const VoxelBucketType empty_bucket{};
    const auto it = map_.find(point_to_voxel(point));
    if (it == map_.end()) return empty_bucket;
    return it->second;
}

// Explicit instantiation for the four canonical specializations exposed by
// the convenience aliases. Non-template members are emitted here once,
// while the SFINAE'd remove_voxels_older_than lives inline in the header.
//
template struct OUSTER_API_FUNCTION VoxelHashMap<
    Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>, impl::first_n_point>;
template struct OUSTER_API_FUNCTION VoxelHashMap<
    Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>, impl::first_n_point>;
template struct OUSTER_API_FUNCTION
    VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, AveragePointBucket<Eigen::Vector3d>,
                 impl::accumulate_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, AveragePointBucket<Eigen::VectorXd>,
                 impl::accumulate_strategy>;
template struct OUSTER_API_FUNCTION VoxelHashMap<
    Eigen::Vector3i, Eigen::VectorXd, PointNormalBucket, impl::accumulate_point_normal_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMap<Eigen::Vector3i, IndexedPoint<Eigen::Vector3d>,
                 DefaultVoxelBucket<IndexedPoint<Eigen::Vector3d>>, impl::first_n_point>;
template struct OUSTER_API_FUNCTION VoxelHashMap<Eigen::Vector3i, IndexedPoint<Eigen::Vector3d>,
                                                 DefaultVoxelBucket<IndexedPoint<Eigen::Vector3d>>,
                                                 impl::indexed_reservoir_strategy>;
template struct OUSTER_API_FUNCTION VoxelHashMap<Eigen::Vector3i, IndexedPoint<Eigen::VectorXd>,
                                                 DefaultVoxelBucket<IndexedPoint<Eigen::VectorXd>>,
                                                 impl::indexed_reservoir_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>,
                 impl::random_selection_strategy>;
template struct OUSTER_API_FUNCTION
    VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                 impl::random_selection_strategy>;

// TODO[UN]: refactor to expose this as a configurable insertion strategy on
// VoxelHashMap rather than a free function.
std::pair<std::vector<Eigen::Vector3d>, std::vector<std::uint32_t>> voxel_downsample(
    const std::vector<Eigen::Vector3d>& frame, const double voxel_size) {
    const std::size_t n = frame.size();
    if (n == 0) return {};

    auto shuffled = frame;

    // Parallel index array — tracks each point's original position through
    // the shuffle. Only 4-byte swaps per step, negligible extra cost.
    std::vector<std::uint32_t> indices(n);
    std::iota(indices.begin(), indices.end(), 0u);

    std::uint32_t rng_state = 42;
    auto fast_rand = [&rng_state]() -> std::uint32_t {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 17;
        rng_state ^= rng_state << 5;
        return rng_state;
    };

    // Fisher-Yates (Durstenfeld) shuffle: O(n), forward pass. Uses Lemire's
    // fast range reduction to avoid integer division.
    for (std::size_t i = 0; i < n - 1; ++i) {
        const auto j =
            i + static_cast<std::size_t>((static_cast<std::uint64_t>(fast_rand()) * (n - i)) >> 32);
        std::swap(shuffled[i], shuffled[j]);
        std::swap(indices[i], indices[j]);
    }

    // First-in-wins on shuffled data. Set-only (12 bytes/voxel vs 36 with a
    // map) since we collect output points in a separate vector.
    tsl::robin_set<Eigen::Vector3i, Vector3iHash> seen;
    seen.reserve(n);

    std::vector<Eigen::Vector3d> points_out;
    std::vector<std::uint32_t> indices_out;
    points_out.reserve(n);
    indices_out.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        const auto voxel = VoxelHashMap3d::point_to_voxel(shuffled[i], 1.0 / voxel_size);
        if (seen.insert(voxel).second) {
            points_out.emplace_back(shuffled[i]);
            indices_out.push_back(indices[i]);
        }
    }

    return {std::move(points_out), std::move(indices_out)};
}

OUSTER_API_FUNCTION ArrayX3dR voxel_downsample_3d(const Eigen::Ref<const ArrayX3dR>& frame,
                                                  const double voxel_size,
                                                  const std::size_t max_points_per_voxel,
                                                  const std::size_t min_pts_threshold,
                                                  const VoxelDownsampleStrategy strategy) {
    if (frame.rows() == 0) return ArrayX3dR(0, 3);
    const double max_dist = std::numeric_limits<double>::max() / 2.0;

    switch (strategy) {
        case VoxelDownsampleStrategy::FIRST_N_POINT: {
            VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>,
                         impl::first_n_point>
                map(voxel_size, max_dist, max_points_per_voxel, min_pts_threshold, 0);
            map.add_points(frame);
            return map.pointcloud();
        }

        case VoxelDownsampleStrategy::AVERAGE_POINT: {
            VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, AveragePointBucket<Eigen::Vector3d>,
                         impl::accumulate_strategy>
                map(voxel_size, max_dist, max_points_per_voxel, min_pts_threshold, 0);
            map.add_points(frame);
            return map.pointcloud();
        }

        case VoxelDownsampleStrategy::RANDOM: {
            VoxelHashMap<Eigen::Vector3i, Eigen::Vector3d, DefaultVoxelBucket<Eigen::Vector3d>,
                         impl::random_selection_strategy>
                map(voxel_size, max_dist, max_points_per_voxel, min_pts_threshold, 0);
            map.add_points(frame);
            return map.pointcloud();
        }

        default:
            throw std::invalid_argument("voxel_downsample_3d: unknown strategy");
    }
}

OUSTER_API_FUNCTION ArrayXXdR voxel_downsample_xd(const Eigen::Ref<const ArrayXXdR>& frame,
                                                  const double voxel_size,
                                                  const std::size_t max_points_per_voxel,
                                                  const std::size_t min_pts_threshold,
                                                  const VoxelDownsampleStrategy strategy) {
    if (frame.rows() == 0) return ArrayXXdR(0, frame.cols());
    if (frame.cols() < 3)
        throw std::invalid_argument("voxel_downsample_xd: frame must have at least 3 columns");

    const std::size_t num_attrs = static_cast<std::size_t>(frame.cols()) - 3;
    const double max_dist = std::numeric_limits<double>::max() / 2.0;

    switch (strategy) {
        case VoxelDownsampleStrategy::FIRST_N_POINT: {
            // VoxelHashMapXd: DefaultVoxelBucket + first_n_point.
            // min_pts_threshold is stored on the map but DefaultVoxelBucket
            // does not filter by it; all voxels with >= 1 admitted point appear.
            VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                         impl::first_n_point>
                map(voxel_size, max_dist, max_points_per_voxel, min_pts_threshold, num_attrs);
            map.add_points(frame);
            return map.pointcloud();
        }

        case VoxelDownsampleStrategy::AVERAGE_POINT: {
            VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, AveragePointBucket<Eigen::VectorXd>,
                         impl::accumulate_strategy>
                map(voxel_size, max_dist, max_points_per_voxel, min_pts_threshold, num_attrs);
            map.add_points(frame);
            return map.pointcloud();
        }

        case VoxelDownsampleStrategy::RANDOM: {
            VoxelHashMap<Eigen::Vector3i, Eigen::VectorXd, DefaultVoxelBucket<Eigen::VectorXd>,
                         impl::random_selection_strategy>
                map(voxel_size, max_dist, max_points_per_voxel, min_pts_threshold, num_attrs);
            map.add_points(frame);
            return map.pointcloud();
        }

        default:
            throw std::invalid_argument("voxel_downsample_xd: unknown strategy");
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
