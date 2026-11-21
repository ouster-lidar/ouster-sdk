#include "lio_slam.h"

#include <ouster/core/impl/logging.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ouster/algorithm/impl/custom_voxel_downsample.h"
#include "ouster/core/impl/dewarp_impl.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/voxel_hash_map.h"
#include "ouster/mapping/deskew_method.h"
#include "ouster/mapping/slam_engine.h"
#include "slam_util.h"

using ouster::sdk::core::logger;
using ouster::sdk::core::VoxelHashMap;

namespace {
// Points the ICP local map keeps per voxel: enough for a dense surface sample
// to match against.
constexpr std::size_t LOCAL_MAP_MAX_POINTS_PER_VOXEL = 20;
}  // namespace

namespace ouster {
namespace sdk {
namespace mapping {

LIOSlam::LIOSlam(const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos,
                 const LIOSlamConfig& config)
    : SlamEngine(sensor_infos), active_time_correction_(sensor_infos_), config_(config) {
    if (sensor_infos_.empty()) {
        throw std::invalid_argument("No sensor info provided for slam");
    }

    if (config.voxel_size > 0.0) {
        initialize_components();
    }

    if (impl::contains_lidar(sensor_infos_)) {
        deskew_method_ = DeskewMethodFactory::create(config_.deskew_method, sensor_infos_,
                                                     config_.initial_pose, config_.plumb_method);
    }
}

/*
 * Update the pose information of each lidar frame based on SLAM pose
 * estimation.
 *
 *  Workflow:
 *  • Dynamic voxel size:
 *      - If a voxel_size was provided then apply it.
 *  • Time correction: Apply time correction to each frame.
 *     - Check inter-sensor synchronization
 *     - Monotonicity check (ignoring zero timestamps)
 *     - Packet-offset handling
 *     - Abort early if offset computation fails.
 *  • Frame-ID handling:
 *      - Compute per-sensor frame-ID diffs (with overflow protection).
 *      - Invalidate any out-of-order frames by zeroing their ranges.
 *      - Track the smallest non-zero diff for SLAM integration.
 *  • Point-cloud aggregation:
 *      - Merge valid lidar points (deskewed/dewarped) and global-shutter
 *        camera RANGE points (no deskew).
 *      - Abort early if no points remain.
 *  • SLAM registration:
 *      - Use ICP to register the aggregated frame to the local map.
 *      - Update the map.
 *  • Pose interpolation:
 *      - Interpolate between the last and new SLAM poses.
 *      - Write per-column poses back into each frame.
 *  • Update internal state:
 *      - Store the new last SLAM pose, last frame-ID array, and timestamp
 * range.
 */
void LIOSlam::update(core::FrameSet& frame_set) {
    constexpr double min_valid_lidar_column_ratio = 0.5;
    constexpr size_t local_map_cull_interval_frames = 10;

    if (config_.voxel_size <= 0) {
        logger().info(
            "voxel size was not specified, generating an estimated "
            "value..");
        auto voxel_size = impl::determine_voxel_size(frame_set);
        config_.voxel_size = voxel_size.value_or(1.0);
        logger().info("Using voxel size {} m", config_.voxel_size);
        initialize_components();
    }

    // Fix frame timestamps
    active_time_correction_.update(frame_set);

    // Min/max timestamp across all frames in the set
    const auto ts_range = impl::compute_frame_ts_range(frame_set);
    const bool has_lidar = impl::contains_lidar(sensor_infos_);

    // Validate timestamp range. Lidar collation needs a non-zero span for
    // deskew/time correction. Global-shutter cameras have a single timestamp,
    // so equal min/max is valid when there is no lidar.
    if (ts_range.first == std::numeric_limits<uint64_t>::max()) {
        logger().warn("None of the frame timestamps are valid. Skipping the frame set!");
        return;
    }
    if (has_lidar && ts_range.second <= ts_range.first) {
        logger().warn(
            "None of the frame timestamps are valid or the frame set collation "
            "duration is zero. This prevents timestamp normalization. Skipping "
            "the frame set!");
        return;
    }

    // Deskew: uses IMU/odometry to set per-column poses to world frame
    if (deskew_method_) {
        deskew_method_->update(frame_set);
    } else if (frame_count_ == 0) {
        // With deskewing disabled nothing seeds the first frame's poses, so
        // anchor it at the configured initial pose. Later frames re-register
        // globally against the accumulated map. frame_count_ only advances once
        // a frame is actually registered, so a skipped first frame still leaves
        // the seed in place for the first frame that survives.
        const core::Matrix4dR seed_pose =
            config_.initial_pose.value_or(core::Matrix4dR::Identity());
        impl::init_valid_column_poses(frame_set, seed_pose);
        impl::init_camera_poses(frame_set, seed_pose);
    }

    // Put cameras into the same world as the (deskewed) lidar before RANGE
    // projection so mixed lidar+camera clouds share a frame.
    impl::update_camera_poses(frame_set);

    // TODO[UN]: refactor this and move it close to where it is actually used
    // If too much lidar data is missing, trust the deskew method's open-loop
    // prediction for this frame instead of letting sparse data produce a bad
    // ICP correction and corrupt the voxel map.
    size_t expected_lidar_columns = 0;
    size_t received_lidar_columns = 0;
    for (size_t idx : frame_set.valid_indices()) {
        auto lidar = std::dynamic_pointer_cast<core::LidarFrame>(frame_set[idx]);
        if (!lidar) {
            continue;
        }
        // sensor_infos_ shares the FrameSet index space, so dispatch on the
        // matching entry's concrete type; this makes no assumption about lidar
        // vs. camera ordering within the set.
        if (idx < sensor_infos_.size()) {
            if (auto lidar_info = std::dynamic_pointer_cast<core::LidarInfo>(sensor_infos_[idx])) {
                expected_lidar_columns +=
                    static_cast<size_t>(lidar_info->format.valid_columns_per_frame());
            }
        }
        Eigen::Ref<const core::LidarFrame::Header<uint32_t>> status = lidar->status();
        received_lidar_columns += impl::count_valid_columns(status);
    }

    // TODO[UN]: move this computed value closer to where it is actually used
    const double valid_lidar_column_ratio = expected_lidar_columns > 0
                                                ? static_cast<double>(received_lidar_columns) /
                                                      static_cast<double>(expected_lidar_columns)
                                                : 1.0;

    // Dewarp every RANGE-carrying frame into one world-frame cloud. Lidar
    // frames use their per-column poses (rolling shutter); camera frames use
    // their single global-shutter pose. Both go through the shared dewarp path.
    std::vector<uint32_t> frame_idxs;
    std::vector<uint32_t> col_idxs;
    std::vector<uint64_t> timestamps_ns;
    std::vector<Eigen::Vector3d> frame = core::impl::dewarp_impl<double>(
        frame_set, config_.min_range, config_.max_range, &frame_idxs, &col_idxs, &timestamps_ns);

    if (frame.empty()) {
        logger().warn("No valid points found in the frame set. Skipping this update.");
        active_time_correction_.reset(frame_set);
        impl::update_camera_poses(frame_set);
        return;
    }

    // Single voxel downsample in world frame; the same cloud is the ICP source
    // and the points added to the map.
    const auto source_ds = algorithm::impl::voxel_downsample_with_index(frame, config_.voxel_size);
    const auto& source = source_ds.first;
    const auto& source_idx = source_ds.second;

    // Pick the column whose timestamp is the median over the ICP source
    // cloud. Weighting the origin toward where returns actually exist gives
    // a more representative pose than the raw frame-set temporal midpoint
    // (e.g. when one side of the frame is empty sky).
    std::vector<size_t> order(source.size());
    std::iota(order.begin(), order.end(), 0);
    const size_t k = order.size() / 2;
    std::nth_element(order.begin(), order.begin() + static_cast<ptrdiff_t>(k), order.end(),
                     [&](size_t a, size_t b) {
                         return timestamps_ns[source_idx[a]] < timestamps_ns[source_idx[b]];
                     });

    const uint32_t median_orig = source_idx[order[k]];
    const size_t median_frame_idx = frame_idxs[median_orig];
    const int median_col_idx = static_cast<int>(col_idxs[median_orig]);
    const uint64_t median_ts_ns = timestamps_ns[median_orig];

    // World-frame pose of the chosen median-timestamp source (lidar column or camera).
    core::Matrix4dR median_point_pose =
        impl::source_point_pose(frame_set, static_cast<uint32_t>(median_frame_idx), median_col_idx);

    if (valid_lidar_column_ratio < min_valid_lidar_column_ratio) {
        logger().warn(
            "Received only {}/{} expected lidar columns. Minimum valid lidar "
            "column ratio is {}. Skipping ICP registration and map update for "
            "this frame.",
            received_lidar_columns, expected_lidar_columns, min_valid_lidar_column_ratio);

        // ICP did not run, so feed back the open-loop deskew pose instead of a
        // post-registration corrected pose.
        if (deskew_method_) {
            deskew_method_->set_last_pose(static_cast<int64_t>(median_ts_ns), median_point_pose);
        }

        active_time_correction_.reset(frame_set);
        impl::update_camera_poses(frame_set);
        return;
    }

    // ICP in world frame; returns near-identity correction
    const double sigma = adaptive_threshold_->compute_threshold();
    const auto icp_correction =
        registration_->align_points_to_map(source, *local_map_, 3 * sigma, sigma / 3);
    adaptive_threshold_->update_model_deviation(icp_correction);

    // Clear the map on the first two frames to avoid adding improperly
    // deskewed frames
    if (frame_count_++ <= 2) {
        local_map_->clear();
    }
    // Apply ICP correction to world-frame points before adding to map
    core::ArrayXXdR icp_corrected_points(static_cast<Eigen::Index>(source.size()), 3);

    // TODO[UN]: cleanup code to use one method of pose_util::transform methods
    for (std::size_t i = 0; i < source.size(); ++i) {
        icp_corrected_points.row(static_cast<Eigen::Index>(i)) =
            (icp_correction.block<3, 3>(0, 0) * source[i] + icp_correction.block<3, 1>(0, 3))
                .transpose();
    }

    local_map_->add_points(icp_corrected_points);

    // TODO[UN]: Refactor this code to use the new_pose_m and use distance based
    // culling instead.
    // TODO[UN]: cleanup code to use one method of pose_util::transform methods
    if (frame_count_ % local_map_cull_interval_frames == 0) {
        local_map_->remove_voxels_far_from_location(icp_correction.block<3, 3>(0, 0) *
                                                        median_point_pose.block<3, 1>(0, 3) +
                                                    icp_correction.block<3, 1>(0, 3));
    }

    // Apply ICP correction to column poses (world -> corrected world)
    impl::transform_inplace(frame_set, icp_correction);

    // Solve a refined initial velocity from the corrected anchor for the next
    // frame's deskew prediction, without modifying the current frame's poses.
    if (deskew_method_ && median_frame_idx < frame_set.size()) {
        core::Matrix4dR pose = frame_set[median_frame_idx]->get_column_pose(median_col_idx);
        deskew_method_->finalize_after_registration(frame_set, static_cast<int64_t>(median_ts_ns),
                                                    pose);
    }

    active_time_correction_.reset(frame_set);

    // Re-anchor object poses to the corrected per-column poses, interpolated at
    // each object's timestamp.
    impl::update_object_poses(frame_set);
    impl::update_camera_poses(frame_set);
}

core::PointCloudXYZf LIOSlam::get_point_cloud() const {
    // Convert the local map to PointCloudXYZf format
    core::PointCloudXYZf points;
    points = local_map_->pointcloud().xyz.matrix().cast<float>();
    return points;
}

core::VoxelHashMap& LIOSlam::get_local_map() const {
    if (!this->local_map_) {
        throw std::logic_error(
            "LIOSlam must be initialized prior to accessing the internal voxel hash map");
    }

    return *this->local_map_;
}

void LIOSlam::initialize_components() {
    adaptive_threshold_ = std::make_unique<AdaptiveThreshold>(config_.max_range);
    local_map_ = VoxelHashMap::create(
        config_.voxel_size, LOCAL_MAP_MAX_POINTS_PER_VOXEL, /*min_points_per_voxel=*/1,
        core::VoxelDownsampleStrategy::FIRST_N_POINT, core::AttributeLayout(),
        config_.voxel_map_size, config_.voxel_storage_mode);
    registration_ = std::make_unique<ICPRegistration>(config_.max_iterations);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
