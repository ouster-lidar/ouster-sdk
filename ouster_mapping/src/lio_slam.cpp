#include "lio_slam.h"

#include <ouster/core/impl/logging.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "ouster/core/pose_util.h"
#include "slam_util.h"

using ouster::sdk::core::logger;
using ouster::sdk::core::VoxelHashMap3d;

namespace ouster {
namespace sdk {
namespace mapping {

LIOSlam::LIOSlam(const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
                 const LIOSlamConfig& config)
    : SlamEngine(infos), active_time_correction_(infos), config_(config) {
    if (infos.empty()) {
        throw std::invalid_argument("No sensor info provided for slam");
    }

    xyz_lut_.reserve(infos.size());
    for (const auto& info : infos) {
        xyz_lut_.emplace_back(*info, true);
    }

    if (config.voxel_size > 0.0) {
        initialize_components();
    }

    deskew_method_ =
        DeskewMethodFactory::create(config_.deskew_method, infos, config_.initial_pose);
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
 *      - Merge valid points and their (possibly offset) timestamps.
 *      - Abort early if no points remain.
 *  • SLAM registration:
 *      - Register the aggregated frame with KISS-ICP.
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

    // Validate timestamp range
    if (ts_range.second <= ts_range.first) {
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
        impl::init_valid_column_poses(frame_set, config_.initial_pose);
    }

    // TODO[UN]: refactor this and move it close to where it is actually used
    // If too much lidar data is missing, trust the deskew method's open-loop
    // prediction for this frame instead of letting sparse data produce a bad
    // ICP correction and corrupt the voxel map.
    size_t expected_lidar_columns = 0;
    size_t received_lidar_columns = 0;
    for (size_t idx : frame_set.valid_indices()) {
        expected_lidar_columns +=
            static_cast<size_t>(infos_[idx]->format.valid_columns_per_frame());
        const auto& frame = frame_set[idx];
        Eigen::Ref<const core::LidarFrame::Header<uint32_t>> status = frame->status();
        received_lidar_columns += impl::count_valid_columns(status);
    }

    // TODO[UN]: move this computed value closer to where it is actually used
    const double valid_lidar_column_ratio = expected_lidar_columns > 0
                                                ? static_cast<double>(received_lidar_columns) /
                                                      static_cast<double>(expected_lidar_columns)
                                                : 1.0;

    // Dewarp: range data + world-frame column poses -> world-frame 3D points,
    // keeping per-point (frame_idx, col_idx, col_timestamp) provenance so we
    // can map the downsampled median back to a concrete source column.
    std::vector<uint32_t> frame_idxs;
    std::vector<uint32_t> col_idxs;
    std::vector<uint64_t> timestamps_ns;
    auto frame =
        core::impl::dewarp_impl<double>(frame_set, xyz_lut_, config_.min_range, config_.max_range,
                                        &frame_idxs, &col_idxs, &timestamps_ns);

    if (frame.empty()) {
        logger().warn("No valid points found in the frame set. Skipping this update.");
        active_time_correction_.reset(frame_set);
        return;
    }

    // Two-level voxel downsample, both in world frame. The fine pass thins
    // each coarse voxel to a roughly-uniform spatial distribution before the
    // coarse pass picks one representative per voxel, so dense regions don't
    // dominate the ICP source cloud. Both passes return original-index maps,
    // which we thread together to recover provenance for any source point.
    const auto fine_ds = core::voxel_downsample(frame, config_.voxel_size * 0.5);
    const auto& frame_downsample = fine_ds.first;
    const auto& fine_idx = fine_ds.second;
    const auto coarse_ds = core::voxel_downsample(frame_downsample, config_.voxel_size * 1.5);
    const auto& source = coarse_ds.first;
    const auto& coarse_idx = coarse_ds.second;

    // Pick the column whose timestamp is the median over the ICP source
    // cloud. Weighting the origin toward where returns actually exist gives
    // a more representative pose than the raw frame-set temporal midpoint
    // (e.g. when one side of the frame is empty sky).
    std::vector<size_t> order(source.size());
    std::iota(order.begin(), order.end(), 0);
    const size_t k = order.size() / 2;
    std::nth_element(order.begin(), order.begin() + static_cast<ptrdiff_t>(k), order.end(),
                     [&](size_t a, size_t b) {
                         return timestamps_ns[fine_idx[coarse_idx[a]]] <
                                timestamps_ns[fine_idx[coarse_idx[b]]];
                     });

    const uint32_t median_orig = fine_idx[coarse_idx[order[k]]];
    const size_t median_frame_idx = frame_idxs[median_orig];
    const int median_col_idx = static_cast<int>(col_idxs[median_orig]);
    const uint64_t median_ts_ns = timestamps_ns[median_orig];

    // World-frame pose of the chosen median-timestamp column
    core::Matrix4dR median_point_pose =
        frame_set[median_frame_idx]->get_column_pose(median_col_idx);

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
    std::vector<Eigen::Vector3d> icp_corrected_points(frame_downsample.size());

    // TODO[UN]: cleanup code to use one method of pose_util::transform methods
    std::transform(frame_downsample.cbegin(), frame_downsample.cend(), icp_corrected_points.begin(),
                   [&](const auto& point) {
                       return icp_correction.block<3, 3>(0, 0) * point +
                              icp_correction.block<3, 1>(0, 3);
                   });

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
    if (deskew_method_) {
        core::Matrix4dR pose = frame_set[median_frame_idx]->get_column_pose(median_col_idx);
        deskew_method_->finalize_after_registration(frame_set, static_cast<int64_t>(median_ts_ns),
                                                    pose);
    }

    active_time_correction_.reset(frame_set);

    // Re-anchor object poses to the corrected per-column poses, interpolated at
    // each object's timestamp.
    impl::update_object_poses(frame_set);
}

core::PointCloudXYZf LIOSlam::get_point_cloud() const {
    // Convert the local map to PointCloudXYZf format
    core::PointCloudXYZf points;
    // TODO[UN]: we should be able to simply cast into the matrix float
    // directly.
    const auto local_map = local_map_->pointcloud_vector();
    points.resize(static_cast<Eigen::Index>(local_map.size()), 3);
    for (size_t i = 0; i < local_map.size(); ++i) {
        points.row(static_cast<Eigen::Index>(i)) = local_map[i].cast<float>();
    }
    return points;
}

void LIOSlam::initialize_components() {
    adaptive_threshold_ = std::make_unique<AdaptiveThreshold>(config_.max_range);
    local_map_ = std::make_unique<VoxelHashMap3d>(config_.voxel_size, config_.max_range);
    registration_ = std::make_unique<ICPRegistration>(config_.max_iterations);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
