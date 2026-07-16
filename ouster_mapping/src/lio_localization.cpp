#include "lio_localization.h"

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

LIOLocalization::LIOLocalization(const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
                                 LIOLocalizationConfig config,
                                 const Eigen::Ref<const core::PointCloudXYZf> map)
    : LocalizationEngine(infos),
      map_(map),
      active_time_correction_(infos),
      config_(std::move(config)) {
    if (infos.empty()) {
        throw std::invalid_argument("No sensor info provided for localization");
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
 * Update the pose information of each lidar frame based on Localization
 * pose estimation.
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
 *      - No map update.
 *  • Pose interpolation:
 *      - Interpolate between the last and new localization poses.
 *      - Write per-column poses back into each frame.
 *  • Update internal state:
 *      - Store the new last localization pose, last frame-ID array, and
 * timestamp range.
 */
void LIOLocalization::update(core::FrameSet& frame_set) {
    constexpr double min_valid_lidar_column_ratio = 0.5;

    if (config_.voxel_size <= 0) {
        logger().info(
            "voxel size was not specified, generating an estimated "
            "value..");
        auto voxel_size = impl::determine_voxel_size(frame_set);
        config_.voxel_size = voxel_size.value_or(1.0);
        logger().info("Using voxel size {} m", config_.voxel_size);
        initialize_components();
    }

    active_time_correction_.update(frame_set);

    // Select the midest point of the frame set
    const auto ts_range = impl::compute_frame_ts_range(frame_set);

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
        // anchor it at the configured initial pose. This seeds the ICP search
        // near the initial pose in the map; later frames re-register globally.
        // frame_count_ only advances once a frame is actually registered, so a
        // skipped first frame still leaves the seed in place.
        impl::init_valid_column_poses(frame_set, config_.initial_pose);
    }

    // If too much lidar data is missing, trust the deskew method's open-loop
    // prediction for this frame instead of letting sparse data produce a bad
    // ICP correction and disrupt localization.
    size_t expected_lidar_columns = 0;
    size_t received_lidar_columns = 0;
    for (size_t idx : frame_set.valid_indices()) {
        expected_lidar_columns +=
            static_cast<size_t>(infos_[idx]->format.valid_columns_per_frame());
        const auto& frame = frame_set[idx];
        Eigen::Ref<const core::LidarFrame::Header<uint32_t>> status = frame->status();
        received_lidar_columns += impl::count_valid_columns(status);
    }

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

    // Single-pass voxel downsample. Localization never updates the map, so
    // the stratifying fine-pass used in SLAM is unnecessary here. The
    // returned index map is used below to thread provenance.
    const auto downsampled = core::voxel_downsample(frame, config_.voxel_size * 1.5);
    const auto& source = downsampled.first;
    const auto& ds_idx = downsampled.second;

    // Pick the column whose timestamp is the median over the ICP source
    // cloud. Weighting the origin toward where returns actually exist gives
    // a more representative pose than the raw frame-set temporal midpoint
    // (e.g. when one side of the frame is empty sky).
    std::vector<size_t> order(source.size());
    std::iota(order.begin(), order.end(), 0);
    const size_t k = order.size() / 2;
    std::nth_element(
        order.begin(), order.begin() + static_cast<ptrdiff_t>(k), order.end(),
        [&](size_t a, size_t b) { return timestamps_ns[ds_idx[a]] < timestamps_ns[ds_idx[b]]; });

    const uint32_t median_orig = ds_idx[order[k]];
    const size_t median_frame_idx = frame_idxs[median_orig];
    const int median_col_idx = static_cast<int>(col_idxs[median_orig]);
    const uint64_t median_ts_ns = timestamps_ns[median_orig];

    // World-frame pose of the chosen median-timestamp column
    core::Matrix4dR median_point_pose =
        frame_set[median_frame_idx]->get_column_pose(median_col_idx);

    if (valid_lidar_column_ratio < min_valid_lidar_column_ratio) {
        logger().warn(
            "Received only {}/{} expected lidar columns. Minimum valid lidar "
            "column ratio is {}. Skipping ICP localization update for this "
            "frame.",
            received_lidar_columns, expected_lidar_columns, min_valid_lidar_column_ratio);

        // ICP did not run, so feed back the open-loop deskew pose instead of a
        // post-registration corrected pose.
        if (deskew_method_) {
            deskew_method_->set_last_pose(static_cast<int64_t>(median_ts_ns), median_point_pose);
        }

        active_time_correction_.reset(frame_set);
        return;
    }

    // Count only frames that pass every skip guard above, so the initial-pose
    // seed stays active until the first frame is actually registered.
    frame_count_++;

    // ICP in world frame; returns near-identity correction
    const double sigma = adaptive_threshold_->compute_threshold();
    const auto icp_correction =
        registration_->align_points_to_map(source, *local_map_, 3 * sigma, sigma / 3);
    adaptive_threshold_->update_model_deviation(icp_correction);

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

void LIOLocalization::initialize_components() {
    adaptive_threshold_ = std::make_unique<AdaptiveThreshold>(config_.max_range);
    local_map_ = std::make_unique<VoxelHashMap3d>(config_.voxel_size, config_.max_range);
    registration_ = std::make_unique<ICPRegistration>(config_.max_iterations);

    std::vector<Eigen::Vector3d> vec3d;
    vec3d.resize(map_.rows());
    for (int i = 0; i < map_.rows(); ++i) {
        vec3d[i] = map_.row(i).cast<double>();
    }
    local_map_->add_points(vec3d);
    // Clear the map points after loading them into the local map
    map_.resize(0, 3);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
