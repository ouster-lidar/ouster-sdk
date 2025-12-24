#include "kiss_localization.h"

#include <ouster/impl/logging.h>

#include <cstddef>
#include <exception>
#include <stdexcept>

#include "ouster/pose_util.h"
#include "slam_util.h"

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace mapping {

KissLocalization::KissLocalization(
    const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
    const LocalizationConfig& config,
    const Eigen::Ref<const core::PointCloudXYZf> map)
    : LocalizationBackend(infos, config),
      last_pose_(Sophus::SE3d(impl::make_ortho(config_.initial_pose))),
      map_(map),
      active_time_correction_(infos) {
    if (infos.empty()) {
        throw std::invalid_argument("No sensor info provided for localization");
    }

    xyz_lut_.resize(infos.size());
    for (size_t i = 0; i < infos.size(); ++i) {
        xyz_lut_[i] = core::XYZLut(*infos[i], true);
    }

    if (config.voxel_size > 0.0) {
        initialize_kiss_icp();
    }

    deskew_method_ = DeskewMethodFactory::create(config_.deskew_method, infos);
}

/*
 * Update the pose information of each lidar scan based on Localization
 * pose estimation.
 *
 *  Workflow:
 *  • Dynamic voxel size:
 *      - If a voxel_size was provided then apply it.
 *  • Time correction: Apply time correction to each scan.
 *     - Check inter-sensor synchronization
 *     - Monotonicity check (ignoring zero timestamps)
 *     - Packet-offset handling
 *     - Abort early if offset computation fails.
 *  • Frame-ID handling:
 *      - Compute per-sensor frame-ID diffs (with overflow protection).
 *      - Invalidate any out-of-order scans by zeroing their ranges.
 *      - Track the smallest non-zero diff for SLAM integration.
 *  • Point-cloud aggregation:
 *      - Merge valid points and their (possibly offset) timestamps.
 *      - Abort early if no points remain.
 *  • SLAM registration:
 *      - Register the aggregated frame with KISS-ICP.
 *      - No map update.
 *  • Pose interpolation:
 *      - Interpolate between the last and new SLAM poses.
 *      - Write per-column poses back into each scan.
 *  • Update internal state:
 *      - Store the new last SLAM pose, last frame-ID array, and timestamp
 * range.
 */
void KissLocalization::update(core::LidarScanSet& scans) {
    if (config_.voxel_size <= 0) {
        logger().info(
            "voxel size was not specified, generating an estimated "
            "value..");
        auto voxel_size = impl::determine_voxel_size(scans);
        config_.voxel_size = voxel_size.value_or(1.0);
        logger().info("Using voxel size {} m", config_.voxel_size);
        initialize_kiss_icp();
    }

    active_time_correction_.update(scans);

    // Select the midest point of the lidar_scan_set
    const auto ts_range = impl::compute_frame_ts_range(scans);

    if (ts_range.second <= ts_range.first) {
        logger().warn(
            "None of the scan time stamps are valid or the lidarscan collation "
            "duration is zero. This prevents timestamp normalization. Skipping "
            "the scan!");
        return;
    }

    if (deskew_method_) {
        deskew_method_->update(scans);
    }

    const auto scan_set_mid = impl::find_scan_set_mid(scans, ts_range);
    core::Matrix4dR mid_point_pose =
        scans[scan_set_mid.first]->get_column_pose(scan_set_mid.second);

    // transform poses into relative space
    impl::transform_inplace(scans, mid_point_pose.inverse());
    auto frame =
        core::dewarp(scans, xyz_lut_, config_.min_range, config_.max_range);

    if (frame.empty()) {
        logger().warn(
            "No valid points found in the scans. Skipping this update.");
        return;
    }

    auto new_pose =
        kiss_icp_->register_frame(frame, Sophus::SE3d(mid_point_pose), false);

    // transform poses to global space.
    core::Matrix4dR new_pose_m = new_pose.matrix();
    impl::transform_inplace(scans, new_pose_m);

    if (deskew_method_) {
        uint64_t ts =
            scans[scan_set_mid.first]->timestamp()[scan_set_mid.second];
        core::Matrix4dR pose =
            scans[scan_set_mid.first]->get_column_pose(scan_set_mid.second);
        deskew_method_->set_last_pose(ts, pose);
    }

    active_time_correction_.reset(scans);
}

void KissLocalization::initialize_kiss_icp() {
    auto config = impl::KissConfig();
    config.max_range = config_.max_range;
    config.min_range = config_.min_range;
    config.voxel_size = config_.voxel_size;
    config.max_num_iterations = 50;
    config.min_motion_th = 0.01;
    kiss_icp_ = std::make_unique<impl::KissICP>(config);

    std::vector<Eigen::Vector3d> vec3d;
    vec3d.resize(map_.rows());
    for (int i = 0; i < map_.rows(); ++i) {
        vec3d[i] = map_.row(i).cast<double>();
    }

    kiss_icp_->add_points_to_local_map(vec3d);
    // Clear the map points after loading them into the odometry
    map_.resize(0, 0);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
