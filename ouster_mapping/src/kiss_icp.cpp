#include "kiss_icp.h"

#include <stdexcept>

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

KissICP::KissICP(const KissConfig& config)
    : config_(config),
      adaptive_threshold_(config.initial_threshold, config.min_motion_th,
                          config.max_range),
      local_map_(config.voxel_size, config.max_range,
                 config.max_points_per_voxel),
      registration_(config.max_num_iterations, config.convergence_criterion,
                    config.max_num_threads) {}

Sophus::SE3d KissICP::register_frame(const std::vector<Eigen::Vector3d>& frame,
                                     const Sophus::SE3d& initial_guess,
                                     bool update_map) {
    auto downsampled = voxelize(frame);
    std::vector<Eigen::Vector3d> source = std::get<0>(downsampled);
    std::vector<Eigen::Vector3d> frame_downsample = std::get<1>(downsampled);

    const double sigma = adaptive_threshold_.ComputeThreshold();

    // Run ICP
    const auto new_pose = registration_.AlignPointsToMap(
        source, local_map_, initial_guess, 3 * sigma, sigma / 3);

    // Compute the difference between the prediction and the actual estimate
    const auto model_deviation = initial_guess.inverse() * new_pose;

    // Update step: threshold, local map, delta, and the last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);

    if (update_map) {
        // Clear the map on the first two frames to avoid adding improperly
        // deskewed frames
        if (frame_count_++ <= 2) {
            local_map_.Clear();
        }
        local_map_.Update(frame_downsample, new_pose);
    }

    return new_pose;
}

kiss_icp::pipeline::KissICP::Vector3dVectorTuple KissICP::voxelize(
    const std::vector<Eigen::Vector3d>& iframe) {
    std::vector<Eigen::Vector3d> frame_downsample =
        kiss_icp::VoxelDownsample(iframe, config_.voxel_size * 0.5);
    std::vector<Eigen::Vector3d> source =
        kiss_icp::VoxelDownsample(frame_downsample, config_.voxel_size * 1.5);
    return std::make_pair(source, frame_downsample);
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
