#pragma once

#include <ouster/core/voxel_hash_map.h>

#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/visibility.h"
#include "ouster/mapping/active_time_correction.h"
#include "ouster/mapping/adaptive_threshold.h"
#include "ouster/mapping/deskew_method.h"
#include "ouster/mapping/icp_registration.h"
#include "ouster/mapping/slam_engine.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class LIOSlam
 * @brief Lidar Inertial Odometry (LIO) based SLAM backend.
 *
 * The LIOSlam class implements the SlamEngine interface
 * using ICP-based scan-to-map registration.
 */
class OUSTER_API_CLASS LIOSlam : public SlamEngine {
   public:
    /**
     * @brief Constructs a LIOSlam object.
     *
     * @param[in] sensor_infos Unified sensor calibration (a mix of LidarInfo
     *            and CameraInfo). The list is reordered lidar-first internally
     *            and dispatched on concrete type, so lidar-only, camera-only,
     *            and mixed sources are all supported.
     * @param[in] config SLAM configuration parameters.
     *
     * @throws std::invalid_argument If no sensor calibration is provided.
     */
    OUSTER_API_FUNCTION
    LIOSlam(const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos,
            const LIOSlamConfig& config);
    /**
     * @brief Updates the pose information of each lidar frame based on SLAM
     * pose estimation.
     *
     * @param[in,out] frame_set A vector of LidarFrame objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void update(core::FrameSet& frame_set) override;

    OUSTER_API_FUNCTION
    core::PointCloudXYZf get_point_cloud() const override;

    /**
     * @brief Get the underlying local voxel hash map
     *
     * This should only be called once the engine has been initialized. If no
     * voxel size is specified (automatic), then only after a frame of data has been processed.
     *
     * @throws std::logic_error if the engine has not been initialized yet
     * @return Reference to the voxel hash map object.
     */
    OUSTER_API_FUNCTION
    core::VoxelHashMap& get_local_map() const override;

   private:
    void initialize_components();

    std::unique_ptr<AdaptiveThreshold> adaptive_threshold_;
    std::unique_ptr<ouster::sdk::core::VoxelHashMap> local_map_;
    std::unique_ptr<ICPRegistration> registration_;
    size_t frame_count_ = 0;

    std::unique_ptr<DeskewMethod> deskew_method_;
    ActiveTimeCorrection active_time_correction_;

    LIOSlamConfig config_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
