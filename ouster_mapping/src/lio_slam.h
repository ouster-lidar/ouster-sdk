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
 * @brief LIO based SLAM backend.
 *
 * The LIOSlam class implements the SlamEngine interface
 * using the LIO algorithm for point cloud registration and SLAM.
 */
class OUSTER_API_CLASS LIOSlam : public SlamEngine {
   public:
    /**
     * @brief Constructs a LIOSlam object.
     *
     * @param[in] infos Vector of shared pointers to sensor information objects.
     * @param[in] config SLAM configuration parameters.
     *
     * @throws std::invalid_argument If no SensorInfo is provided.
     */
    OUSTER_API_FUNCTION
    LIOSlam(const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
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

   private:
    void initialize_components();

    std::unique_ptr<AdaptiveThreshold> adaptive_threshold_;
    std::unique_ptr<ouster::sdk::core::VoxelHashMap3d> local_map_;
    std::unique_ptr<ICPRegistration> registration_;
    size_t frame_count_ = 0;

    std::vector<core::XYZLut> xyz_lut_;

    std::unique_ptr<DeskewMethod> deskew_method_;
    ActiveTimeCorrection active_time_correction_;

    LIOSlamConfig config_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
