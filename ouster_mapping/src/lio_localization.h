#pragma once

#include <ouster/core/voxel_hash_map.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/visibility.h"
#include "ouster/mapping/active_time_correction.h"
#include "ouster/mapping/adaptive_threshold.h"
#include "ouster/mapping/deskew_method.h"
#include "ouster/mapping/icp_registration.h"
#include "ouster/mapping/localization_engine.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class LIOLocalization
 * @brief Lidar Inertial Odometry (LIO) based localization backend.
 *
 * The LIOLocalization class implements the LocalizationEngine interface
 * using the KISS-ICP algorithm for point cloud registration and localization.
 */
class OUSTER_API_CLASS LIOLocalization : public LocalizationEngine {
   public:
    /**
     * @brief Constructs a LIOLocalization object.
     *
     * @param[in] infos Vector of shared pointers to sensor information objects.
     * @param[in] config Localization configuration parameters.
     * @param[in] map Reference to the map points used for localization.
     *
     * @throws std::invalid_argument If no SensorInfo is provided.
     */
    OUSTER_API_FUNCTION
    LIOLocalization(const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
                    LIOLocalizationConfig config, const Eigen::Ref<const core::PointCloudXYZf> map);

    /**
     * @brief Let LIOLocalization process LidarFrames and rely on current world
     * map to estimate the new state (pose).
     *
     * This function processes a set of LidarFrames provided as vector of shared
     * pointers of LidarFrame, The LIOLocalization updates the per-column pose
     * of input LidarFrames after registration.
     *
     * @param[in,out] frame_set A vector of LidarFrame objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void update(core::FrameSet& frame_set) override;

   private:
    void initialize_components();

    std::unique_ptr<AdaptiveThreshold> adaptive_threshold_;
    std::unique_ptr<ouster::sdk::core::VoxelHashMap3d> local_map_;
    std::unique_ptr<ICPRegistration> registration_;
    size_t frame_count_ = 0;

    std::vector<core::XYZLut> xyz_lut_;
    core::PointCloudXYZf map_;  // mainly used for delayed load

    std::unique_ptr<DeskewMethod> deskew_method_;
    ActiveTimeCorrection active_time_correction_;
    LIOLocalizationConfig config_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
