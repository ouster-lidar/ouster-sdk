#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "kiss_icp.h"
#include "ouster/active_time_correction.h"
#include "ouster/deskew_method.h"
#include "ouster/lidar_scan_set.h"
#include "ouster/localization_backend.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class KissLocalization
 * @brief KISS-ICP based localization backend.
 *
 * The KissLocalization class implements the LocalizationBackend interface
 * using the KISS-ICP algorithm for point cloud registration and localization.
 */
class OUSTER_API_CLASS KissLocalization : public LocalizationBackend {
   public:
    /**
     * @brief Constructs a KissLocalization object.
     *
     * @param[in] infos Vector of shared pointers to sensor information objects.
     * @param[in] config Localization configuration parameters.
     * @param[in] map Reference to the map points used for localization.
     *
     * @throws std::invalid_argument If no SensorInfo is provided.
     */
    OUSTER_API_FUNCTION
    KissLocalization(
        const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
        const LocalizationConfig& config,
        const Eigen::Ref<const core::PointCloudXYZf> map);

    /**
     * @brief Let KissLocalization process LidarScans and rely on current world
     * map to estimate the new state (pose).
     *
     * This function processes a set of LidarScans provided as vector of shared
     * pointers of LidarScan, The KissLocalization updates the per-column pose
     * of input LidarScans after registration.
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void update(core::LidarScanSet& scans) override;

   private:
    void initialize_kiss_icp();

    std::unique_ptr<impl::KissICP> kiss_icp_;
    Sophus::SE3d last_pose_;
    std::vector<core::XYZLut> xyz_lut_;
    core::PointCloudXYZf map_;  // mainly used for delayed load

    std::unique_ptr<DeskewMethod> deskew_method_;
    ActiveTimeCorrection active_time_correction_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
