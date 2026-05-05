#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "kiss_icp.h"
#include "ouster/active_time_correction.h"
#include "ouster/deskew_method.h"
#include "ouster/lidar_scan_set.h"
#include "ouster/slam_backend.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class KissSlam
 * @brief KISS-ICP based SLAM backend.
 *
 * The KissSlam class implements the SlamBackend interface
 * using the KISS-ICP algorithm for point cloud registration and SLAM.
 */
class OUSTER_API_CLASS KissSlam : public SlamBackend {
   public:
    /**
     * @brief Constructs a KissSlam object.
     *
     * @param[in] infos Vector of shared pointers to sensor information objects.
     * @param[in] config SLAM configuration parameters.
     *
     * @throws std::invalid_argument If no SensorInfo is provided.
     */
    OUSTER_API_FUNCTION
    KissSlam(const std::vector<std::shared_ptr<core::SensorInfo>>& infos,
             const SlamConfig& config);
    /**
     * @brief Updates the pose information of each lidar scan based on SLAM pose
     * estimation.
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void update(core::LidarScanSet& scans) override;

    OUSTER_API_FUNCTION
    core::PointCloudXYZf get_point_cloud() const override;

   private:
    void initialize_kiss_icp();

    std::unique_ptr<impl::KissICP> kiss_icp_;
    Sophus::SE3d last_pose_;
    std::vector<core::XYZLut> xyz_lut_;

    std::unique_ptr<DeskewMethod> deskew_method_;
    ActiveTimeCorrection active_time_correction_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
