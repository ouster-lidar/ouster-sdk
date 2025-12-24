#pragma once

#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <ouster/visibility.h>

#include <string>
#include <vector>

#include "ouster/lidar_scan_set.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @brief Configuration parameters for the Localization backend.
 */
struct OUSTER_API_CLASS LocalizationConfig {
    double min_range = 0.0;    ///< Minimum range of sensor measurements to be
                               ///< considered (meters)
    double max_range = 150.0;  ///< Maximum range of sensor measurements to be
                               ///< considered (meters)
    double voxel_size = 0.0;   ///< Size of the voxel grid for downsampling
    Eigen::Matrix4d initial_pose =
        Eigen::Matrix4d::Identity();  ///< Initial pose of the sensor
    std::string backend = "kiss";  ///< The backend to be used for Localization
    std::string deskew_method = "auto";  ///< Deskewing method to be applied
};

/**
 * @class LocalizationBackend
 * @brief Abstract base class for Localization backend.
 *
 * The LocalizationBackend class provides an interface for processing
 * LidarScans, estimating poses, and generating point clouds using a preloaded
 * map. Implementations of this class are responsible for processing incoming
 * lidar data, retrieving the latest estimated pose, and providing the current
 * point cloud.
 *
 * @note This class is intended to be subclassed; all core methods are pure
 * virtual and must be implemented by derived classes.
 *
 * @see LidarScan
 * @see LocalizationConfig
 * @see LocalizationEngine
 */
class OUSTER_API_CLASS LocalizationBackend {
   public:
    /**
     * @brief Constructs the LocalizationBackend.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects.
     * @param[in] config The LocalizationConfig object containing configuration
     *                   parameters for the localization.
     */
    OUSTER_API_FUNCTION
    LocalizationBackend(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos,
        const LocalizationConfig& config)
        : infos_(infos), config_(config) {}

    /**
     * @brief LocalizationBackend destructor.
     */
    OUSTER_API_FUNCTION
    virtual ~LocalizationBackend() = default;

    /**
     * @brief Let Localization process LidarScans and rely on current world map
     * to estimate the new state (pose).
     *
     * This function processes a set of LidarScans provided as vector of shared
     * pointers of LidarScan, The engine returns these LidarScans with their
     * per-column pose corrected.
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::LidarScanSet& scans) = 0;

   protected:
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> infos_;
    LocalizationConfig config_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
