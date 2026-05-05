#pragma once

#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <ouster/visibility.h>

#include <string>
#include <vector>

#include "localization_backend.h"
#include "ouster/lidar_scan_set.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class LocalizationEngine
 * @brief Engine for performing localization using LidarScans and a pre-computed
 * map.
 *
 * The LocalizationEngine class provides functionality to estimate the pose of a
 * sensor by processing Lidar scans and matching them against a world map. It
 * supports initialization with either a map file or a preloaded map.
 *
 * @note Only the "kiss" backend is supported at this time.
 *
 * @see LidarScan
 * @see LocalizationConfig
 * @see LocalizationBackend
 */
class OUSTER_API_CLASS LocalizationEngine {
   public:
    /**
     * @brief Constructs the LocalizationEngine.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects.
     * @param[in] config The LocalizationConfig object containing configuration
     *                   parameters for the localization.
     * @param[in] map_path The file path containing the map to be used for
     *                     localization.
     *
     * @throws std::runtime_error If the specified backend is not supported.
     * @throws std::runtime_error If the map file cannot be loaded.
     */
    OUSTER_API_FUNCTION
    LocalizationEngine(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos,
        const LocalizationConfig& config, const std::string& map_path);

    /**
     * @brief Constructs the LocalizationEngine.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects.
     * @param[in] config The LocalizationConfig object containing configuration
     *                   parameters for the localization.
     * @param[in] map The preloaded map as a PointCloudXYZf object.
     */
    OUSTER_API_FUNCTION
    LocalizationEngine(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos,
        const LocalizationConfig& config,
        const Eigen::Ref<const ouster::sdk::core::PointCloudXYZf> map);

    /**
     * @brief Let LocalizationEngin process LidarScans and rely on current world
     * map to estimate the new state (pose).
     *
     * This function processes a set of LidarScans provided as vector of shared
     * pointers of LidarScan, The engine returns these LidarScans with their
     * per-column pose corrected.
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void update(ouster::sdk::core::LidarScanSet& scans);

   private:
    ouster::sdk::core::PointCloudXYZf load_map(const std::string& map_file);

   private:
    std::unique_ptr<LocalizationBackend> backend_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
