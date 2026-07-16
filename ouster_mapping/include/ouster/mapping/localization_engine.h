#pragma once

#include <ouster/core/lidar_frame.h>
#include <ouster/core/types.h>
#include <ouster/core/visibility.h>

#include <string>
#include <vector>

#include "ouster/core/frame_set.h"

namespace ouster {
namespace sdk {
namespace mapping {

/// @brief Common configuration parameters for LocalizationEngine
/// implementations.
struct OUSTER_API_CLASS LocalizationConfig {
   protected:
    LocalizationConfig() = default;

   public:
    double min_range = 0.0;    ///< Minimum range of sensor measurements to be
                               ///< considered (meters)
    double max_range = 150.0;  ///< Maximum range of sensor measurements to be
                               ///< considered (meters)
    double voxel_size = 0.0;   ///< Size of the voxel grid for downsampling
    int max_iterations = 500;  ///< Maximum number of ICP registration iterations
    core::Matrix4dR initial_pose = core::Matrix4dR::Identity();  ///< Initial pose of the sensor
    std::string deskew_method = "auto";                          ///< Deskewing method to be applied
};

/**
 * @brief Configuration class for the LIO-based LocalizationEngine.
 */
struct OUSTER_API_CLASS LIOLocalizationConfig : public LocalizationConfig {
    /// Default constructor for LIOLocalizationConfig.
    OUSTER_API_FUNCTION
    LIOLocalizationConfig() = default;
};

/**
 * @class LocalizationEngine
 * @brief Engine for performing localization using LidarFrames and a
 * pre-computed map.
 *
 * The LocalizationEngine class provides functionality to estimate the pose of a
 * sensor by processing LiDAR frames and matching them against a world map. It
 * supports initialization with either a map file or a preloaded map.
 *
 * @note Only the "lio" backend is supported at this time.
 *
 * @see LidarFrame
 * @see LocalizationConfig
 */
class OUSTER_API_CLASS LocalizationEngine {
   public:
    /// Destructor for LocalizationEngine.
    OUSTER_API_FUNCTION
    virtual ~LocalizationEngine() = default;

    /**
     * @brief Constructs the LocalizationEngine with a map file, using a
     * default config.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects.
     * @param[in] map_path The file path to the pre-computed map.
     * @param[in] config Configuration parameters. Defaults to
     * LIOLocalizationConfig{}.
     * @throws std::runtime_error If the map file cannot be loaded.
     * @return A unique pointer to the created LocalizationEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<LocalizationEngine> create(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos,
        const std::string& map_path, const LIOLocalizationConfig& config = LIOLocalizationConfig{});

    /**
     * @brief Constructs the LocalizationEngine with a preloaded map, using a
     * default config.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects.
     * @param[in] map A preloaded point cloud map to be used for localization.
     * @param[in] config Configuration parameters. Defaults to
     * LIOLocalizationConfig{}.
     * @return A unique pointer to the created LocalizationEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<LocalizationEngine> create(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos,
        const Eigen::Ref<const ouster::sdk::core::PointCloudXYZf> map,
        const LIOLocalizationConfig& config = LIOLocalizationConfig{});

    /**
     * @brief Let LocalizationEngine process LidarFrames and rely on current
     * world map to estimate the new state (pose).
     *
     * This function processes a set of LidarFrames provided as vector of shared
     * pointers of LidarFrame, The engine returns these LidarFrames with their
     * per-column body_to_world transform corrected.
     *
     * @param[in,out] frame_set A vector of LidarFrame objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::FrameSet& frame_set) = 0;

   protected:
    LocalizationEngine(const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos);

    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> infos_;

   private:
    static ouster::sdk::core::PointCloudXYZf load_map(const std::string& map_file);
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
