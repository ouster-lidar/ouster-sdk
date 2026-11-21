#pragma once

#include <ouster/core/camera_info.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/sensor_info.h>
#include <ouster/core/types.h>
#include <ouster/core/visibility.h>
#include <ouster/core/voxel_hash_map.h>

#include <limits>
#include <memory>
#include <nonstd/optional.hpp>
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
    double voxel_map_size =
        std::numeric_limits<double>::infinity();  ///< Maximum distance from
                                                  ///< the initial pose at which points are retained
                                                  ///< in the local voxel map (meters). Defaults to
                                                  ///< infinity, i.e. no trimming of the preloaded
                                                  ///< map.
    int max_iterations = 500;                     ///< Maximum number of ICP registration iterations
    nonstd::optional<core::Matrix4dR> initial_pose;  ///< Body-to-world pose in the map frame,
                                                     ///< used as the first anchor. Unset starts
                                                     ///< at identity
    std::string deskew_method = "auto";              ///< Deskewing method to be applied
    std::string plumb_method = "all";  ///< How the IMU deskew method plumbs gravity on the
                                       ///< first frame set: "imu", "walls" or "all" (IMU
                                       ///< estimate refined by wall geometry)
    core::VHMInternalStorageMode voxel_storage_mode =
        core::VHMInternalStorageMode::AUTO;  ///< Internal storage representation for the voxels. By
                                             ///< default it will choose the best for the voxel
                                             ///< size.
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
     * @brief Constructs the LocalizationEngine from a unified list of sensor
     * metadata and a map file.
     *
     * Accepts a single vector of SensorInfo (as returned by
     * FrameSetSource::sensor_info()), which may contain a mix of LidarInfo and
     * CameraInfo entries in any order. The list shares its index space with the
     * FrameSet (sensor i describes frame i) and is dispatched on concrete type
     * internally, so lidar-only, camera-only, and mixed sources are all
     * supported through one call. Entries that are neither LidarInfo nor
     * CameraInfo are ignored.
     *
     * @param[in] sensor_infos Mixed lidar/camera calibration.
     * @param[in] map_path The file path to the pre-computed map.
     * @param[in] config Configuration parameters. Defaults to
     * LIOLocalizationConfig{}.
     * @throws std::runtime_error If the map file cannot be loaded.
     * @return A unique pointer to the created LocalizationEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<LocalizationEngine> create(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const std::string& map_path, const LIOLocalizationConfig& config = LIOLocalizationConfig{});

    /**
     * @brief Constructs the LocalizationEngine from a unified list of sensor
     * metadata and a preloaded map.
     *
     * See the map-file overload above for how the unified sensor list is
     * handled.
     *
     * @param[in] sensor_infos Mixed lidar/camera calibration.
     * @param[in] map A preloaded point cloud map to be used for localization.
     * @param[in] config Configuration parameters. Defaults to
     * LIOLocalizationConfig{}.
     * @return A unique pointer to the created LocalizationEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<LocalizationEngine> create(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const Eigen::Ref<const ouster::sdk::core::PointCloudXYZf> map,
        const LIOLocalizationConfig& config = LIOLocalizationConfig{});

    /**
     * @brief Let LocalizationEngine process LidarFrames and rely on current
     * world map to estimate the new state (pose).
     *
     * This function processes a FrameSet of LidarFrames (and any attached
     * CameraFrames). The engine writes per-column body_to_world transforms
     * onto each lidar frame and interpolates a single body_to_world pose
     * onto each camera frame at its timestamp.
     *
     * @param[in,out] frame_set FrameSet of lidar and optional camera frames to
     * be processed.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::FrameSet& frame_set) = 0;

    /**
     * @brief Get the underlying local voxel hash map
     * @throws std::logic_error if the map is not available yet (e.g. engine has not been
     * initialized due to lazy initialization)
     * @return Reference to the voxel hash map object.
     */
    OUSTER_API_FUNCTION
    virtual ouster::sdk::core::VoxelHashMap& get_local_map() const = 0;

   protected:
    /// Constructs the engine from a unified sensor list. Lidar and camera
    /// calibration are stored together and dispatched on concrete type
    /// internally. The list shares the FrameSet index space (sensor i describes
    /// frame i) in whatever order the caller provides; no lidar-first ordering
    /// is assumed.
    explicit LocalizationEngine(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos);

    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_infos_;

   private:
    static ouster::sdk::core::PointCloudXYZf load_map(const std::string& map_file);
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
