#pragma once

#include <ouster/core/camera_info.h>
#include <ouster/core/frame_set.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/sensor_info.h>
#include <ouster/core/types.h>
#include <ouster/core/visibility.h>
#include <ouster/core/voxel_hash_map.h>

#include <memory>
#include <nonstd/optional.hpp>
#include <vector>

namespace ouster {
namespace sdk {
namespace mapping {

/// @brief Common configuration parameters for SlamEngine implementations.
struct OUSTER_API_CLASS SlamConfig {
   protected:
    SlamConfig() = default;

   public:
    double min_range = 0.0;         ///< Minimum range of sensor measurements to be
                                    ///< considered (meters)
    double max_range = 150.0;       ///< Maximum range of sensor measurements to be
                                    ///< considered (meters)
    double voxel_size = 0.0;        ///< Size of the voxel grid for downsampling
    double voxel_map_size = 150.0;  ///< Maximum distance from the sensor origin at which
                                    ///< points are retained in the local voxel map (meters)
    int max_iterations = 500;       ///< Maximum number of ICP registration iterations
    nonstd::optional<core::Matrix4dR> initial_pose;  ///< Body-to-world pose in a
                                                     ///< gravity-levelled world frame, used as
                                                     ///< the first anchor; plumbing checks it
                                                     ///< and warns above 10 degrees, keeping
                                                     ///< the pose. Unset lets plumbing level the
                                                     ///< first frame
    std::string deskew_method = "auto";              ///< Deskewing method to be applied
    std::string plumb_method = "all";  ///< How the IMU deskew method plumbs the first frame
                                       ///< set: "imu", "walls" or "all" (IMU levelling
                                       ///< refined by wall geometry). With an initial_pose,
                                       ///< only the IMU plumb runs, as a check
    core::VHMInternalStorageMode voxel_storage_mode =
        core::VHMInternalStorageMode::AUTO;  ///< Internal storage representation for the voxels. By
                                             ///< default it will choose the best for the voxel
                                             ///< size.
};

/**
 * @brief Configuration class for the LIO-based SlamEngine.
 */
struct OUSTER_API_CLASS LIOSlamConfig : public SlamConfig {
    /// Default constructor for LIOSlamConfig.
    OUSTER_API_FUNCTION
    LIOSlamConfig() = default;
};

/**
 * @class SlamEngine
 * @brief SLAM engine for processing sensor data using specified backend.
 *
 * This class provides an interface for running SLAM (Simultaneous Localization
 * and Mapping) algorithms on lidar and other sensor data.
 *
 * @note Only the "lio" backend is supported at this time.
 *
 * @see LidarFrame
 * @see SlamConfig
 */
class OUSTER_API_CLASS SlamEngine {
   public:
    /**
     * @brief Constructs the SlamEngine from a unified list of sensor metadata.
     *
     * Accepts a single vector of SensorInfo (as returned by
     * FrameSetSource::sensor_info()), which may contain a mix of LidarInfo and
     * CameraInfo entries. The list is dispatched on concrete type internally
     * and reordered lidar-first (matching the unified FrameSet ordering), so
     * lidar-only, camera-only, and mixed sources are all supported through one
     * call. Entries that are neither LidarInfo nor CameraInfo are ignored.
     *
     * @param[in] sensor_infos Mixed lidar/camera calibration.
     * @param[in] config Configuration parameters. Defaults to LIOSlamConfig{}.
     * @return A unique pointer to the created SlamEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<SlamEngine> create(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const LIOSlamConfig& config = LIOSlamConfig{});

    /// Destructor for SlamEngine.
    OUSTER_API_FUNCTION
    virtual ~SlamEngine() = default;

    /**
     * @brief Let SlamEngine process LidarFrames to estimate the new state
     * (pose).
     *
     * This function processes a FrameSet of LidarFrames (and any attached
     * CameraFrames). The engine writes per-column body_to_world transforms
     * onto each lidar frame. Camera frames with RANGE contribute a
     * global-shutter point cloud to ICP; their body_to_world pose is written
     * from the ICP correction, or interpolated from lidar when lidar is
     * present.
     *
     * @param[in,out] frame_set FrameSet of lidar and optional camera frames to
     * be processed.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::FrameSet& frame_set) = 0;

    /**
     * @brief Retrieves the current point cloud from the SLAM engine.
     *
     * This method returns a point cloud representation as a `PointCloudXYZf`
     * object, which contains the latest set of 3D points generated by the SLAM
     * engine.
     *
     * @return A `PointCloudXYZf` object containing the current point cloud
     * data.
     */
    OUSTER_API_FUNCTION
    virtual ouster::sdk::core::PointCloudXYZf get_point_cloud() const = 0;

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
    /// internally, so lidar-only, camera-only, and mixed sources are handled
    /// uniformly. The list shares the FrameSet index space (sensor i describes
    /// frame i) in whatever order the caller provides; no lidar-first ordering
    /// is assumed.
    explicit SlamEngine(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos)
        : sensor_infos_(sensor_infos) {}

    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_infos_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
