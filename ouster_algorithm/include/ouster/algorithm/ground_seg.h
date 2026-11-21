#pragma once

#include <memory>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
class FrameSet;
}  // namespace core
}  // namespace sdk
}  // namespace ouster

namespace ouster {
namespace sdk {
namespace algorithm {

/// @brief Configuration parameters for the ground segmentation engine.
struct OUSTER_API_CLASS GroundSegConfig {
    double grid_size = 0.5;  ///< Grid cell size in meters for the 2.5-D height
                             ///< map used by ground segmentation.
};

/**
 * @class GroundSegEngine
 * @brief Ground segmentation engine for classifying ground points in sensor
 *        frames.
 *
 * This class provides an interface for running ground segmentation on
 * FrameSet data, annotating each frame with a per-pixel ground mask. It
 * operates on any Frame that carries a RANGE field and sensor metadata,
 * covering both LidarFrame and CameraFrame. Lidar frames may provide multiple
 * returns (GROUND, GROUND2, ...) and a staggered pixel layout; camera frames
 * have a single return and an unstaggered image. Frames without a RANGE field
 * or sensor info are skipped.
 *
 * Coordinate frame selection:
 * - If any valid pose in a frame is non-identity, the engine treats those
 * poses as SLAM poses and segments in that global frame. A lidar frame
 * provides one pose per column (rolling shutter); a camera frame provides a
 * single rigid pose applied to every column.
 * - Otherwise, the engine uses sensor extrinsics and frame-local column poses.
 *
 * The engine does not compute IMU gravity alignment internally. If gravity
 * alignment is required, plumb the frame set source first and write the
 * resulting transform into sensor extrinsics before calling update().
 */
class OUSTER_API_CLASS GroundSegEngine {
   public:
    /**
     * @brief Create a GroundSegEngine with the given configuration.
     *
     * @param[in] config Configuration parameters for the engine.
     * @return A unique pointer to the created GroundSegEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<GroundSegEngine> create(const GroundSegConfig& config = {});

    OUSTER_API_FUNCTION
    virtual ~GroundSegEngine() = default;

    /**
     * @brief Run ground segmentation on a FrameSet, annotating each supported
     *        frame (lidar or camera) with a "GROUND" field (uint8: 1 = ground,
     *        0 = non-ground).
     *
     * Example:
     * @code{.cpp}
     * // If IMU gravity alignment is desired, apply it before segmentation:
     * // sensor_info.sensor_to_body = T_plumb * sensor_info.sensor_to_body;
     *
     * auto ground = ouster::sdk::algorithm::GroundSegEngine::create();
     * ground->update(frames);
     * @endcode
     *
     * If a frame contains precomputed "NORMALS" or "NORMALS2" fields, their
     * vectors must be expressed in the global coordinate frame used for
     * segmentation.
     *
     * @param[in,out] frames FrameSet to annotate in place.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::FrameSet& frames) = 0;

   protected:
    GroundSegEngine() = default;
};

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
