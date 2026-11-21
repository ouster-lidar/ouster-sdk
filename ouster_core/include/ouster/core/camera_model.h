#pragma once

#include <cstddef>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Pinhole intrinsics for a camera.
 *
 * The intrinsics (fx, fy, cx, cy) describe the standard pinhole/optical image
 * convention (pixel origin top-left, +u right, +v down, +Z along the optical
 * axis). The shared XYZLut derives a camera's per-pixel direction/offset from
 * them. Points produced by that LUT are returned in the **Ouster sensor frame**
 * (X forward, Y left, Z up) so camera clouds are interoperable with lidar
 * clouds and the rest of the SDK.
 *
 * There are deliberately no distortion coefficients: nothing in the SDK
 * undistorts imagery, so a CameraModel asserts that its pixels are already
 * rectified. Unrectified imagery must either be rectified before it reaches
 * the SDK, or carried on a CameraInfo with no model at all, which makes it
 * viewable but not projectable.
 */
struct OUSTER_API_CLASS CameraModel {
    /**
     * Construct a pinhole model from its intrinsics.
     *
     * There is deliberately no default constructor: a model with a zero focal
     * length has no usable projection (it divides by zero in the pixel-to-ray
     * math), so every model is required to carry one from the start.
     *
     * @throws std::invalid_argument if focal_x or focal_y is not greater than
     * zero.
     *
     * @param[in] focal_x focal length in pixels along x; must be > 0.
     * @param[in] focal_y focal length in pixels along y; must be > 0.
     * @param[in] center_x optical center in pixels along x.
     * @param[in] center_y optical center in pixels along y.
     */
    OUSTER_API_FUNCTION
    CameraModel(float focal_x, float focal_y, float center_x, float center_y);

    float fx;  ///< focal length in pixels; greater than zero as constructed
    float fy;  ///< focal length in pixels; greater than zero as constructed
    float cx;  ///< optical center in pixels
    float cy;  ///< optical center in pixels
};

/**
 * Equality for CameraModel (per-field exact comparison).
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const CameraModel& lhs, const CameraModel& rhs);

/**
 * Inequality for CameraModel.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const CameraModel& lhs, const CameraModel& rhs);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
