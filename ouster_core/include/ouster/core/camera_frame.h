/**
 * @file
 * @brief Camera frame data structure
 */

#pragma once

#include <memory>
#include <string>

#include "ouster/core/camera_info.h"
#include "ouster/core/frame.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Data structure for camera image frames and associated metadata.
 *
 * Stores image fields (for example RANGE) and timing/pose headers. Inherits
 * common field, timestamp, and pose accessors from Frame.
 */
class OUSTER_API_CLASS CameraFrame : public TypedFrame<CameraInfo> {
   public:
    // The CameraInfo associated with this frame is reached through the
    // inherited TypedFrame<CameraInfo>::sensor_info() / set_sensor_info().

    /**
     * The default constructor creates an invalid 0 x 0 frame with a single
     * timestamp and identity body-to-world pose.
     */
    OUSTER_API_FUNCTION
    CameraFrame();

    /**
     * Initialize camera frame with camera info.
     *
     * @param[in] info a shared_ptr to the CameraInfo object
     */
    OUSTER_API_FUNCTION
    explicit CameraFrame(std::shared_ptr<CameraInfo> info);

    /**
     * Camera frame destructor.
     */
    OUSTER_API_FUNCTION
    ~CameraFrame() override = default;

    /**
     * Copy constructor.
     */
    OUSTER_API_FUNCTION
    CameraFrame(const CameraFrame&) = default;

    /**
     * Copy assignment operator.
     */
    OUSTER_API_FUNCTION
    CameraFrame& operator=(const CameraFrame&) = default;

    /**
     * Move constructor.
     */
    OUSTER_API_FUNCTION
    CameraFrame(CameraFrame&&) = default;

    /**
     * Move assignment operator.
     */
    OUSTER_API_FUNCTION
    CameraFrame& operator=(CameraFrame&&) = default;

    /**
     * Deep-copy this camera frame as a base Frame pointer.
     *
     * @return a shared pointer to a newly allocated CameraFrame copy.
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<Frame> clone() const override;

    /**
     * Get a string representation of this camera frame.
     *
     * @return the string representation of the camera frame.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const override;

    // Frame::equals() is deliberately not overridden. A CameraFrame adds no
    // members of its own, so the base comparison of the dimensions, headers,
    // fields, and objects — which already refuses to equate frames of
    // different concrete types — is complete for camera frames.
};

/**
 * Equality for CameraFrame.
 *
 * Forwards to the inherited Frame::equals(), which compares the frame
 * dimensions, timestamp / pose / status headers, fields, and objects.
 *
 * @param[in] a The first camera frame to compare.
 * @param[in] b The second camera frame to compare.
 *
 * @return if a == b.
 */
OUSTER_API_FUNCTION
bool operator==(const CameraFrame& a, const CameraFrame& b);

/**
 * Inequality for CameraFrame.
 *
 * @param[in] a The first camera frame to compare.
 * @param[in] b The second camera frame to compare.
 *
 * @return if a != b.
 */
OUSTER_API_FUNCTION
bool operator!=(const CameraFrame& a, const CameraFrame& b);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
