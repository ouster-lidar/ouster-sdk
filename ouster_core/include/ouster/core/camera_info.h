#pragma once

#include <cstddef>
#include <cstdint>
#include <nonstd/optional.hpp>
#include <string>

#include "ouster/core/camera_model.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/** Stores data format information. */
struct OUSTER_API_CLASS CameraDataFormat {
    uint32_t pixels_per_column{};  ///< pixels per column
    uint32_t columns_per_frame{};  ///< columns per frame, should match with lidar mode
    uint16_t fps{};                ///< frames per second
};

/**
 * Metadata describing a camera sensor (intrinsics, extrinsics, image size).
 *
 * Inherits shared calibration fields from SensorInfo (sn, sensor_to_body,
 * imu_to_sensor_transform). 3D projection is derived from the pinhole
 * intrinsics (model) by the shared XYZLut; cameras do not carry the
 * lidar-specific beam intrinsics or beam/lidar transforms.
 *
 * The model is optional, and the distinction matters: with one, the camera's
 * RANGE can be projected into a point cloud; without one, the imagery can only
 * be viewed, and an XYZLut built from it is empty (0x0) rather than inventing
 * intrinsics. The same applies to a camera with a zero width or height.
 */
struct OUSTER_API_CLASS CameraInfo : public SensorInfo {
   public:
    /**
     * The default constructor creates an invalid camera info.
     */
    OUSTER_API_FUNCTION
    CameraInfo();

    /**
     * Construct a CameraInfo by parsing a JSON metadata string.
     *
     * @param[in] json_string JSON-formatted string representing camera metadata.
     *
     * @throws std::runtime_error if the metadata string is empty or malformed.
     */
    OUSTER_API_FUNCTION
    explicit CameraInfo(const std::string& json_string);

    /**
     * Copy constructor.
     *
     * @param[in] other CameraInfo to copy from.
     */
    OUSTER_API_FUNCTION
    CameraInfo(const CameraInfo& other);

    /**
     * Copy assignment operator.
     */
    OUSTER_API_FUNCTION
    CameraInfo& operator=(const CameraInfo& other);

    /**
     * Move constructor.
     *
     * Not marked noexcept for the same reason as SensorInfo's: it is defaulted
     * out-of-line and the base's move is not noexcept, so the compiler must
     * deduce the exception specification.
     *
     * @param[in] other CameraInfo to move from.
     */
    // NOLINTBEGIN(cppcoreguidelines-noexcept-move-operations)
    OUSTER_API_FUNCTION
    CameraInfo(CameraInfo&& other);
    // NOLINTEND(cppcoreguidelines-noexcept-move-operations)

    /**
     * Move assignment operator.
     *
     * Not marked noexcept for the same reason as the move constructor above.
     */
    // NOLINTBEGIN(cppcoreguidelines-noexcept-move-operations)
    OUSTER_API_FUNCTION
    CameraInfo& operator=(CameraInfo&& other);
    // NOLINTEND(cppcoreguidelines-noexcept-move-operations)

    /**
     * Destructor.
     */
    OUSTER_API_FUNCTION
    ~CameraInfo() override;

    CameraDataFormat format{};  ///< data format for this camera

    /**
     * Pinhole intrinsics for this camera, when it is calibrated.
     *
     * Absent means the imagery can be visualized but not projected: nothing
     * guesses a focal length on the caller's behalf. Present asserts that the
     * imagery is rectified, since the projection path applies no undistortion.
     */
    nonstd::optional<CameraModel> model;

    /**
     * Return a JSON serialized version of this CameraInfo.
     *
     * @return JSON serialized version of this CameraInfo.
     */
    OUSTER_API_FUNCTION
    std::string to_json_string() const override;

    /**
     * Frame width (number of columns), i.e. image width in pixels.
     *
     * @return image width in pixels.
     */
    OUSTER_API_FUNCTION
    size_t w() const override;

    /**
     * Frame height (number of rows), i.e. image height in pixels.
     *
     * @return image height in pixels.
     */
    OUSTER_API_FUNCTION
    size_t h() const override;

    /**
     * Frame rate of this camera.
     *
     * @return the value of frame_rate, in frames per second.
     */
    OUSTER_API_FUNCTION
    int fps() const override;

    /**
     * Whether this camera can be synchronized with other sensors.
     *
     * @return false; cameras carry no synchronization source.
     */
    OUSTER_API_FUNCTION
    bool could_be_synchronized() const override;

    /**
     * Delay between the start of a capture and the frame's timestamp.
     *
     * @return 0.0; camera capture latency is not currently modeled.
     */
    OUSTER_API_FUNCTION
    double latency() const override;

    /**
     * Polymorphic copy.
     *
     * @return a shared_ptr to a copy of this CameraInfo.
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<SensorInfo> clone() const override;
};

/**
 * Parse camera metadata from a JSON file path.
 *
 * @throws std::runtime_error if the file cannot be read or is malformed.
 *
 * @param[in] json_file path to a JSON file containing camera metadata.
 *
 * @return a CameraInfo populated from the given metadata file.
 */
OUSTER_API_FUNCTION
CameraInfo camera_metadata_from_json(const std::string& json_file);

/**
 * Equality for CameraInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const CameraInfo& lhs, const CameraInfo& rhs);

/**
 * Inequality for CameraInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const CameraInfo& lhs, const CameraInfo& rhs);

/**
 * Equality for CameraDataFormat.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const CameraDataFormat& lhs, const CameraDataFormat& rhs);

/**
 * Inequality for CameraDataFormat.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const CameraDataFormat& lhs, const CameraDataFormat& rhs);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
