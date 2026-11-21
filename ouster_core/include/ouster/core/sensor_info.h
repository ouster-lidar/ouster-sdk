/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Common base for sensor metadata (LidarInfo, CameraInfo, ...)
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "ouster/core/impl/cache_ptr.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

template <typename T>
class XYZLutT;

namespace impl {
struct SensorInfoCache;
}  // namespace impl

/**
 * Common base for sensor metadata types such as LidarInfo and CameraInfo.
 *
 * Holds calibration shared by all sensor kinds. For now this is limited to the
 * sensor-to-body extrinsic transform; other shared fields may be hoisted here
 * as the API grows.
 *
 * @note Polymorphic base: instances may be held and iterated as
 * std::shared_ptr<SensorInfo> (e.g. from FrameSetSource::sensor_info()) and
 * downcast to their concrete type (LidarInfo, CameraInfo, ...) via
 * std::dynamic_pointer_cast.
 */
struct OUSTER_API_CLASS SensorInfo {
   private:
    mutable impl::CachePtr<ouster::sdk::core::impl::SensorInfoCache> cache_;

   public:
    /// Default constructor.
    ///
    /// Defaulted out-of-line (see lidar_info.cpp) because the PIMPL cache_
    /// member requires a complete SensorInfoCache type.
    OUSTER_API_FUNCTION
    SensorInfo();

    /// Virtual destructor so SensorInfo can be used polymorphically.
    ///
    /// Defaulted out-of-line (see lidar_info.cpp) because the PIMPL cache_
    /// member requires a complete SensorInfoCache type.
    OUSTER_API_FUNCTION
    virtual ~SensorInfo();

    /// Copy constructor.
    OUSTER_API_FUNCTION
    SensorInfo(const SensorInfo&);

    /// Copy assignment operator.
    OUSTER_API_FUNCTION
    SensorInfo& operator=(const SensorInfo&);

    /// Move constructor.
    ///
    /// Defaulted out-of-line (see lidar_info.cpp) because the PIMPL cache_
    /// member requires a complete SensorInfoCache type.
    ///
    /// Not marked noexcept: CachePtr's move allocates a fresh cache for the
    /// moved-from object, so the implicit specification is not noexcept, and
    /// GCC rejects a defaulted move whose explicit exception-specification
    /// differs from the implicit one. Let the compiler deduce it.
    // NOLINTBEGIN(cppcoreguidelines-noexcept-move-operations)
    OUSTER_API_FUNCTION
    SensorInfo(SensorInfo&&);
    // NOLINTEND(cppcoreguidelines-noexcept-move-operations)

    /// Move assignment operator.
    ///
    /// Not marked noexcept for the same reason as the move constructor above.
    // NOLINTBEGIN(cppcoreguidelines-noexcept-move-operations)
    OUSTER_API_FUNCTION
    SensorInfo& operator=(SensorInfo&&);
    // NOLINTEND(cppcoreguidelines-noexcept-move-operations)

    /// Sensor serial number.
    uint64_t sn{};

    /// Homogeneous transform from the sensor coordinate frame to the body frame.
    /// Defaults to the identity, so the body frame coincides with the sensor
    /// frame until extrinsics are supplied.
    mat4d sensor_to_body = mat4d::Identity();

    /// Homogeneous transform from the sensor coordinate frame to the IMU frame.
    /// Lidar metadata provides this directly; other sensor kinds (e.g. cameras)
    /// may leave it as the zero/identity default when no separate IMU extrinsic
    /// exists.
    mat4d imu_to_sensor_transform = mat4d::Zero();

    // NOTE: the lidar-specific beam intrinsics (beam_azimuth_angles /
    // beam_altitude_angles) and the beam_to_lidar_transform /
    // lidar_to_sensor_transform live on LidarInfo, not this shared base, so a
    // CameraInfo never carries them. Camera projection is derived from the
    // pinhole intrinsics instead (see xyzlut.h / camera_info.h).

    /**
     * Width (number of columns) of a frame from this sensor.
     *
     * Overridden by concrete sensor types; the base returns 0.
     *
     * @return frame width in columns, or 0 in the base implementation.
     */
    OUSTER_API_FUNCTION
    virtual size_t w() const {
        return 0;
    }

    /**
     * Height (number of rows) of a frame from this sensor.
     *
     * Overridden by concrete sensor types; the base returns 0.
     *
     * @return frame height in rows, or 0 in the base implementation.
     */
    OUSTER_API_FUNCTION
    virtual size_t h() const {
        return 0;
    }

    /**
     * Rate at which this sensor produces frames.
     *
     * @return frame rate in frames per second.
     */
    OUSTER_API_FUNCTION
    virtual int fps() const = 0;

    /**
     * Whether this sensor is capable of synchronizing with an external time
     * source, and so may share a timebase with other sensors.
     *
     * @return true if the sensor supports an external timing source.
     */
    OUSTER_API_FUNCTION
    virtual bool could_be_synchronized() const = 0;

    /**
     * Delay between the start of a capture and the timestamp reported for the
     * resulting frame.
     *
     * @return capture latency in seconds.
     */
    OUSTER_API_FUNCTION
    virtual double latency() const = 0;

    /**
     * Polymorphic copy. Returns a new SensorInfo of the concrete type (e.g.
     * LidarInfo or CameraInfo), letting callers duplicate a sensor without
     * knowing its kind.
     *
     * @return a shared_ptr to a copy of this SensorInfo.
     */
    OUSTER_API_FUNCTION
    virtual std::shared_ptr<SensorInfo> clone() const = 0;

    /**
     * Return a JSON serialized representation of this sensor metadata.
     *
     * Implemented by concrete sensor types (LidarInfo, CameraInfo, ...).
     *
     * @return JSON serialized metadata string for this sensor.
     */
    OUSTER_API_FUNCTION
    virtual std::string to_json_string() const = 0;

    /**
     * Construct the appropriate concrete SensorInfo subtype from a metadata
     * JSON string, inferring the sensor kind from the JSON schema.
     *
     * A camera is recognized by the presence of a top-level `camera_info`
     * object; anything else is parsed as lidar metadata (LidarInfo). Both
     * subtypes share the SensorInfo base, so callers can hold the result as a
     * std::shared_ptr and downcast via std::dynamic_pointer_cast when a
     * concrete type is required.
     *
     * @param[in] json_string JSON-formatted sensor metadata string.
     *
     * @return a shared_ptr to a LidarInfo or CameraInfo populated from @p json_string.
     */
    OUSTER_API_FUNCTION
    static std::shared_ptr<SensorInfo> from_json_string(const std::string& json_string);

    /**
     * Clears cached data such as xyzluts calculated from other members.
     *
     * Call after changing members the cache is derived from, such as
     * sensor_to_body. Pointers returned by xyzlut() earlier keep the LUT they
     * share alive, so they stay valid but keep projecting with the old values.
     */
    OUSTER_API_FUNCTION
    void clear_cache();

    /**
     * Gets a XYZLut for this sensor, caching between calls.
     *
     * The LUT is built on first use and shared with every pointer returned
     * afterwards, until clear_cache() is called. Safe to call concurrently with
     * other calls to xyzlut() and clear_cache().
     *
     * @tparam T floating point type to use for the XYZLut: float or double
     * @return non-null pointer sharing ownership of the cached xyzlut; it stays
     * valid even if the cache is cleared or this SensorInfo is destroyed
     *
     * @see clear_cache()
     */
    template <typename T>
    std::shared_ptr<const XYZLutT<T>> xyzlut() const {
        // this check is redundant, but need one that depends on T and always
        // fails or else we get a compiler error even if this function isn't used
        static_assert(std::is_same<T, float>() || std::is_same<T, double>(),
                      "Unsupported type. Must use float or double.");
    }
};

/// \cond
template <>
OUSTER_API_FUNCTION std::shared_ptr<const XYZLutT<float>> SensorInfo::xyzlut() const;
template <>
OUSTER_API_FUNCTION std::shared_ptr<const XYZLutT<double>> SensorInfo::xyzlut() const;
/// \endcond

}  // namespace core
}  // namespace sdk
}  // namespace ouster
