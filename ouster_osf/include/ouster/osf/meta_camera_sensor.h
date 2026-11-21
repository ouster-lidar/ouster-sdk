/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file meta_camera_sensor.h
 * @brief Metadata entry CameraSensor
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/camera_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Metadata entry storing a camera sensor's metadata
 * (ouster::sdk::core::CameraInfo) as a JSON string in the OSF footer.
 *
 * This mirrors LidarSensor but uses a distinct OSF type string
 * ("ouster/v1/os_sensor/CameraSensor"). Keeping camera metadata under its own
 * type is deliberate: OSF readers that predate camera support don't recognize
 * this type and therefore skip it as an opaque (unknown) metadata entry rather
 * than attempting to validate a CameraInfo as a LidarInfo - which would fail
 * (missing beam intrinsics / columns_per_frame) and abort opening the whole
 * file. As a result, older readers can still consume the lidar streams while
 * silently ignoring the camera streams.
 *
 * OSF type:
 *   ouster/v1/os_sensor/CameraSensor
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/camera_sensor.fbs
 */
class OUSTER_API_CLASS CameraSensor : public MetadataEntryHelper<CameraSensor> {
   public:
    /**
     * @param[in] camera_info Initialize the entry with a CameraInfo object.
     */
    OUSTER_API_FUNCTION
    explicit CameraSensor(const ouster::sdk::core::CameraInfo& camera_info);

    /**
     * @param[in] sensor_metadata Initialize the entry with a json string
     *                            representation of a CameraInfo object.
     */
    OUSTER_API_FUNCTION
    explicit CameraSensor(const std::string& sensor_metadata);

    /**
     * Returns the sensor metadata associated with this entry as the polymorphic
     * base type. Downcast with std::dynamic_pointer_cast to obtain the concrete
     * CameraInfo.
     *
     * @return The SensorInfo associated with this entry.
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info() const;

    /**
     * Returns the CameraInfo associated with the entry.
     *
     * @throws std::runtime_error if this entry does not store a CameraInfo.
     *
     * @return The CameraInfo associated with the entry.
     */
    OUSTER_API_FUNCTION
    const ouster::sdk::core::CameraInfo& info() const;

    /**
     * Returns the json string representation of the sensor_info associated
     * with the CameraSensor.
     *
     * @return The json string representation of the sensor_info object.
     */
    OUSTER_API_FUNCTION
    const std::string& metadata() const;

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a CameraSensor object from a byte array.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new CameraSensor cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const OsfBuffer& buf);

    /**
     * Get the string representation for the CameraSensor object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the CameraSensor object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;

    /**
     * @copydoc CameraSensor::repr
     */
    OUSTER_API_FUNCTION
    std::string to_string() const override;

   private:
    /**
     * The internal sensor metadata object (a CameraInfo), held via the
     * polymorphic base type.
     */
    std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info_;

    /**
     * The internal json string representation of the sensor metadata object.
     */
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    const std::string metadata_;
};

/** @defgroup OSFTraitsCameraSensor Templated struct for traits */

/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsCameraSensor
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<CameraSensor> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/CameraSensor".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/CameraSensor";
    }
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
