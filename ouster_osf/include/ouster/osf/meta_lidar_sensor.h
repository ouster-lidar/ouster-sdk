/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file meta_lidar_sensor.h
 * @brief Metadata entry LidarSensor
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/camera_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Metadata entry storing a sensor's metadata (calibration/config) as a JSON
 * string in the OSF footer.
 *
 * @note Despite the historical name, this entry stores metadata for *any*
 * sensor kind - both ouster::sdk::core::LidarInfo and
 * ouster::sdk::core::CameraInfo are serialized through this single type. The
 * concrete subtype is inferred from the JSON schema on read (see
 * ouster::sdk::core::SensorInfo::from_json_string); the name and OSF type string are kept
 * for backward compatibility. Use sensor_info() for the generic (base) accessor
 * and info() only when the entry is known to describe a lidar.
 *
 * OSF type:
 *   ouster/v1/os_sensor/LidarSensor
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/lidar_sensor.fbs
 */
class OUSTER_API_CLASS LidarSensor : public MetadataEntryHelper<LidarSensor> {
   public:
    /**
     * @param[in] sensor_info Initialize the LidarSensor with a LidarInfo
     * object.
     */
    OUSTER_API_FUNCTION
    explicit LidarSensor(const ouster::sdk::core::LidarInfo& sensor_info);

    /**
     * @param[in] camera_info Initialize the entry with a CameraInfo object.
     * Cameras reuse this same footer metadata type as lidars.
     */
    OUSTER_API_FUNCTION
    explicit LidarSensor(const ouster::sdk::core::CameraInfo& camera_info);

    /**
     * @param[in] sensor_metadata Initialize the entry with a json string
     *                            representation of a LidarInfo or CameraInfo
     *                            object. The concrete type is inferred from the
     *                            JSON schema.
     */
    OUSTER_API_FUNCTION
    explicit LidarSensor(const std::string& sensor_metadata);

    /**
     * Returns the sensor metadata associated with this entry as the polymorphic
     * base type. Downcast with std::dynamic_pointer_cast to obtain the concrete
     * LidarInfo or CameraInfo.
     *
     * @return The SensorInfo associated with this entry.
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info() const;

    /**
     * Returns the LidarInfo associated with the entry.
     *
     * @throws std::runtime_error if this entry stores a non-lidar sensor (e.g.
     *         a CameraInfo). Prefer sensor_info() when the kind is not known.
     *
     * @return The LidarInfo associated with the entry.
     */
    OUSTER_API_FUNCTION
    const ouster::sdk::core::LidarInfo& info() const;

    /**
     * Returns the json string representation sensor_info associated
     * with the LidarSensor.
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
     * Create a LidarSensor object from a byte array.
     *
     * @todo Figure out why this wasn't just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new LidarSensor cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const OsfBuffer& buf);

    /**
     * Get the string representation for the LidarSensor object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the LidarSensor object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;

    /**
     * @todo Figure out why we have both repr and to_string
     *
     * @relates MetadataEntry::to_string
     *
     * @copydoc LidarSensor::repr
     */
    OUSTER_API_FUNCTION
    std::string to_string() const override;

   private:
    /**
     * The internal sensor metadata object (LidarInfo or CameraInfo), held via
     * the polymorphic base type.
     */
    std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info_;

    /**
     * The internal json string representation of the sensor metadata object.
     */
    const std::string metadata_;
};

/** @defgroup OSFTraitsLidarSensor Templated struct for traits */

/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsLidarSensor
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<LidarSensor> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/LidarSensor".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/LidarSensor";
    }
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
