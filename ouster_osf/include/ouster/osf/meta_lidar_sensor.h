/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file meta_lidar_sensor.h
 * @brief Metadata entry LidarSensor
 *
 */
#pragma once

#include <iostream>
#include <memory>

#include "ouster/osf/metadata.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

/**
 * Metadata entry to store lidar sensor_info, i.e. Ouster sensor configuration.
 *
 * OSF type:
 *   ouster/v1/os_sensor/LidarSensor
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/lidar_sensor.fbs
 */
class LidarSensor : public MetadataEntryHelper<LidarSensor> {
    using sensor_info = ouster::sensor::sensor_info;

   public:
    /**
     * @param[in] si Initialize the LidarSensor with a sensor_info object.
     */
    explicit LidarSensor(const sensor_info& si);

    /**
     * @param[in] sensor_metadata Initialize the LidarSensor with a json string
     *                            representation of the sensor_info object.
     */
    explicit LidarSensor(const std::string& sensor_metadata);

    /**
     * Returns the sensor_info associated with the LidarSensor.
     *
     * @return The sensor_info associated with the LidarSensor.
     */
    const sensor_info& info() const;

    /**
     * Returns the json string representation sensor_info associated
     * with the LidarSensor.
     *
     * @return  ///< The json string representation of the
     *          ///< sensor_info object.
     */
    const std::string& metadata() const;

    /**
     * @copydoc MetadataEntry::buffer
     */
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a LidarSensor object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new LidarSensor cast as a MetadataEntry
     */
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    /**
     * Get the string representation for the LidarSensor object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the LidarSensor object.
     */
    std::string repr() const override;

    /**
     * @todo Figure out why we have both repr and to_string
     *
     * @relates MetadataEntry::to_string
     *
     * @copydoc LidarSensor::repr
     */
    std::string to_string() const override;

   private:
    /**
     * The internal sensor_info object.
     */
    sensor_info sensor_info_;

    /**
     * The internal json string representation of the sensor_info object.
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
struct MetadataTraits<LidarSensor> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/LidarSensor".
     */
    static const std::string type() {
        return "ouster/v1/os_sensor/LidarSensor";
    }
};

}  // namespace osf
}  // namespace ouster
