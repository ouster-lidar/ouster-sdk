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
 * @verbatim
 * Fields:
 *   metadata: string - lidar metadata in json
 *
 * OSF type:
 *   ouster/v1/os_sensor/LidarSensor
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_sensor.fbs
 * @endverbatim
 *
 */
class LidarSensor : public MetadataEntryHelper<LidarSensor> {
    using sensor_info = ouster::sensor::sensor_info;

   public:
    explicit LidarSensor(const sensor_info& si)
        : sensor_info_(si), metadata_(si.updated_metadata_string()) {}

    explicit LidarSensor(const std::string& sensor_metadata)
        : sensor_info_(sensor::parse_metadata(sensor_metadata)),
          metadata_(sensor_metadata) {}

    const sensor_info& info() const { return sensor_info_; }

    const std::string& metadata() const { return metadata_; }

    // === Simplified with MetadataEntryHelper<Sensor>: type()+clone()
    // std::string type() const override;
    // std::unique_ptr<MetadataEntry> clone() const override;

    std::vector<uint8_t> buffer() const final;

    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    std::string repr() const override;
    std::string to_string() const override;

   private:
    sensor_info sensor_info_;
    const std::string metadata_;
};

template <>
struct MetadataTraits<LidarSensor> {
    static const std::string type() {
        return "ouster/v1/os_sensor/LidarSensor";
    }
};

}  // namespace osf
}  // namespace ouster
