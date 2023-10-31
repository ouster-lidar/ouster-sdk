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
    /// TODO]pb]: This is soft DEPRECATED until we have an updated sensor_info,
    ///           since we are not encouraging storing the serialized metadata
    explicit LidarSensor(const sensor_info& si)
        : sensor_info_(si), metadata_(si.original_string()) {
        throw std::invalid_argument(
            "\nERROR: `osf::LidarSensor()` constructor accepts only "
            "metadata_json "
            "(full string of the file metadata.json or what was received from "
            "sensor) and not a `sensor::sensor_info` object.\n\n"
            "We are so sorry that we deprecated it's so hardly but the thing "
            "is that `sensor::sensor_info` object doesn't equal the original "
            "metadata.json file (or string) that we used to construct it.\n"
            "However, Data App when tries to get metadata from OSF looks for "
            "fields (like `image_rev`) that only present in metadata.json but "
            "not `sensor::sensor_info` which effectively leads to OSF that "
            "couldn't be uploaded to Data App.\n");
    }

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
