/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/meta_lidar_sensor.h"

#include <cstdint>
#include <jsoncons/json.hpp>
#include <jsoncons/json_parser.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "flatbuffers/flatbuffers.h"
// We need this for the v2 namespace.
#include "ouster/osf/buffer.h"
#include "ouster/osf/impl/basics.h"  // NOLINT(misc-include-cleaner)
#include "ouster/osf/metadata.h"
#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace osf {

using sensor_info = ouster::sdk::core::SensorInfo;

namespace {

// === Lidar Sensor stream/msgs functions ====================

// NOLINTBEGIN(misc-include-cleaner)
/**
 * Internal helper function for creating flatbuffer blobs to represent
 * LidarSensor objects.
 *
 * @param[in] fbb The flatbufferbuilder to use when generating the blob.
 * @param[in] sensor_metadata ///< The json string representation of the
 *                            ///< sensor_info to use when creating
 *                            ///< the flatbuffer blob.
 * @return The offset pointer inside the flatbufferbuilder to the new section.
 */
flatbuffers::Offset<ouster::sdk::osf::impl::gen::LidarSensor>
create_lidar_sensor(flatbuffers::FlatBufferBuilder& fbb,
                    const std::string& sensor_metadata) {
    auto ls_offset = ouster::sdk::osf::impl::gen::CreateLidarSensorDirect(
        fbb, sensor_metadata.c_str());
    return ls_offset;
}
// NOLINTEND(misc-include-cleaner)

/**
 * Internal helper function for restoring a LidarSensor's json string
 * representation of a sensor_info from a raw flatbuffer byte vector.
 *
 * @param[in] buf The flatbuffer byte vector.
 * @return ///< The json string representation of the sensor_info object
 *         ///< contained within the flatbuffer blob.
 */

std::unique_ptr<std::string> restore_lidar_sensor(const OsfBuffer buf) {
    auto lidar_sensor = v2::GetSizePrefixedLidarSensor(
        buf.data());  // NOLINT(misc-include-cleaner)

    std::string sensor_metadata{};
    if (lidar_sensor->metadata() != nullptr) {
        sensor_metadata = lidar_sensor->metadata()->str();
    }

    return std::make_unique<std::string>(sensor_metadata);
}

}  // namespace

LidarSensor::LidarSensor(const sensor_info& sensor_info)
    : sensor_info_(sensor_info), metadata_(sensor_info.to_json_string()) {}

LidarSensor::LidarSensor(const std::string& sensor_metadata)
    : sensor_info_(sensor_metadata), metadata_(sensor_metadata) {}

const sensor_info& LidarSensor::info() const { return sensor_info_; }

const std::string& LidarSensor::metadata() const { return metadata_; }

std::vector<uint8_t> LidarSensor::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto ls_offset = create_lidar_sensor(fbb, metadata_);
    fbb.FinishSizePrefixed(
        ls_offset,
        ouster::sdk::osf::impl::gen::
            LidarSensorIdentifier());  // NOLINT(misc-include-cleaner)
    const uint8_t* buf = fbb.GetBufferPointer();
    const uint32_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> LidarSensor::from_buffer(const OsfBuffer buf) {
    auto sensor_metadata = restore_lidar_sensor(buf);
    if (sensor_metadata) {
        return std::make_unique<LidarSensor>(*sensor_metadata);
    }
    return nullptr;
}

std::string LidarSensor::repr() const {
    jsoncons::json root;

    std::istringstream temp(sensor_info_.to_json_string());

    jsoncons::json_decoder<jsoncons::json> temp_decoder;
    jsoncons::json_stream_reader reader(temp, temp_decoder);

    std::error_code temp_error_code;
    reader.read(temp_error_code);
    if (!temp_error_code) {
        root["sensor_info"] = temp_decoder.get_result();
    } else {
        root["sensor_info"] = metadata_;
    }

    std::string out;
    root.dump(out);
    return out;
};

std::string LidarSensor::to_string() const { return repr(); };

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
