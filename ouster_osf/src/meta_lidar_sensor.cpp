/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/meta_lidar_sensor.h"

#include "fb_utils.h"
#include "flatbuffers/flatbuffers.h"
#include "json_utils.h"
#include "ouster/osf/basics.h"

namespace ouster {
namespace osf {

// === Lidar Sensor stream/msgs functions ====================

flatbuffers::Offset<ouster::osf::gen::LidarSensor> create_lidar_sensor(
    flatbuffers::FlatBufferBuilder& fbb, const std::string& sensor_metadata) {
    auto ls_offset =
        ouster::osf::gen::CreateLidarSensorDirect(fbb, sensor_metadata.c_str());
    return ls_offset;
}

std::unique_ptr<std::string> restore_lidar_sensor(
    const std::vector<uint8_t> buf) {
    auto lidar_sensor = v2::GetSizePrefixedLidarSensor(buf.data());

    std::string sensor_metadata{};
    if (lidar_sensor->metadata())
        sensor_metadata = lidar_sensor->metadata()->str();

    return std::make_unique<std::string>(sensor_metadata);
}

std::vector<uint8_t> LidarSensor::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto ls_offset = create_lidar_sensor(fbb, metadata_);
    fbb.FinishSizePrefixed(ls_offset,
                           ouster::osf::gen::LidarSensorIdentifier());
    const uint8_t* buf = fbb.GetBufferPointer();
    const uint32_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> LidarSensor::from_buffer(
    const std::vector<uint8_t>& buf) {
    auto sensor_metadata = restore_lidar_sensor(buf);
    if (sensor_metadata) {
        return std::make_unique<LidarSensor>(*sensor_metadata);
    }
    return nullptr;
}

std::string LidarSensor::repr() const {
    Json::Value lidar_sensor_obj{};
    Json::Value sensor_info_obj{};

    if (!parse_json(metadata_, sensor_info_obj)) {
        lidar_sensor_obj["sensor_info"] = metadata_;
    } else {
        lidar_sensor_obj["sensor_info"] = sensor_info_obj;
    }

    return json_string(lidar_sensor_obj);
};

}  // namespace osf
}  // namespace ouster
