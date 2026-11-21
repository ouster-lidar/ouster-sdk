/**
 * Copyright(c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/meta_camera_sensor.h"

#include <cstdint>
#include <jsoncons/json.hpp>
#include <jsoncons/json_parser.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/sensor_info.h"
// We need this for the v2 namespace.
#include "ouster/core/types.h"  // NOLINT(misc-include-cleaner)
#include "ouster/osf/buffer.h"
#include "ouster/osf/impl/basics.h"  // NOLINT(misc-include-cleaner)
#include "ouster/osf/metadata.h"

namespace ouster {
namespace sdk {
namespace osf {

using ouster::sdk::core::CameraInfo;
using ouster::sdk::core::SensorInfo;

namespace {

// === Camera Sensor stream/msgs functions ===================

// NOLINTBEGIN(misc-include-cleaner)
/**
 * Internal helper function for creating flatbuffer blobs to represent
 * CameraSensor objects.
 *
 * @param[in] fbb The flatbufferbuilder to use when generating the blob.
 * @param[in] sensor_metadata The json string representation of the CameraInfo
 *                            to use when creating the flatbuffer blob.
 * @return The offset pointer inside the flatbufferbuilder to the new section.
 */
flatbuffers::Offset<ouster::sdk::osf::impl::gen::CameraSensor> create_camera_sensor(
    flatbuffers::FlatBufferBuilder& fbb, const std::string& sensor_metadata) {
    auto cs_offset =
        ouster::sdk::osf::impl::gen::CreateCameraSensorDirect(fbb, sensor_metadata.c_str());
    return cs_offset;
}
// NOLINTEND(misc-include-cleaner)

/**
 * Internal helper function for restoring a CameraSensor's json string
 * representation of a sensor_info from a raw flatbuffer byte vector.
 *
 * @param[in] buf The flatbuffer byte vector.
 * @return The json string representation of the sensor_info object contained
 *         within the flatbuffer blob.
 */
std::unique_ptr<std::string> restore_camera_sensor(const OsfBuffer& buf) {
    // NOLINTNEXTLINE(misc-include-cleaner)
    auto camera_sensor = v2::GetSizePrefixedCameraSensor(buf.data());

    std::string sensor_metadata{};
    if (camera_sensor->metadata() != nullptr) {
        sensor_metadata = camera_sensor->metadata()->str();
    }

    return std::make_unique<std::string>(sensor_metadata);
}

}  // namespace

CameraSensor::CameraSensor(const CameraInfo& camera_info)
    : sensor_info_(std::make_shared<CameraInfo>(camera_info)),
      metadata_(camera_info.to_json_string()) {}

CameraSensor::CameraSensor(const std::string& sensor_metadata)
    : sensor_info_(std::make_shared<CameraInfo>(sensor_metadata)), metadata_(sensor_metadata) {}

std::shared_ptr<SensorInfo> CameraSensor::sensor_info() const {
    return sensor_info_;
}

const CameraInfo& CameraSensor::info() const {
    auto* camera = dynamic_cast<const CameraInfo*>(sensor_info_.get());
    if (camera == nullptr) {
        throw std::runtime_error(
            "CameraSensor::info(): this metadata entry does not store a CameraInfo.");
    }
    return *camera;
}

const std::string& CameraSensor::metadata() const {
    return metadata_;
}

std::vector<uint8_t> CameraSensor::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto cs_offset = create_camera_sensor(fbb, metadata_);
    fbb.FinishSizePrefixed(
        cs_offset,
        ouster::sdk::osf::impl::gen::CameraSensorIdentifier());  // NOLINT(misc-include-cleaner)
    const uint8_t* buf = fbb.GetBufferPointer();
    const uint32_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> CameraSensor::from_buffer(const OsfBuffer& buf) {
    auto sensor_metadata = restore_camera_sensor(buf);
    if (sensor_metadata) {
        return std::make_unique<CameraSensor>(*sensor_metadata);
    }
    return nullptr;
}

std::string CameraSensor::repr() const {
    jsoncons::json root;

    // Normalize by re-serializing through the concrete sensor metadata type.
    std::string canonical;
    if (sensor_info_) {
        canonical = sensor_info_->to_json_string();
    } else {
        canonical = metadata_;
    }

    std::istringstream temp(canonical);

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

std::string CameraSensor::to_string() const {
    return repr();
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
