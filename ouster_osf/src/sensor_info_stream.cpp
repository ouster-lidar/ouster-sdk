#include "ouster/osf/impl/sensor_info_stream.h"

#include <zstd.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "os_sensor/sensor_info_stream_generated.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/lidar_info.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"

using namespace std::literals::chrono_literals;

namespace ouster {
namespace sdk {
namespace osf {

class MessageRef;

std::vector<uint8_t> SensorInfoStream::make_msg(const ouster::sdk::osf::SensorInfoMessage& msg) {
    if (!msg.sensor_info) {
        throw std::invalid_argument("SensorInfoMessage::sensor_info must not be null");
    }
    auto string = msg.sensor_info->to_json_string();

    // Tag the payload so recovery knows which SensorInfo subtype to rebuild;
    // the json alone is what distinguishes them otherwise.
    const auto sensor_type =
        (std::dynamic_pointer_cast<const ouster::sdk::core::CameraInfo>(msg.sensor_info) != nullptr)
            ? ouster::sdk::osf::v2::SensorInfoType::Camera
            : ouster::sdk::osf::v2::SensorInfoType::Lidar;

    // compress the string
    auto max_size = ZSTD_compressBound(string.length());
    std::vector<uint8_t> compressed;
    compressed.resize(max_size);
    auto compressed_size =
        ZSTD_compress(compressed.data(), max_size, string.data(), string.length(), 1);
    compressed.resize(compressed_size);

    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto lss_offset = ouster::sdk::osf::v2::CreateSensorInfoDirect(
        fbb, msg.sensor_meta_id, msg.frame_stream_id, &compressed, sensor_type);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<SensorInfoStream::obj_type> SensorInfoStream::from_buffer(
    const std::vector<uint8_t>& buf) {
    auto si_msg = flatbuffers::GetSizePrefixedRoot<ouster::sdk::osf::v2::SensorInfo>(buf.data());
    auto out = std::make_unique<ouster::sdk::osf::SensorInfoMessage>();
    out->frame_stream_id = si_msg->scan_stream_id();
    out->sensor_meta_id = si_msg->lidar_sensor_id();

    // decompress
    auto sensor_info = si_msg->sensor_info();
    auto size = ZSTD_getFrameContentSize(sensor_info->data(), sensor_info->size());
    std::string string_info;
    string_info.resize(size);
    auto dsize =
        ZSTD_decompress(&string_info.front(), size, sensor_info->data(), sensor_info->size());
    if (dsize != size) {
        throw std::runtime_error("Unexpected decompressed size.");
    }
    if (si_msg->sensor_type() == ouster::sdk::osf::v2::SensorInfoType::Camera) {
        out->sensor_info = std::make_shared<ouster::sdk::core::CameraInfo>(string_info);
    } else {
        out->sensor_info = std::make_shared<ouster::sdk::core::LidarInfo>(string_info);
    }
    return out;
}

std::unique_ptr<SensorInfoStream::obj_type> SensorInfoStream::decode_msg(
    const MessageRef& msg, const meta_type& /*meta*/, const MetadataStore& /*meta_provider*/) {
    const auto& buf = msg.buffer();
    return from_buffer(buf);
}

void SensorInfoStream::save(const SensorInfoMessage& sensor_info_message, ts_t timestamp) {
    if (writer_ == nullptr) {
        throw std::logic_error("writer_ must not be null");
    }

    const auto& msg_buf = make_msg(sensor_info_message);
    ts_t sensor_ts{0ns};
    writer_->save_message(meta_.id(), timestamp, sensor_ts, msg_buf,
                          MetadataTraits<SensorInfoStreamMeta>::type());
}

std::vector<uint8_t> SensorInfoStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    auto lss_offset = ouster::sdk::osf::v2::CreateSensorInfoStream(fbb);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<MetadataEntry> SensorInfoStreamMeta::from_buffer(
    const ouster::sdk::osf::OsfBuffer& /*buf*/) {
    // the metadata is empty, don't bother trying to decode
    return std::make_unique<SensorInfoStreamMeta>();
}

std::string SensorInfoStreamMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "SensorInfoStreamMeta";
    return string_stream.str();
}

SensorInfoStream::SensorInfoStream(Writer* writer) : writer_{writer} {
    if (writer_ == nullptr) {
        throw std::invalid_argument("writer_ must not be null");
    }

    stream_meta_id_ = writer_->add_metadata(meta_);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
