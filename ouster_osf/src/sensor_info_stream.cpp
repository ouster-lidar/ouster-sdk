#include "ouster/osf/sensor_info_stream.h"

#include <zstd.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "os_sensor/sensor_info_stream_generated.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"
#include "ouster/zone_monitor.h"

using namespace std::literals::chrono_literals;

namespace ouster {
namespace sdk {
namespace osf {

class MessageRef;

std::vector<uint8_t> SensorInfoStream::make_msg(
    const ouster::sdk::osf::SensorInfoMessage& msg) {
    auto string = msg.sensor_info.to_json_string();

    // compress the string
    auto max_size = ZSTD_compressBound(string.length());
    std::vector<uint8_t> compressed;
    compressed.resize(max_size);
    auto compressed_size = ZSTD_compress(compressed.data(), max_size,
                                         string.data(), string.length(), 1);
    compressed.resize(compressed_size);

    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto lss_offset = ouster::sdk::osf::v2::CreateSensorInfoDirect(
        fbb, msg.lidar_sensor_id, msg.scan_stream_id, &compressed);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<SensorInfoStream::obj_type> SensorInfoStream::from_buffer(
    const std::vector<uint8_t>& buf) {
    auto si_msg =
        flatbuffers::GetSizePrefixedRoot<ouster::sdk::osf::v2::SensorInfo>(
            buf.data());
    auto out = std::make_unique<ouster::sdk::osf::SensorInfoMessage>();
    out->scan_stream_id = si_msg->scan_stream_id();
    out->lidar_sensor_id = si_msg->lidar_sensor_id();

    // decompress
    auto sensor_info = si_msg->sensor_info();
    auto size =
        ZSTD_getFrameContentSize(sensor_info->data(), sensor_info->size());
    std::string string_info;
    string_info.resize(size);
    auto dsize = ZSTD_decompress(&string_info.front(), size,
                                 sensor_info->data(), sensor_info->size());
    if (dsize != size) {
        throw std::runtime_error("Unexpected decompressed size.");
    }
    out->sensor_info = ouster::sdk::core::SensorInfo(string_info);
    return out;
}

std::unique_ptr<SensorInfoStream::obj_type> SensorInfoStream::decode_msg(
    const MessageRef& msg, const meta_type& /*meta*/,
    const MetadataStore& /*meta_provider*/) {
    const auto& buf = msg.buffer();
    return from_buffer(buf);
}

void SensorInfoStream::save(const obj_type& zone_set_config, ts_t timestamp) {
    if (writer_ == nullptr) {
        throw std::logic_error("writer_ must not be null");
    }

    const auto& msg_buf = make_msg(zone_set_config);
    ts_t sensor_ts{0ns};
    writer_->save_message(meta_.id(), timestamp, sensor_ts, msg_buf,
                          MetadataTraits<SensorInfoStreamMeta>::type());
}

SensorInfoStreamMeta::SensorInfoStreamMeta() {}

std::vector<uint8_t> SensorInfoStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    auto lss_offset = ouster::sdk::osf::v2::CreateSensorInfoStream(fbb);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<MetadataEntry> SensorInfoStreamMeta::from_buffer(
    const ouster::sdk::osf::OsfBuffer /*buf*/) {
    // the metadata is empty, dont bother trying to decode
    return std::make_unique<SensorInfoStreamMeta>();
}

std::string SensorInfoStreamMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "SensorInfoStreamMeta";
    return string_stream.str();
}

SensorInfoStream::SensorInfoStream(Token /*key*/, Writer* writer)
    : writer_{writer} {
    if (writer_ == nullptr) {
        throw std::invalid_argument("writer_ must not be null");
    }

    stream_meta_id_ = writer_->add_metadata(meta_);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
