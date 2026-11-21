/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "common.h"
#include "fb_common.h"
#include "os_sensor/camera_frame_stream_generated.h"
#include "os_sensor/common_generated.h"
#include "os_sensor/lidar_scan_stream_generated.h"
#include "osf_test.h"
#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/error_handler.h"
#include "ouster/core/field.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/meta_camera_sensor.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_camera_frame.h"
#include "ouster/osf/stream_lidar_frame.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace {

using ouster::sdk::core::CameraFrame;
using ouster::sdk::core::CameraInfo;
using ouster::sdk::core::CameraModel;
using ouster::sdk::core::fd_array;
using ouster::sdk::core::Field;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::LidarInfo;
using ouster::sdk::core::metadata_from_json;
using ouster::sdk::core::Severity;

namespace gen = ouster::sdk::osf::impl::gen;

// Enum values this SDK version doesn't know about, standing in for ones a newer
// SDK might add and write into an OSF.
constexpr auto UNKNOWN_FIELD_TYPE = static_cast<gen::CHAN_FIELD_TYPE>(200);
constexpr auto UNKNOWN_CHAN_FIELD = static_cast<gen::CHAN_FIELD>(200);

class OsfUnknownFieldTypeTest : public OsfTestWithDataAndFiles {
   protected:
    LidarInfo lidar_info() {
        return metadata_from_json(
            path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    }
};

template <typename T>
Field make_field(size_t rows, size_t cols) {
    Field field{fd_array<T>(rows, cols), FieldClass::PIXEL_FIELD};
    auto* data = field.get<T>();
    std::iota(data, data + field.size(), T{1});
    return field;
}

std::vector<uint8_t> finish(flatbuffers::FlatBufferBuilder& fbb, flatbuffers::Offset<void> root) {
    fbb.FinishSizePrefixed(root);
    return {fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize()};
}

uint32_t add_lidar_stream(Writer& writer, const LidarInfo& info) {
    LidarSensor sensor(info);
    const uint32_t sensor_id = writer.add_metadata(sensor);
    LidarFrameStreamMeta stream(sensor_id);
    return writer.add_metadata(stream);
}

struct Warnings {
    std::vector<std::string> messages;

    ouster::sdk::core::error_handler_t handler() {
        return [this](Severity severity, const std::string& message) {
            EXPECT_EQ(severity, Severity::OUSTER_WARNING);
            messages.push_back(message);
        };
    }

    std::string joined() const {
        std::string result;
        for (const auto& message : messages) {
            result += message + "\n";
        }
        return result;
    }
};

template <typename Stream>
std::vector<std::unique_ptr<typename Stream::obj_type>> decode_all(Reader& reader) {
    std::vector<std::unique_ptr<typename Stream::obj_type>> result;
    for (const auto& msg : reader.messages()) {
        if (msg.is<Stream>()) {
            result.push_back(msg.decode_msg<Stream>());
        }
    }
    return result;
}

// Runs `func` and returns what it wrote to stderr, including via the SDK
// logger. On Windows the logger's stderr sink writes to the OS handle it was
// created with, which CaptureStderr() closes and GetCapturedStderr() replaces,
// so the sink is recreated on both sides of the capture.
template <typename Func>
std::string capture_logged_stderr(Func&& func) {
    testing::internal::CaptureStderr();
    ouster::sdk::core::logger().configure_stderr_sink("info");
    func();
    std::string output = testing::internal::GetCapturedStderr();
    ouster::sdk::core::logger().configure_stderr_sink("info");
    return output;
}

// Writes an OSF with a single lidar frame having two custom fields: "VALID"
// and "FROM_THE_FUTURE", the latter with a field type this SDK doesn't know.
Field write_lidar_custom_fields_osf(const std::string& path, const LidarInfo& info) {
    Field valid = make_field<uint16_t>(info.h(), info.w());

    Writer writer(path);
    const uint32_t stream_id = add_lidar_stream(writer, info);
    auto valid_data = writer.encoder().frame_encoder().encode_field(valid);
    std::vector<uint64_t> shape{valid.shape().begin(), valid.shape().end()};

    flatbuffers::FlatBufferBuilder fbb;
    std::vector<flatbuffers::Offset<gen::Field>> fields{
        gen::CreateFieldDirect(fbb, "VALID", gen::CHAN_FIELD_TYPE::UINT16, &shape,
                               gen::FIELD_CLASS::PIXEL_FIELD, &valid_data, valid.bytes()),
        gen::CreateFieldDirect(fbb, "FROM_THE_FUTURE", UNKNOWN_FIELD_TYPE, &shape,
                               gen::FIELD_CLASS::PIXEL_FIELD, &valid_data, valid.bytes())};
    auto fields_off = fbb.CreateVector(fields);
    auto channels_off = fbb.CreateVector(std::vector<flatbuffers::Offset<gen::ChannelData>>{});
    gen::LidarScanMsgBuilder msg(fbb);
    msg.add_channels(channels_off);
    msg.add_custom_fields(fields_off);

    writer.save_message(stream_id, ts_t{100}, ts_t{100}, finish(fbb, msg.Finish().Union()),
                        MetadataTraits<LidarFrameStreamMeta>::type());
    writer.close();
    return valid;
}

// Writes an OSF with a single lidar frame having two legacy (standard) channels:
// `unknown` first, whose data is garbage and must never be decoded, followed by
// a valid REFLECTIVITY channel.
Field write_lidar_channels_osf(const std::string& path, const LidarInfo& info,
                               gen::ChannelField unknown) {
    Field reflectivity = make_field<uint8_t>(info.h(), info.w());

    Writer writer(path);
    const uint32_t stream_id = add_lidar_stream(writer, info);
    auto reflectivity_data =
        writer.encoder().frame_encoder().encode_field(reflectivity, info.format.pixel_shift_by_row);
    std::vector<uint8_t> garbage{1, 2, 3, 4};

    flatbuffers::FlatBufferBuilder fbb;
    std::vector<flatbuffers::Offset<gen::ChannelData>> channels{
        gen::CreateChannelDataDirect(fbb, &garbage),
        gen::CreateChannelDataDirect(fbb, &reflectivity_data)};
    std::vector<gen::ChannelField> field_types{
        unknown, gen::ChannelField(gen::CHAN_FIELD::REFLECTIVITY, gen::CHAN_FIELD_TYPE::UINT8)};
    auto channels_off = fbb.CreateVector(channels);
    auto field_types_off = create_vector_of_structs(fbb, field_types.data(), field_types.size());
    gen::LidarScanMsgBuilder msg(fbb);
    msg.add_channels(channels_off);
    msg.add_field_types(field_types_off);

    writer.save_message(stream_id, ts_t{100}, ts_t{100}, finish(fbb, msg.Finish().Union()),
                        MetadataTraits<LidarFrameStreamMeta>::type());
    writer.close();
    return reflectivity;
}

TEST_F(OsfUnknownFieldTypeTest, LidarCustomFieldWithUnknownTypeIsSkipped) {
    const std::string path = tmp_file("unknown_lidar_custom_field_type.osf");
    const Field valid = write_lidar_custom_fields_osf(path, lidar_info());

    Warnings warnings;
    Reader reader(path, warnings.handler());
    auto frames = decode_all<LidarFrameStream>(reader);

    ASSERT_EQ(frames.size(), 1u);
    ASSERT_NE(frames[0], nullptr);
    EXPECT_TRUE(frames[0]->has_field("VALID"));
    EXPECT_FALSE(frames[0]->has_field("FROM_THE_FUTURE"));
    EXPECT_EQ(frames[0]->field("VALID"), valid);

    EXPECT_NE(warnings.joined().find(
                  "Skipping field 'FROM_THE_FUTURE' with unsupported field type (tag=200)"),
              std::string::npos)
        << "warnings: " << warnings.joined();
}

TEST_F(OsfUnknownFieldTypeTest, UnknownFieldTypeWarningIsLoggedByDefaultHandler) {
    const std::string path = tmp_file("unknown_field_type_default_handler.osf");
    const Field valid = write_lidar_custom_fields_osf(path, lidar_info());

    Reader reader(path);
    std::vector<std::unique_ptr<LidarFrame>> frames;
    const std::string console =
        capture_logged_stderr([&] { frames = decode_all<LidarFrameStream>(reader); });

    ASSERT_EQ(frames.size(), 1u);
    ASSERT_NE(frames[0], nullptr);
    EXPECT_EQ(frames[0]->field("VALID"), valid);
    EXPECT_NE(
        console.find("Skipping field 'FROM_THE_FUTURE' with unsupported field type (tag=200)"),
        std::string::npos)
        << "console output: " << console;
}

TEST_F(OsfUnknownFieldTypeTest, LidarChannelWithUnknownTypeIsSkipped) {
    const std::string path = tmp_file("unknown_lidar_channel_type.osf");
    const Field reflectivity = write_lidar_channels_osf(
        path, lidar_info(), gen::ChannelField(gen::CHAN_FIELD::RANGE, UNKNOWN_FIELD_TYPE));

    Warnings warnings;
    Reader reader(path, warnings.handler());
    auto frames = decode_all<LidarFrameStream>(reader);

    ASSERT_EQ(frames.size(), 1u);
    ASSERT_NE(frames[0], nullptr);
    EXPECT_FALSE(frames[0]->has_field("RANGE"));
    ASSERT_TRUE(frames[0]->has_field("REFLECTIVITY"));
    EXPECT_EQ(frames[0]->field("REFLECTIVITY"), reflectivity);

    EXPECT_NE(
        warnings.joined().find("Skipping field 'RANGE' with unsupported field type (tag=200)"),
        std::string::npos)
        << "warnings: " << warnings.joined();
}

TEST_F(OsfUnknownFieldTypeTest, LidarChannelWithUnknownChanFieldIsSkipped) {
    const std::string path = tmp_file("unknown_lidar_chan_field.osf");
    const Field reflectivity = write_lidar_channels_osf(
        path, lidar_info(), gen::ChannelField(UNKNOWN_CHAN_FIELD, gen::CHAN_FIELD_TYPE::UINT32));

    Warnings warnings;
    Reader reader(path, warnings.handler());
    auto frames = decode_all<LidarFrameStream>(reader);

    ASSERT_EQ(frames.size(), 1u);
    ASSERT_NE(frames[0], nullptr);
    ASSERT_TRUE(frames[0]->has_field("REFLECTIVITY"));
    EXPECT_EQ(frames[0]->fields().size(), 1u);
    EXPECT_EQ(frames[0]->field("REFLECTIVITY"), reflectivity);

    EXPECT_NE(warnings.joined().find("Skipping field with unsupported channel field id (200)"),
              std::string::npos)
        << "warnings: " << warnings.joined();
}

TEST_F(OsfUnknownFieldTypeTest, LidarStreamMetaWithUnknownTypeIsSkipped) {
    flatbuffers::FlatBufferBuilder fbb;
    std::vector<gen::ChannelField> field_types{
        gen::ChannelField(gen::CHAN_FIELD::RANGE, UNKNOWN_FIELD_TYPE),
        gen::ChannelField(gen::CHAN_FIELD::REFLECTIVITY, gen::CHAN_FIELD_TYPE::UINT8)};
    auto stream_off = gen::CreateLidarScanStream(
        fbb, 1, create_vector_of_structs(fbb, field_types.data(), field_types.size()));
    OsfBuffer buf;
    buf.load_data(finish(fbb, stream_off.Union()));

    std::unique_ptr<MetadataEntry> meta;
    const std::string console =
        capture_logged_stderr([&] { meta = LidarFrameStreamMeta::from_buffer(buf); });

    ASSERT_NE(meta, nullptr);
    EXPECT_EQ(meta->repr(),
              "LidarFrameStreamMeta: sensor_id = 1, field_types = {REFLECTIVITY:UINT8}");
    EXPECT_NE(console.find("LidarScanStream metadata: skipping field"), std::string::npos)
        << "console output: " << console;
}

TEST_F(OsfUnknownFieldTypeTest, CameraFieldWithUnknownTypeIsSkipped) {
    const std::string path = tmp_file("unknown_camera_field_type.osf");
    const Field valid = make_field<uint8_t>(3, 4);
    {
        CameraInfo info;
        info.model = CameraModel(100.0f, 100.0f, 0.0f, 0.0f);
        Writer writer(path);
        CameraSensor sensor(info);
        const uint32_t sensor_id = writer.add_metadata(sensor);
        CameraFrameStreamMeta stream(sensor_id);
        const uint32_t stream_id = writer.add_metadata(stream);

        auto valid_data = writer.encoder().frame_encoder().encode_field(valid);
        std::vector<uint64_t> shape{valid.shape().begin(), valid.shape().end()};

        flatbuffers::FlatBufferBuilder fbb;
        std::vector<flatbuffers::Offset<gen::Field>> fields{
            gen::CreateFieldDirect(fbb, "FROM_THE_FUTURE", UNKNOWN_FIELD_TYPE, &shape,
                                   gen::FIELD_CLASS::PIXEL_FIELD, &valid_data, valid.bytes()),
            gen::CreateFieldDirect(fbb, "VALID", gen::CHAN_FIELD_TYPE::UINT8, &shape,
                                   gen::FIELD_CLASS::PIXEL_FIELD, &valid_data, valid.bytes())};
        auto msg = gen::CreateCameraFrameMsg(fbb, 4, 3, fbb.CreateVector(fields));

        writer.save_message(stream_id, ts_t{100}, ts_t{100}, finish(fbb, msg.Union()),
                            MetadataTraits<CameraFrameStreamMeta>::type());
        writer.close();
    }

    Warnings warnings;
    Reader reader(path, warnings.handler());
    auto frames = decode_all<CameraFrameStream>(reader);

    ASSERT_EQ(frames.size(), 1u);
    ASSERT_NE(frames[0], nullptr);
    EXPECT_FALSE(frames[0]->has_field("FROM_THE_FUTURE"));
    ASSERT_TRUE(frames[0]->has_field("VALID"));
    EXPECT_EQ(frames[0]->field("VALID"), valid);

    EXPECT_NE(warnings.joined().find(
                  "Skipping field 'FROM_THE_FUTURE' with unsupported field type (tag=200)"),
              std::string::npos)
        << "warnings: " << warnings.joined();
}

}  // namespace
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
