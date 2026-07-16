/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/writer.h"

#include <gtest/gtest.h>

#include <string>

#include "common.h"
#include "osf_test.h"
#include "ouster/core/class_map.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/file.h"
#include "ouster/osf/impl/sensor_info_stream.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata_stream.h"
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_frame.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace osf {
namespace {

using ouster::sdk::core::SensorInfo;
using ouster::sdk::osf::get_random_lidar_frame;

class WriterTest : public osf::OsfTestWithDataAndFiles {};

namespace {

std::string write_minimal_streaming_osf(const std::string& test_data_dir,
                                        const std::string& output_path) {
    const SensorInfo sinfo =
        metadata_from_json(path_concat(test_data_dir, "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    Writer writer(output_path);
    const auto sensor_id = writer.add_sensor(sinfo);
    writer.save(sensor_id, get_random_lidar_frame(sinfo), ts_t{1});
    writer.save(sensor_id, get_random_lidar_frame(sinfo), ts_t{2});
    writer.close();
    return output_path;
}

}  // namespace

TEST_F(WriterTest, ChunksLayoutEnum) {
    ChunksLayout cl = ChunksLayout::STANDARD;
    EXPECT_EQ(to_string(cl), "STANDARD");
    ChunksLayout cl1{};
    EXPECT_EQ(to_string(cl1), "STANDARD");
    ChunksLayout cl2{ChunksLayout::STREAMING};
    EXPECT_EQ(to_string(cl2), "STREAMING");
    EXPECT_EQ(chunks_layout_of_string("STREAMING"), ChunksLayout::STREAMING);
    EXPECT_EQ(chunks_layout_of_string("STANDARD"), ChunksLayout::STANDARD);
    EXPECT_EQ(chunks_layout_of_string("RRR"), ChunksLayout::STANDARD);
}

TEST_F(WriterTest, WriteSingleLidarFrame) {
    const SensorInfo sinfo =
        metadata_from_json(path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarFrame ls = get_random_lidar_frame(sinfo);

    std::string output_osf_filename = tmp_file("writer_simple.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarFrame
    Writer writer(output_osf_filename);
    writer.set_metadata_id("test_session");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::STREAMING);

    writer.add_sensor(sinfo);
    writer.save(0, ls, ts_t{123});
    writer.close();

    Reader reader(output_osf_filename);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto lidar_stream = reader.meta_store().get<LidarFrameStreamMeta>();
    auto lidar_messages = reader.messages({lidar_stream->id()});

    auto msg_it = lidar_messages.begin();
    EXPECT_NE(msg_it, lidar_messages.end());

    auto ls_recovered = msg_it->decode_msg<LidarFrameStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);

    auto si_stream = reader.meta_store().get<SensorInfoStreamMeta>();
    auto si_messages = reader.messages({si_stream->id()});

    auto si_msg_it = si_messages.begin();
    EXPECT_NE(si_msg_it, si_messages.end());  // sensor info message
    EXPECT_EQ(++si_msg_it, si_messages.end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its SensorInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo_str);

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteLidarSensorWithExtrinsics) {
    SensorInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    std::string output_osf_filename = tmp_file("writer_lidar_sensor_extrinsics.osf");

    std::string sinfo_str = sinfo.to_json_string();

    sinfo.sensor_to_body(0, 3) = 10.0;
    sinfo.sensor_to_body(0, 1) = 0.756;
    sinfo.sensor_to_body(1, 0) = 0.756;
    sinfo.sensor_to_body(0, 0) = 0.0;

    // Writing LidarSensor
    Writer writer(output_osf_filename);

    auto sensor_meta_id = writer.add_metadata<LidarSensor>(sinfo_str);
    EXPECT_TRUE(sensor_meta_id != 0);

    writer.add_metadata<Extrinsics>(sinfo.sensor_to_body, sensor_meta_id);

    writer.close();

    Reader reader(output_osf_filename);

    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its SensorInfo
    auto sinfo_recovered = sensors.begin()->second->info();

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);

    auto extrinsics = reader.meta_store().find<Extrinsics>();
    EXPECT_EQ(extrinsics.size(), 1);
    EXPECT_EQ(extrinsics.begin()->second->repr(),
              "ExtrinsicsMeta: ref_id = 1, name = , extrinsics = 0 0.756 0 10 "
              "0.756 1 0 0 0 0 1 0 0 0 0 1");

    auto ext_mat_recovered = extrinsics.begin()->second->extrinsics();
    EXPECT_EQ(sinfo.sensor_to_body, ext_mat_recovered);
    EXPECT_EQ(sensor_meta_id, extrinsics.begin()->second->ref_meta_id());
}

TEST_F(WriterTest, WriteSingleLidarFrameStreamingLayout) {
    const SensorInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarFrame ls = get_random_lidar_frame(sinfo);

    std::string output_osf_filename = tmp_file("writer_simple_streaming.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarFrame
    Writer writer(output_osf_filename);
    writer.set_metadata_id("test_session");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::STREAMING);

    writer.add_sensor(sinfo);
    writer.save(0, ls, ts_t{123});
    writer.close();

    Reader reader(output_osf_filename);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    // TODO[pb]: Add reader validation CRC
    auto lidar_stream = reader.meta_store().get<LidarFrameStreamMeta>();
    auto lidar_messages = reader.messages({lidar_stream->id()});

    auto msg_it = lidar_messages.begin();
    EXPECT_NE(msg_it, lidar_messages.end());

    auto ls_recovered = msg_it->decode_msg<LidarFrameStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);
    EXPECT_EQ(++msg_it, lidar_messages.end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Check that it's an OSF with StreamingLayout
    EXPECT_EQ(1, reader.meta_store().count<osf::StreamingInfo>());
    auto streaming_info = reader.meta_store().get<osf::StreamingInfo>();

    EXPECT_TRUE(streaming_info != nullptr);

    // One stream LidarFrameStream and the SensorInfoStream
    EXPECT_EQ(2, streaming_info->stream_stats().size());

    // Use first sensor and get its SensorInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteSlicedLidarFrame) {
    const SensorInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarFrame ls = get_random_lidar_frame(sinfo);

    // Subset of fields to leave in LidarFrame
    core::LidarFrameFieldTypes field_types;
    field_types.emplace_back(ls.field_type(core::ChanField::RANGE));
    field_types.emplace_back(ls.field_type(core::ChanField::REFLECTIVITY));

    // Make a reduced field LidarFrame
    ls = slice_and_cast(ls, field_types);

    EXPECT_EQ(field_types.size(), ls.field_types().size());

    std::string output_osf_filename = tmp_file("writer_sliced.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarFrame
    Writer writer(output_osf_filename, sinfo, {"RANGE", "REFLECTIVITY"});
    writer.set_metadata_id("test_session");
    writer.save(0, ls, ts_t{123});
    writer.close();

    Reader reader(output_osf_filename);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto lidar_stream = reader.meta_store().get<LidarFrameStreamMeta>();
    auto lidar_messages = reader.messages({lidar_stream->id()});

    auto msg_it = lidar_messages.begin();
    EXPECT_NE(msg_it, lidar_messages.end());

    auto ls_recovered = msg_it->decode_msg<LidarFrameStream>();

    EXPECT_EQ(field_types.size(), ls_recovered->field_types().size());

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);
    EXPECT_EQ(++msg_it, lidar_messages.end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its SensorInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteSlicedLegacyLidarFrame) {
    const SensorInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "metadata/2_5_0_os-992146000760-128_legacy.json"));
    LidarFrame ls_orig = get_random_lidar_frame(sinfo);

    // Subset of fields to leave in LidarFrame during writing
    LidarFrameFieldTypes field_types;
    field_types.emplace_back(core::ChanField::RANGE, core::ChanFieldType::UINT32);
    field_types.emplace_back(core::ChanField::SIGNAL, core::ChanFieldType::UINT16);
    field_types.emplace_back(core::ChanField::REFLECTIVITY, core::ChanFieldType::UINT8);

    std::cout << "LidarFrame field_types: " << ouster::sdk::core::to_string(field_types)
              << std::endl;

    // Make a reduced/extended fields LidarFrame
    // that will be compared with a recovered LidarFrame from OSF
    auto ls_reference = slice_and_cast(ls_orig, field_types);

    EXPECT_EQ(field_types.size(), ls_reference.field_types().size());

    std::string output_osf_filename = tmp_file("writer_sliced_legacy.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarFrame with custom field types
    Writer writer(output_osf_filename, sinfo, {"RANGE", "SIGNAL", "REFLECTIVITY"});
    writer.set_metadata_id("test_session");

    writer.save(0, ls_orig, ts_t{123});
    writer.close();

    Reader reader(output_osf_filename);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto lidar_stream = reader.meta_store().get<LidarFrameStreamMeta>();
    auto lidar_messages = reader.messages({lidar_stream->id()});

    auto msg_it = lidar_messages.begin();
    EXPECT_NE(msg_it, lidar_messages.end());

    auto ls_recovered = msg_it->decode_msg<LidarFrameStream>();

    EXPECT_TRUE(ls_recovered);

    EXPECT_EQ(field_types.size(), ls_recovered->field_types().size());

    EXPECT_EQ(*ls_recovered, ls_reference);
    EXPECT_EQ(++msg_it, lidar_messages.end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its SensorInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteCustomLidarFrameWithFlags) {
    const SensorInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "metadata/3_0_1_os-122246000293-128_legacy.json"));

    LidarFrameFieldTypes field_types_with_flags;
    field_types_with_flags.emplace_back(core::ChanField::RANGE, core::ChanFieldType::UINT32);
    field_types_with_flags.emplace_back(core::ChanField::SIGNAL, core::ChanFieldType::UINT16);
    field_types_with_flags.emplace_back(core::ChanField::RANGE2, core::ChanFieldType::UINT32);
    field_types_with_flags.emplace_back(core::ChanField::SIGNAL2, core::ChanFieldType::UINT16);
    field_types_with_flags.emplace_back(core::ChanField::REFLECTIVITY, core::ChanFieldType::UINT8);
    field_types_with_flags.emplace_back(core::ChanField::NEAR_IR, core::ChanFieldType::UINT16);
    field_types_with_flags.emplace_back(core::ChanField::FLAGS, core::ChanFieldType::UINT8);
    field_types_with_flags.emplace_back(core::ChanField::FLAGS2, core::ChanFieldType::UINT8);
    field_types_with_flags.emplace_back("CUSTOM0", core::ChanFieldType::UINT64);
    field_types_with_flags.emplace_back("CUSTOM7", core::ChanFieldType::UINT16);

    LidarFrame ls = get_random_lidar_frame(sinfo, field_types_with_flags);

    std::cout << "LidarFrame field_types_with_flags: "
              << ouster::sdk::core::to_string(field_types_with_flags) << std::endl;

    // Check that we have non zero FLAGS
    img_t<uint8_t> flags{ls.h, ls.w};
    core::impl::visit_field(ls, ChanField::FLAGS, core::impl::read_and_cast(), flags);
    EXPECT_FALSE((flags == 0).all());
    // and non zero FLAGS2
    core::impl::visit_field(ls, ChanField::FLAGS2, core::impl::read_and_cast(), flags);
    EXPECT_FALSE((flags == 0).all());

    // Check that we have non zero CUSTOM7
    img_t<uint16_t> custom{ls.h, ls.w};
    core::impl::visit_field(ls, "CUSTOM7", core::impl::read_and_cast(), custom);
    EXPECT_FALSE((custom == 0).all());

    EXPECT_EQ(field_types_with_flags.size(), ls.field_types().size());

    std::string output_osf_filename = tmp_file("writer_with_flags.osf");

    // Writing LidarFrame
    Writer writer(output_osf_filename, sinfo);
    writer.set_metadata_id("test_session");
    writer.save(0, ls, ts_t{123});
    writer.close();

    Reader reader(output_osf_filename);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto lidar_stream = reader.meta_store().get<LidarFrameStreamMeta>();
    auto lidar_messages = reader.messages({lidar_stream->id()});

    auto msg_it = lidar_messages.begin();
    EXPECT_NE(msg_it, lidar_messages.end());

    auto ls_recovered = msg_it->decode_msg<LidarFrameStream>();

    EXPECT_TRUE(ls_recovered);

    EXPECT_EQ(field_types_with_flags.size(), ls_recovered->field_types().size());

    EXPECT_EQ(*ls_recovered, ls);
    EXPECT_EQ(++msg_it, lidar_messages.end());
}

// Used in WriteExample test below
void ReadExample(const std::string filename) {
    // Open output: OSF v2
    Reader reader(filename);

    // Read all messages from OSF file
    for (const auto m : reader.messages()) {
        auto ts = m.ts();              // << message timestamp
        auto stream_meta_id = m.id();  // << link to the stream meta

        EXPECT_GT(ts.count(), 0);
        EXPECT_GT(stream_meta_id, uint32_t{0});

        // Decoding messages
        if (m.is<LidarFrameStream>()) {
            auto ls = m.decode_msg<LidarFrameStream>();
            EXPECT_TRUE(ls != nullptr);
            // std::cout << "ls = " << *ls << std::endl;
        }
    }

    // Get meta objects by type map of (meta_id, meta_ptr)
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(1, sensors.size());

    // Get LidarSensor metadata
    auto lidar_sensor = reader.meta_store().get<LidarSensor>();
    EXPECT_TRUE(lidar_sensor);
}

TEST_F(WriterTest, WriteExample) {
    // Get SensorInfo
    const SensorInfo sinfo =
        metadata_from_json(path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    std::string output_osf_filename = tmp_file("write_example.osf");

    // Create OSF v2 Writer
    osf::Writer writer(output_osf_filename);
    writer.set_metadata_id("Example Session 1234");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::STREAMING);

    // Create LidarSensor record
    auto sensor_id = writer.add_sensor(sinfo);

    const int LOOP_CNT = 7;

    int timestamp = 0;
    while (timestamp++ < LOOP_CNT) {
        LidarFrame ls = get_random_lidar_frame(sinfo);

        // Save LidarFrame
        writer.save(sensor_id, ls, ts_t{timestamp});
    }

    writer.close();

    // Quick test that number of messages in result file is what we wrote
    // including the SensorInfo message
    Reader reader(output_osf_filename);
    EXPECT_EQ(LOOP_CNT + 1, std::distance(reader.messages().begin(), reader.messages().end()));

    // Check that it's an OSF with StreamingLayout
    auto streaming_info_entry = reader.meta_store().find<StreamingInfo>();
    EXPECT_EQ(1, streaming_info_entry.size());
    auto streaming_info = streaming_info_entry.begin()->second;

    // std::cout << "streaming_info = " << streaming_info->to_string() <<
    // std::endl;

    // Two stream: LidarFrameStream and SensorInfoStream
    EXPECT_EQ(2, streaming_info->stream_stats().size());

    EXPECT_TRUE(reader.has_message_idx());
    EXPECT_TRUE(reader.has_timestamp_idx());

    auto lsm = reader.meta_store().get<LidarFrameStreamMeta>();

    auto stream_msg_count = streaming_info->stream_stats()[lsm->id()].message_count;
    EXPECT_EQ(stream_msg_count, LOOP_CNT);

    for (size_t msg_idx = 0; msg_idx < stream_msg_count; ++msg_idx) {
        auto msg_ts = reader.ts_by_message_idx(lsm->id(), msg_idx);
        EXPECT_TRUE(msg_ts);

        EXPECT_TRUE(msg_ts >= reader.start_ts());
        EXPECT_TRUE(msg_ts <= reader.end_ts());

        // by construction of the test
        EXPECT_EQ(msg_idx + 1, msg_ts->count());

        // if we start reading from that msg_ts using stream_id, there is indeed
        // the first message returned with this timestamp
        auto first_msg = reader.messages({lsm->id()}, *msg_ts, reader.end_ts()).begin();
        EXPECT_EQ(first_msg->ts(), msg_ts);
    }

    auto msg_ts100 = reader.ts_by_message_idx(lsm->id(), 100);
    EXPECT_FALSE(msg_ts100);

    auto msg_ts_count = reader.ts_by_message_idx(lsm->id(), stream_msg_count);
    EXPECT_FALSE(msg_ts_count);

    auto msg_ts_no_stream = reader.ts_by_message_idx(0, 0);
    EXPECT_FALSE(msg_ts_no_stream);

    auto msg_ts_no_stream2 = reader.ts_by_message_idx(100, 0);
    EXPECT_FALSE(msg_ts_no_stream2);

    ReadExample(output_osf_filename);
}

TEST_F(WriterTest, FileNameOnlyWriterTest) {
    osf::Writer writer("FOOBARBAT");
    EXPECT_EQ(writer.filename(), "FOOBARBAT");
    EXPECT_EQ(writer.metadata_id(), "ouster_sdk");
}

TEST_F(WriterTest, WriteCustomFieldsTest) {
    const SensorInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarFrame ls = get_random_lidar_frame(sinfo);
    auto& f1 =
        ls.add_field("custom_field_1", fd_array<double>(85, 129, 344), FieldClass::FRAME_FIELD);
    auto& f2 =
        ls.add_field("custom_field_2", fd_array<uint16_t>(111, 333), FieldClass::FRAME_FIELD);

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> nd_d{100.0, 10.0};
    std::uniform_int_distribution<uint16_t> ud_u8{0, 150};
    randomize_field<double>(f1, gen, nd_d);
    randomize_field<uint16_t>(f2, gen, ud_u8);

    std::string output_osf_filename = tmp_file("writer_simple.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarFrame
    Writer writer(output_osf_filename);
    writer.set_metadata_id("test_session");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::STREAMING);

    writer.add_sensor(sinfo);
    writer.save(0, ls, ts_t{123});
    writer.close();

    Reader reader(output_osf_filename);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto lidar_stream = reader.meta_store().get<LidarFrameStreamMeta>();
    auto lidar_messages = reader.messages({lidar_stream->id()});

    auto msg_it = lidar_messages.begin();
    EXPECT_NE(msg_it, lidar_messages.end());

    auto ls_recovered = msg_it->decode_msg<LidarFrameStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);
}

TEST_F(WriterTest, WriteCollationTest) {
    const auto info1 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const auto info2 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"));
    const auto info3 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-2-128-U1_v2.3.0_1024x10.json"));

    auto ls1 = std::make_shared<LidarFrame>(get_random_lidar_frame(info1));
    auto ls2 = std::make_shared<LidarFrame>(get_random_lidar_frame(info2));

    // we want a collation with a few frames and one skipped frame
    FrameSet collation({ls1, ls2, nullptr});

    auto& f1 = collation.add_field("custom1", fd_array<double>(85, 129, 344));
    auto& f2 = collation.add_field("custom2", fd_array<uint16_t>(111, 333));

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> nd_d{100.0, 10.0};
    std::uniform_int_distribution<uint16_t> ud_u8{0, 150};
    randomize_field<double>(f1, gen, nd_d);
    randomize_field<uint16_t>(f2, gen, ud_u8);

    std::string output_osf_filename = tmp_file("writer_collation.osf");
    Writer writer(output_osf_filename, {info1, info2, info3});
    writer.save(collation);
    writer.close();

    Reader reader(output_osf_filename);
    auto collation_meta = reader.meta_store().get<CollationStreamMeta>();
    ASSERT_NE(collation_meta, nullptr);
    auto collation_stream_id = collation_meta->id();

    auto msg_it = reader.messages({collation_stream_id}).begin();
    EXPECT_NE(msg_it, reader.messages().end());

    std::map<uint32_t, uint32_t> stream_ids;
    {
        auto streams = reader.meta_store().find<ouster::sdk::osf::LidarFrameStreamMeta>();
        uint32_t i = 0;
        for (const auto& item : streams) {
            stream_ids[i++] = item.first;
        }
    }

    ResolveFrameFn resolve_frame =
        [&reader, &stream_ids](FrameUniqueId frame_id) -> std::shared_ptr<LidarFrame> {
        if (frame_id == INVALID_FRAME_UID) {
            return nullptr;
        }

        auto lidar_stream_id = stream_ids.at(frame_id.first);
        auto start = reader.ts_by_message_idx(lidar_stream_id, frame_id.second).value();
        auto range = reader.messages({lidar_stream_id}, start, reader.end_ts());
        auto msg_it = range.begin();

        std::unique_ptr<LidarFrame> frame = msg_it->decode_msg<LidarFrameStream>();

        return std::shared_ptr<LidarFrame>(frame.release());
    };

    auto collation_recovered = msg_it->decode_msg<CollationStream>(resolve_frame);

    EXPECT_EQ(collation, *collation_recovered);
    EXPECT_FALSE((*collation_recovered)[2]);

    // TODO: test that skips over some other lidarframes after we normalize
    //       timestamps
}

TEST_F(WriterTest, frameSetSourceMetadataStreamTest) {
    const auto sensor_info = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    std::string output_osf_filename = tmp_file("writer_frame_set_source_metadata.osf");
    ouster::sdk::osf::Writer writer(output_osf_filename, {sensor_info});

    ClassMap class_map1;
    class_map1.class_map.emplace(1, "dog");
    class_map1.class_map.emplace(2, "cat");
    ClassMap class_map2;
    class_map2.class_map.emplace(1, "tree");
    class_map2.class_map.emplace(2, "bush");
    ClassMapSet class_maps;
    class_maps.class_maps.emplace("four_legs", class_map1);
    class_maps.class_maps.emplace("zero_legs", class_map2);
    FrameSetSourceMetadataSet frame_set_source_metadata_set;
    frame_set_source_metadata_set.entries.emplace("class_maps", class_maps);
    std::string additional_info = "Test additional info";
    frame_set_source_metadata_set.entries.emplace("additional_info", additional_info);
    writer.save(frame_set_source_metadata_set);
    writer.close();

    Reader reader(output_osf_filename);
    for (const auto& msg : reader.messages()) {
        if (msg.is<FrameSetSourceMetadataStream>()) {
            auto collection = msg.decode_msg<FrameSetSourceMetadataStream>();
            EXPECT_TRUE(collection != nullptr);
            EXPECT_EQ(collection->entries.size(), 2);
            auto it = collection->entries.find("class_maps");
            EXPECT_TRUE(it != collection->entries.end());
            auto retrieved_class_maps = it->second.as<ClassMapSet>();
            EXPECT_EQ(retrieved_class_maps, class_maps);
            auto it2 = collection->entries.find("additional_info");
            EXPECT_EQ(it2->second.as<std::string>(), additional_info);
        }
    }

    auto src = open_source(output_osf_filename);
    // In C++, FrameSetSource metadata uses type erasure to allow storing
    // different types of metadata So the type must be checked and the value
    // casted to the appropriate type.
    if (!src.metadata("class_maps").is<ClassMapSet>()) {
        throw std::runtime_error("Unexpected metadata type!");
    }
    auto class_map_set = src.metadata("class_maps").as<ClassMapSet>();
    auto dog_class = class_map_set.class_maps["four_legs"].class_map[1];

    EXPECT_EQ(src.metadata_keys(), std::set<std::string>({"class_maps", "additional_info"}));

    EXPECT_EQ(class_map_set, class_maps);
    EXPECT_EQ(src.metadata("additional_info").as<std::string>(), additional_info);
}

TEST_F(WriterTest, HasTimestampIdxTrueForWriterOutput) {
    const std::string path =
        write_minimal_streaming_osf(test_data_dir(), tmp_file("ts_idx_ok.osf"));
    Reader reader(path);
    EXPECT_EQ(1, reader.meta_store().count<StreamingInfo>());
    EXPECT_TRUE(reader.has_message_idx());
    EXPECT_TRUE(reader.has_timestamp_idx());
}

}  // namespace
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
