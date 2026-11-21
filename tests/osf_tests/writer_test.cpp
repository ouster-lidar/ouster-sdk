/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/writer.h"

#include <gtest/gtest.h>

#include <string>

#include "common.h"
#include "osf_test.h"
#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/class_map.h"
#include "ouster/core/field.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/file.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/sensor_info_stream.h"
#include "ouster/osf/meta_camera_sensor.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata_stream.h"
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_camera_frame.h"
#include "ouster/osf/stream_lidar_frame.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace osf {
namespace {

using ouster::sdk::core::LidarInfo;
using ouster::sdk::osf::get_random_lidar_frame;

class WriterTest : public osf::OsfTestWithDataAndFiles {};

namespace {

std::string write_minimal_streaming_osf(const std::string& test_data_dir,
                                        const std::string& output_path) {
    const LidarInfo sinfo =
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
    const LidarInfo sinfo =
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

    // Use first sensor and get its LidarInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo_str);

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteLidarSensorWithExtrinsics) {
    LidarInfo sinfo = core::metadata_from_json(
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

    // Use first sensor and get its LidarInfo
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
    const LidarInfo sinfo = core::metadata_from_json(
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

    // Use first sensor and get its LidarInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteSlicedLidarFrame) {
    const LidarInfo sinfo = core::metadata_from_json(
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

    // Use first sensor and get its LidarInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteSlicedLegacyLidarFrame) {
    const LidarInfo sinfo = core::metadata_from_json(
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

    // Use first sensor and get its LidarInfo
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, SliceAndCastPreservesFrameKind) {
    // slice_and_cast copies through Frame::clone(), so a camera frame slices
    // like a lidar frame: the concrete type, headers, and metadata survive and
    // only the field set changes.
    auto cam_info = std::make_shared<CameraInfo>();
    cam_info->format.columns_per_frame = 4;
    cam_info->format.pixels_per_column = 3;

    CameraFrame cf{cam_info};
    cf.w = cam_info->w();
    cf.h = cam_info->h();
    cf.timestamp()[0] = 1234;
    cf.receive_timestamp()[0] = 5678;
    cf.set_column_pose(0, Matrix4dR(Matrix4dR::Identity() * 2.0));
    cf.add_field(core::ChanField::RANGE, fd_array<uint32_t>(cf.h, cf.w));
    // A three-channel image: the per-pixel dimension has to come across intact.
    cf.add_field("LEFT_BGR", fd_array<uint8_t>(cf.h, cf.w, 3));
    cf.add_field("DROPPED", fd_array<uint16_t>(cf.h, cf.w));
    cf.field<uint32_t>(core::ChanField::RANGE).setConstant(0xffffff01);
    cf.field("LEFT_BGR").get<uint8_t>()[0] = 42;

    LidarFrameFieldTypes field_types;
    field_types.emplace_back(core::ChanField::RANGE, core::ChanFieldType::UINT8);
    field_types.emplace_back(cf.field_type("LEFT_BGR"));

    auto sliced = slice_and_cast(static_cast<const core::Frame&>(cf), field_types);
    ASSERT_NE(sliced, nullptr);
    auto sliced_camera = std::dynamic_pointer_cast<CameraFrame>(sliced);
    ASSERT_NE(sliced_camera, nullptr);

    // Exactly the requested fields, and nothing else.
    EXPECT_EQ(sliced->fields().size(), 2u);
    EXPECT_TRUE(sliced->has_field(core::ChanField::RANGE));
    EXPECT_TRUE(sliced->has_field("LEFT_BGR"));
    EXPECT_FALSE(sliced->has_field("DROPPED"));

    // RANGE was narrowed to its bottom 8 bits; the image came across untouched.
    EXPECT_EQ(sliced->field_type(core::ChanField::RANGE).element_type, core::ChanFieldType::UINT8);
    EXPECT_TRUE(sliced->field<uint8_t>(core::ChanField::RANGE).isConstant(0x01));
    EXPECT_EQ(sliced->field_type("LEFT_BGR"), cf.field_type("LEFT_BGR"));
    EXPECT_EQ(sliced->field("LEFT_BGR"), cf.field("LEFT_BGR"));

    // Dimensions, headers and metadata are the source's.
    EXPECT_EQ(sliced->w, cf.w);
    EXPECT_EQ(sliced->h, cf.h);
    EXPECT_EQ(sliced->timestamp()[0], 1234u);
    EXPECT_EQ(sliced->receive_timestamp()[0], 5678u);
    EXPECT_EQ(sliced->get_column_pose(0), cf.get_column_pose(0));
    EXPECT_EQ(sliced_camera->sensor_info(), cam_info);

    // A requested field the frame doesn't have is an error, as for lidar.
    LidarFrameFieldTypes missing;
    missing.emplace_back("NOT_THERE", core::ChanFieldType::UINT8);
    EXPECT_THROW(slice_and_cast(static_cast<const core::Frame&>(cf), missing),
                 std::invalid_argument);
}

TEST_F(WriterTest, SliceAndCastLidarFrameThroughFrameBase) {
    const LidarInfo sinfo = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const LidarFrame ls = get_random_lidar_frame(sinfo);

    LidarFrameFieldTypes field_types;
    field_types.emplace_back(core::ChanField::RANGE, core::ChanFieldType::UINT8);
    field_types.emplace_back(ls.field_type(core::ChanField::REFLECTIVITY));

    // Reaching slice_and_cast through the Frame base yields a LidarFrame, with
    // the lidar-only headers intact...
    auto sliced = slice_and_cast(static_cast<const core::Frame&>(ls), field_types);
    auto sliced_lidar = std::dynamic_pointer_cast<LidarFrame>(sliced);
    ASSERT_NE(sliced_lidar, nullptr);
    EXPECT_EQ(sliced_lidar->frame_id, ls.frame_id);
    EXPECT_EQ(sliced_lidar->frame_status, ls.frame_status);
    EXPECT_TRUE(sliced_lidar->measurement_id().isApprox(ls.measurement_id()));
    EXPECT_TRUE(sliced_lidar->alert_flags().isApprox(ls.alert_flags()));

    // ...and matches what the LidarFrame overload returns by value.
    EXPECT_EQ(*sliced_lidar, slice_and_cast(ls, field_types));
}

TEST_F(WriterTest, WriteCustomLidarFrameWithFlags) {
    const LidarInfo sinfo = core::metadata_from_json(
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
    // Get LidarInfo
    const LidarInfo sinfo =
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
    // including the LidarInfo message
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
    const LidarInfo sinfo = core::metadata_from_json(
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

    // Index-based collation round-trips through the OsfFrameSetSource. info3
    // registered a sensor that never produced a frame, so its slot stays null.
    OsfFrameSetSource source(output_osf_filename);
    EXPECT_TRUE(source.is_collated());
    std::vector<FrameSet> recovered_all;
    for (auto fs : source) {
        recovered_all.push_back(std::move(fs));
    }
    ASSERT_EQ(recovered_all.size(), 1u);
    const auto& collation_recovered = recovered_all.front();

    EXPECT_EQ(collation, collation_recovered);
    ASSERT_EQ(collation_recovered.size(), 3u);
    EXPECT_FALSE(collation_recovered[2]);

    // TODO: test that skips over some other lidarframes after we normalize
    //       timestamps
}

TEST_F(WriterTest, WriteCollationWithCameraFramesTest) {
    // Round-trip test for FrameSet with both lidar and camera frames,
    // including a null slot on each side.
    const auto info1 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const auto info2 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"));

    auto ls1 = std::make_shared<LidarFrame>(get_random_lidar_frame(info1));
    auto ls2 = std::make_shared<LidarFrame>(get_random_lidar_frame(info2));

    // Build two CameraInfos with distinguishable intrinsics so we can verify
    // that each camera's CameraInfo round-trips per-stream via the footer.
    ouster::sdk::core::CameraInfo cam_info_a;
    cam_info_a.model = ouster::sdk::core::CameraModel(500.0f, 501.0f, 320.0f, 240.0f);

    ouster::sdk::core::CameraInfo cam_info_b;
    cam_info_b.model = ouster::sdk::core::CameraModel(800.0f, 801.0f, 640.0f, 360.0f);

    // Build a CameraFrame with a plausible 2D image field. Camera frames need a
    // valid (non-zero) timestamp to participate in a collation, exactly like
    // lidar frames.
    auto make_camera_frame = [](std::size_t w, std::size_t h, uint64_t ts) {
        auto frame = std::make_shared<core::CameraFrame>();
        frame->w = w;
        frame->h = h;
        frame->add_field("image", fd_array<uint8_t>(h, w), FieldClass::PIXEL_FIELD);
        frame->timestamp()[0] = ts;
        frame->receive_timestamp()[0] = ts;
        return frame;
    };
    auto cf_a = make_camera_frame(640, 480, 1000);
    // Second slot is null - we want to verify null-slot preservation across
    // the camera side of the round-trip.

    // The collation frame order must match the registration order below:
    // [lidar0, lidar1, camera0, camera1] (camera1 null).
    std::vector<std::shared_ptr<core::Frame>> collation_frames{ls1, ls2, cf_a, nullptr};
    FrameSet collation(std::move(collation_frames));

    std::string output_osf_filename = tmp_file("writer_collation_with_cameras.osf");
    Writer writer(output_osf_filename, {info1, info2});
    auto cam_a_idx = writer.add_sensor(cam_info_a);
    auto cam_b_idx = writer.add_sensor(cam_info_b);
    // add_sensor returns the unified (registration-order) index; the two
    // cameras follow the two lidars registered via the constructor.
    EXPECT_EQ(cam_a_idx, 2u);
    EXPECT_EQ(cam_b_idx, 3u);
    writer.save(collation);
    writer.close();

    // Iterate through OsfFrameSetSource and pick up the (single) collation.
    OsfFrameSetSource source(output_osf_filename);
    EXPECT_TRUE(source.is_collated());
    std::vector<FrameSet> recovered_all;
    for (auto fs : source) {
        recovered_all.push_back(std::move(fs));
    }
    ASSERT_EQ(recovered_all.size(), 1u);
    const auto& recovered = recovered_all.front();

    // Positional order is preserved: [lidar, lidar, camera, null].
    ASSERT_EQ(recovered.size(), 4u);
    EXPECT_NE(std::dynamic_pointer_cast<LidarFrame>(recovered[0]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<LidarFrame>(recovered[1]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<core::CameraFrame>(recovered[2]), nullptr);
    EXPECT_EQ(recovered[3], nullptr);

    // Lidar side: 2 frames, both present.
    const auto recovered_lidars = recovered.valid_lidar_frames();
    ASSERT_EQ(recovered_lidars.size(), 2u);

    // Camera side: only the first slot was populated.
    const auto recovered_cameras = recovered.valid_camera_frames();
    ASSERT_EQ(recovered_cameras.size(), 1u);

    // Verify camera frame dimensions and field preservation.
    const auto& recovered_cf = *recovered_cameras[0];
    EXPECT_EQ(recovered_cf.w, 640u);
    EXPECT_EQ(recovered_cf.h, 480u);
    EXPECT_TRUE(recovered_cf.has_field("image"));

    // Verify each camera's CameraInfo was stored in the OSF footer via a
    // dedicated CameraSensor entry (distinct from the lidar LidarSensor type)
    // that the CameraFrameStream references by id.
    Reader reader(output_osf_filename);
    auto camera_streams = reader.meta_store().find<CameraFrameStreamMeta>();
    ASSERT_EQ(camera_streams.size(), 2u);
    std::map<uint32_t, ouster::sdk::core::CameraInfo> recovered_camera_infos;
    for (const auto& kv : camera_streams) {
        auto sensor = reader.meta_store().get<CameraSensor>(kv.second->sensor_meta_id());
        ASSERT_NE(sensor, nullptr);
        auto ci = std::dynamic_pointer_cast<ouster::sdk::core::CameraInfo>(sensor->sensor_info());
        ASSERT_NE(ci, nullptr);
        recovered_camera_infos.emplace(kv.first, *ci);
    }
    // Both cameras should be present.
    EXPECT_EQ(recovered_camera_infos.size(), 2u);
    // Round-trip via JSON equality (avoiding sensitivity to enumeration
    // order in the map).
    bool saw_a = false;
    bool saw_b = false;
    for (const auto& kv : recovered_camera_infos) {
        if (kv.second == cam_info_a) saw_a = true;
        if (kv.second == cam_info_b) saw_b = true;
    }
    EXPECT_TRUE(saw_a);
    EXPECT_TRUE(saw_b);
}

TEST_F(WriterTest, WriteCollationInterleavedLidarCameraTest) {
    // The canonical per-slot ordering is reconstructed on read from the sensor
    // footer registration order (stream_ids is no longer stored). The hard case
    // is a camera registered *between* two lidars: frame-stream metadata ids
    // don't follow registration order (lidar streams are created lazily, camera
    // streams eagerly), so ordering must come from the footer entries. This test
    // registers [lidar, camera, lidar] and verifies the interleaved order
    // round-trips positionally.
    const auto info1 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const auto info2 = core::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"));

    auto ls1 = std::make_shared<LidarFrame>(get_random_lidar_frame(info1));
    auto ls2 = std::make_shared<LidarFrame>(get_random_lidar_frame(info2));

    ouster::sdk::core::CameraInfo cam_info;
    cam_info.model = ouster::sdk::core::CameraModel(500.0f, 501.0f, 320.0f, 240.0f);

    auto cf = std::make_shared<core::CameraFrame>();
    cf->w = 640;
    cf->h = 480;
    cf->add_field("image", fd_array<uint8_t>(480, 640), FieldClass::PIXEL_FIELD);
    cf->timestamp()[0] = 1500;
    cf->receive_timestamp()[0] = 1500;

    // Registration order: lidar, camera, lidar (interleaved).
    std::string output_osf_filename = tmp_file("writer_collation_interleaved.osf");
    Writer writer(output_osf_filename);
    writer.add_sensor(info1);
    writer.add_sensor(cam_info);
    writer.add_sensor(info2);

    // Collation frame order must match the registration order above.
    FrameSet collation(std::vector<std::shared_ptr<core::Frame>>{ls1, cf, ls2});
    writer.save(collation);
    writer.close();

    OsfFrameSetSource source(output_osf_filename);
    EXPECT_TRUE(source.is_collated());

    // sensor_info() must be exposed in the interleaved registration order too.
    const auto& infos = source.sensor_info();
    ASSERT_EQ(infos.size(), 3u);
    EXPECT_NE(std::dynamic_pointer_cast<ouster::sdk::core::LidarInfo>(infos[0]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<ouster::sdk::core::CameraInfo>(infos[1]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<ouster::sdk::core::LidarInfo>(infos[2]), nullptr);

    std::vector<FrameSet> recovered_all;
    for (auto fs : source) {
        recovered_all.push_back(std::move(fs));
    }
    ASSERT_EQ(recovered_all.size(), 1u);
    const auto& recovered = recovered_all.front();

    // Positional order preserved: [lidar, camera, lidar].
    ASSERT_EQ(recovered.size(), 3u);
    EXPECT_NE(std::dynamic_pointer_cast<LidarFrame>(recovered[0]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<core::CameraFrame>(recovered[1]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<LidarFrame>(recovered[2]), nullptr);
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

std::shared_ptr<core::CameraFrame> make_recovery_camera_frame(size_t index, size_t height,
                                                              size_t width) {
    auto camera_frame = std::make_shared<core::CameraFrame>();
    camera_frame->w = width;
    camera_frame->h = height;
    camera_frame->add_field("image", fd_array<uint8_t>(height, width), FieldClass::PIXEL_FIELD);
    camera_frame->timestamp()[0] = 1000 * (index + 1);
    camera_frame->receive_timestamp()[0] = camera_frame->timestamp()[0];
    return camera_frame;
}

// The metadata json of each frame read from source, or an empty string for any
// frame that is not a camera frame carrying metadata.
std::vector<std::string> read_camera_frame_infos(OsfFrameSetSource& source) {
    std::vector<std::string> infos;
    for (auto frame_set : source) {
        for (const auto& frame : frame_set.valid_frames()) {
            auto camera_frame = std::dynamic_pointer_cast<core::CameraFrame>(frame);
            if (camera_frame && camera_frame->sensor_info()) {
                infos.push_back(camera_frame->sensor_info()->to_json_string());
            } else {
                infos.emplace_back();
            }
        }
    }
    return infos;
}

// A camera OSF whose metadata footer never reached disk still recovers its
// CameraInfo, because save_camera_internal() duplicates it into the data
// section on the first frame the same way the lidar path does.
TEST_F(WriterTest, CameraMetadataRecoveredWithoutFooter) {
    const size_t height = 480;
    const size_t width = 640;

    ouster::sdk::core::CameraInfo cam_info;
    cam_info.model = ouster::sdk::core::CameraModel(500.0f, 501.0f, 320.0f, 240.0f);
    cam_info.format.columns_per_frame = static_cast<uint32_t>(width);
    cam_info.format.pixels_per_column = static_cast<uint32_t>(height);
    cam_info.sn = 987654;

    const std::string output_osf_filename = tmp_file("writer_camera_recovery.osf");
    {
        Writer writer(output_osf_filename);
        writer.add_sensor(cam_info);
        for (size_t i = 0; i < 3; i++) {
            writer.save(FrameSet(std::vector<std::shared_ptr<core::Frame>>{
                make_recovery_camera_frame(i, height, width)}));
        }
        writer.close();
    }

    // Drop the footer, as a crash mid-recording would.
    const auto full_size = ouster::sdk::osf::file_size(output_osf_filename);
    ASSERT_GT(full_size, 10u);
    ouster::sdk::osf::truncate_file(output_osf_filename, full_size - 10);

    OsfFrameSetSource source(output_osf_filename);
    const auto& infos = source.sensor_info();
    ASSERT_EQ(infos.size(), 1u);
    auto recovered_info = std::dynamic_pointer_cast<ouster::sdk::core::CameraInfo>(infos[0]);
    ASSERT_NE(recovered_info, nullptr);
    EXPECT_EQ(recovered_info->to_json_string(), cam_info.to_json_string());

    // And the frames read back are associated with that CameraInfo.
    const auto infos_read = read_camera_frame_infos(source);
    ASSERT_FALSE(infos_read.empty());
    for (const auto& info_json : infos_read) {
        EXPECT_EQ(info_json, cam_info.to_json_string());
    }
}

// Decoding a camera message directly attaches its CameraInfo, the way the
// lidar path does. The stream's sensor id refers to a CameraSensor footer
// entry, so looking it up as a LidarSensor finds nothing and silently leaves
// the frame without metadata. OsfFrameSetSource attaches metadata itself and
// so would hide that; this reads the messages straight from the Reader.
TEST_F(WriterTest, DecodedCameraFrameCarriesItsCameraInfo) {
    const size_t height = 480;
    const size_t width = 640;

    ouster::sdk::core::CameraInfo cam_info;
    cam_info.format.columns_per_frame = static_cast<uint32_t>(width);
    cam_info.format.pixels_per_column = static_cast<uint32_t>(height);
    cam_info.sn = 13579;

    const std::string output_osf_filename = tmp_file("writer_camera_decode_msg.osf");
    {
        Writer writer(output_osf_filename);
        writer.add_sensor(cam_info);
        writer.save(FrameSet(std::vector<std::shared_ptr<core::Frame>>{
            make_recovery_camera_frame(0, height, width)}));
        writer.close();
    }

    Reader reader(output_osf_filename);
    size_t decoded = 0;
    for (const auto& msg : reader.messages()) {
        if (!msg.is<CameraFrameStream>()) {
            continue;
        }
        auto camera_frame = msg.decode_msg<CameraFrameStream>();
        ASSERT_NE(camera_frame, nullptr);
        ASSERT_NE(camera_frame->sensor_info(), nullptr);
        EXPECT_EQ(camera_frame->sensor_info()->to_json_string(), cam_info.to_json_string());
        ++decoded;
    }
    EXPECT_EQ(decoded, 1u);
}

}  // namespace
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
