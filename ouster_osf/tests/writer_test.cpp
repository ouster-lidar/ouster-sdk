/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/writer.h"

#include <gtest/gtest.h>

#include <string>

#include "common.h"
#include "osf_test.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/file.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {
namespace {

using ouster::osf::get_random_lidar_scan;
using ouster::sensor::sensor_info;

class WriterTest : public osf::OsfTestWithDataAndFiles {};

TEST_F(WriterTest, ChunksLayoutEnum) {
    ChunksLayout cl = ChunksLayout::LAYOUT_STANDARD;
    EXPECT_EQ(to_string(cl), "STANDARD");
    ChunksLayout cl1{};
    EXPECT_EQ(to_string(cl1), "STANDARD");
    ChunksLayout cl2{LAYOUT_STREAMING};
    EXPECT_EQ(to_string(cl2), "STREAMING");
    EXPECT_EQ(chunks_layout_of_string("STREAMING"), LAYOUT_STREAMING);
    EXPECT_EQ(chunks_layout_of_string("STANDARD"), LAYOUT_STANDARD);
    EXPECT_EQ(chunks_layout_of_string("RRR"), LAYOUT_STANDARD);
}

TEST_F(WriterTest, WriteSingleLidarScan) {
    const sensor_info sinfo = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(sinfo);

    std::string output_osf_filename = tmp_file("writer_simple.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarScan
    Writer writer(output_osf_filename);
    writer.set_metadata_id("test_session");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::LAYOUT_STREAMING);

    writer.add_sensor(sinfo);
    writer.save(0, ls, ts_t{123});
    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);

    EXPECT_EQ(++msg_it, reader.messages().end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its sensor_info
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo_str);

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteLidarSensorWithExtrinsics) {
    sensor_info sinfo = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    std::string output_osf_filename =
        tmp_file("writer_lidar_sensor_extrinsics.osf");

    std::string sinfo_str = sinfo.to_json_string();

    sinfo.extrinsic(0, 3) = 10.0;
    sinfo.extrinsic(0, 1) = 0.756;
    sinfo.extrinsic(1, 0) = 0.756;
    sinfo.extrinsic(0, 0) = 0.0;

    // Writing LidarSensor
    Writer writer(output_osf_filename);

    auto sensor_meta_id = writer.add_metadata<LidarSensor>(sinfo_str);
    EXPECT_TRUE(sensor_meta_id != 0);

    writer.add_metadata<Extrinsics>(sinfo.extrinsic, sensor_meta_id);

    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);

    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its sensor_info
    auto sinfo_recovered = sensors.begin()->second->info();

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);

    auto extrinsics = reader.meta_store().find<Extrinsics>();
    EXPECT_EQ(extrinsics.size(), 1);
    EXPECT_EQ(extrinsics.begin()->second->repr(),
              "ExtrinsicsMeta: ref_id = 1, name = , extrinsics = 0 0.756 0 10 "
              "0.756 1 0 0 0 0 1 0 0 0 0 1");

    auto ext_mat_recovered = extrinsics.begin()->second->extrinsics();
    EXPECT_EQ(sinfo.extrinsic, ext_mat_recovered);
    EXPECT_EQ(sensor_meta_id, extrinsics.begin()->second->ref_meta_id());
}

TEST_F(WriterTest, WriteSingleLidarScanStreamingLayout) {
    const sensor_info sinfo = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(sinfo);

    std::string output_osf_filename = tmp_file("writer_simple_streaming.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarScan
    Writer writer(output_osf_filename);
    writer.set_metadata_id("test_session");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::LAYOUT_STREAMING);

    writer.add_sensor(sinfo);
    writer.save(0, ls, ts_t{123});
    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    // TODO[pb]: Add reader validation CRC

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);

    EXPECT_EQ(++msg_it, reader.messages().end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Check that it's an OSF with StreamingLayout
    EXPECT_EQ(1, reader.meta_store().count<osf::StreamingInfo>());
    auto streaming_info = reader.meta_store().get<osf::StreamingInfo>();

    EXPECT_TRUE(streaming_info != nullptr);

    // One stream LidarScanStream
    EXPECT_EQ(1, streaming_info->stream_stats().size());

    // Use first sensor and get its sensor_info
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteSlicedLidarScan) {
    const sensor_info sinfo = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(sinfo);

    // Subset of fields to leave in LidarScan
    ouster::LidarScanFieldTypes field_types;
    field_types.emplace_back(ls.field_type(sensor::ChanField::RANGE));
    field_types.emplace_back(ls.field_type(sensor::ChanField::REFLECTIVITY));

    // Make a reduced field LidarScan
    ls = slice_with_cast(ls, field_types);

    EXPECT_EQ(field_types.size(), ls.field_types().size());

    std::string output_osf_filename = tmp_file("writer_sliced.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarScan
    Writer writer(output_osf_filename, sinfo, {"RANGE", "REFLECTIVITY"});
    writer.set_metadata_id("test_session");
    writer.save(0, ls, ts_t{123});
    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_EQ(field_types.size(), ls_recovered->field_types().size());

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);

    EXPECT_EQ(++msg_it, reader.messages().end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its sensor_info
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteSlicedLegacyLidarScan) {
    const sensor_info sinfo = sensor::metadata_from_json(path_concat(
        test_data_dir(), "metadata/2_5_0_os-992146000760-128_legacy.json"));
    LidarScan ls_orig = get_random_lidar_scan(sinfo);

    // Subset of fields to leave in LidarScan during writing
    LidarScanFieldTypes field_types;
    field_types.emplace_back(sensor::ChanField::RANGE,
                             sensor::ChanFieldType::UINT32);
    field_types.emplace_back(sensor::ChanField::SIGNAL,
                             sensor::ChanFieldType::UINT16);
    field_types.emplace_back(sensor::ChanField::REFLECTIVITY,
                             sensor::ChanFieldType::UINT8);

    std::cout << "LidarScan field_types: " << ouster::to_string(field_types)
              << std::endl;

    // Make a reduced/extended fields LidarScan
    // that will be compared with a recovered LidarScan from OSF
    auto ls_reference = slice_with_cast(ls_orig, field_types);

    EXPECT_EQ(field_types.size(), ls_reference.field_types().size());

    std::string output_osf_filename = tmp_file("writer_sliced_legacy.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarScan with custom field types
    Writer writer(output_osf_filename, sinfo,
                  {"RANGE", "SIGNAL", "REFLECTIVITY"});
    writer.set_metadata_id("test_session");

    writer.save(0, ls_orig, ts_t{123});
    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_TRUE(ls_recovered);

    EXPECT_EQ(field_types.size(), ls_recovered->field_types().size());

    EXPECT_EQ(*ls_recovered, ls_reference);

    EXPECT_EQ(++msg_it, reader.messages().end());

    // Map of all MetadataEntries of type LidarSensor
    auto sensors = reader.meta_store().find<LidarSensor>();
    EXPECT_EQ(sensors.size(), 1);

    // Use first sensor and get its sensor_info
    auto sinfo_recovered = sensors.begin()->second->info();
    EXPECT_EQ(sinfo_recovered.to_json_string(), sinfo.to_json_string());

    auto metadata_recovered = sensors.begin()->second->metadata();
    EXPECT_EQ(metadata_recovered, sinfo_str);
}

TEST_F(WriterTest, WriteCustomLidarScanWithFlags) {
    const sensor_info sinfo = sensor::metadata_from_json(path_concat(
        test_data_dir(), "metadata/3_0_1_os-122246000293-128_legacy.json"));

    LidarScanFieldTypes field_types_with_flags;
    field_types_with_flags.emplace_back(sensor::ChanField::RANGE,
                                        sensor::ChanFieldType::UINT32);
    field_types_with_flags.emplace_back(sensor::ChanField::SIGNAL,
                                        sensor::ChanFieldType::UINT16);
    field_types_with_flags.emplace_back(sensor::ChanField::RANGE2,
                                        sensor::ChanFieldType::UINT32);
    field_types_with_flags.emplace_back(sensor::ChanField::SIGNAL2,
                                        sensor::ChanFieldType::UINT16);
    field_types_with_flags.emplace_back(sensor::ChanField::REFLECTIVITY,
                                        sensor::ChanFieldType::UINT8);
    field_types_with_flags.emplace_back(sensor::ChanField::NEAR_IR,
                                        sensor::ChanFieldType::UINT16);
    field_types_with_flags.emplace_back(sensor::ChanField::FLAGS,
                                        sensor::ChanFieldType::UINT8);
    field_types_with_flags.emplace_back(sensor::ChanField::FLAGS2,
                                        sensor::ChanFieldType::UINT8);
    field_types_with_flags.emplace_back("CUSTOM0",
                                        sensor::ChanFieldType::UINT64);
    field_types_with_flags.emplace_back("CUSTOM7",
                                        sensor::ChanFieldType::UINT16);

    LidarScan ls = get_random_lidar_scan(sinfo.format.columns_per_frame,
                                         sinfo.format.pixels_per_column,
                                         field_types_with_flags);

    std::cout << "LidarScan field_types_with_flags: "
              << ouster::to_string(field_types_with_flags) << std::endl;

    // Check that we have non zero FLAGS
    img_t<uint8_t> flags{ls.h, ls.w};
    impl::visit_field(ls, sensor::ChanField::FLAGS,
                      ouster::impl::read_and_cast(), flags);
    EXPECT_FALSE((flags == 0).all());
    // and non zero FLAGS2
    impl::visit_field(ls, sensor::ChanField::FLAGS2,
                      ouster::impl::read_and_cast(), flags);
    EXPECT_FALSE((flags == 0).all());

    // Check that we have non zero CUSTOM7
    img_t<uint16_t> custom{ls.h, ls.w};
    impl::visit_field(ls, "CUSTOM7", ouster::impl::read_and_cast(), custom);
    EXPECT_FALSE((custom == 0).all());

    EXPECT_EQ(field_types_with_flags.size(), ls.field_types().size());

    std::string output_osf_filename = tmp_file("writer_with_flags.osf");

    // Writing LidarScan
    Writer writer(output_osf_filename, sinfo);
    writer.set_metadata_id("test_session");
    writer.save(0, ls, ts_t{123});
    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_TRUE(ls_recovered);

    EXPECT_EQ(field_types_with_flags.size(),
              ls_recovered->field_types().size());

    EXPECT_EQ(*ls_recovered, ls);

    EXPECT_EQ(++msg_it, reader.messages().end());
}

// Used in WriteExample test below
void ReadExample(const std::string filename) {
    // Open output: OSF v2
    OsfFile file(filename);
    Reader reader(file);

    // Read all messages from OSF file
    for (const auto m : reader.messages()) {
        auto ts = m.ts();              // << message timestamp
        auto stream_meta_id = m.id();  // << link to the stream meta

        EXPECT_GT(ts.count(), 0);
        EXPECT_GT(stream_meta_id, uint32_t{0});

        // Decoding messages
        if (m.is<LidarScanStream>()) {
            auto ls = m.decode_msg<LidarScanStream>();
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
    // Get sensor_info
    const sensor_info sinfo = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    std::string output_osf_filename = tmp_file("write_example.osf");

    // Create OSF v2 Writer
    osf::Writer writer(output_osf_filename);
    writer.set_metadata_id("Example Session 1234");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::LAYOUT_STREAMING);

    // Create LidarSensor record
    auto sensor_id = writer.add_sensor(sinfo);

    const int LOOP_CNT = 7;

    int timestamp = 0;
    while (timestamp++ < LOOP_CNT) {
        LidarScan ls = get_random_lidar_scan(sinfo);

        // Save LidarScan
        writer.save(sensor_id, ls, ts_t{timestamp});
    }

    writer.close();

    // Quick test the we have result file
    OsfFile result(output_osf_filename);
    EXPECT_TRUE(result.good());

    // Quick test that number of messages in result file is what we wrote
    Reader reader(result);
    EXPECT_EQ(LOOP_CNT, std::distance(reader.messages().begin(),
                                      reader.messages().end()));

    // Check that it's an OSF with StreamingLayout
    auto streaming_info_entry = reader.meta_store().find<StreamingInfo>();
    EXPECT_EQ(1, streaming_info_entry.size());
    auto streaming_info = streaming_info_entry.begin()->second;

    // std::cout << "streaming_info = " << streaming_info->to_string() <<
    // std::endl;

    // One stream: LidarScanStream
    EXPECT_EQ(1, streaming_info->stream_stats().size());

    EXPECT_TRUE(reader.has_message_idx());

    auto lsm = reader.meta_store().get<LidarScanStreamMeta>();

    auto stream_msg_count =
        streaming_info->stream_stats()[lsm->id()].message_count;
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
        auto first_msg =
            reader.messages({lsm->id()}, *msg_ts, reader.end_ts()).begin();
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
    const sensor_info sinfo = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(sinfo);
    auto& f1 =
        ls.add_field("custom_field_1", fd_array<double>(85, 129, 344), {});
    auto& f2 = ls.add_field("custom_field_2", fd_array<uint16_t>(111, 333), {});

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> nd_d{100.0, 10.0};
    std::uniform_int_distribution<uint16_t> ud_u8{0, 150};
    randomize_field<double>(f1, gen, nd_d);
    randomize_field<uint16_t>(f2, gen, ud_u8);

    std::string output_osf_filename = tmp_file("writer_simple.osf");

    std::string sinfo_str = sinfo.to_json_string();

    // Writing LidarScan
    Writer writer(output_osf_filename);
    writer.set_metadata_id("test_session");
    EXPECT_EQ(writer.chunks_layout(), ChunksLayout::LAYOUT_STREAMING);

    writer.add_sensor(sinfo);
    writer.save(0, ls, ts_t{123});
    writer.close();

    OsfFile osf_file(output_osf_filename);
    EXPECT_TRUE(osf_file.good());

    Reader reader(osf_file);
    EXPECT_EQ(reader.metadata_id(), "test_session");

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);
}

}  // namespace
}  // namespace osf
}  // namespace ouster
