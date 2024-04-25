/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

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
#include "ouster/osf/writer.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {
namespace {

using ouster::osf::get_random_lidar_scan;
using ouster::sensor::sensor_info;

class WriterV2Test : public osf::OsfTestWithDataAndFiles {};

TEST_F(WriterV2Test, WriterV2AccessorTest) {
    const int chunk_size = 1234;
    std::string output_osf_filename = tmp_file("WriterV2AccessorTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const sensor::sensor_info info2 = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"));
    {
        std::vector<ouster::sensor::sensor_info> info_compare = {info};
        Writer writer(output_osf_filename, info, LidarScanFieldTypes(),
                      chunk_size);
        EXPECT_EQ(writer.chunk_size(), chunk_size);
        EXPECT_EQ(writer.sensor_info_count(), 1);
        EXPECT_EQ(writer.filename(), output_osf_filename);
        EXPECT_EQ(writer.sensor_info(), info_compare);
        EXPECT_EQ(writer.sensor_info(0), info);
    }
    {
        std::vector<ouster::sensor::sensor_info> info_compare = {info, info2};
        Writer writer(output_osf_filename, info_compare, LidarScanFieldTypes(),
                      chunk_size);
        EXPECT_EQ(writer.sensor_info_count(), 2);

        EXPECT_EQ(writer.sensor_info(), info_compare);
        EXPECT_EQ(writer.sensor_info(0), info);
        EXPECT_EQ(writer.sensor_info(1), info2);
    }
}

TEST_F(WriterV2Test, WriterV2BoundingTest) {
    const int chunk_size = 1234;
    std::string output_osf_filename = tmp_file("WriterV2BoundingTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    Writer writer(output_osf_filename, info, LidarScanFieldTypes(), chunk_size);

    bool caught = false;
    try {
        LidarScan one;
        writer.save(1, one);
    } catch (const std::logic_error& e) {
        EXPECT_EQ(std::string(e.what()), "ERROR: Bad Stream ID");
        caught = true;
    } catch (...) {
        FAIL();
    }
    EXPECT_TRUE(caught);
    caught = false;
    try {
        LidarScan one;
        LidarScan two;
        writer.save({one, two});
    } catch (const std::logic_error& e) {
        EXPECT_EQ(std::string(e.what()),
                  "ERROR: Scans passed in to writer "
                  "does not match number of sensor infos");
        caught = true;
    } catch (...) {
        FAIL();
    }
    EXPECT_TRUE(caught);
}

TEST_F(WriterV2Test, WriterV2CloseTest) {
    std::string output_osf_filename = tmp_file("WriterV2CloseTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(info);
    Writer writer(output_osf_filename, info);
    writer.save(0, ls);
    EXPECT_FALSE(writer.is_closed());
    writer.close();
    EXPECT_TRUE(writer.is_closed());
    bool caught = false;
    try {
        writer.save({ls});
    } catch (const std::logic_error& e) {
        EXPECT_EQ(std::string(e.what()), "ERROR: Writer is closed");
        caught = true;
    } catch (...) {
        FAIL();
    }
    EXPECT_TRUE(caught);
}

void test_single_file(std::string& output_osf_filename, LidarScan& ls) {
    Reader reader(output_osf_filename);

    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());

    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();

    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);

    EXPECT_EQ(++msg_it, reader.messages().end());
}

TEST_F(WriterV2Test, WriterV2SingleIndexedTest) {
    std::string output_osf_filename = tmp_file("WriterV2SingleIndexedTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(info);
    {
        Writer writer(output_osf_filename, info);
        writer.save(0, ls);
    }
    test_single_file(output_osf_filename, ls);
}

TEST_F(WriterV2Test, WriterV2SingleVectorTest) {
    std::string output_osf_filename = tmp_file("WriterV2SingleVectorTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(info);
    {
        Writer writer(output_osf_filename, info);
        writer.save({ls});
    }
    test_single_file(output_osf_filename, ls);
}

void test_multi_file(std::string& output_osf_filename, LidarScan& ls,
                     LidarScan& ls2) {
    Reader reader(output_osf_filename);
    auto msg_it = reader.messages().begin();
    EXPECT_NE(msg_it, reader.messages().end());
    auto ls_recovered = msg_it->decode_msg<LidarScanStream>();
    EXPECT_TRUE(ls_recovered);
    EXPECT_EQ(*ls_recovered, ls);
    EXPECT_NE(++msg_it, reader.messages().end());
    auto ls_recovered2 = msg_it->decode_msg<LidarScanStream>();
    EXPECT_TRUE(ls_recovered2);
    EXPECT_EQ(*ls_recovered2, ls2);
    EXPECT_EQ(++msg_it, reader.messages().end());
}

TEST_F(WriterV2Test, WriterV2MultiIndexedTest) {
    std::string output_osf_filename = tmp_file("WriterV2MultiIndexedTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const sensor::sensor_info info2 = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"));

    LidarScan ls = get_random_lidar_scan(info);
    LidarScan ls2 = get_random_lidar_scan(info2);
    {
        Writer writer(output_osf_filename, {info, info2});
        writer.save(0, ls);
        writer.save(1, ls2);
    }
    test_multi_file(output_osf_filename, ls2, ls);
}

TEST_F(WriterV2Test, WriterV2MultiVectorTest) {
    std::string output_osf_filename = tmp_file("WriterV2MultiVectorTest.osf");
    const sensor::sensor_info info = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    const sensor::sensor_info info2 = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"));

    LidarScan ls = get_random_lidar_scan(info);
    LidarScan ls2 = get_random_lidar_scan(info2);
    {
        Writer writer(output_osf_filename, {info, info2});
        writer.save({ls, ls2});
    }
    test_multi_file(output_osf_filename, ls2, ls);
}
}  // namespace
}  // namespace osf
}  // namespace ouster
