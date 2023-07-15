/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/operations.h"

#include <gtest/gtest.h>

#include "json_utils.h"
#include "osf_test.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/file.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"

namespace ouster {
namespace osf {
namespace {

class OperationsTest : public OsfTestWithDataAndFiles {};

TEST_F(OperationsTest, GetOsfDumpInfo) {
    std::string osf_info_str = dump_metadata(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
        true);

    Json::Value osf_info_obj{};

    EXPECT_TRUE(parse_json(osf_info_str, osf_info_obj));

    ASSERT_TRUE(osf_info_obj.isMember("header"));
    EXPECT_TRUE(osf_info_obj["header"].isMember("status"));
    EXPECT_TRUE(osf_info_obj["header"].isMember("version"));
    EXPECT_TRUE(osf_info_obj["header"].isMember("size"));
    EXPECT_TRUE(osf_info_obj["header"].isMember("metadata_offset"));
    EXPECT_TRUE(osf_info_obj["header"].isMember("chunks_offset"));

    ASSERT_TRUE(osf_info_obj.isMember("metadata"));
    EXPECT_TRUE(osf_info_obj["metadata"].isMember("id"));
    EXPECT_EQ("from_pcap pythonic", osf_info_obj["metadata"]["id"].asString());
    EXPECT_TRUE(osf_info_obj["metadata"].isMember("start_ts"));
    EXPECT_TRUE(osf_info_obj["metadata"].isMember("end_ts"));
    EXPECT_TRUE(osf_info_obj["metadata"].isMember("entries"));
    EXPECT_EQ(3, osf_info_obj["metadata"]["entries"].size());
}

TEST_F(OperationsTest, ParseAndPrintSmoke) {
    parse_and_print(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));
}

// TODO[pb]: Remove this test and remove PcapRawSource since it's not mathing
// the python impl.
TEST_F(OperationsTest, PcapToOsf) {
    std::string pcap_file = path_concat(
        test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10_lb_n3.pcap");
    std::string meta_file =
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json");

    std::string output_osf_filename = tmp_file("pcap_to_osf_test.osf");

    bool res = pcap_to_osf(pcap_file, meta_file, 7502, output_osf_filename);

    EXPECT_TRUE(res);

    OsfFile output_osf_file{output_osf_filename};
    EXPECT_TRUE(output_osf_file.valid());

    Reader reader{output_osf_file};

    auto msgs_count =
        std::distance(reader.messages().begin(), reader.messages().end());
    EXPECT_EQ(2, msgs_count);
}

}  // namespace

}  // namespace osf
}  // namespace ouster
