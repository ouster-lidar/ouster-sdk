/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/reader.h"

#include <gtest/gtest.h>

#include "common.h"
#include "osf_test.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/stream_lidar_scan.h"

namespace ouster {
namespace osf {
namespace {

class ReaderTest : public osf::OsfTestWithData {};

TEST_F(ReaderTest, Basics) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    EXPECT_EQ("from_pcap pythonic", reader.metadata_id());
    EXPECT_EQ(991587364520LL, reader.start_ts().count());
    EXPECT_EQ(991787323080LL, reader.end_ts().count());

    // Get first sensor (it's the first by metadata_id) (i.e. first added)
    auto sensor = reader.meta_store().get<osf::LidarSensor>();
    EXPECT_TRUE(sensor);
    EXPECT_EQ(
        sensor->to_string(),
        "{\n  \"sensor_info\": \n  {\n    \"base_pn\": \"\",\n    \"base_sn\": "
        "\"\",\n    \"beam_altitude_angles\": \n    [\n      20.95,\n      "
        "20.67,\n      20.36,\n      20.03,\n      19.73,\n      19.41,\n      "
        "19.11,\n      18.76,\n      18.47,\n      18.14,\n      17.82,\n      "
        "17.5,\n      17.19,\n      16.86,\n      16.53,\n      16.2,\n      "
        "15.89,\n      15.56,\n      15.23,\n      14.9,\n      14.57,\n      "
        "14.23,\n      13.9,\n      13.57,\n      13.25,\n      12.91,\n      "
        "12.57,\n      12.22,\n      11.9,\n      11.55,\n      11.2,\n      "
        "10.87,\n      10.54,\n      10.18,\n      9.84,\n      9.51,\n      "
        "9.15,\n      8.81,\n      8.47,\n      8.11,\n      7.78,\n      "
        "7.43,\n      7.08,\n      6.74,\n      6.39,\n      6.04,\n      "
        "5.7,\n      5.34,\n      4.98,\n      4.64,\n      4.29,\n      "
        "3.93,\n      3.58,\n      3.24,\n      2.88,\n      2.53,\n      "
        "2.17,\n      1.82,\n      1.47,\n      1.12,\n      0.78,\n      "
        "0.41,\n      0.07,\n      -0.28,\n      -0.64,\n      -0.99,\n      "
        "-1.35,\n      -1.7,\n      -2.07,\n      -2.4,\n      -2.75,\n      "
        "-3.11,\n      -3.46,\n      -3.81,\n      -4.15,\n      -4.5,\n      "
        "-4.86,\n      -5.22,\n      -5.57,\n      -5.9,\n      -6.27,\n      "
        "-6.61,\n      -6.97,\n      -7.3,\n      -7.67,\n      -8.01,\n      "
        "-8.35,\n      -8.69,\n      -9.05,\n      -9.38,\n      -9.71,\n      "
        "-10.07,\n      -10.42,\n      -10.76,\n      -11.09,\n      -11.43,\n "
        "     -11.78,\n      -12.12,\n      -12.46,\n      -12.78,\n      "
        "-13.15,\n      -13.46,\n      -13.8,\n      -14.12,\n      -14.48,\n  "
        "    -14.79,\n      -15.11,\n      -15.46,\n      -15.79,\n      "
        "-16.12,\n      -16.45,\n      -16.76,\n      -17.11,\n      -17.44,\n "
        "     -17.74,\n      -18.06,\n      -18.39,\n      -18.72,\n      "
        "-19.02,\n      -19.32,\n      -19.67,\n      -19.99,\n      -20.27,\n "
        "     -20.57,\n      -20.92,\n      -21.22,\n      -21.54,\n      "
        "-21.82\n    ],\n    \"beam_azimuth_angles\": \n    [\n      4.21,\n   "
        "   1.41,\n      -1.4,\n      -4.22,\n      4.22,\n      1.41,\n      "
        "-1.4,\n      -4.23,\n      4.21,\n      1.4,\n      -1.42,\n      "
        "-4.2,\n      4.22,\n      1.41,\n      -1.4,\n      -4.23,\n      "
        "4.21,\n      1.41,\n      -1.41,\n      -4.21,\n      4.22,\n      "
        "1.4,\n      -1.41,\n      -4.2,\n      4.22,\n      1.42,\n      "
        "-1.4,\n      -4.2,\n      4.22,\n      1.41,\n      -1.42,\n      "
        "-4.21,\n      4.22,\n      1.41,\n      -1.4,\n      -4.21,\n      "
        "4.2,\n      1.4,\n      -1.4,\n      -4.22,\n      4.21,\n      "
        "1.41,\n      -1.41,\n      -4.21,\n      4.22,\n      1.41,\n      "
        "-1.4,\n      -4.21,\n      4.21,\n      1.41,\n      -1.4,\n      "
        "-4.21,\n      4.2,\n      1.41,\n      -1.4,\n      -4.21,\n      "
        "4.2,\n      1.4,\n      -1.41,\n      -4.21,\n      4.22,\n      "
        "1.4,\n      -1.4,\n      -4.21,\n      4.22,\n      1.42,\n      "
        "-1.4,\n      -4.2,\n      4.2,\n      1.42,\n      -1.4,\n      "
        "-4.22,\n      4.22,\n      1.41,\n      -1.4,\n      -4.2,\n      "
        "4.23,\n      1.41,\n      -1.4,\n      -4.2,\n      4.21,\n      "
        "1.41,\n      -1.4,\n      -4.21,\n      4.21,\n      1.41,\n      "
        "-1.4,\n      -4.21,\n      4.22,\n      1.41,\n      -1.39,\n      "
        "-4.21,\n      4.23,\n      1.41,\n      -1.39,\n      -4.22,\n      "
        "4.23,\n      1.4,\n      -1.4,\n      -4.2,\n      4.21,\n      "
        "1.41,\n      -1.41,\n      -4.2,\n      4.22,\n      1.42,\n      "
        "-1.39,\n      -4.22,\n      4.24,\n      1.41,\n      -1.41,\n      "
        "-4.22,\n      4.23,\n      1.41,\n      -1.39,\n      -4.21,\n      "
        "4.23,\n      1.41,\n      -1.39,\n      -4.2,\n      4.23,\n      "
        "1.4,\n      -1.39,\n      -4.2,\n      4.22,\n      1.42,\n      "
        "-1.39,\n      -4.2\n    ],\n    \"build_date\": "
        "\"2022-04-14T21:11:47Z\",\n    \"build_rev\": \"v2.3.0\",\n    "
        "\"client_version\": \"ouster_client 0.3.0\",\n    \"data_format\": \n "
        "   {\n      \"column_window\": \n      [\n        0,\n        1023\n  "
        "    ],\n      \"columns_per_frame\": 1024,\n      "
        "\"columns_per_packet\": 16,\n      \"pixel_shift_by_row\": \n      "
        "[\n        24,\n        16,\n        8,\n        0,\n        24,\n    "
        "    16,\n        8,\n        0,\n        24,\n        16,\n        "
        "8,\n        0,\n        24,\n        16,\n        8,\n        0,\n    "
        "    24,\n        16,\n        8,\n        0,\n        24,\n        "
        "16,\n        8,\n        0,\n        24,\n        16,\n        8,\n   "
        "     0,\n        24,\n        16,\n        8,\n        0,\n        "
        "24,\n        16,\n        8,\n        0,\n        24,\n        16,\n  "
        "      8,\n        0,\n        24,\n        16,\n        8,\n        "
        "0,\n        24,\n        16,\n        8,\n        0,\n        24,\n   "
        "     16,\n        8,\n        0,\n        24,\n        16,\n        "
        "8,\n        0,\n        24,\n        16,\n        8,\n        0,\n    "
        "    24,\n        16,\n        8,\n        0,\n        24,\n        "
        "16,\n        8,\n        0,\n        24,\n        16,\n        8,\n   "
        "     0,\n        24,\n        16,\n        8,\n        0,\n        "
        "24,\n        16,\n        8,\n        0,\n        24,\n        16,\n  "
        "      8,\n        0,\n        24,\n        16,\n        8,\n        "
        "0,\n        24,\n        16,\n        8,\n        0,\n        24,\n   "
        "     16,\n        8,\n        0,\n        24,\n        16,\n        "
        "8,\n        0,\n        24,\n        16,\n        8,\n        0,\n    "
        "    24,\n        16,\n        8,\n        0,\n        24,\n        "
        "16,\n        8,\n        0,\n        24,\n        16,\n        8,\n   "
        "     0,\n        24,\n        16,\n        8,\n        0,\n        "
        "24,\n        16,\n        8,\n        0,\n        24,\n        16,\n  "
        "      8,\n        0\n      ],\n      \"pixels_per_column\": 128,\n    "
        "  \"udp_profile_imu\": \"LEGACY\",\n      \"udp_profile_lidar\": "
        "\"RNG15_RFL8_NIR8\"\n    },\n    \"hostname\": \"\",\n    "
        "\"image_rev\": \"ousteros-image-prod-aries-v2.3.0+20220415163956\",\n "
        "   \"imu_to_sensor_transform\": \n    [\n      1,\n      0,\n      "
        "0,\n      6.253,\n      0,\n      1,\n      0,\n      -11.775,\n      "
        "0,\n      0,\n      1,\n      7.645,\n      0,\n      0,\n      0,\n  "
        "    1\n    ],\n    \"initialization_id\": 7109750,\n    "
        "\"json_calibration_version\": 4,\n    \"lidar_mode\": \"1024x10\",\n  "
        "  \"lidar_origin_to_beam_origin_mm\": 15.806,\n    "
        "\"lidar_to_sensor_transform\": \n    [\n      -1,\n      0,\n      "
        "0,\n      0,\n      0,\n      -1,\n      0,\n      0,\n      0,\n     "
        " 0,\n      1,\n      36.18,\n      0,\n      0,\n      0,\n      1\n  "
        "  ],\n    \"prod_line\": \"OS-1-128\",\n    \"prod_pn\": "
        "\"840-103575-06\",\n    \"prod_sn\": \"122201000998\",\n    "
        "\"proto_rev\": \"\",\n    \"status\": \"RUNNING\",\n    "
        "\"udp_port_imu\": 7503,\n    \"udp_port_lidar\": 7502\n  }\n}");
    EXPECT_EQ(1, reader.meta_store().count<osf::LidarSensor>());

    EXPECT_EQ(
        3, std::distance(reader.messages().begin(), reader.messages().end()));

    const MetadataStore& meta_store = reader.meta_store();
    EXPECT_EQ(3, meta_store.size());
}

TEST_F(ReaderTest, ChunksReading) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    auto chunks = reader.chunks();

    EXPECT_EQ(chunks.to_string(), "ChunksRange: [ba = 0, ea = 1013976]");
    EXPECT_EQ(chunks.begin().to_string(), "ChunksIter: [ca = 0, ea = 1013976]");
    std::cout << chunks.begin()->end_ts().count() << std::endl;
    EXPECT_EQ(chunks.begin()->start_ts(), ts_t(991587364520L));
    EXPECT_EQ(chunks.begin()->end_ts(), ts_t(991787323080L));
    EXPECT_EQ(chunks.begin()->to_string(),
              "ChunkRef: [msgs_size = 3, state = ("
              "{offset = 0, next_offset = 18446744073709551615,"
              " start_ts = 991587364520, end_ts = 991787323080,"
              " status = 1}), chunk_buf_ = nullptr]");
    EXPECT_EQ(((ChunkRef)*chunks.begin())[0].to_string(),
              "MessageRef: [id = 2, ts = 991587364520, buffer = 0c"
              " 2b 05 00 14 00 00 00 10 00 1c 00 04 00 08 00 0c 00"
              " 10 00 14 00 18 00 10 00 00 00 34 38 00 00 24 38 00"
              " 00 18 18 00 00 10 10 00 00 08 00 00 00 03 07 00 00"
              " 00 04 00 00 01 00 00 00 01 00 00 00 01 00 00 00 01"
              " 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00"
              " 00 00 01 00 00 00 01 00 00 00 01 00 00 00 ... and"
              " 338604 more ...]");
    EXPECT_EQ(chunks.begin()->begin().to_string(),
              "MessagesChunkIter: [chunk_ref = ChunkRef:"
              " [msgs_size = 3, state = ({offset = 0,"
              " next_offset = 18446744073709551615,"
              " start_ts = 991587364520, end_ts = 991787323080,"
              " status = 1}), chunk_buf_ = nullptr], msg_idx = 0]");
    EXPECT_EQ(to_string(*chunks.begin()->state()),
              "{offset = 0, next_offset = 18446744073709551615,"
              " start_ts = 991587364520, end_ts = 991787323080, status = 1}");
    EXPECT_EQ(to_string(*chunks.begin()->info()),
              "{offset = 0, next_offset = 18446744073709551615,"
              " stream_id = 2, message_count = 3, message_start_idx = 0}");

    EXPECT_EQ(1, std::distance(chunks.begin(), chunks.end()));

    auto first_chunk_it = chunks.begin();
    EXPECT_EQ(3, first_chunk_it->size());

    EXPECT_EQ(3, std::distance(first_chunk_it->begin(), first_chunk_it->end()));

    auto msg_it = first_chunk_it->begin();
    auto msg0 = *msg_it;
    auto msg1 = *(++msg_it);
    EXPECT_NE(msg0, msg1);

    auto msg00 = *(--msg_it);
    EXPECT_EQ(msg0, msg00);
    EXPECT_EQ(msg0, *(--msg_it));
}

TEST_F(ReaderTest, ChunksPileBasics) {
    ChunkState st{};
    EXPECT_EQ(st.status, ChunkValidity::UNKNOWN);

    ChunksPile cp{};
    cp.add(0, ts_t{1}, ts_t{2});
    cp.add(10, ts_t{10}, ts_t{20});
    EXPECT_FALSE(cp.get(2));
    EXPECT_TRUE(cp.get(0));
    EXPECT_EQ(2, cp.get(0)->end_ts.count());
    EXPECT_EQ(ChunkValidity::UNKNOWN, cp.get(0)->status);

    cp.add(12, ts_t{12}, ts_t{22});
    EXPECT_EQ(3, cp.size());
}

TEST_F(ReaderTest, MessagesReadingStreaming) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    // Reader iterator reads as messages from chunk as they appears on a disk
    int it_cnt = 0;
    ts_t it_prev{0};
    bool it_ordered = true;
    for (const auto msg : reader.messages()) {
        it_ordered = it_ordered && (it_prev <= msg.ts());
        ++it_cnt;
        it_prev = msg.ts();
    }

    EXPECT_EQ(3, it_cnt);

    // only for this test file, ordering by timestamp while reading by chunks
    // order as they layout in the file is not guaranteed
    EXPECT_TRUE(it_ordered);

    // StreammingReader reads StreamingLayout OSFs in timestamp order
    int sit_cnt = 0;
    ts_t sit_prev{0};
    bool sit_ordered = true;
    EXPECT_EQ((*reader.messages().begin()).to_string(),
              "MessageRef: [id = 2, ts = 991587364520, buffer ="
              " 0c 2b 05 00 14 00 00 00 10 00 1c 00 04 00 08 00"
              " 0c 00 10 00 14 00 18 00 10 00 00 00 34 38 00 00"
              " 24 38 00 00 18 18 00 00 10 10 00 00 08 00 00 00"
              " 03 07 00 00 00 04 00 00 01 00 00 00 01 00 00 00"
              " 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00"
              " 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00"
              " 01 00 00 00 ... and 338604 more ...]");
    EXPECT_EQ(reader.messages().begin().to_string(),
              "MessagesStreamingIter: [curr_ts = 991587364520,"
              " end_ts = 991787323081, curr_chunks_.size = 1,"
              " stream_ids_hash_ = 0, top = (ts = 991587364520, id = 2)]");

    for (const auto msg : reader.messages()) {
        sit_ordered = sit_ordered && (sit_prev <= msg.ts());
        ++sit_cnt;
        sit_prev = msg.ts();
    }
    EXPECT_EQ(3, sit_cnt);
    EXPECT_TRUE(sit_ordered);

    EXPECT_EQ(reader.messages().to_string(),
              "MessagesStreamingRange:"
              " [start_ts = 991587364520, end_ts = 991787323080]");
    EXPECT_EQ(
        3, std::distance(reader.messages().begin(), reader.messages().end()));

    // Read time based range of messages from StreamingLayout
    ts_t begin_ts{991587364520LL};
    ts_t end_ts{991587364520LL + 1 * 100'000'000LL};  // start_ts + 1 * 0.1s
    for (const auto msg : reader.messages(begin_ts, end_ts)) {
        EXPECT_TRUE(begin_ts <= msg.ts());
        EXPECT_TRUE(msg.ts() <= end_ts);
    }
    auto some_msgs = reader.messages(begin_ts, end_ts);
    EXPECT_EQ(2, std::distance(some_msgs.begin(), some_msgs.end()));

    // Get any first LidarScan stream from OSF
    auto lidar_scan_stream = reader.meta_store().get<LidarScanStreamMeta>();
    EXPECT_TRUE(lidar_scan_stream);

    // Get a stream of LidarScan messages only
    auto scan_msgs = reader.messages({lidar_scan_stream->id()});
    for (const auto msg : scan_msgs) {
        EXPECT_EQ(lidar_scan_stream->id(), msg.id());
    }
    EXPECT_EQ(3, std::distance(scan_msgs.begin(), scan_msgs.end()));

    // Get a stream of LidarScan messages only with start/end_ts params
    auto scan_msgs_full = reader.messages({lidar_scan_stream->id()},
                                          reader.start_ts(), reader.end_ts());
    EXPECT_EQ(3, std::distance(scan_msgs_full.begin(), scan_msgs_full.end()));
}

TEST_F(ReaderTest, MetadataFromBufferTest) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    auto sensor = reader.meta_store().entries().begin()->second;

    std::vector<uint8_t> buf;
    std::stringstream output_stream;
    std::streambuf* old_output_stream = std::cout.rdbuf();

    std::cout.rdbuf(output_stream.rdbuf());
    auto result = sensor->from_buffer(buf, "NON EXISTENT");
    std::cout.rdbuf(old_output_stream);
    EXPECT_EQ(output_stream.str(), "UNKNOWN TYPE: NON EXISTENT\n");
    EXPECT_EQ(result, nullptr);
    std::cout.rdbuf(old_output_stream);

    result = sensor->from_buffer(sensor->buffer(),
                                 "ouster/v1/os_sensor/LidarSensor");
    EXPECT_NE(result, nullptr);
    EXPECT_EQ(result->id(), 0);
    EXPECT_EQ(result->type(), "ouster/v1/os_sensor/LidarSensor");
}

}  // namespace
}  // namespace osf
}  // namespace ouster
