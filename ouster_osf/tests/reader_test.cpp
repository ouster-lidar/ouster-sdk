/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/reader.h"

#include <gtest/gtest.h>

#include "common.h"
#include "osf_test.h"
#include "ouster/impl/logging.h"
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

    EXPECT_EQ("ouster_sdk", reader.metadata_id());
    EXPECT_EQ(991587364520LL, reader.start_ts().count());
    EXPECT_EQ(991787323080LL, reader.end_ts().count());

    // Get first sensor (it's the first by metadata_id) (i.e. first added)
    auto sensor = reader.meta_store().get<osf::LidarSensor>();
    EXPECT_TRUE(sensor);
    std::stringstream actual;
    actual
        << "{\"sensor_info\":{\"beam_intrinsics\":{\"beam_altitude_angles\":["
           "20."
           "95,20.67,20.36,20.03,19.73,19.41,19.11,18.76,18.47,18.14,17.82,17."
           "5,"
           "17.19,16.86,16.53,16.2,15.89,15.56,15.23,14.9,14.57,14.23,13.9,13."
           "57,"
           "13.25,12.91,12.57,12.22,11.9,11.55,11.2,10.87,10.54,10.18,9.84,9."
           "51,9."
           "15,8.81,8.47,8.11,7.78,7.43,7.08,6.74,6.39,6.04,5.7,5.34,4.98,4.64,"
           "4."
           "29,3.93,3.58,3.24,2.88,2.53,2.17,1.82,1.47,1.12,0.78,0.41,0.07,-0."
           "28,-"
           "0.64,-0.99,-1.35,-1.7,-2.07,-2.4,-2.75,-3.11,-3.46,-3.81,-4.15,-4."
           "5,-"
           "4.86,-5.22,-5.57,-5.9,-6.27,-6.61,-6.97,-7.3,-7.67,-8.01,-8.35,-8."
           "69,-"
           "9.05,-9.38,-9.71,-10.07,-10.42,-10.76,-11.09,-11.43,-11.78,-12.12,-"
           "12."
           "46,-12.78,-13.15,-13.46,-13.8,-14.12,-14.48,-14.79,-15.11,-15.46,-"
           "15."
           "79,-16.12,-16.45,-16.76,-17.11,-17.44,-17.74,-18.06,-18.39,-18.72,-"
           "19."
           "02,-19.32,-19.67,-19.99,-20.27,-20.57,-20.92,-21.22,-21.54,-21.82],"
           "\"beam_azimuth_angles\":[4.21,1.41,-1.4,-4.22,4.22,1.41,-1.4,-4.23,"
           "4."
           "21,1.4,-1.42,-4.2,4.22,1.41,-1.4,-4.23,4.21,1.41,-1.41,-4.21,4.22,"
           "1.4,"
           "-1.41,-4.2,4.22,1.42,-1.4,-4.2,4.22,1.41,-1.42,-4.21,4.22,1.41,-1."
           "4,-"
           "4.21,4.2,1.4,-1.4,-4.22,4.21,1.41,-1.41,-4.21,4.22,1.41,-1.4,-4.21,"
           "4."
           "21,1.41,-1.4,-4.21,4.2,1.41,-1.4,-4.21,4.2,1.4,-1.41,-4.21,4.22,1."
           "4,-"
           "1.4,-4.21,4.22,1.42,-1.4,-4.2,4.2,1.42,-1.4,-4.22,4.22,1.41,-1.4,-"
           "4.2,"
           "4.23,1.41,-1.4,-4.2,4.21,1.41,-1.4,-4.21,4.21,1.41,-1.4,-4.21,4.22,"
           "1."
           "41,-1.39,-4.21,4.23,1.41,-1.39,-4.22,4.23,1.4,-1.4,-4.2,4.21,1.41,-"
           "1."
           "41,-4.2,4.22,1.42,-1.39,-4.22,4.24,1.41,-1.41,-4.22,4.23,1.41,-1."
           "39,-"
           "4.21,4.23,1.41,-1.39,-4.2,4.23,1.4,-1.39,-4.2,4.22,1.42,-1.39,-4.2]"
           ","
           "\"beam_to_lidar_transform\":[1.0,0.0,0.0,15.806,0.0,1.0,0.0,0.0,0."
           "0,0."
           "0,1.0,0.0,0.0,0.0,0.0,1.0],\"lidar_origin_to_beam_origin_mm\":15."
           "806},"
           "\"calibration_status\":{},\"config_params\":{\"lidar_mode\":"
           "\"1024x10\",\"udp_port_imu\":7503,\"udp_port_lidar\":7502},\"imu_"
           "intrinsics\":{\"imu_to_sensor_transform\":[1.0,0.0,0.0,6.253,0.0,1."
           "0,"
           "0.0,-11.775,0.0,0.0,1.0,7.645,0.0,0.0,0.0,1.0]},\"lidar_data_"
           "format\":"
           "{\"column_window\":[0,1023],\"columns_per_frame\":1024,\"columns_"
           "per_"
           "packet\":16,\"fps\":10,\"pixel_shift_by_row\":[24,16,8,0,24,16,8,0,"
           "24,"
           "16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,"
           "24,"
           "16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,"
           "24,"
           "16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,"
           "24,"
           "16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,24,16,8,0,"
           "24,"
           "16,8,0,24,16,8,0],\"pixels_per_column\":128,\"udp_profile_imu\":"
           "\"LEGACY\",\"udp_profile_lidar\":\"RNG15_RFL8_NIR8\"},\"lidar_"
           "intrinsics\":{\"lidar_to_sensor_transform\":[-1.0,0.0,0.0,0.0,0.0,-"
           "1."
           "0,0.0,0.0,0.0,0.0,1.0,36.18,0.0,0.0,0.0,1.0]},\"ouster-sdk\":{"
           "\"client_version\":\"";
    actual << ouster::sensor::client_version();
    actual
        << "\",\"extrinsic\":[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0."
           "0,0.0,0.0,0.0,1.0],\"output_source\":\"sensor_info_to_string\"},"
           "\"sensor_info\":{\"build_date\":\"2022-04-14T21:11:47Z\",\"build_"
           "rev\":\"v2.3.0\",\"image_rev\":\"ousteros-image-prod-aries-v2.3.0+"
           "20220415163956\",\"initialization_id\":7109750,\"prod_line\":\"OS-"
           "1-"
           "128\",\"prod_pn\":\"840-103575-06\",\"prod_sn\":\"122201000998\","
           "\"status\":\"RUNNING\"},\"user_data\":\"\"}}";
    EXPECT_EQ(sensor->to_string(), actual.str());
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

    EXPECT_EQ(chunks.to_string(), "ChunksRange: [ba = 0, ea = 1015824]");
    EXPECT_EQ(chunks.begin().to_string(), "ChunksIter: [ca = 0, ea = 1015824]");
    std::cout << chunks.begin()->end_ts().count() << std::endl;
    EXPECT_EQ(chunks.begin()->start_ts(), ts_t(991587364520L));
    EXPECT_EQ(chunks.begin()->end_ts(), ts_t(991787323080L));
    EXPECT_EQ(chunks.begin()->to_string(),
              "ChunkRef: [msgs_size = 3, state = ("
              "{offset = 0, next_offset = 18446744073709551615,"
              " start_ts = 991587364520, end_ts = 991787323080,"
              " status = 1}), chunk_buf_ = nullptr]");
    EXPECT_EQ(((ChunkRef)*chunks.begin())[0].to_string(),
              "MessageRef: [id = 2, ts = 991587364520, buffer = 74 2d 05 00 28 "
              "00 00 00 00 00 00 00 00 00 1e 00 24 00 04 00 08 00 0c 00 10 00 "
              "14 00 18 00 00 00 1c 00 00 00 00 00 00 00 00 00 20 00 1e 00 00 "
              "00 88 3a 00 00 78 3a 00 00 6c 1a 00 00 64 12 00 00 5c 02 00 00 "
              "03 07 00 00 4c 00 00 00 04 00 00 00 40 00 00 00 00 00 00 00 00 "
              "00 00 00 00 00 00 00 00 00 00 00 ... and 339220 more ...]");
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
    EXPECT_EQ(
        (*reader.messages().begin()).to_string(),
        "MessageRef: [id = 2, ts = 991587364520, buffer = 74 2d 05 00 28 00 00 "
        "00 00 00 00 00 00 00 1e 00 24 00 04 00 08 00 0c 00 10 00 14 00 18 00 "
        "00 00 1c 00 00 00 00 00 00 00 00 00 20 00 1e 00 00 00 88 3a 00 00 78 "
        "3a 00 00 6c 1a 00 00 64 12 00 00 5c 02 00 00 03 07 00 00 4c 00 00 00 "
        "04 00 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
        "00 ... and 339220 more ...]");
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

// @TODO Reeanble this test when we have generic spdlog functonality.
// TEST_F(ReaderTest, MetadataFromBufferTest) {
//     OsfFile osf_file(
//         path_concat(test_data_dir(),
//         "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

//     Reader reader(osf_file);

//     auto sensor = reader.meta_store().entries().begin()->second;

//     std::vector<uint8_t> buf;
//     std::stringstream output_stream;
//     auto ostream_sink = std::make_shared<
//         spdlog::sinks::ostream_sink<spdlog::details::null_mutex>>(
//         output_stream);
//     ouster::sensor::impl::Logger::instance().configure_generic_sink(
//         ostream_sink, "info");

//     auto result = sensor->from_buffer(buf, "NON EXISTENT");

//     std::string output_error = output_stream.str();
//     auto error_loc = output_error.find("[error]");
//     EXPECT_NE(error_loc, std::string::npos);

//     output_error = output_error.substr(error_loc);
// #ifdef _WIN32
//     EXPECT_EQ(output_error, "[error] UNKNOWN TYPE: NON EXISTENT\r\n");
// #else
//     EXPECT_EQ(output_error, "[error] UNKNOWN TYPE: NON EXISTENT\n");
// #endif
//     EXPECT_EQ(result, nullptr);

//     result = sensor->from_buffer(sensor->buffer(),
//                                  "ouster/v1/os_sensor/LidarSensor");
//     EXPECT_NE(result, nullptr);
//     EXPECT_EQ(result->id(), 0);
//     EXPECT_EQ(result->type(), "ouster/v1/os_sensor/LidarSensor");
// }

}  // namespace
}  // namespace osf
}  // namespace ouster
