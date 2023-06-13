/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/reader.h"

#include <gtest/gtest.h>

#include "common.h"
#include "osf_test.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/stream_lidar_scan.h"

namespace ouster {
namespace osf {
namespace {

class ReaderTest : public osf::OsfTestWithData {};

TEST_F(ReaderTest, Basics) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    EXPECT_EQ("from_pcap pythonic", reader.id());
    EXPECT_EQ(991587364520LL, reader.start_ts().count());
    EXPECT_EQ(991787323080LL, reader.end_ts().count());

    // Get first sensor (it's the first by metadata_id) (i.e. first added)
    auto sensor = reader.meta_store().get<osf::LidarSensor>();
    EXPECT_TRUE(sensor);

    EXPECT_EQ(1, reader.meta_store().count<osf::LidarSensor>());

    EXPECT_EQ(3, std::distance(reader.messages_standard().begin(),
                               reader.messages_standard().end()));

    const MetadataStore& meta_store = reader.meta_store();
    EXPECT_EQ(3, meta_store.size());
}

TEST_F(ReaderTest, ChunksReading) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    auto chunks = reader.chunks();

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

TEST_F(ReaderTest, MessagesReadingStandard) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    const auto msgs = reader.messages_standard();
    EXPECT_EQ(3, std::distance(msgs.begin(), msgs.end()));

    // Chunks Iterator
    auto chunks = reader.chunks();
    EXPECT_EQ(1, std::distance(chunks.begin(), chunks.end()));

    // Get messages from first chunks
    auto first_chunk_it = chunks.begin();
    EXPECT_EQ(3, first_chunk_it->size());
    EXPECT_EQ(3, std::distance(first_chunk_it->begin(), first_chunk_it->end()));
}

TEST_F(ReaderTest, MessagesReadingStreaming) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));

    Reader reader(osf_file);

    // Reader iterator reads as messages from chunk as they appears on a disk
    int it_cnt = 0;
    ts_t it_prev{0};
    bool it_ordered = true;
    for (const auto msg : reader.messages_standard()) {
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

    for (const auto msg : reader.messages()) {
        sit_ordered = sit_ordered && (sit_prev <= msg.ts());
        ++sit_cnt;
        sit_prev = msg.ts();
    }
    EXPECT_EQ(3, sit_cnt);
    EXPECT_TRUE(sit_ordered);

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

}  // namespace
}  // namespace osf
}  // namespace ouster
