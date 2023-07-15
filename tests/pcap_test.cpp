/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/pcap.h"

#include <gtest/gtest.h>

#include <iostream>

#include "ouster/indexed_pcap_reader.h"
#include "ouster/os_pcap.h"

namespace ouster {
namespace sensor_utils {

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}

/// it is equal to or occurs before the first byte in the packet
TEST(PcapReader, file_offset) {
    // this file has one packet in it and is 8530 bytes long
    auto data_dir = getenvs("DATA_DIR");
    PcapReader pcap(data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap");

    // get offset of the first packet;
    // since we haven't read any packets yet, call next_packet to read the first
    // one then read the cached info
    pcap.next_packet();
    packet_info current_info = pcap.current_info();
    EXPECT_EQ(current_info.file_offset,
              24);  // the length of a pcap file header

    size_t packet_size = pcap.next_packet();
    EXPECT_EQ(packet_size, 0);  // no packets left
    current_info = pcap.current_info();
    EXPECT_EQ(current_info.file_offset, 8530);  // which is the end of the file
}

/// it seeks to the specified offset
TEST(PcapReader, seek) {
    auto data_dir = getenvs("DATA_DIR");
    PcapReader pcap(data_dir + "/OS-0-128-U1_v2.3.0_1024x10.pcap");

    // get offset of the first packet;
    // since we haven't read any packets yet, call next_packet to read the first
    // one then read the cached info
    pcap.next_packet();
    packet_info current_info = pcap.current_info();

    // file_offset is supposed to be the position of the packet in the file
    auto offset = current_info.file_offset;

    // so if we seek to that offset, we should be back at the beginning of the
    // first packet
    pcap.seek(offset);

    // read the packet again, read its info again
    pcap.next_packet();
    current_info = pcap.current_info();

    // it should have the same offset as the first packet we read
    EXPECT_EQ(offset, current_info.file_offset);
}

inline uint64_t file_size(const std::string& filename) {
    FILE* f = fopen(filename.c_str(), "rb");
    fseek(f, 0, SEEK_END);
    uint64_t size = ftell(f);
    fclose(f);
    return size;
}

TEST(PcapReader, seek_to_0) {
    // it should be able to read packets after an attempt to seek before the
    // pcap header this file has one packet in it and is 8530 bytes long
    auto data_dir = getenvs("DATA_DIR");
    std::string filename =
        data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap";
    PcapReader pcap(filename);

    EXPECT_EQ(pcap.next_packet(), 8448);
    pcap.seek(0);

    // attempting to read past end of file is not an error, either
    EXPECT_EQ(pcap.next_packet(), 8448);
}

TEST(PcapReader, seek_middle_of_packet) {
    // seeking to the middle of a packet results in undefined behavior when
    // `next_packet()` is called again
    auto data_dir = getenvs("DATA_DIR");
    std::string filename = data_dir + "/OS-0-128-U1_v2.3.0_1024x10.pcap";
    PcapReader pcap(filename);

    pcap.seek(4000);  // roughly the middle of the first packet in the file
    EXPECT_EQ(pcap.next_packet(), 0);
    EXPECT_EQ(pcap.next_packet(), 0);
}

TEST(PcapReader, seek_past_end_of_file) {
    // it does not raise an exception if the caller seeks past the end of file
    // this file has one packet in it and is 8530 bytes long
    auto data_dir = getenvs("DATA_DIR");
    std::string filename =
        data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap";
    PcapReader pcap(filename);
    pcap.seek(file_size(filename) + 1);

    // attempting to read past end of file is not an error, either
    EXPECT_EQ(pcap.next_packet(), 0);
}

TEST(IndexedPcapReader, constructor) {
    // it should be constructed with the correct number of indices
    // and previous frame counts (one for each metadata file)
    auto data_dir = getenvs("DATA_DIR");
    std::string filename =
        data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap";
    std::string meta_filename = data_dir + "/OS-0-32-U1_v2.2.0_1024x10.json";

    IndexedPcapReader pcap(filename,
                           {meta_filename, meta_filename, meta_filename});
    EXPECT_EQ(pcap.index_.frame_indices_.size(), 3);
    EXPECT_EQ(pcap.previous_frame_ids_.size(), 3);
}

TEST(IndexedPcapReader, frame_count) {
    // it should raise std::out_of_range if there is no sensor at that position
    auto data_dir = getenvs("DATA_DIR");
    std::string filename =
        data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap";
    IndexedPcapReader pcap(filename, {});
    pcap.index_.frame_indices_.push_back(PcapIndex::frame_index());
    pcap.index_.frame_indices_.at(0).push_back(0);

    EXPECT_EQ(pcap.index_.frame_count(0), 1);
    EXPECT_THROW(pcap.index_.frame_count(1), std::out_of_range);
}

TEST(IndexedPcapReader, seek_to_frame) {
    // it should raise std::out_of_range if there is no sensor or frame at that
    // position
    auto data_dir = getenvs("DATA_DIR");
    std::string filename = data_dir + "/OS-2-128-U1_v2.3.0_1024x10.pcap";
    std::string meta_filename = data_dir + "/OS-2-128-U1_v2.3.0_1024x10.json";
    IndexedPcapReader pcap(filename, {meta_filename});

    std::vector<int> progress;
    while (pcap.next_packet()) {
        progress.push_back(pcap.update_index_for_current_packet());
    }
    ASSERT_GT(progress.size(), 2);
    // progress values are in the range and are increasing [1, 100]
    int prev_progress = 0l;
    for (size_t i = 0; i < progress.size(); i++) {
        EXPECT_GE(progress[i], 1);
        EXPECT_LE(progress[i], 100);
        EXPECT_GE(progress[i], prev_progress);
        prev_progress = progress[i];
    }
    EXPECT_EQ(progress[progress.size() - 1],
              100);  // last progress value is 100
    pcap.index_.frame_indices_.push_back(PcapIndex::frame_index());

    EXPECT_EQ(pcap.index_.frame_count(0), 1);
    EXPECT_NO_THROW(pcap.index_.seek_to_frame(pcap, 0, 0));
    EXPECT_THROW(pcap.index_.seek_to_frame(pcap, 0, 1), std::out_of_range);
    EXPECT_THROW(pcap.index_.seek_to_frame(pcap, 1, 1), std::out_of_range);
}

TEST(IndexedPcapReader, frame_id_rolled_over) {
    EXPECT_TRUE(IndexedPcapReader::frame_id_rolled_over(65535, 0));
    EXPECT_TRUE(IndexedPcapReader::frame_id_rolled_over(65290, 100));
    EXPECT_FALSE(IndexedPcapReader::frame_id_rolled_over(65000, 65535));
    EXPECT_FALSE(IndexedPcapReader::frame_id_rolled_over(1, 0));
    EXPECT_FALSE(IndexedPcapReader::frame_id_rolled_over(12, 12));
}

}  // namespace sensor_utils
}  // namespace ouster
