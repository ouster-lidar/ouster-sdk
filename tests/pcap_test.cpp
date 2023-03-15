/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "ouster/pcap.h"
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
    // since we haven't read any packets yet, call next_packet to read the first one
    // then read the cached info
    pcap.next_packet();
    packet_info current_info = pcap.current_info();
    EXPECT_EQ(current_info.file_offset, 24); // the length of a pcap file header

    size_t packet_size = pcap.next_packet();
    EXPECT_EQ(packet_size, 0); // no packets left
    current_info = pcap.current_info();
    EXPECT_EQ(current_info.file_offset, 8530); // which is the end of the file
}

/// it seeks to the specified offset
TEST(PcapReader, seek) {
    auto data_dir = getenvs("DATA_DIR");
    PcapReader pcap(data_dir + "/OS-0-128-U1_v2.3.0_1024x10.pcap");

    // get offset of the first packet;
    // since we haven't read any packets yet, call next_packet to read the first one
    // then read the cached info
    pcap.next_packet();
    packet_info current_info = pcap.current_info();

    // file_offset is supposed to be the position of the packet in the file
    auto offset = current_info.file_offset;

    // so if we seek to that offset, we should be back at the beginning of the first packet
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
    // it should be able to read packets after an attempt to seek before the pcap header
    // this file has one packet in it and is 8530 bytes long
    auto data_dir = getenvs("DATA_DIR");
    std::string filename = data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap";
    PcapReader pcap(filename);

    EXPECT_EQ(pcap.next_packet(), 8448);
    pcap.seek(0);

    // attempting to read past end of file is not an error, either
    EXPECT_EQ(pcap.next_packet(), 8448);
}

TEST(PcapReader, seek_middle_of_packet) {
    // seeking to the middle of a packet results in undefined behavior when `next_packet()` is called again
    auto data_dir = getenvs("DATA_DIR");
    std::string filename = data_dir + "/OS-0-128-U1_v2.3.0_1024x10.pcap";
    PcapReader pcap(filename);

    pcap.seek(4000); // roughly the middle of the first packet in the file
    EXPECT_EQ(pcap.next_packet(), 0);
    EXPECT_EQ(pcap.next_packet(), 0);
}

TEST(PcapReader, seek_past_end_of_file) {
    // it does not raise an exception if the caller seeks past the end of file
    // this file has one packet in it and is 8530 bytes long
    auto data_dir = getenvs("DATA_DIR");
    std::string filename = data_dir + "/OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap";
    PcapReader pcap(filename);
    pcap.seek(file_size(filename) + 1);

    // attempting to read past end of file is not an error, either
    EXPECT_EQ(pcap.next_packet(), 0);
}

} // namespace sensor_utils
} // namespace ouster

