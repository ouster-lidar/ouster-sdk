/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "pcap.h"

using ouster::sdk::pcap::PacketInfo;
using ouster::sdk::pcap::PcapReader;
using ouster::sdk::pcap::PcapWriter;

namespace {

uint32_t ipv4(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    const uint8_t bytes[4] = {a, b, c, d};
    uint32_t value = 0;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

std::vector<uint8_t> read_payload(const PcapReader& reader) {
    return {reader.current_data(), reader.current_data() + reader.current_length()};
}

}  // namespace

TEST(PcapWriterTest, WritesRawIpv4Addresses) {
    const std::string path = ::testing::TempDir() + "pcap_writer_raw.pcap";
    const std::vector<uint8_t> payload(100, 0xab);
    {
        PcapWriter writer(path, PcapWriter::ETHERNET, 1500);
        writer.write_packet(payload.data(), payload.size(), ipv4(192, 168, 1, 10),
                            ipv4(10, 0, 0, 255), 7502, 7503, PacketInfo::ts{1000});
    }

    PcapReader reader(path);
    ASSERT_EQ(reader.next_packet(), payload.size());
    const auto& info = reader.current_info();
    EXPECT_EQ(info.ip_version, 4);
    EXPECT_EQ(info.src_ip(), "192.168.1.10");
    EXPECT_EQ(info.dst_ip(), "10.0.0.255");
    EXPECT_EQ(info.src_port, 7502);
    EXPECT_EQ(info.dst_port, 7503);
    EXPECT_EQ(read_payload(reader), payload);
    EXPECT_EQ(reader.next_packet(), 0u);
}

TEST(PcapWriterTest, PacketInfoRoundTrip) {
    const std::string first = ::testing::TempDir() + "pcap_writer_first.pcap";
    const std::string second = ::testing::TempDir() + "pcap_writer_second.pcap";
    const std::vector<uint8_t> payload(64, 0x5a);
    {
        PcapWriter writer(first, PcapWriter::ETHERNET, 1500);
        writer.write_packet(payload.data(), payload.size(), "172.16.0.1", "172.16.0.2", 7504, 7505,
                            PacketInfo::ts{2000});
    }

    PcapReader first_reader(first);
    ASSERT_EQ(first_reader.next_packet(), payload.size());
    {
        PcapWriter writer(second, PcapWriter::ETHERNET, 1500);
        writer.write_packet(first_reader.current_data(), first_reader.current_length(),
                            first_reader.current_info());
    }

    PcapReader second_reader(second);
    ASSERT_EQ(second_reader.next_packet(), payload.size());
    const auto& info = second_reader.current_info();
    EXPECT_EQ(info.src_ip(), "172.16.0.1");
    EXPECT_EQ(info.dst_ip(), "172.16.0.2");
    EXPECT_EQ(info.src_port, 7504);
    EXPECT_EQ(info.dst_port, 7505);
    EXPECT_EQ(info.timestamp, PacketInfo::ts{2000});
    EXPECT_EQ(read_payload(second_reader), payload);
}

TEST(PcapWriterTest, RejectsIpv6PacketInfo) {
    const std::string path = ::testing::TempDir() + "pcap_writer_ipv6.pcap";
    const std::vector<uint8_t> payload(16, 0);
    PcapWriter writer(path, PcapWriter::ETHERNET, 1500);
    PacketInfo info{};
    info.ip_version = 6;
    EXPECT_THROW(writer.write_packet(payload.data(), payload.size(), info), std::invalid_argument);
}
