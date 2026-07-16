/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <map>
#include <numeric>
#include <random>
#include <set>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/profile_extension.h"
#include "ouster/core/types.h"
#include "ouster/pcap/pcap.h"
#include "util.h"

using namespace ouster::sdk::core;
using namespace ouster::sdk::core::impl;
using namespace ouster::sdk::pcap;

using test_param = std::tuple<UDPProfileLidar, HeaderType, uint32_t, uint32_t, uint32_t>;
class FrameBatcherTest : public ::testing::TestWithParam<test_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    FrameBatcherTests,
    FrameBatcherTest,
    ::testing::Combine(
        ::testing::Values(
            UDPProfileLidar::LEGACY,
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
            UDPProfileLidar::RNG15_RFL8_NIR8,
            UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL
            ),
        ::testing::Values(
            HeaderType::STANDARD,
            HeaderType::FUSA),
        ::testing::Values(1024), // columns_per_frame
        ::testing::Values(128),  // pixels_per_column
        ::testing::Values(16))); // columns_per_packet
// clang-format on

std::vector<Packet> random_frame(const SensorInfo& info) {
    const auto& format = info.format;
    PacketFormat packet_format{format};

    auto ls = LidarFrame(format.pixels_per_column, format.columns_per_frame,
                         format.udp_profile_lidar, format.columns_per_packet);

    ls.frame_id = 700;
    std::iota(ls.measurement_id().data(), ls.measurement_id().data() + ls.measurement_id().size(),
              0);
    std::iota(ls.packet_timestamp().data(),
              ls.packet_timestamp().data() + ls.packet_timestamp().size(), 10);
    std::iota(ls.timestamp().data(), ls.timestamp().data() + ls.timestamp().size(), 1000);
    std::fill(ls.status().data(), ls.status().data() + ls.status().size(), 0x1);

    auto randomise = [&](auto ref_field, const std::string& i) {
        randomize_field(ref_field, packet_format.field_value_mask(i));
    };
    impl::foreach_channel_field(ls, packet_format, randomise);

    auto packets = std::vector<Packet>{};
    impl::frame_to_packets(ls, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets), info.init_id, info.sn);

    return packets;
}

TEST_P(FrameBatcherTest, frame_batcher_skips_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    HeaderType profile_type = std::get<1>(param);
    uint32_t columns_per_frame = std::get<2>(param);
    uint32_t pixels_per_column = std::get<3>(param);
    uint32_t columns_per_packet = std::get<4>(param);

    const DataFormat df{pixels_per_column,
                        columns_per_packet,
                        columns_per_frame,
                        0,
                        0,
                        {},
                        {0, columns_per_frame - 1},
                        profile,
                        UDPProfileIMU::LEGACY,
                        profile_type,
                        10};
    SensorInfo info;
    info.format = df;
    const PacketFormat pf{info};
    PacketFormat packet_format{pf};

    auto packets = random_frame(info);

    auto reference = LidarFrame(info);
    {
        FrameBatcher batcher(info);
        for (size_t i = 0; i < packets.size(); i++) {
            const auto& p = packets.at(i);
            if (i == 63) {
                EXPECT_TRUE(batcher(p, reference));
            } else {
                EXPECT_FALSE(batcher(p, reference));
            }
        }
    }

    uint32_t frame_id = pf.frame_id(packets.at(0).buf.data());

    auto next_frame_packet = std::make_unique<LidarPacket>(pf.lidar_packet_size);
    packet_format.set_frame_id(next_frame_packet->buf.data(), frame_id + 1);

    // produce a reordered packet from "previous frame" with data from one of
    // the dropped packets, therefore we will know if it gets parsed
    auto reordered_packet = std::make_unique<LidarPacket>();
    *reordered_packet = packets.back().as<LidarPacket>();
    packet_format.set_frame_id(reordered_packet->buf.data(), frame_id - 1);

    std::set<uint16_t> invalid_m_ids;
    // dropping in reverse order for easier erase
    std::vector<size_t> dropped_packets = {packets.size() - 1, packets.size() / 2, 0};
    std::set<uint16_t> dropped_m_ids;
    for (const auto& p_id : dropped_packets) {
        auto& packet = packets.at(p_id);
        for (size_t icol = 0; icol < columns_per_packet; ++icol) {
            const uint8_t* col_buf = pf.nth_col(icol, packet.buf.data());
            dropped_m_ids.insert(pf.col_measurement_id(col_buf));
        }
        packets.erase(packets.begin() + p_id);
    }
    {  // invalidate every fourth m_id;
        for (auto& packet : packets) {
            for (size_t icol = 0; icol < columns_per_packet; icol += 4) {
                uint8_t* col_buf = packet_format.nth_col(icol, packet.buf.data());
                invalid_m_ids.insert(pf.col_measurement_id(col_buf));
                packet_format.set_col_status(col_buf, 0);
            }
        }
    }
    {  // invalidate first half of second packet
        auto& packet = packets.at(2);
        for (size_t icol = 0; icol < columns_per_packet / 2; ++icol) {
            uint8_t* col_buf = packet_format.nth_col(icol, packet.buf.data());
            invalid_m_ids.insert(pf.col_measurement_id(col_buf));
            packet_format.set_col_status(col_buf, 0);
        }
    }
    {  // invalidate last half of fourth packet
        auto& packet = packets.at(4);
        for (size_t icol = columns_per_packet / 2; icol < columns_per_packet; ++icol) {
            uint8_t* col_buf = packet_format.nth_col(icol, packet.buf.data());
            invalid_m_ids.insert(pf.col_measurement_id(col_buf));
            packet_format.set_col_status(col_buf, 0);
        }
    }
    {  // invalidate entire tenth packet (packet_timestamp should stay)
        auto& packet = packets.at(10);
        for (size_t icol = 0; icol < columns_per_packet; ++icol) {
            uint8_t* col_buf = packet_format.nth_col(icol, packet.buf.data());
            invalid_m_ids.insert(pf.col_measurement_id(col_buf));
            packet_format.set_col_status(col_buf, 0);
        }
    }
    // swap two packets
    std::swap(packets.at(4), packets.at(5));

    std::set<uint16_t> valid_m_ids;
    for (uint16_t m_id = 0; m_id < columns_per_frame; ++m_id) {
        if (!invalid_m_ids.count(m_id) && !dropped_m_ids.count(m_id)) valid_m_ids.insert(m_id);
    }

    auto ls = LidarFrame(info);

    {  // pre-fill lidar frame so we know which fields/headers are changed
        auto fill = [](auto ref_field, const std::string&) { ref_field = 1; };
        impl::foreach_channel_field(ls, pf, fill);
        ls.packet_timestamp() = 2000;
        ls.timestamp() = 100;
        ls.status() = 0x0f;
        ls.measurement_id() = 10000;
    }

    FrameBatcher batcher(info);
    for (const auto& p : packets) {
        EXPECT_FALSE(batcher(p, ls));
    }
    // this should just be dropped, we check that the values are not read in
    // dropped_m_ids checks
    EXPECT_FALSE(batcher(*reordered_packet, ls));

    // fill up the cache with the next frame's packets,
    // which should cause the previous frame to be finalized
    // (which zeros the dropped m_ids)
    for (size_t p_id = 0; p_id < batcher.get_max_cache_size() - 1; ++p_id) {
        EXPECT_FALSE(batcher(*next_frame_packet, ls));
    }
    // this last addition should finalize the frame
    EXPECT_TRUE(batcher(*next_frame_packet, ls));

    // the frame's id should be the one from the original packets, not the new
    // frame's packets
    EXPECT_EQ(ls.frame_id, frame_id);

    auto test_skipped_fields = [&](auto ref_field, const std::string& chan) {
        // dropped frames should all be zero, RAW_HEADERS or not
        for (auto& m_id : dropped_m_ids) {
            EXPECT_TRUE((ref_field.col(m_id) == 0).all());
        }

        if (chan == ChanField::RAW_HEADERS) {
            for (auto& m_id : valid_m_ids) {
                EXPECT_FALSE((ref_field.col(m_id) == 0).all());
            }
            return;
        }

        using T = typename decltype(ref_field)::Scalar;
        const auto& f = reference.field<T>(chan);
        for (auto& m_id : valid_m_ids) {
            EXPECT_TRUE((ref_field.col(m_id) == f.col(m_id)).all());
        }

        // these should be zero unless RAW_HEADERS
        for (auto& m_id : invalid_m_ids) {
            EXPECT_TRUE((ref_field.col(m_id) == 0).all());
        }
    };

    auto test_headers = [&](const LidarFrame& frame) {
        for (auto& m_id : dropped_m_ids) {
            EXPECT_EQ(frame.timestamp()[m_id], 0);
            EXPECT_EQ(frame.status()[m_id], 0);
            EXPECT_EQ(frame.measurement_id()[m_id], 0);
        }

        for (auto& m_id : invalid_m_ids) {
            EXPECT_EQ(frame.timestamp()[m_id], 0);
            EXPECT_EQ(frame.status()[m_id], 0);
            EXPECT_EQ(frame.measurement_id()[m_id], 0);
        }

        for (auto& m_id : valid_m_ids) {
            EXPECT_NE(frame.timestamp()[m_id], 0);
            EXPECT_NE(frame.status()[m_id], 0);
            EXPECT_EQ(frame.measurement_id()[m_id], m_id);
        }

        for (const auto& p_id : dropped_packets) EXPECT_EQ(frame.packet_timestamp()[p_id], 0);
        EXPECT_EQ((frame.packet_timestamp() == 0).count(), dropped_packets.size());
    };

    impl::foreach_channel_field(ls, pf, test_skipped_fields);
    test_headers(ls);

    // now repeat for RAW_HEADERS and CUSTOM fields
    LidarFrameFieldTypes rh_types(ls.field_types());
    rh_types.emplace_back(ChanField::RAW_HEADERS, ChanFieldType::UINT32);
    rh_types.emplace_back("CUSTOM0", ChanFieldType::UINT32);
    rh_types.emplace_back("CUSTOM9", ChanFieldType::UINT32);
    auto rh_ls = LidarFrame(pixels_per_column, columns_per_frame, rh_types, columns_per_packet);

    {  // pre-fill lidar frame so we know which fields/headers are changed
        auto fill = [](auto ref_field, const std::string&) { ref_field = 1; };
        impl::foreach_channel_field(rh_ls, pf, fill);
        impl::visit_field(rh_ls, "CUSTOM0", fill, "");
        impl::visit_field(rh_ls, "CUSTOM9", fill, "");
        rh_ls.packet_timestamp() = 2000;
        rh_ls.timestamp() = 100;
        rh_ls.status() = 0x0f;
        rh_ls.measurement_id() = 10000;
    }

    FrameBatcher rh_batcher(info);
    for (const auto& p : packets) {
        EXPECT_FALSE(rh_batcher(p, rh_ls));
    }
    EXPECT_FALSE(rh_batcher(*reordered_packet, rh_ls));
    // fill up the cache with the next frame's packets,
    // which should cause the previous frame to be finalized
    // (which zeros the dropped m_ids)
    for (size_t p_id = 0; p_id < batcher.get_max_cache_size() - 1; ++p_id) {
        EXPECT_FALSE(rh_batcher(*next_frame_packet, rh_ls));
    }
    ASSERT_TRUE(rh_batcher(*next_frame_packet, rh_ls));

    EXPECT_EQ(rh_ls.frame_id, frame_id);

    auto test_custom_fields = [](auto ref_field) { EXPECT_TRUE((ref_field == 1).all()); };

    impl::visit_field(rh_ls, "CUSTOM0", test_custom_fields);
    impl::visit_field(rh_ls, "CUSTOM9", test_custom_fields);

    impl::foreach_channel_field(rh_ls, pf, test_skipped_fields);
    impl::visit_field(rh_ls, ChanField::RAW_HEADERS, test_skipped_fields, ChanField::RAW_HEADERS);
    test_headers(rh_ls);
}

/**
 * repeat the above test for block traversal case (no invalid m_ids in packets)
 */
TEST_P(FrameBatcherTest, frame_batcher_block_parse_dropped_packets_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    HeaderType profile_type = std::get<1>(param);
    uint32_t columns_per_frame = std::get<2>(param);
    uint32_t pixels_per_column = std::get<3>(param);
    uint32_t columns_per_packet = std::get<4>(param);

    DataFormat df{pixels_per_column,
                  columns_per_packet,
                  columns_per_frame,
                  0,
                  0,
                  {},
                  {0, columns_per_frame - 1},
                  profile,
                  UDPProfileIMU::LEGACY,
                  profile_type,
                  10};
    SensorInfo info;
    info.format = df;
    PacketFormat pf{info};
    PacketFormat packet_format{pf};

    auto packets = random_frame(info);

    auto reference = LidarFrame(info);
    {
        FrameBatcher batcher(info);
        for (size_t i = 0; i < packets.size(); i++) {
            const auto& p = packets.at(i);
            if (i == 63) {
                EXPECT_TRUE(batcher(p, reference));
            } else {
                EXPECT_FALSE(batcher(p, reference));
            }
        }
    }

    uint32_t frame_id = pf.frame_id(packets.at(0).buf.data());

    auto next_frame_packet = std::make_unique<LidarPacket>(pf.lidar_packet_size);
    packet_format.set_frame_id(next_frame_packet->buf.data(), frame_id + 1);

    // dropping in reverse order for easier erase
    std::vector<size_t> dropped_packets = {packets.size() - 1, packets.size() / 2, 0};
    std::set<uint16_t> dropped_m_ids;
    for (const auto& p_id : dropped_packets) {
        auto& packet = packets.at(p_id);
        for (size_t icol = 0; icol < columns_per_packet; ++icol) {
            const uint8_t* col_buf = pf.nth_col(icol, packet.buf.data());
            dropped_m_ids.insert(pf.col_measurement_id(col_buf));
        }
        packets.erase(packets.begin() + p_id);
    }
    // swap two packets
    std::swap(packets.at(4), packets.at(5));

    std::set<uint16_t> valid_m_ids;
    for (uint16_t m_id = 0; m_id < columns_per_frame; ++m_id) {
        if (!dropped_m_ids.count(m_id)) valid_m_ids.insert(m_id);
    }

    auto ls = LidarFrame(info);

    {  // pre-fill lidar frame so we know which fields/headers are changed
        auto fill = [](auto ref_field, const std::string&) { ref_field = 1; };
        impl::foreach_channel_field(ls, pf, fill);
        ls.packet_timestamp() = 2000;
        ls.timestamp() = 100;
        ls.status() = 0x0f;
        ls.measurement_id() = 10000;
    }

    FrameBatcher batcher(info);
    for (const auto& p : packets) {
        EXPECT_FALSE(batcher(p, ls));
    }

    // fill up the cache with the next frame's packets,
    // which should cause the previous frame to be finalized
    // (which zeros the dropped m_ids)
    for (size_t p_id = 0; p_id < batcher.get_max_cache_size() - 1; ++p_id) {
        EXPECT_FALSE(batcher(*next_frame_packet, ls));
    }

    // once the cache is filled, another packet from the next frame should
    // finalize the current frame
    EXPECT_TRUE(batcher(*next_frame_packet, ls));

    auto test_skipped_fields = [&](auto ref_field, const std::string& chan) {
        // dropped frames should all be zero, RAW_HEADERS or not
        for (auto& m_id : dropped_m_ids) {
            EXPECT_TRUE((ref_field.col(m_id) == 0).all());
        }

        using T = typename decltype(ref_field)::Scalar;
        const auto& f = reference.field<T>(chan);
        for (auto& m_id : valid_m_ids) {
            EXPECT_TRUE((ref_field.col(m_id) == f.col(m_id)).all());
        }
    };

    auto test_headers = [&](const LidarFrame& frame) {
        for (auto& m_id : dropped_m_ids) {
            EXPECT_EQ(frame.timestamp()[m_id], 0);
            EXPECT_EQ(frame.status()[m_id], 0);
            EXPECT_EQ(frame.measurement_id()[m_id], 0);
        }

        for (auto& m_id : valid_m_ids) {
            EXPECT_NE(frame.timestamp()[m_id], 0);
            EXPECT_NE(frame.status()[m_id], 0);
            EXPECT_EQ(frame.measurement_id()[m_id], m_id);
        }

        for (const auto& p_id : dropped_packets) EXPECT_EQ(frame.packet_timestamp()[p_id], 0);
        EXPECT_EQ((frame.packet_timestamp() == 0).count(), dropped_packets.size());
    };

    impl::foreach_channel_field(ls, pf, test_skipped_fields);
    test_headers(ls);

    // now repeat for CUSTOM fields
    LidarFrameFieldTypes custom_types(ls.field_types());
    custom_types.emplace_back("CUSTOM0", ChanFieldType::UINT32);
    custom_types.emplace_back("CUSTOM9", ChanFieldType::UINT32);
    auto custom_ls =
        LidarFrame(pixels_per_column, columns_per_frame, custom_types, columns_per_packet);

    {  // pre-fill lidar frame so we know which fields/headers are changed
        auto fill = [](auto ref_field, const std::string&) { ref_field = 1; };
        impl::foreach_channel_field(custom_ls, pf, fill);
        impl::visit_field(custom_ls, "CUSTOM0", fill, "");
        impl::visit_field(custom_ls, "CUSTOM9", fill, "");
        custom_ls.packet_timestamp() = 2000;
        custom_ls.timestamp() = 100;
        custom_ls.status() = 0x0f;
        custom_ls.measurement_id() = 10000;
    }

    FrameBatcher custom_batcher(info);
    for (const auto& p : packets) {
        EXPECT_FALSE(custom_batcher(p, custom_ls));
    }

    // fill up the cache with the next frame's packets,
    // which should cause the previous frame to be finalized
    // (which zeros the dropped m_ids)
    for (size_t p_id = 0; p_id < batcher.get_max_cache_size() - 1; ++p_id) {
        EXPECT_FALSE(custom_batcher(*next_frame_packet, custom_ls));
    }

    // once the cache is filled, another packet from the next frame should
    // finalize the current frame
    EXPECT_TRUE(custom_batcher(*next_frame_packet, custom_ls));

    EXPECT_EQ(custom_ls.frame_id, frame_id);

    auto test_custom_fields = [](auto ref_field) { EXPECT_TRUE((ref_field == 1).all()); };

    impl::visit_field(custom_ls, "CUSTOM0", test_custom_fields);
    impl::visit_field(custom_ls, "CUSTOM9", test_custom_fields);

    impl::foreach_channel_field(custom_ls, pf, test_skipped_fields);
    test_headers(custom_ls);
}

TEST_P(FrameBatcherTest, frame_batcher_wraparound_test) {
    // A packet added from a previous frame won't zero out the fields and
    // headers.
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    HeaderType profile_type = std::get<1>(param);
    uint32_t columns_per_frame = std::get<2>(param);
    uint32_t pixels_per_column = std::get<3>(param);
    uint32_t columns_per_packet = std::get<4>(param);

    auto ls = LidarFrame(pixels_per_column, columns_per_frame, profile, columns_per_packet);

    DataFormat df{pixels_per_column,
                  columns_per_packet,
                  columns_per_frame,
                  0,                           // imu measurements per packet
                  0,                           // imu packets per frame
                  {},                          // pixel shift by row
                  {0, columns_per_frame - 1},  // column window
                  profile,
                  UDPProfileIMU::LEGACY,
                  profile_type,
                  10};
    SensorInfo info;
    info.format = df;
    PacketFormat pf{df};
    PacketFormat packet_format{pf};

    auto packet = std::make_unique<LidarPacket>(pf.lidar_packet_size);
    std::memset(packet->buf.data(), 0, packet->buf.size());
    packet->host_timestamp = 100;
    packet_format.set_frame_id(packet->buf.data(), packet_format.max_frame_id);
    uint16_t m_id = columns_per_frame - columns_per_packet;
    uint64_t ts = 100;
    for (size_t icol = 0; icol < columns_per_packet; ++icol) {
        uint8_t* col_buf = packet_format.nth_col(icol, packet->buf.data());
        packet_format.set_col_status(col_buf, 0x01 /*valid*/);
        packet_format.set_col_measurement_id(col_buf, m_id++);
        packet_format.set_col_timestamp(col_buf, ts++);
    }

    FrameBatcher batcher(info);
    // The first packet will reset the frame since it's a brand new batcher.
    // But we're not testing this.
    EXPECT_FALSE(batcher.batch(*packet, ls));

    // pre-fill lidar frame so we know which fields/headers are changed
    auto fill = [](auto ref_field, const std::string&) { ref_field = 1; };
    impl::foreach_channel_field(ls, pf, fill);
    ls.packet_timestamp() = 2000;
    ls.timestamp() = 100;
    ls.status() = 0x0f;
    ls.measurement_id() = 10000;
    ls.frame_id = 0;

    // This packet is from the previous frame, but it won't zero out the frame's
    // headers.
    EXPECT_FALSE(batcher.batch(*packet, ls));

    EXPECT_EQ(ls.frame_id, 0);
    EXPECT_TRUE((ls.packet_timestamp() == 2000).all());
    EXPECT_TRUE((ls.timestamp() == 100).all());
    EXPECT_TRUE((ls.status() == 0x0f).all());
    EXPECT_TRUE((ls.measurement_id() == 10000).all());
    auto test_fields = [](auto ref_field, const std::string&) {
        EXPECT_TRUE((ref_field == 1).all());
    };
    impl::foreach_channel_field(ls, pf, test_fields);
}

using HashMap = std::map<std::string, size_t>;
using snapshot_param = std::tuple<std::string, std::string, HashMap>;
class FrameBatcherSnapshotTest : public ::testing::TestWithParam<snapshot_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    FrameBatcherSnapshots,
    FrameBatcherSnapshotTest,
    ::testing::Values(
        // low bandwidth
        snapshot_param{"OS-0-128-U1_v2.3.0_1024x10.pcap",
                       "OS-0-128-U1_v2.3.0_1024x10.json",
                       {{ChanField::RANGE, 0xf605c68634d4d496},
                        {ChanField::REFLECTIVITY, 0x308446ce12113b5c},
                        {ChanField::NEAR_IR, 0xacbe4e6963b1d6c7},
                        {ChanField::FLAGS, 6373750807750774351}}},
        // dual return
        snapshot_param{"OS-0-32-U1_v2.2.0_1024x10.pcap",
                       "OS-0-32-U1_v2.2.0_1024x10.json",
                       {{ChanField::RANGE, 0xda815ba0ea0173dd},
                        {ChanField::RANGE2, 0x9d07c3e610c99239},
                        {ChanField::SIGNAL, 0xb2d846ac47621f7b},
                        {ChanField::SIGNAL2, 0x4553138a62c59e37},
                        {ChanField::REFLECTIVITY, 0x63d4c6e69ced4423},
                        {ChanField::REFLECTIVITY2, 0x415f5e481688fe5a},
                        {ChanField::NEAR_IR, 0x2c32a3e5be6b01d5},
                        {ChanField::FLAGS, 6902511898004997142},
                        {ChanField::FLAGS2, 14986456617710294519U}}},
        // fusa dual return
        snapshot_param{"OS-1-128_767798045_1024x10_20230712_120049.pcap",
                       "OS-1-128_767798045_1024x10_20230712_120049.json",
                       {{ChanField::RANGE, 0x8327b9d4c44c45a3},
                        {ChanField::RANGE2, 0x87288b444ddb9c9e},
                        {ChanField::REFLECTIVITY, 0x6912ca3fa04b0d1f},
                        {ChanField::REFLECTIVITY2, 0xf58aa5594d9749dc},
                        {ChanField::NEAR_IR, 0xc99384623c5d9feb},
                        {ChanField::FLAGS, 15585490641324286966U},
                        {ChanField::FLAGS2, 3655442015794344596}}},
        // single return
        snapshot_param{"OS-2-128-U1_v2.3.0_1024x10.pcap",
                       "OS-2-128-U1_v2.3.0_1024x10.json",
                       {{ChanField::RANGE, 0x5940899c1190d02d},
                        {ChanField::SIGNAL, 0x4446bddd21f14dd4},
                        {ChanField::REFLECTIVITY, 0xea599b8814d2eac1},
                        {ChanField::NEAR_IR, 0x8a5a3df8896e317a},
                        {ChanField::FLAGS, 3655442015794344596}}},
        // legacy
        snapshot_param{"OS-2-32-U0_v2.0.0_1024x10.pcap",
                       "OS-2-32-U0_v2.0.0_1024x10.json",
                       {{ChanField::RANGE, 0x5937f3d8f3762184},
                        {ChanField::SIGNAL, 0xbb4b7f22d1231e80},
                        {ChanField::REFLECTIVITY, 0x3D37AAEB2792F714},
                        {ChanField::NEAR_IR, 0xe972940ca8b204f0},
                        {ChanField::FLAGS, 13284364481018348283U}}}));
// clang-format on

// picked up from
// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
struct matrix_hash {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> matrix, const std::string& f, HashMap& map) const {
        size_t seed = 0;
        for (int i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<T>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        map[f] = seed;
    }
};

TEST_P(FrameBatcherSnapshotTest, snapshot_test) {
#ifdef _WIN32
    // technically speaking, std::hash is implementation dependent,
    // so a better solution would be to have a cross-platform hash
    GTEST_SKIP();
#endif

    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));
    auto pf = PacketFormat(info);
    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));

    const HashMap snapshot_hashes = std::get<2>(test_params);

    auto ls = LidarFrame(info);
    uint64_t packet_ts = 1234;  // arbitrary, irrelevant for test
    FrameBatcher batcher(info);
    int packet_index = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            LidarPacket packet;
            packet.host_timestamp = packet_ts;
            packet.buf.resize(pcap.current_length());
            memcpy(packet.buf.data(), pcap.current_data(), pcap.current_length());
            if (packet_index++ == 63) {
                EXPECT_TRUE(batcher(packet, ls));
            } else {
                EXPECT_FALSE(batcher(packet, ls));
            }
        }

    HashMap hashes;
    impl::foreach_channel_field(ls, pf, matrix_hash{}, hashes);

    EXPECT_EQ(hashes, snapshot_hashes);
}

namespace alternatives {
using Fields = std::vector<std::pair<std::string, FieldDecodeInfo>>;

static const Fields lb_field_info{
    {ChanField::RANGE, {ChanFieldType::UINT32, 0, 0x7fff, -3}},  // uint16 => uint32
    {ChanField::FLAGS, {ChanFieldType::UINT8, 1, 0b10000000, 7}},
    {ChanField::REFLECTIVITY, {ChanFieldType::UINT8, 1, 0xff00, 8}},
    {ChanField::NEAR_IR, {ChanFieldType::UINT16, 2, 0xff00, 4}}  // uint8  => uint16
};

static const Fields single_field_info{
    {ChanField::RANGE, {ChanFieldType::UINT32, 0, 0x0007ffff, 0}},
    {ChanField::FLAGS, {ChanFieldType::UINT8, 2, 0b11111000, 3}},
    {ChanField::REFLECTIVITY, {ChanFieldType::UINT8, 3, 0xff00, 8}},
    {ChanField::SIGNAL, {ChanFieldType::UINT16, 6, 0, 0}},
    {ChanField::NEAR_IR, {ChanFieldType::UINT16, 8, 0, 0}},
    {ChanField::WINDOW, {ChanFieldType::UINT8, 11, 0, 0}}};

static const Fields fusa_info{
    {ChanField::RANGE, {ChanFieldType::UINT32, 0, 0x7fff, -3}},  // uint16 => uint32
    {ChanField::REFLECTIVITY, {ChanFieldType::UINT8, 2, 0xff, 0}},
    {ChanField::NEAR_IR, {ChanFieldType::UINT16, 3, 0xff, -4}},
    {ChanField::RANGE2, {ChanFieldType::UINT32, 4, 0x7fff, -3}},  // uint16 => uint32
    {ChanField::REFLECTIVITY2, {ChanFieldType::UINT8, 6, 0xff, 0}},
    {ChanField::WINDOW, {ChanFieldType::UINT8, 7, 0xff, 0}},
    {ChanField::FLAGS, {ChanFieldType::UINT8, 1, 0b10000000, 7}},
    {ChanField::FLAGS2, {ChanFieldType::UINT8, 5, 0b10000000, 7}},
};

std::map<UDPProfileLidar, UDPProfileLidar> add_profiles() {
    std::map<UDPProfileLidar, UDPProfileLidar> orig_to_custom;
    orig_to_custom[UDPProfileLidar::RNG15_RFL8_NIR8] =
        add_custom_profile("PROFILE_LOWBAND_ALT", lb_field_info, 4);
    orig_to_custom[UDPProfileLidar::RNG19_RFL8_SIG16_NIR16] =
        add_custom_profile("PROFILE_SINGLE_ALT", single_field_info, 12);
    orig_to_custom[UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL] =
        add_custom_profile("PROFILE_FUSA_ALT", fusa_info, 8);
    return orig_to_custom;
}

}  // namespace alternatives

TEST_P(FrameBatcherSnapshotTest, extended_profile_comp_test) {
    static const std::map<UDPProfileLidar, UDPProfileLidar> orig_to_custom =
        alternatives::add_profiles();

    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));
    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));

    UDPProfileLidar prof = info.format.udp_profile_lidar;

    // skip legacy because some parsing specifics are very different
    if (prof == UDPProfileLidar::LEGACY) GTEST_SKIP();
    // skip dual returns because it doesn't have type inconsistencies
    if (prof == UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL) GTEST_SKIP();

    auto get_hashes = [&](UDPProfileLidar profile) -> HashMap {
        HashMap hashes;

        DataFormat format = info.format;
        format.udp_profile_lidar = profile;

        PacketFormat pf(format);

        auto ls = LidarFrame(pf.pixels_per_column, info.format.columns_per_frame,
                             pf.udp_profile_lidar, pf.columns_per_packet);
        uint64_t packet_ts = 1234;  // arbitrary, irrelevant for test
        FrameBatcher batcher(info);

        int packet_index = 0;
        while (pcap.next_packet()) {
            if (pcap.current_info().dst_port == 7502) {
                LidarPacket packet;
                packet.host_timestamp = packet_ts;
                packet.buf.resize(pcap.current_length());
                memcpy(packet.buf.data(), pcap.current_data(), pcap.current_length());
                if (packet_index++ == 63) {
                    EXPECT_TRUE(batcher(packet, ls));
                } else {
                    EXPECT_FALSE(batcher(packet, ls));
                }
            }
        }
        pcap.seek(0);
        impl::foreach_channel_field(ls, pf, matrix_hash{}, hashes);
        return hashes;
    };

    HashMap hashes_orig = get_hashes(prof);
    HashMap hashes_alt = get_hashes(orig_to_custom.find(prof)->second);

    EXPECT_EQ(hashes_orig, hashes_alt);
}

TEST(FrameBatcherLegacyTest, legacy_col_status) {
    std::string pcap_file = "OS-2-32-U0_v2.0.0_1024x10.pcap";
    std::string meta_file = "OS-2-32-U0_v2.0.0_1024x10.json";

    auto data_dir = getenvs("DATA_DIR");

    auto info = metadata_from_json(data_dir + "/" + meta_file);
    PcapReader pcap(data_dir + "/" + pcap_file);

    PacketFormat pf(info);

    while (pcap.next_packet()) {
        if (pcap.current_info().dst_port == 7502) {
            for (uint32_t icol = 0; icol < pf.columns_per_packet; icol++) {
                const uint8_t* col_buf = pf.nth_col(icol, pcap.current_data());
                const uint32_t status = pf.col_status(col_buf);
                EXPECT_EQ(status, 0xFFFFFFFF);
            }
        }
    }
}

TEST(FrameBatcherTest, cached_packet_test) {
    DataFormat df{128,
                  16,
                  1024,
                  0,
                  0,
                  {},
                  {0, 1023},
                  UDPProfileLidar::RNG15_RFL8_NIR8,
                  UDPProfileIMU::LEGACY,
                  HeaderType::STANDARD,
                  10};
    SensorInfo meta{};
    meta.format = df;
    PacketFormat packet_format{meta};

    uint16_t frame_id = 1337;

    auto packets_first = random_frame(meta);
    // drop one last packet so that the frame does not finalize
    packets_first.pop_back();

    auto packets_second = random_frame(meta);

    for (auto& p : packets_first) {
        packet_format.set_frame_id(p.buf.data(), frame_id);
    }
    for (auto& p : packets_second) {
        packet_format.set_frame_id(p.buf.data(), frame_id + 1);
    }

    auto ref_second = LidarFrame(df.pixels_per_column, df.columns_per_frame, df.udp_profile_lidar,
                                 df.columns_per_packet);
    {  // parse ref_second only for final reference
        FrameBatcher batcher{meta};
        for (const auto& p : packets_second) {
            batcher(p, ref_second);
        }
    }

    auto ls = LidarFrame(df.pixels_per_column, df.columns_per_frame, df.udp_profile_lidar,
                         df.columns_per_packet);
    FrameBatcher batcher{meta};
    for (const auto& p : packets_first) {
        ASSERT_FALSE(batcher(p, ls));
    }

    LidarFrame ref_first = ls;

    // get packet to cache and check lidar frame did not change
    for (size_t i = 0; i < batcher.get_max_cache_size() - 1; ++i) {
        ASSERT_FALSE(batcher(packets_second[i], ls));
        EXPECT_EQ(ls, ref_first);
    }

    // this packet should finalize the first frame
    EXPECT_TRUE(batcher(packets_second[0], ls));
    EXPECT_EQ(ls, ref_first);

    std::for_each(packets_second.begin() + 1, packets_second.end() - 1,
                  [&batcher, &ls](const auto& packet) { ASSERT_FALSE(batcher(packet, ls)); });
    EXPECT_TRUE(batcher(packets_second.back(), ls));

    // check frame gets fully batched
    EXPECT_EQ(ls, ref_second);
}

TEST(FrameBatcherImuParsingTest, imu_parsing_test) {
    auto df = DataFormat{128,
                         16,
                         1024,
                         64,
                         1,
                         {},
                         {0, 1023},
                         UDPProfileLidar::OFF,
                         UDPProfileIMU::ACCEL32_GYRO32_NMEA,
                         HeaderType::STANDARD,
                         10};
    SensorInfo info;
    info.format = df;
    auto packet_format = PacketFormat(df);
    auto batcher = FrameBatcher(info);

    // TODO: update from imu_packet_size once we have it
    ImuPacket packet{65535};
    uint8_t* data = packet.buf.data();

    uint64_t timestamp = 133701337;
    std::string sentence = "Freddy Mercury";
    int64_t frame_id = 1337;
    uint8_t alert_flags_val = 0x7F;

    packet_format.set_packet_type(data, 0x2);
    packet_format.set_imu_nmea_ts(data, timestamp);
    packet_format.set_imu_nmea_sentence(data, sentence);
    packet_format.set_frame_id(data, frame_id);
    packet_format.set_alert_flags(data, alert_flags_val);
    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        uint8_t* measurement = packet_format.imu_nth_measurement(i, data);
        packet_format.set_col_measurement_id(measurement, 16 * i);
        packet_format.set_col_timestamp(measurement, 1000 + i);
        packet_format.set_col_status(measurement, 0x1);
        packet_format.set_imu_la_x(measurement, 100 + i);
        packet_format.set_imu_la_y(measurement, 200 + i);
        packet_format.set_imu_la_z(measurement, 300 + i);
        packet_format.set_imu_av_x(measurement, 400 + i);
        packet_format.set_imu_av_y(measurement, 500 + i);
        packet_format.set_imu_av_z(measurement, 600 + i);
    }

    LidarFrame ls(df);
    ASSERT_EQ(ls.frame_id, -1);

    // Check parsing correctness

    batcher(packet, ls);

    ArrayView2<float> acc = ls.field(ChanField::IMU_ACC);
    ArrayView2<float> gyro = ls.field(ChanField::IMU_GYRO);
    ArrayView1<uint64_t> imu_ts = ls.field(ChanField::IMU_TIMESTAMP);
    ArrayView1<uint16_t> imu_m_id = ls.field(ChanField::IMU_MEASUREMENT_ID);
    ArrayView1<uint16_t> imu_status = ls.field(ChanField::IMU_STATUS);

    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        EXPECT_EQ(imu_ts(i), 1000 + i);
        EXPECT_EQ(imu_m_id(i), 16 * i);
        EXPECT_EQ(imu_status(i), 0x1);
        EXPECT_EQ(acc(i, 0), 100 + i);
        EXPECT_EQ(acc(i, 1), 200 + i);
        EXPECT_EQ(acc(i, 2), 300 + i);
        EXPECT_EQ(gyro(i, 0), 400 + i);
        EXPECT_EQ(gyro(i, 1), 500 + i);
        EXPECT_EQ(gyro(i, 2), 600 + i);
    }

    ArrayView1<uint64_t> nmea_ts = ls.field(ChanField::POSITION_TIMESTAMP);
    ArrayView2<char> nmea_sentence = ls.field(ChanField::POSITION_STRING);
    ArrayView2<double> lat_long = ls.field(ChanField::POSITION_LAT_LONG);
    ArrayView1<uint8_t> alert_flags = ls.field(ChanField::IMU_ALERT_FLAGS);

    sentence.resize(NMEA_SENTENCE_LENGTH, '\0');
    EXPECT_EQ(nmea_ts(0), timestamp);
    EXPECT_EQ(alert_flags(0), alert_flags_val);
    EXPECT_EQ(std::strcmp(nmea_sentence.subview(0).data(), sentence.data()), 0);
    EXPECT_TRUE(std::isnan(lat_long(0, 0)));
    EXPECT_TRUE(std::isnan(lat_long(0, 1)));
}

TEST(FrameBatcherTests, serial_number_mismatch) {
    // FIXME[tws] 20260223
    // The FrameBatcher will happily accept packets with the wrong serial
    // number.
    auto df = DataFormat{128,
                         16,
                         1024,
                         0,
                         0,
                         {},
                         {0, 1023},
                         UDPProfileLidar::RNG15_RFL8_NIR8,
                         UDPProfileIMU::LEGACY,
                         HeaderType::STANDARD,
                         10};
    SensorInfo info;
    info.sn = 123;
    EXPECT_EQ(info.sn, 123);
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto packets = random_frame(info);
    info.sn = 456;
    auto batcher = FrameBatcher(info);
    EXPECT_EQ(packets.at(0).prod_sn(), 123);
    LidarFrame ls(df);
    EXPECT_EQ(ls.sensor_info, nullptr);
    EXPECT_FALSE(batcher.batch(packets.at(0), ls));
    ASSERT_EQ(ls.sensor_info->sn, 456);
}

TEST(FrameBatcherTests, reset) {
    // Test that reset properly clears the cache and allows a new frame to be
    // batched
    auto df = DataFormat{128,
                         16,
                         1024,
                         0,
                         0,
                         {},
                         {0, 1023},
                         UDPProfileLidar::RNG15_RFL8_NIR8,
                         UDPProfileIMU::LEGACY,
                         HeaderType::STANDARD,
                         10};
    SensorInfo info;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    auto packets = random_frame(info);

    LidarFrame ls(df);
    // Start a frame but don't finalize it
    for (size_t packet_idx = 0; packet_idx < packets.size() - 1; ++packet_idx) {
        const auto& packet = packets.at(packet_idx);
        ASSERT_FALSE(batcher.batch(packet, ls));
    }
    ASSERT_EQ(ls.frame_id, 700);

    // Create a "future" frame with a new frame id
    LidarFrame future_frame = ls;
    future_frame.frame_id = 702;

    // Add a packet from the future frame to the batcher; it will be cached
    // since the current frame is not finalized
    std::vector<Packet> packets2{};
    impl::frame_to_packets(future_frame, packet_format, std::back_inserter(packets2), info.init_id,
                           info.sn);
    batcher.batch(packets2.at(0), ls);

    // Since the packet is cached, the frame id should not have updated
    ASSERT_EQ(ls.frame_id, 700);

    // Reset the batcher
    batcher.reset();

    // Create a new future frame with a different frame id
    LidarFrame future_frame2 = ls;
    future_frame2.frame_id = 701;
    std::vector<Packet> packets3;
    impl::frame_to_packets(future_frame2, packet_format, std::back_inserter(packets3), info.init_id,
                           info.sn);

    // The packets produced by frame_to_packets should have the new frame_id
    for (const auto& packet : packets3) {
        ASSERT_EQ(packet_format->frame_id(packet.buf.data()), 701);
    }

    // But packets will be cached
    batcher.batch(packets3.at(0), ls);
    ASSERT_EQ(ls.frame_id, 701);
}

TEST(FrameBatcherTests, new_framebatcher_clears_frame) {
    // A brand new FrameBatcher will start a new frame, overwriting any headers
    // in the LidarFrame provided
    auto df = DataFormat{128,
                         16,
                         1024,
                         0,
                         0,
                         {},
                         {0, 1023},
                         UDPProfileLidar::RNG15_RFL8_NIR8,
                         UDPProfileIMU::LEGACY,
                         HeaderType::STANDARD,
                         10};
    SensorInfo info;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    auto packets = random_frame(info);

    LidarFrame ls(df);
    ls.packet_timestamp() = 2000;
    ls.timestamp() = 100;
    ls.status() = 0x0f;
    ls.measurement_id() = 10000;
    ls.frame_id = 123;

    const auto& packet = packets.at(0);
    batcher.batch(packet, ls);

    // First packet timestamp will be set; the rest will be zero
    EXPECT_EQ(ls.packet_timestamp()[0], packet.host_timestamp);
    EXPECT_TRUE((ls.packet_timestamp().segment(1, df.lidar_packets_per_frame() - 1) == 0).all());

    // Columns from the first column should be filled in
    for (uint32_t i = 0; i < df.columns_per_packet; ++i) {
        const uint8_t* col_buf = packet_format->nth_col(i, packet.buf.data());
        uint16_t m_id = packet_format->col_measurement_id(col_buf);
        uint64_t ts = packet_format->col_timestamp(col_buf);
        EXPECT_EQ(ls.timestamp()[m_id], ts);
        EXPECT_EQ(ls.status()[m_id], packet_format->col_status(col_buf));
        EXPECT_EQ(ls.measurement_id()[m_id], m_id);
    }

    // Everything else should be zero
    EXPECT_TRUE(
        ls.timestamp()
            .segment(df.columns_per_packet, df.columns_per_frame - df.columns_per_packet - 1)
            .all() == 0);
    EXPECT_TRUE(
        ls.status()
            .segment(df.columns_per_packet, df.columns_per_frame - df.columns_per_packet - 1)
            .all() == 0);
    EXPECT_TRUE(
        ls.measurement_id()
            .segment(df.columns_per_packet, df.columns_per_frame - df.columns_per_packet - 1)
            .all() == 0);
}

TEST(FrameBatcherTests, init_id) {
    // it properly handles changes to init id
    auto df = DataFormat{128,
                         16,
                         1024,
                         0,
                         0,
                         {},
                         {0, 1023},
                         UDPProfileLidar::RNG15_RFL8_NIR8,
                         UDPProfileIMU::OFF,
                         HeaderType::STANDARD,
                         10};
    SensorInfo info;
    info.init_id = 42;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    auto packets = random_frame(info);
    auto lidar_frame = LidarFrame(info);

    // finalize a frame
    for (size_t packet_idx = 0; packet_idx < packets.size() - 1; ++packet_idx) {
        const auto& packet = packets.at(packet_idx);
        ASSERT_FALSE(batcher.batch(packet, lidar_frame));
    }
    auto& packet = packets.at(packets.size() - 1);
    ASSERT_TRUE(batcher.batch(packet, lidar_frame));

    // change the init_id
    info.init_id += 1;
    auto packets2 = random_frame(info);
    for (size_t packet_idx = 0; packet_idx < packets2.size() - 1; ++packet_idx) {
        const auto& packet = packets2.at(packet_idx);
        ASSERT_FALSE(batcher.batch(packet, lidar_frame));
    }
    packet = packets2.at(packets2.size() - 1);
    ASSERT_TRUE(batcher.batch(packet, lidar_frame));
}

TEST(FrameBatcherTests, init_id_2) {
    // it properly handles a mid-frame change to init id
    auto df = DataFormat{128,
                         16,
                         1024,
                         0,
                         0,
                         {},
                         {0, 1023},
                         UDPProfileLidar::RNG15_RFL8_NIR8,
                         UDPProfileIMU::OFF,
                         HeaderType::STANDARD,
                         10};
    SensorInfo info;
    info.init_id = 42;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    auto packets = random_frame(info);
    auto lidar_frame = LidarFrame(info);

    // batch an incomplete frame
    for (size_t packet_idx = 0; packet_idx < packets.size() - 1; ++packet_idx) {
        const auto& packet = packets.at(packet_idx);
        ASSERT_FALSE(batcher.batch(packet, lidar_frame));
    }

    // change the init_id, which will cause the next packet to finalize the
    // previous frame
    LidarPacket packet(packet_format->lidar_packet_size);
    packet_format->set_frame_id(packet.buf.data(), 701);
    packet_format->set_init_id(packet.buf.data(), info.init_id + 1);
    ASSERT_TRUE(batcher.batch(packet, lidar_frame));
    ASSERT_EQ(lidar_frame.frame_id, 700);  // from random_frame
}

TEST(FrameBatcherTests, lost_frame) {
    // A frame will get "skipped" if the first packet from a later frame arrives
    // first.
    DataFormat df{128,
                  16,
                  1024,
                  0,
                  0,
                  {},
                  {0, 1023},
                  UDPProfileLidar::OFF,
                  UDPProfileIMU::OFF,
                  HeaderType::STANDARD,
                  10};
    df.zone_monitoring_enabled = true;
    SensorInfo info;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    ASSERT_EQ(df.zone_monitoring_enabled, packet_format->zone_monitoring_enabled);
    ZonePacket zm_packet(packet_format);
    LidarFrame ls(df);

    // Add the one required packet - starts and finalizes frame 700
    packet_format->set_frame_id(zm_packet.buf.data(), 700);
    ASSERT_TRUE(batcher.batch(zm_packet, ls));
    // Add the one required packet - starts and finalizes frame 702
    packet_format->set_frame_id(zm_packet.buf.data(), 702);
    ASSERT_TRUE(batcher.batch(zm_packet, ls));
    // Add a packet from the lost frame, and it's ignored.
    packet_format->set_frame_id(zm_packet.buf.data(), 701);
    ASSERT_FALSE(batcher.batch(zm_packet, ls));
}

TEST(FrameBatcherTests, constructors_sensor_info_side_effects) {
    // FIXME [tws] 20260225
    // The FrameBatcher constructors have inconsistent behavior with regards to
    // modifying the SensorInfo passed in.
    {  // scenario 1: SensorInfo provided as a const ref.
        DataFormat df{128,
                      16,
                      1024,
                      0,
                      0,
                      {},
                      {0, 1023},
                      UDPProfileLidar::OFF,
                      UDPProfileIMU::OFF,
                      HeaderType::STANDARD,
                      10};
        df.zone_monitoring_enabled = true;
        SensorInfo info;
        info.sn = 123;
        info.format = df;
        auto packet_format = std::make_shared<PacketFormat>(df);
        FrameBatcher batcher(info);

        // batch a packet
        ZonePacket zm_packet(packet_format);
        packet_format->set_frame_id(zm_packet.buf.data(), 700);
        LidarFrame ls(df);
        ASSERT_TRUE(batcher.batch(zm_packet, ls));
        // the sn from the lidar frame matches the info
        ASSERT_EQ(ls.sensor_info->sn, 123);

        info.sn = 456;
        // batch a packet
        packet_format->set_frame_id(zm_packet.buf.data(), 701);
        ASSERT_TRUE(batcher.batch(zm_packet, ls));
        // the sn from the lidar frame doesn't match the info
        ASSERT_EQ(ls.sensor_info->sn, 123);
    }
    {  // scenario 2: SensorInfo provided as a shared ptr
        DataFormat df{128,
                      16,
                      1024,
                      0,
                      0,
                      {},
                      {0, 1023},
                      UDPProfileLidar::OFF,
                      UDPProfileIMU::OFF,
                      HeaderType::STANDARD,
                      10};
        df.zone_monitoring_enabled = true;
        auto info = std::make_shared<SensorInfo>();
        info->sn = 123;
        info->format = df;
        auto packet_format = std::make_shared<PacketFormat>(df);
        FrameBatcher batcher(info);

        // batch a packet
        ZonePacket zm_packet(packet_format);
        packet_format->set_frame_id(zm_packet.buf.data(), 700);
        LidarFrame ls(df);
        ASSERT_TRUE(batcher.batch(zm_packet, ls));
        // the sn from the lidar frame matches the info
        ASSERT_EQ(ls.sensor_info->sn, 123);

        info->sn = 456;
        // batch a packet
        packet_format->set_frame_id(zm_packet.buf.data(), 701);
        ASSERT_TRUE(batcher.batch(zm_packet, ls));
        // the sn from the lidar frame _DOES_ match the info
        ASSERT_EQ(ls.sensor_info->sn, 456);
    }
}

class FrameBatcherRollOverTest : public ::testing::TestWithParam<std::tuple<HeaderType>> {};
INSTANTIATE_TEST_CASE_P(FrameBatcherRollOverTests, FrameBatcherRollOverTest,
                        ::testing::Values(HeaderType::STANDARD, HeaderType::FUSA));

TEST_P(FrameBatcherRollOverTest, bad_roll_over_32_bit) {
    auto header_type = std::get<0>(GetParam());
    // It should throw an exception if roll-over happens for a 32-bit frame id.
    // Expects a single lidar packet per frame.
    DataFormat df{16,
                  16,
                  16,
                  0,
                  0,
                  {},
                  {0, 15},
                  UDPProfileLidar::RNG15_RFL8_NIR8,
                  UDPProfileIMU::OFF,
                  header_type,
                  10};
    SensorInfo info;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    LidarFrame ls(info);
    EXPECT_EQ(ls.frame_id, -1);

    LidarPacket packet(packet_format);

    // Frame with max frame id - should be accepted
    packet_format->set_frame_id(packet.buf.data(), packet_format->max_frame_id);
    packet_format->set_init_id(packet.buf.data(), info.init_id);
    packet.host_timestamp = 1234;
    EXPECT_TRUE(batcher.batch(packet, ls));
    EXPECT_EQ(ls.frame_id, packet_format->max_frame_id);

    // Roll-over occured - should throw
    packet_format->set_frame_id(packet.buf.data(), 0);
    packet_format->set_init_id(packet.buf.data(), info.init_id);
    packet.host_timestamp = 1234;
    if (header_type == HeaderType::STANDARD) {
        // Standard header should allow roll-over, so this should not throw
        EXPECT_TRUE(batcher.batch(packet, ls));
        EXPECT_EQ(ls.frame_id, 0);
    } else {
        EXPECT_THROW(
            {
                try {
                    batcher.batch(packet, ls);
                } catch (const std::runtime_error& e) {
                    // Check that the exception message contains "frame id
                    // roll-over"
                    EXPECT_TRUE(std::string(e.what()).find("32-bit frame id did not increase since "
                                                           "the last frame") != std::string::npos);
                    throw;
                }
            },
            std::runtime_error);

        // Frame id decreases but the init id is different - doesn't throw
        packet_format->set_init_id(packet.buf.data(), info.init_id + 1);
        EXPECT_TRUE(batcher.batch(packet, ls));
    }
}

TEST(FrameBatcherTest, reused_lidarframe_fields_cleared) {
    using namespace ouster::sdk::core::ChanField;

    // It should throw an exception if roll-over happens for a 32-bit frame id.
    // Expects a single lidar packet per frame.
    DataFormat df{16,
                  16,
                  16,
                  0,
                  0,
                  {},
                  {0, 15},
                  UDPProfileLidar::RNG15_RFL8_NIR8,
                  UDPProfileIMU::ACCEL32_GYRO32_NMEA,
                  HeaderType::STANDARD,
                  10};
    df.zone_monitoring_enabled = true;
    SensorInfo info;
    info.format = df;
    auto packet_format = std::make_shared<PacketFormat>(df);
    auto batcher = FrameBatcher(info);
    LidarFrame lidar_frame(info);
    EXPECT_EQ(lidar_frame.frame_id, -1);

    // fill the fields with some made-up data
    lidar_frame.field(IMU_TIMESTAMP).fill(uint64_t{1234});
    lidar_frame.field(IMU_PACKET_TIMESTAMP).fill(uint64_t{5678});
    lidar_frame.field(IMU_MEASUREMENT_ID).fill(uint16_t{42});
    lidar_frame.field(IMU_STATUS).fill(uint16_t{0x1});
    lidar_frame.field(IMU_ALERT_FLAGS).fill(uint8_t{0x7F});
    lidar_frame.field(IMU_ACC).fill(41.0f);
    lidar_frame.field(IMU_GYRO).fill(42.0f);
    lidar_frame.field(ZONE_ALERT_FLAGS).fill(uint8_t{0x7F});
    lidar_frame.field(ZONE_PACKET_TIMESTAMP).fill(uint64_t{4321});
    lidar_frame.field(ZONE_TIMESTAMP).fill(uint64_t{8765});
    lidar_frame.field(LIVE_ZONESET_HASH).fill(uint8_t{0xFF});
    ZoneState zone_state;
    zone_state.id = 7;
    lidar_frame.field(ZONE_STATES).fill(zone_state);

    ZonePacket zm_packet(packet_format);

    // because lidar_frame.frame_id is -1, this will start a new frame, which
    // zeros out the headers
    batcher.batch(zm_packet, lidar_frame);

    // check all fields that should have been zeroed
    ArrayView1<uint64_t> imu_ts = lidar_frame.field(ChanField::IMU_TIMESTAMP);
    ArrayView1<uint64_t> imu_packet_ts = lidar_frame.field(ChanField::IMU_PACKET_TIMESTAMP);
    ArrayView1<uint16_t> imu_m_id = lidar_frame.field(ChanField::IMU_MEASUREMENT_ID);
    ArrayView1<uint16_t> imu_status = lidar_frame.field(ChanField::IMU_STATUS);
    ArrayView1<uint8_t> imu_alert_flags = lidar_frame.field(ChanField::IMU_ALERT_FLAGS);
    ArrayView2<float> acc = lidar_frame.field(ChanField::IMU_ACC);
    ArrayView2<float> gyro = lidar_frame.field(ChanField::IMU_GYRO);
    ArrayView1<uint8_t> zone_alert_flags = lidar_frame.field(ZONE_ALERT_FLAGS);
    ArrayView1<uint64_t> zone_packet_ts = lidar_frame.field(ZONE_PACKET_TIMESTAMP);
    ArrayView1<uint64_t> zone_ts = lidar_frame.field(ZONE_TIMESTAMP);
    ArrayView1<uint8_t> live_zoneset_hash = lidar_frame.field(LIVE_ZONESET_HASH);
    ArrayView1<ZoneState> zone_states = lidar_frame.field(ZONE_STATES);

    auto check_zero = [](auto& field) {
        for (size_t i = 0; i < field.size(); ++i) {
            EXPECT_EQ(field.data()[i], 0);
        }
    };
    check_zero(imu_ts);
    check_zero(imu_packet_ts);
    check_zero(imu_m_id);
    check_zero(imu_status);
    check_zero(imu_alert_flags);
    check_zero(acc);
    check_zero(gyro);
    check_zero(zone_alert_flags);
    check_zero(zone_packet_ts);
    check_zero(zone_ts);
    check_zero(live_zoneset_hash);

    for (size_t i = 0; i < zone_states.size(); ++i) {
        EXPECT_EQ(zone_states.data()[i], ZoneState{});
    }
}
