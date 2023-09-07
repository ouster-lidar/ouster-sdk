/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/impl/packet_writer.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>

#include "ouster/lidar_scan.h"
#include "ouster/pcap.h"
#include "util.h"

using namespace ouster;
using namespace ouster::sensor;
using namespace ouster::sensor::impl;

// TODO: we should just make FieldInfo's publicly available
namespace ouster {
namespace sensor {
namespace impl {

struct FieldInfo {
    ChanFieldType ty_tag;
    size_t offset;
    uint64_t mask;
    int shift;
};

struct ProfileEntry {
    const std::pair<ChanField, FieldInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> profiles;

uint64_t get_value_mask(const FieldInfo& f);
int get_bitness(const FieldInfo& f);

std::map<ChanField, FieldInfo> get_fields(UDPProfileLidar profile) {
    auto end = profiles.end();
    auto it =
        std::find_if(impl::profiles.begin(), end,
                     [profile](const auto& kv) { return kv.first == profile; });

    auto& entry = it->second;
    return {entry.fields, entry.fields + entry.n_fields};
}

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

using bitness_param = std::tuple<UDPProfileLidar, std::map<ChanField, int>>;
class FieldInfoSanityTest : public ::testing::TestWithParam<bitness_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    FieldInfoSanityTests,
    FieldInfoSanityTest,
    ::testing::Values(
        bitness_param{UDPProfileLidar::PROFILE_LIDAR_LEGACY,
                      {{ChanField::RANGE, 20},
                       {ChanField::FLAGS, 4},
                       {ChanField::REFLECTIVITY, 16},
                       {ChanField::SIGNAL, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32}}},
        bitness_param{UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
                      {{ChanField::RANGE, 15},
                       {ChanField::FLAGS, 1},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::NEAR_IR, 8},
                       {ChanField::RAW32_WORD1, 32}}},
        bitness_param{UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
                      {{ChanField::RANGE, 19},
                       {ChanField::FLAGS, 5},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32}}},
        bitness_param{UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
                      {{ChanField::RANGE, 19},
                       {ChanField::FLAGS, 5},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::RANGE2, 19},
                       {ChanField::FLAGS2, 5},
                       {ChanField::REFLECTIVITY2, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::SIGNAL2, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32},
                       {ChanField::RAW32_WORD4, 32}}},
        bitness_param{UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL,
                      {{ChanField::RANGE, 15},
                       {ChanField::FLAGS, 1},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::RANGE2, 15},
                       {ChanField::FLAGS2, 1},
                       {ChanField::REFLECTIVITY2, 8},
                       {ChanField::NEAR_IR, 8},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32}}}));
// clang-format on

TEST_P(FieldInfoSanityTest, field_info_sanity_checks) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    auto chan_bitness_map = std::get<1>(param);

    auto fields = get_fields(profile);

    for (const auto& kv : fields) {
        auto f = kv.second;
        uint64_t type_mask =
            (uint64_t{1} << (field_type_size(f.ty_tag) * 8)) - 1;
        uint64_t value_mask = get_value_mask(f);

        EXPECT_EQ(get_bitness(f), chan_bitness_map[kv.first]);
        if (f.shift < 0) {
            if (f.mask) {
                EXPECT_EQ(value_mask, f.mask << std::abs(f.shift));
            } else {
                EXPECT_EQ(value_mask, type_mask << std::abs(f.shift));
            }
        } else {
            if (f.mask) {
                EXPECT_EQ(value_mask, f.mask >> f.shift);
            } else {
                EXPECT_EQ(value_mask, type_mask >> f.shift);
            }
        }
    }
}

using test_param = std::tuple<UDPProfileLidar, size_t, size_t, size_t>;
class PacketWriterTest : public ::testing::TestWithParam<test_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    PacketWriterTests,
    PacketWriterTest,
    ::testing::Combine(
        ::testing::Values(
            UDPProfileLidar::PROFILE_LIDAR_LEGACY,
            UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
            UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
            UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
            UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL),
        ::testing::Values(1024), // columns_per_frame
        ::testing::Values(128),  // pixels_per_column
        ::testing::Values(16))); // columns_per_packet
// clang-format on

struct cmp_field {
    LidarScan& ls;

    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField i) {
        // FUSA hacks
        if (i >= ChanField::RAW_HEADERS) return;

        EXPECT_TRUE((ls.field<T>(i) == field).all());
    }
};

TEST_P(PacketWriterTest, packet_writer_headers_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    size_t pixels_per_column = std::get<2>(param);
    size_t columns_per_packet = std::get<3>(param);

    LidarPacket p;
    packet_format pf(profile, pixels_per_column, columns_per_packet);
    packet_writer pw{pf};

    pw.set_col_status(pw.nth_col(9, p.buf.data()), 123);
    EXPECT_EQ(pf.col_status(pf.nth_col(9, p.buf.data())), 123);

    pw.set_col_timestamp(pw.nth_col(11, p.buf.data()), 80899);
    EXPECT_EQ(pf.col_timestamp(pf.nth_col(11, p.buf.data())), 80899);

    pw.set_col_measurement_id(pw.nth_col(7, p.buf.data()), 613);
    EXPECT_EQ(pf.col_measurement_id(pf.nth_col(7, p.buf.data())), 613);

    pw.set_frame_id(p.buf.data(), 777);
    EXPECT_EQ(pf.frame_id(p.buf.data()), 777);
}

TEST_P(PacketWriterTest, packet_writer_randomize_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    size_t columns_per_frame = std::get<1>(param);
    size_t pixels_per_column = std::get<2>(param);
    size_t columns_per_packet = std::get<3>(param);

    packet_format pf(profile, pixels_per_column, columns_per_packet);
    packet_writer pw{pf};

    // create randomised lidar scan
    auto ls = LidarScan(columns_per_frame, pixels_per_column, profile,
                        columns_per_packet);
    std::iota(ls.measurement_id().data(),
              ls.measurement_id().data() + ls.measurement_id().size(), 0);
    std::iota(ls.packet_timestamp().data(),
              ls.packet_timestamp().data() + ls.packet_timestamp().size(), 10);
    std::iota(ls.timestamp().data(),
              ls.timestamp().data() + ls.timestamp().size(), 1000);
    std::fill(ls.status().data(), ls.status().data() + ls.status().size(), 0x1);
    ls.frame_id = 700;

    auto randomise = [&](auto ref_field, ChanField i) {
        // need this to skip RAW32_WORD*
        if (i >= ChanField::RAW_HEADERS) return;

        // use static seed so that the test does not flake some day and
        // collectively piss off a bunch of angry developers
        randomize_field(ref_field, pw.field_value_mask(i), 0xdeadbeef);
    };
    ouster::impl::foreach_field(ls, randomise);

    auto fields = get_fields(profile);
    auto verify_field = [&](auto ref_field, ChanField i) {
        // need this to skip RAW32_WORD*
        if (i >= ChanField::RAW_HEADERS) return;

        // field should not be all zeros
        EXPECT_FALSE((ref_field == 0).all());

        FieldInfo f = fields.at(i);
        // value mask tells us how the output needs to look like
        uint64_t value_mask = get_value_mask(f);

        using T = typename decltype(ref_field)::Scalar;
        T* data = ref_field.data();
        uint64_t field_mask = 0;
        for (int i = 0; i < ref_field.size(); ++i) {
            T value = *(data + i);
            // output must perfectly fit into the value mask
            EXPECT_EQ(value, value & value_mask);
            field_mask |= value;
        }
        // verify all possible bits were covered
        EXPECT_EQ(field_mask, value_mask);
    };
    ouster::impl::foreach_field(ls, verify_field);

    // produced and re-parsed packets should result in the same scan
    auto packets = std::vector<LidarPacket>{};
    ouster::impl::scan_to_packets(ls, pw, std::back_inserter(packets));

    ASSERT_EQ(packets.size(), 64);

    auto ls2 = LidarScan(columns_per_frame, pixels_per_column, profile,
                         columns_per_packet);
    ScanBatcher batcher(columns_per_frame, pf);
    for (const auto& p : packets) EXPECT_FALSE(batcher(p, ls2));

    EXPECT_EQ(ls.frame_id, ls2.frame_id);
    EXPECT_TRUE((ls.packet_timestamp() == ls2.packet_timestamp()).all());
    EXPECT_TRUE((ls.status() == ls2.status()).all());
    EXPECT_TRUE((ls.timestamp() == ls2.timestamp()).all());
    EXPECT_TRUE((ls.measurement_id() == ls2.measurement_id()).all());

    ouster::impl::foreach_field(ls2, cmp_field{ls});
}

TEST_P(PacketWriterTest, scans_to_packets_skips_dropped_packets_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    size_t columns_per_frame = std::get<1>(param);
    size_t pixels_per_column = std::get<2>(param);
    size_t columns_per_packet = std::get<3>(param);

    packet_format pf(profile, pixels_per_column, columns_per_packet);
    packet_writer pw{pf};

    // create randomised lidar scan
    auto ls = LidarScan(columns_per_frame, pixels_per_column, profile,
                        columns_per_packet);
    std::iota(ls.measurement_id().data(),
              ls.measurement_id().data() + ls.measurement_id().size(), 0);
    std::iota(ls.packet_timestamp().data(),
              ls.packet_timestamp().data() + ls.packet_timestamp().size(), 10);
    std::iota(ls.timestamp().data(),
              ls.timestamp().data() + ls.timestamp().size(), 1000);
    std::fill(ls.status().data(), ls.status().data() + ls.status().size(), 0x1);
    ls.frame_id = 700;

    auto randomise = [&](auto ref_field, ChanField i) {
        // need this to skip RAW32_WORD*
        if (i >= ChanField::RAW_HEADERS) return;

        // use static seed so that the test does not flake some day and
        // collectively piss off a bunch of angry developers
        randomize_field(ref_field, pw.field_value_mask(i), 0xdeadbeef);
    };
    ouster::impl::foreach_field(ls, randomise);

    auto packets_orig = std::vector<LidarPacket>{};
    ouster::impl::scan_to_packets(ls, pw, std::back_inserter(packets_orig));

    ASSERT_EQ(packets_orig.size(), 64);

    // drop one packet
    packets_orig.erase(packets_orig.begin() + 14);

    // disable every second column in first packet
    for (size_t icol = 0; icol < columns_per_packet; ++icol) {
        auto& p = packets_orig[0];
        if (icol % 2 == 0) pw.set_col_status(pw.nth_col(icol, p.buf.data()), 0);
    }

    auto ls_repr = LidarScan(columns_per_frame, pixels_per_column, profile,
                             columns_per_packet);
    ScanBatcher batcher(columns_per_frame, pf);
    for (const auto& p : packets_orig) {
        EXPECT_FALSE(batcher(p, ls_repr));
    }

    auto packets_repr = std::vector<LidarPacket>{};
    ouster::impl::scan_to_packets(ls_repr, pw,
                                  std::back_inserter(packets_repr));
    EXPECT_EQ(packets_repr.size(), 63);
    EXPECT_EQ(packets_repr[14].host_timestamp, 25);

    // check disabled column channel data blocks are completely empty
    for (size_t icol = 0; icol < columns_per_packet; ++icol) {
        auto& p = packets_repr[0];
        uint8_t* col_buf = pw.nth_col(icol, p.buf.data());
        if (icol % 2 == 0) {
            const uint8_t* begin = pw.nth_px(0, col_buf);
            const uint8_t* end = pw.nth_col(icol + 1, p.buf.data());
            EXPECT_TRUE(
                std::all_of(begin, end, [](uint8_t v) { return v == 0; }));
        }
    }
}

using data_param = std::tuple<std::string, std::string>;
class PacketWriterDataTest : public ::testing::TestWithParam<data_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    PacketWriterDataTests,
    PacketWriterDataTest,
    ::testing::Values(
        // low bandwidth
        data_param{"OS-0-128-U1_v2.3.0_1024x10.pcap",
                   "OS-0-128-U1_v2.3.0_1024x10.json"},
        // dual return
        data_param{"OS-0-32-U1_v2.2.0_1024x10.pcap",
                   "OS-0-32-U1_v2.2.0_1024x10.json"},
        // fusa dual return
        data_param{"OS-1-128_767798045_1024x10_20230712_120049.pcap",
                   "OS-1-128_767798045_1024x10_20230712_120049.json"},
        // single return
        data_param{"OS-2-128-U1_v2.3.0_1024x10.pcap",
                   "OS-2-128-U1_v2.3.0_1024x10.json"},
        // legacy
        data_param{"OS-2-32-U0_v2.0.0_1024x10.pcap",
                   "OS-2-32-U0_v2.0.0_1024x10.json"}));
// clang-format on

using namespace ouster::sensor_utils;

TEST_P(PacketWriterDataTest, packet_writer_data_repr_test) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));

    auto pw = packet_writer(info);

    auto ls_orig =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  pw.udp_profile_lidar, pw.columns_per_packet);

    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
    ScanBatcher batcher(info.format.columns_per_frame, pw);
    int n_packets = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            ASSERT_FALSE(batcher(pcap.current_data(), 0, ls_orig));
            ++n_packets;
        }

    // produced and re-parsed fields should match
    auto packets = std::vector<LidarPacket>{};
    ouster::impl::scan_to_packets(ls_orig, pw, std::back_inserter(packets));
    ASSERT_EQ(packets.size(), n_packets);

    auto ls_repr =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  pw.udp_profile_lidar, pw.columns_per_packet);
    ScanBatcher repr_batcher(info.format.columns_per_frame, pw);
    for (auto& p : packets) {
        ASSERT_FALSE(repr_batcher(p, ls_repr));
    }

    ouster::impl::foreach_field(ls_repr, cmp_field{ls_orig});
}

TEST_P(PacketWriterDataTest, packet_writer_raw_headers_match_test) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));

    auto pw = packet_writer(info);

    auto rh_types = get_field_types(info);
    rh_types.emplace_back(ChanField::RAW_HEADERS, ChanFieldType::UINT32);

    auto rh_ls_orig =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  rh_types.begin(), rh_types.end(), pw.columns_per_packet);

    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
    ScanBatcher batcher(info.format.columns_per_frame, pw);
    int n_packets = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            ASSERT_FALSE(batcher(pcap.current_data(), 0, rh_ls_orig));
            ++n_packets;
        }

    // produced and re-parsed RAW_HEADERS fields should match
    auto packets = std::vector<LidarPacket>{};
    ouster::impl::scan_to_packets(rh_ls_orig, pw, std::back_inserter(packets));
    ASSERT_EQ(packets.size(), n_packets);

    auto rh_ls_repr =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  rh_types.begin(), rh_types.end(), pw.columns_per_packet);
    ScanBatcher repr_batcher(info.format.columns_per_frame, pw);
    for (auto& p : packets) {
        ASSERT_FALSE(repr_batcher(p, rh_ls_repr));
    }

    auto rh_orig = rh_ls_orig.field<uint32_t>(ChanField::RAW_HEADERS);
    auto rh_repr = rh_ls_repr.field<uint32_t>(ChanField::RAW_HEADERS);
    EXPECT_TRUE((rh_orig == rh_repr).all());
}
