/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>

#include "ouster/core/field_decode_info.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/pcap/pcap.h"
#include "util.h"

using namespace ouster::sdk::core;
using namespace ouster::sdk::core::impl;
using namespace ouster::sdk::pcap;

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

struct ProfileEntry {
    const std::pair<std::string, FieldDecodeInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

OUSTER_API_FUNCTION Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> get_profiles();

/**
 * @todo please find some way around removing this, this is causing shared lib
 * issues
 */
uint64_t get_value_mask(const FieldDecodeInfo& f);
/**
 * @todo please find some way around removing this, this is causing shared lib
 * issues
 */
int get_bitness(const FieldDecodeInfo& f);

std::map<std::string, FieldDecodeInfo> get_fields(UDPProfileLidar profile) {
    auto profiles = get_profiles();
    auto end = profiles.end();
    auto it = std::find_if(profiles.begin(), end,
                           [profile](const auto& kv) { return kv.first == profile; });

    auto& entry = it->second;
    return {entry.fields, entry.fields + entry.n_fields};
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

using bitness_param = std::tuple<UDPProfileLidar, std::map<std::string, int>>;
class FieldDecodeInfoSanityTest : public ::testing::TestWithParam<bitness_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    FieldDecodeInfoSanityTests,
    FieldDecodeInfoSanityTest,
    ::testing::Values(
        bitness_param{UDPProfileLidar::LEGACY,
                      {{ChanField::RANGE, 20},
                       {ChanField::FLAGS, 4},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32}}},
        bitness_param{UDPProfileLidar::RNG15_RFL8_NIR8,
                      {{ChanField::RANGE, 15},
                       {ChanField::FLAGS, 1},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::NEAR_IR, 8},
                       {ChanField::RAW32_WORD1, 32}}},
        bitness_param{UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
                      {{ChanField::RANGE, 19},
                       {ChanField::FLAGS, 5},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::WINDOW, 8},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32}}},
        bitness_param{UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
                      {{ChanField::RANGE, 19},
                       {ChanField::FLAGS, 5},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::RANGE2, 19},
                       {ChanField::FLAGS2, 5},
                       {ChanField::REFLECTIVITY2, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::SIGNAL2, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::WINDOW, 8},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32},
                       {ChanField::RAW32_WORD4, 32}}},
        bitness_param{UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL,
                      {{ChanField::RANGE, 15},
                       {ChanField::FLAGS, 1},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::RANGE2, 15},
                       {ChanField::FLAGS2, 1},
                       {ChanField::REFLECTIVITY2, 8},
                       {ChanField::NEAR_IR, 8},
                       {ChanField::WINDOW, 8},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32}}}));
// clang-format on

TEST_P(FieldDecodeInfoSanityTest, field_info_sanity_checks) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    auto chan_bitness_map = std::get<1>(param);

    auto fields = get_fields(profile);

    for (const auto& kv : fields) {
        auto f = kv.second;
        uint64_t type_mask = (uint64_t{1} << (field_type_size(f.ty_tag) * 8)) - 1;
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

using test_param = std::tuple<UDPProfileLidar, HeaderType, uint32_t, uint32_t, uint32_t>;
class PacketFormatTest : public ::testing::TestWithParam<test_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    PacketFormatTests,
    PacketFormatTest,
    ::testing::Combine(
        ::testing::Values(
            UDPProfileLidar::LEGACY,
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
            UDPProfileLidar::RNG15_RFL8_NIR8,
            UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL),
        ::testing::Values(
            HeaderType::STANDARD,
            HeaderType::FUSA),
        ::testing::Values(1024), // columns_per_frame
        ::testing::Values(128),  // pixels_per_column
        ::testing::Values(16))); // columns_per_packet
// clang-format on

struct cmp_field {
    LidarFrame& ls;

    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& i) {
        EXPECT_TRUE((ls.field<T>(i) == field).all());
    }
};

TEST_P(PacketFormatTest, packet_packet_format_headers_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    HeaderType profile_type = std::get<1>(param);
    uint32_t columns_per_frame = std::get<2>(param);
    uint32_t pixels_per_column = std::get<3>(param);
    uint32_t columns_per_packet = std::get<4>(param);

    DataFormat df{pixels_per_column, columns_per_packet,    columns_per_frame, 0, 0, {}, {},
                  profile,           UDPProfileIMU::LEGACY, profile_type,      10};
    PacketFormat packet_format{df};
    LidarPacket p(packet_format.lidar_packet_size);

    packet_format.set_col_status(packet_format.nth_col(9, p.buf.data()), 123);
    EXPECT_EQ(packet_format.col_status(packet_format.nth_col(9, p.buf.data())), 123);

    packet_format.set_col_timestamp(packet_format.nth_col(11, p.buf.data()), 80899);
    EXPECT_EQ(packet_format.col_timestamp(packet_format.nth_col(11, p.buf.data())), 80899);

    packet_format.set_col_measurement_id(packet_format.nth_col(7, p.buf.data()), 613);
    EXPECT_EQ(packet_format.col_measurement_id(packet_format.nth_col(7, p.buf.data())), 613);

    packet_format.set_frame_id(p.buf.data(), 777);
    EXPECT_EQ(packet_format.frame_id(p.buf.data()), 777);

    if (profile != UDPProfileLidar::LEGACY) {
        packet_format.set_init_id(p.buf.data(), 0x123456);
        EXPECT_EQ(packet_format.init_id(p.buf.data()), 0x123456);

        packet_format.set_prod_sn(p.buf.data(), 0x1234567890);
        EXPECT_EQ(packet_format.prod_sn(p.buf.data()), 0x1234567890);
    }
}

TEST_P(PacketFormatTest, packet_packet_format_randomize_test) {
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

    auto g = std::mt19937(0xdeadbeef);
    auto dinit_id = std::uniform_int_distribution<uint32_t>(0, 0xFFFFFF);
    auto dserial_no = std::uniform_int_distribution<uint64_t>(0, 0xFFFFFFFFFF);

    uint32_t init_id = dinit_id(g);      // 24 bits
    uint64_t serial_no = dserial_no(g);  // 40 bits

    SensorInfo info;
    info.init_id = init_id;
    info.sn = serial_no;
    info.format = df;
    PacketFormat packet_format{info};

    // create randomised lidar frame
    auto ls = LidarFrame(pixels_per_column, columns_per_frame, profile, columns_per_packet);
    std::iota(ls.measurement_id().data(), ls.measurement_id().data() + ls.measurement_id().size(),
              0);
    std::iota(ls.packet_timestamp().data(),
              ls.packet_timestamp().data() + ls.packet_timestamp().size(), 10);
    std::iota(ls.timestamp().data(), ls.timestamp().data() + ls.timestamp().size(), 1000);
    std::fill(ls.status().data(), ls.status().data() + ls.status().size(), 0x1);
    ls.frame_id = 700;

    auto randomise = [&](auto ref_field, const std::string& i) {
        // use static seed so that the test does not flake some day and
        // collectively piss off a bunch of angry developers
        randomize_field(ref_field, packet_format.field_value_mask(i), 0xdeadbeef);
    };
    impl::foreach_channel_field(ls, packet_format, randomise);

    auto field_decoders = get_fields(profile);
    auto verify_field = [&](auto ref_field, const std::string& i) {
        // field should not be all zeros
        EXPECT_FALSE((ref_field == 0).all());

        FieldDecodeInfo f = field_decoders.at(i);
        // value mask tells us how the output needs to look like
        uint64_t value_mask = get_value_mask(f);

        using T = typename decltype(ref_field)::Scalar;
        T* data = ref_field.data();
        uint64_t field_mask = 0;
        for (int i = 0; i < ref_field.size(); ++i) {
            T value = *(data + i);
            uint64_t value_bits = 0;
            memcpy(&value_bits, &value, sizeof(T));
            // output must perfectly fit into the value mask
            EXPECT_EQ(value_bits, value_bits & value_mask);
            field_mask |= value_bits;
        }
        // verify all possible bits were covered
        EXPECT_EQ(field_mask, value_mask);
    };
    impl::foreach_channel_field(ls, packet_format, verify_field);

    // produced and re-parsed packets should result in the same frame
    auto packets = std::vector<Packet>{};
    impl::frame_to_packets(ls, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets), init_id, serial_no);

    ASSERT_EQ(packets.size(), 64);

    // validate the init id and serial no in each packet if supported
    if (profile != UDPProfileLidar::LEGACY) {
        for (const auto& p : packets) {
            ASSERT_EQ(init_id, packet_format.init_id(p.buf.data()));
            ASSERT_EQ(serial_no, packet_format.prod_sn(p.buf.data()));
        }
    }

    auto ls2 = LidarFrame(pixels_per_column, columns_per_frame, profile, columns_per_packet);
    FrameBatcher batcher(info);
    for (size_t i = 0; i < packets.size(); i++) {
        const auto& p = packets[i];
        if (i == 63) {
            EXPECT_TRUE(batcher(p, ls2));
        } else {
            EXPECT_FALSE(batcher(p, ls2));
        }
    }

    EXPECT_EQ(ls.frame_id, ls2.frame_id);
    EXPECT_TRUE((ls.packet_timestamp() == ls2.packet_timestamp()).all());
    EXPECT_TRUE((ls.status() == ls2.status()).all());
    EXPECT_TRUE((ls.timestamp() == ls2.timestamp()).all());
    EXPECT_TRUE((ls.measurement_id() == ls2.measurement_id()).all());

    impl::foreach_channel_field(ls2, packet_format, cmp_field{ls});
}

TEST_P(PacketFormatTest, frames_to_packets_skips_dropped_packets_test) {
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
    PacketFormat packet_format{info};

    // create randomised lidar frame
    auto ls = LidarFrame(info);
    std::iota(ls.measurement_id().data(), ls.measurement_id().data() + ls.measurement_id().size(),
              0);
    std::iota(ls.packet_timestamp().data(),
              ls.packet_timestamp().data() + ls.packet_timestamp().size(), 10);
    std::iota(ls.timestamp().data(), ls.timestamp().data() + ls.timestamp().size(), 1000);
    std::fill(ls.status().data(), ls.status().data() + ls.status().size(), 0x1);
    ls.frame_id = 700;

    auto randomise = [&](auto ref_field, const std::string& i) {
        // use static seed so that the test does not flake some day and
        // collectively piss off a bunch of angry developers
        randomize_field(ref_field, packet_format.field_value_mask(i), 0xdeadbeef);
    };
    impl::foreach_channel_field(ls, packet_format, randomise);

    auto packets_orig = std::vector<Packet>{};
    impl::frame_to_packets(ls, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets_orig), 0, 0);

    ASSERT_EQ(packets_orig.size(), 64);

    // drop one packet
    packets_orig.erase(packets_orig.begin() + 14);

    // disable every second column in first packet
    for (size_t icol = 0; icol < columns_per_packet; ++icol) {
        auto& p = packets_orig[0];
        if (icol % 2 == 0)
            packet_format.set_col_status(packet_format.nth_col(icol, p.buf.data()), 0);
    }

    auto ls_repr = LidarFrame(info);
    FrameBatcher batcher(info);
    for (const auto& p : packets_orig) {
        EXPECT_FALSE(batcher(p, ls_repr));
    }

    auto packets_repr = std::vector<Packet>{};
    impl::frame_to_packets(ls_repr, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets_repr), 0, 0);
    EXPECT_EQ(packets_repr.size(), 63);
    EXPECT_EQ(packets_repr[14].host_timestamp, 25);

    // check disabled column channel data blocks are completely empty
    for (size_t icol = 0; icol < columns_per_packet; ++icol) {
        auto& p = packets_repr[0];
        uint8_t* col_buf = packet_format.nth_col(icol, p.buf.data());
        if (icol % 2 == 0) {
            const uint8_t* begin = packet_format.nth_px(0, col_buf);
            const uint8_t* end = packet_format.nth_col(icol + 1, p.buf.data());
            EXPECT_TRUE(std::all_of(begin, end, [](uint8_t v) { return v == 0; }));
        }
    }
}

using data_param = std::tuple<std::string, std::string>;
class PacketFormatDataTest : public ::testing::TestWithParam<data_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    PacketFormatDataTests,
    PacketFormatDataTest,
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

TEST_P(PacketFormatDataTest, packet_packet_format_data_repr_test) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));

    auto packet_format = PacketFormat(info);

    auto ls_orig = LidarFrame(info);

    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
    FrameBatcher batcher(info);
    int n_packets = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            LidarPacket packet;
            packet.host_timestamp = 0;
            packet.buf.resize(pcap.current_length());
            memcpy(packet.buf.data(), pcap.current_data(), pcap.current_length());
            ASSERT_FALSE(batcher(packet, ls_orig));
            ++n_packets;
        }

    // produced and re-parsed fields should match
    auto packets = std::vector<Packet>{};
    impl::frame_to_packets(ls_orig, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets), info.init_id, info.sn);
    ASSERT_EQ(packets.size(), n_packets);

    auto ls_repr = LidarFrame(info);
    FrameBatcher repr_batcher(info);
    for (auto& p : packets) {
        ASSERT_FALSE(repr_batcher(p, ls_repr));
    }

    impl::foreach_channel_field(ls_repr, packet_format, cmp_field{ls_orig});
}

TEST_P(PacketFormatDataTest, packet_packet_format_raw_headers_match_test) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));

    auto packet_format = PacketFormat(info);

    auto rh_types = get_field_types(info);
    rh_types.emplace_back(ChanField::RAW_HEADERS, ChanFieldType::UINT32);

    auto rh_ls_orig = LidarFrame(info.format.pixels_per_column, info.format.columns_per_frame,
                                 rh_types, info.format.columns_per_packet);

    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
    FrameBatcher batcher(info);
    int n_packets = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            LidarPacket packet;
            packet.host_timestamp = 0;
            packet.buf.resize(pcap.current_length());
            memcpy(packet.buf.data(), pcap.current_data(), pcap.current_length());
            ASSERT_FALSE(batcher(packet, rh_ls_orig));
            ++n_packets;
        }

    // produced and re-parsed RAW_HEADERS fields should match
    auto packets = std::vector<Packet>{};
    impl::frame_to_packets(rh_ls_orig, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets), 0, 0);
    ASSERT_EQ(packets.size(), n_packets);

    auto rh_ls_repr = LidarFrame(info.format.pixels_per_column, info.format.columns_per_frame,
                                 rh_types, info.format.columns_per_packet);
    FrameBatcher repr_batcher(info);
    for (auto& p : packets) {
        ASSERT_FALSE(repr_batcher(p, rh_ls_repr));
    }

    auto rh_orig = rh_ls_orig.field<uint32_t>(ChanField::RAW_HEADERS);
    auto rh_repr = rh_ls_repr.field<uint32_t>(ChanField::RAW_HEADERS);
    EXPECT_TRUE((rh_orig == rh_repr).all());
}

TEST(PacketFormatImuTest, packet_packet_format_imu_test) {
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
    auto packet_format = PacketFormat(df);

    // TODO: unmagic once things are settled with the format
    size_t imu_packet_size = 65535;

    std::vector<uint8_t> packet_buf(imu_packet_size, 0);
    uint8_t* data = packet_buf.data();

    uint64_t timestamp = 133701337;
    std::string sentence = "Freddie Mercury";

    packet_format.set_imu_nmea_ts(data, timestamp);
    packet_format.set_imu_nmea_sentence(data, sentence);

    sentence.resize(NMEA_SENTENCE_LENGTH, '\0');
    EXPECT_EQ(packet_format.imu_nmea_ts(data), timestamp);
    EXPECT_EQ(sentence, packet_format.imu_nmea_sentence(data));

    EXPECT_THROW(
        packet_format.set_imu_nmea_sentence(data, std::string(NMEA_SENTENCE_LENGTH + 1, 'a')),
        std::invalid_argument);

    uint32_t interval = df.columns_per_frame / df.imu_measurements_per_packet;

    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        uint8_t* measurement = packet_format.imu_nth_measurement(i, data);
        packet_format.set_col_measurement_id(measurement, interval * i);
        packet_format.set_col_timestamp(measurement, 1000 + i);
        packet_format.set_col_status(measurement, 0x1);
        packet_format.set_imu_la_x(measurement, 100 + i);
        packet_format.set_imu_la_y(measurement, 200 + i);
        packet_format.set_imu_la_z(measurement, 300 + i);
        packet_format.set_imu_av_x(measurement, 400 + i);
        packet_format.set_imu_av_y(measurement, 500 + i);
        packet_format.set_imu_av_z(measurement, 600 + i);
    }

    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        const uint8_t* measurement = packet_format.imu_nth_measurement(i, data);
        EXPECT_EQ(packet_format.col_measurement_id(measurement), interval * i);
        EXPECT_EQ(packet_format.col_timestamp(measurement), 1000 + i);
        EXPECT_EQ(packet_format.col_status(measurement), 0x1);
        EXPECT_EQ(packet_format.imu_la_x(measurement), 100 + i);
        EXPECT_EQ(packet_format.imu_la_y(measurement), 200 + i);
        EXPECT_EQ(packet_format.imu_la_z(measurement), 300 + i);
        EXPECT_EQ(packet_format.imu_av_x(measurement), 400 + i);
        EXPECT_EQ(packet_format.imu_av_y(measurement), 500 + i);
        EXPECT_EQ(packet_format.imu_av_z(measurement), 600 + i);
    }

    auto accel = Field(fd_array<float>(64, 3));
    auto gyro = Field(fd_array<float>(64, 3));

    packet_format.parse_accel(0, data, accel);
    packet_format.parse_gyro(0, data, gyro);

    ArrayView2<float> la = accel;
    ArrayView2<float> av = gyro;

    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        EXPECT_EQ(la(i, 0), 100 + i);
        EXPECT_EQ(la(i, 1), 200 + i);
        EXPECT_EQ(la(i, 2), 300 + i);
        EXPECT_EQ(av(i, 0), 400 + i);
        EXPECT_EQ(av(i, 1), 500 + i);
        EXPECT_EQ(av(i, 2), 600 + i);
    }
}

TEST(PacketFormatImuTest, frame_to_packets_imu_zm_test) {
    auto format = DataFormat{128,
                             16,
                             1024,
                             64,
                             1,
                             {},
                             {0, 1023},
                             UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
                             UDPProfileIMU::ACCEL32_GYRO32_NMEA,
                             HeaderType::STANDARD,
                             10};
    uint32_t init_id = 1007;
    uint64_t prod_sn = 123456789;
    uint8_t alert_flags = 0x7F;
    format.zone_monitoring_enabled = true;
    auto packet_format = PacketFormat{format};
    SensorInfo info{};
    info.format = format;
    info.init_id = init_id;
    info.sn = prod_sn;

    auto frame = LidarFrame(format);
    frame.status() = 0x1;
    std::iota(frame.measurement_id().data(),
              frame.measurement_id().data() + frame.measurement_id().size(), 0);
    frame.frame_id = 995;

    auto frame_alert_flags = frame.alert_flags();
    for (int i = 0; i < frame_alert_flags.size(); ++i) {
        frame_alert_flags.data()[i] = alert_flags;
    }

    using namespace ChanField;

    ArrayView1<uint64_t> ts = frame.field(IMU_TIMESTAMP);
    for (size_t i = 0; i < ts.shape[0]; ++i) {
        ts(i) = 10000 + i;
    }
    uint32_t interval = format.columns_per_frame / format.imu_measurements_per_packet;
    ArrayView1<uint16_t> m_id = frame.field(IMU_MEASUREMENT_ID);
    for (size_t i = 0; i < m_id.shape[0]; ++i) {
        m_id(i) = interval * i;
    }
    ArrayView1<uint16_t> status = frame.field(IMU_STATUS);
    for (size_t i = 0; i < status.shape[0]; ++i) {
        status(i) = 0x1;
    }

    ArrayView2<float> acc = frame.field(IMU_ACC);
    ArrayView2<float> gyro = frame.field(IMU_GYRO);
    for (size_t i = 0; i < acc.shape[0]; ++i) {
        acc(i, 0) = 100 + i;
        acc(i, 1) = 200 + i;
        acc(i, 2) = 300 + i;
        gyro(i, 0) = 400 + i;
        gyro(i, 1) = 500 + i;
        gyro(i, 2) = 600 + i;
    }

    ArrayView1<uint64_t> packet_ts = frame.field(IMU_PACKET_TIMESTAMP);
    for (size_t i = 0; i < packet_ts.shape[0]; ++i) {
        packet_ts(i) = 20000 + i;
    }

    ArrayView1<uint8_t> imu_alert_flags = frame.field(IMU_ALERT_FLAGS);
    for (size_t i = 0; i < imu_alert_flags.shape[0]; ++i) {
        imu_alert_flags(i) = alert_flags;
    }

    std::string s = "Freddie Mercury";
    ArrayView2<char> nmea_string = frame.field(POSITION_STRING);
    for (size_t i = 0; i < nmea_string.shape[0]; ++i) {
        char* ptr = nmea_string.subview(i).data();
        std::memcpy(ptr, s.data(), s.length());
    }
    ArrayView1<uint64_t> nmea_ts = frame.field(POSITION_TIMESTAMP);
    for (size_t i = 0; i < nmea_ts.shape[0]; ++i) {
        nmea_ts(i) = 30000 + i;
    }

    ArrayView1<uint64_t> zone_packet_ts = frame.field(ZONE_PACKET_TIMESTAMP);
    zone_packet_ts(0) = 40000;

    ArrayView1<uint64_t> zone_ts = frame.field(ZONE_TIMESTAMP);
    zone_ts(0) = 50000;

    ArrayView1<uint8_t> zone_hash = frame.field(LIVE_ZONESET_HASH);
    for (size_t i = 0; i < zone_hash.shape[0]; ++i) {
        zone_hash(i) = i;
    }

    ArrayView1<uint8_t> zone_alert_flags = frame.field(ZONE_ALERT_FLAGS);
    for (size_t i = 0; i < zone_alert_flags.shape[0]; ++i) {
        zone_alert_flags(i) = alert_flags;
    }

    ArrayView1<ZoneState> zone_states = frame.field(ZONE_STATES);
    for (size_t i = 0; i < zone_states.shape[0]; ++i) {
        zone_states(i).live = 0x1;
        zone_states(i).id = i;
        zone_states(i).error_flags = 0xff;
        zone_states(i).trigger_type = 0b11;
        zone_states(i).trigger_status = 0b1;
        zone_states(i).triggered_frames = 1337;
        zone_states(i).count = 100 * i;
        zone_states(i).occlusion_count = 11 * i + 1;
        zone_states(i).invalid_count = 22 * i + 2;
        zone_states(i).max_count = 33 * i + 3;
        zone_states(i).min_range = 10 * i;
        zone_states(i).max_range = 100 * i;
        zone_states(i).mean_range = 55 * i;
    }

    auto packets = std::vector<Packet>{};
    impl::frame_to_packets(frame, std::make_shared<PacketFormat>(packet_format),
                           std::back_inserter(packets), init_id, prod_sn);
    size_t total_packets_expected =
        format.lidar_packets_per_frame() + format.imu_packets_per_frame + 1 /*zm*/;
    EXPECT_EQ(packets.size(), total_packets_expected);

    size_t total_lidar = 0;
    size_t total_imu = 0;
    size_t total_zm = 0;

    for (auto&& p : packets) {
        const uint8_t* buf = p.buf.data();
        switch (p.type()) {
            case PacketType::Lidar:
                ++total_lidar;
                EXPECT_EQ(packet_format.packet_type(buf), 0x1);
                break;
            case PacketType::Imu:
                ++total_imu;
                EXPECT_EQ(packet_format.packet_type(buf), 0x2);
                break;
            case PacketType::Zone:
                ++total_zm;
                EXPECT_EQ(packet_format.packet_type(buf), 0x3);
                break;
            default:
                break;
        }
        EXPECT_EQ(packet_format.frame_id(buf), frame.frame_id);
        EXPECT_EQ(packet_format.init_id(buf), init_id);
        EXPECT_EQ(packet_format.prod_sn(buf), prod_sn);
        EXPECT_EQ(packet_format.alert_flags(buf), alert_flags);
        EXPECT_EQ(packet_format.calculate_crc(buf, p.buf.size()),
                  packet_format.crc(buf, p.buf.size()).value());
    }

    EXPECT_EQ(total_lidar, format.lidar_packets_per_frame());
    EXPECT_EQ(total_imu, format.imu_packets_per_frame);
    EXPECT_EQ(total_zm, 1);

    FrameBatcher batcher{info};
    LidarFrame frame2{format};
    for (auto&& p : packets) {
        batcher(p, frame2);
    }

    // wipe out that one because our nmea field comes as garbage
    ArrayView2<double> lat_long = frame.field(POSITION_LAT_LONG);
    for (size_t i = 0; i < lat_long.shape[0]; ++i) {
        lat_long(i, 0) = std::numeric_limits<double>::quiet_NaN();
        lat_long(i, 1) = std::numeric_limits<double>::quiet_NaN();
    }

    for (auto& field_name_and_value : frame.fields()) {
        auto& field_name = field_name_and_value.first;
        auto& expected_value = field_name_and_value.second;
        EXPECT_EQ(frame2.field(field_name), expected_value)
            << "Field " << field_name << " does not match!";
    }

    // the second frame should have batched to be the same
    EXPECT_TRUE(frame == frame2);
}

TEST(PacketFormat, frame_id_difference) {
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
    PacketFormat pf(df);
    ASSERT_EQ(pf.max_frame_id, 0xFFFF);
    EXPECT_EQ(pf.frame_id_difference(0, 0), 0);
    EXPECT_EQ(pf.frame_id_difference(0, 1), 1);
    EXPECT_EQ(pf.frame_id_difference(0xF000, 0xFF00), 0x0F00);
    EXPECT_EQ(pf.frame_id_difference(pf.max_frame_id, 0), 1);
    EXPECT_EQ(pf.frame_id_difference(pf.max_frame_id, 1), 2);
    EXPECT_EQ(pf.frame_id_difference(0, pf.max_frame_id), -1);
    EXPECT_EQ(pf.frame_id_difference(1, pf.max_frame_id), -2);
}

TEST(PacketFormat, frame_id_difference_fusa) {
    auto df = DataFormat{128,
                         16,
                         1024,
                         64,
                         1,
                         {},
                         {0, 1023},
                         UDPProfileLidar::OFF,
                         UDPProfileIMU::ACCEL32_GYRO32_NMEA,
                         HeaderType::FUSA,
                         10};
    PacketFormat pf(df);
    ASSERT_EQ(pf.max_frame_id, 0xFFFFFFFF);
    EXPECT_EQ(pf.frame_id_difference(0, 0), 0);
    EXPECT_EQ(pf.frame_id_difference(0, 1), 1);
    EXPECT_EQ(pf.frame_id_difference(0xF000, 0xFF00), 0x0F00);
    EXPECT_EQ(pf.frame_id_difference(pf.max_frame_id, 0), 1);
    EXPECT_EQ(pf.frame_id_difference(pf.max_frame_id, 1), 2);
    EXPECT_EQ(pf.frame_id_difference(0, pf.max_frame_id), -1);
    EXPECT_EQ(pf.frame_id_difference(1, pf.max_frame_id), -2);
}

TEST(PacketFormat, legacy_imu_data_fix) {
    auto data_dir = getenvs("DATA_DIR");
    auto info = metadata_from_json(data_dir + "/OS-2-128-U1_v2.3.0_1024x10.json");
    PcapReader pcap(data_dir + "/OS-2-128-U1_v2.3.0_1024x10.pcap");

    auto packet_format = PacketFormat(info);

    std::vector<float> expected_values{-0.45776367,  -0.633239746, -1.2512207,   -0.465393066,
                                       0.0076293945, -0.54931641,  -0.564575195, -0.40435791,
                                       -0.198364258, -0.65612793};

    size_t i = 0;
    while (pcap.next_packet()) {
        if (pcap.current_info().dst_port == 7503) {
            EXPECT_FLOAT_EQ(expected_values[i], packet_format.imu_av_z(pcap.current_data()));
            ++i;
        }
    }
}
