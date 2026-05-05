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

using namespace ouster::sdk::core;
using namespace ouster::sdk::core::impl;
using namespace ouster::sdk::pcap;

// TODO: we should just make FieldInfo's publicly available
namespace ouster {
namespace sdk {
namespace core {
namespace impl {

struct FieldInfo {
    ChanFieldType ty_tag;
    size_t offset;
    uint64_t mask;
    int shift;
};

struct ProfileEntry {
    const std::pair<std::string, FieldInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

OUSTER_API_FUNCTION Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES>
get_profiles();

/**
 * @todo please find some way around removing this, this is causing shared lib
 * issues
 */
uint64_t get_value_mask(const FieldInfo& f);
/**
 * @todo please find some way around removing this, this is causing shared lib
 * issues
 */
int get_bitness(const FieldInfo& f);

std::map<std::string, FieldInfo> get_fields(UDPProfileLidar profile) {
    auto profiles = get_profiles();
    auto end = profiles.end();
    auto it = std::find_if(profiles.begin(), end, [profile](const auto& kv) {
        return kv.first == profile;
    });

    auto& entry = it->second;
    return {entry.fields, entry.fields + entry.n_fields};
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

using bitness_param = std::tuple<UDPProfileLidar, std::map<std::string, int>>;
class FieldInfoSanityTest : public ::testing::TestWithParam<bitness_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    FieldInfoSanityTests,
    FieldInfoSanityTest,
    ::testing::Values(
        bitness_param{PROFILE_LIDAR_LEGACY,
                      {{ChanField::RANGE, 20},
                       {ChanField::FLAGS, 4},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32}}},
        bitness_param{PROFILE_RNG15_RFL8_NIR8,
                      {{ChanField::RANGE, 15},
                       {ChanField::FLAGS, 1},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::NEAR_IR, 8},
                       {ChanField::RAW32_WORD1, 32}}},
        bitness_param{PROFILE_RNG19_RFL8_SIG16_NIR16,
                      {{ChanField::RANGE, 19},
                       {ChanField::FLAGS, 5},
                       {ChanField::REFLECTIVITY, 8},
                       {ChanField::SIGNAL, 16},
                       {ChanField::NEAR_IR, 16},
                       {ChanField::WINDOW, 8},
                       {ChanField::RAW32_WORD1, 32},
                       {ChanField::RAW32_WORD2, 32},
                       {ChanField::RAW32_WORD3, 32}}},
        bitness_param{PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
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
        bitness_param{PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL,
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

using test_param =
    std::tuple<UDPProfileLidar, HeaderType, uint32_t, uint32_t, uint32_t>;
class PacketWriterTest : public ::testing::TestWithParam<test_param> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    PacketWriterTests,
    PacketWriterTest,
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
    LidarScan& ls;

    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& i) {
        EXPECT_TRUE((ls.field<T>(i) == field).all());
    }
};

TEST_P(PacketWriterTest, packet_writer_headers_test) {
    auto param = GetParam();
    UDPProfileLidar profile = std::get<0>(param);
    HeaderType profile_type = std::get<1>(param);
    uint32_t columns_per_frame = std::get<2>(param);
    uint32_t pixels_per_column = std::get<3>(param);
    uint32_t columns_per_packet = std::get<4>(param);

    DataFormat df{
        pixels_per_column, columns_per_packet, columns_per_frame, 0, 0, {}, {},
        profile,           PROFILE_IMU_LEGACY, profile_type,      10};
    PacketFormat pf{df};
    PacketWriter pw{pf};
    LidarPacket p(pf.lidar_packet_size);

    pw.set_col_status(pw.nth_col(9, p.buf.data()), 123);
    EXPECT_EQ(pf.col_status(pf.nth_col(9, p.buf.data())), 123);

    pw.set_col_timestamp(pw.nth_col(11, p.buf.data()), 80899);
    EXPECT_EQ(pf.col_timestamp(pf.nth_col(11, p.buf.data())), 80899);

    pw.set_col_measurement_id(pw.nth_col(7, p.buf.data()), 613);
    EXPECT_EQ(pf.col_measurement_id(pf.nth_col(7, p.buf.data())), 613);

    pw.set_frame_id(p.buf.data(), 777);
    EXPECT_EQ(pf.frame_id(p.buf.data()), 777);

    if (profile != PROFILE_LIDAR_LEGACY) {
        pw.set_init_id(p.buf.data(), 0x123456);
        EXPECT_EQ(pf.init_id(p.buf.data()), 0x123456);

        pw.set_prod_sn(p.buf.data(), 0x1234567890);
        EXPECT_EQ(pf.prod_sn(p.buf.data()), 0x1234567890);
    }
}

TEST_P(PacketWriterTest, packet_writer_randomize_test) {
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
                  PROFILE_IMU_LEGACY,
                  profile_type,
                  10};
    SensorInfo info;
    info.format = df;
    PacketFormat pf{info};
    PacketWriter pw{pf};

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

    auto randomise = [&](auto ref_field, const std::string& i) {
        // use static seed so that the test does not flake some day and
        // collectively piss off a bunch of angry developers
        randomize_field(ref_field, pw.field_value_mask(i), 0xdeadbeef);
    };
    impl::foreach_channel_field(ls, pf, randomise);

    auto fields = get_fields(profile);
    auto verify_field = [&](auto ref_field, const std::string& i) {
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
            uint64_t value_bits = 0;
            memcpy(&value_bits, &value, sizeof(T));
            // output must perfectly fit into the value mask
            EXPECT_EQ(value_bits, value_bits & value_mask);
            field_mask |= value_bits;
        }
        // verify all possible bits were covered
        EXPECT_EQ(field_mask, value_mask);
    };
    impl::foreach_channel_field(ls, pf, verify_field);

    auto g = std::mt19937(0xdeadbeef);
    auto dinit_id = std::uniform_int_distribution<uint32_t>(0, 0xFFFFFF);
    auto dserial_no = std::uniform_int_distribution<uint64_t>(0, 0xFFFFFFFFFF);

    uint32_t init_id = dinit_id(g);      // 24 bits
    uint64_t serial_no = dserial_no(g);  // 40 bits

    // produced and re-parsed packets should result in the same scan
    auto packets = std::vector<Packet>{};
    impl::scan_to_packets(ls, pw, std::back_inserter(packets), init_id,
                          serial_no);

    ASSERT_EQ(packets.size(), 64);

    // validate the init id and serial no in each packet if supported
    if (profile != PROFILE_LIDAR_LEGACY) {
        for (const auto& p : packets) {
            ASSERT_EQ(init_id, pf.init_id(p.buf.data()));
            ASSERT_EQ(serial_no, pf.prod_sn(p.buf.data()));
        }
    }

    auto ls2 = LidarScan(columns_per_frame, pixels_per_column, profile,
                         columns_per_packet);
    ScanBatcher batcher(info);
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

    impl::foreach_channel_field(ls2, pf, cmp_field{ls});
}

TEST_P(PacketWriterTest, scans_to_packets_skips_dropped_packets_test) {
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
                  PROFILE_IMU_LEGACY,
                  profile_type,
                  10};
    SensorInfo info;
    info.format = df;
    PacketFormat pf{info};
    PacketWriter pw{pf};

    // create randomised lidar scan
    auto ls = LidarScan(info);
    std::iota(ls.measurement_id().data(),
              ls.measurement_id().data() + ls.measurement_id().size(), 0);
    std::iota(ls.packet_timestamp().data(),
              ls.packet_timestamp().data() + ls.packet_timestamp().size(), 10);
    std::iota(ls.timestamp().data(),
              ls.timestamp().data() + ls.timestamp().size(), 1000);
    std::fill(ls.status().data(), ls.status().data() + ls.status().size(), 0x1);
    ls.frame_id = 700;

    auto randomise = [&](auto ref_field, const std::string& i) {
        // use static seed so that the test does not flake some day and
        // collectively piss off a bunch of angry developers
        randomize_field(ref_field, pw.field_value_mask(i), 0xdeadbeef);
    };
    impl::foreach_channel_field(ls, pf, randomise);

    auto packets_orig = std::vector<Packet>{};
    impl::scan_to_packets(ls, pw, std::back_inserter(packets_orig), 0, 0);

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
    ScanBatcher batcher(info);
    for (const auto& p : packets_orig) {
        EXPECT_FALSE(batcher(p, ls_repr));
    }

    auto packets_repr = std::vector<Packet>{};
    impl::scan_to_packets(ls_repr, pw, std::back_inserter(packets_repr), 0, 0);
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

TEST_P(PacketWriterDataTest, packet_writer_data_repr_test) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));

    auto pw = PacketWriter(info);

    auto ls_orig =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  pw.udp_profile_lidar, pw.columns_per_packet);

    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
    ScanBatcher batcher(info);
    int n_packets = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            LidarPacket packet;
            packet.host_timestamp = 0;
            packet.buf.resize(pcap.current_length());
            memcpy(packet.buf.data(), pcap.current_data(),
                   pcap.current_length());
            ASSERT_FALSE(batcher(packet, ls_orig));
            ++n_packets;
        }

    // produced and re-parsed fields should match
    auto packets = std::vector<Packet>{};
    impl::scan_to_packets(ls_orig, pw, std::back_inserter(packets), 0, 0);
    ASSERT_EQ(packets.size(), n_packets);

    auto ls_repr =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  pw.udp_profile_lidar, pw.columns_per_packet);
    ScanBatcher repr_batcher(info);
    for (auto& p : packets) {
        ASSERT_FALSE(repr_batcher(p, ls_repr));
    }

    impl::foreach_channel_field(ls_repr, pw, cmp_field{ls_orig});
}

TEST_P(PacketWriterDataTest, packet_writer_raw_headers_match_test) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));

    auto pw = PacketWriter(info);

    auto rh_types = get_field_types(info);
    rh_types.emplace_back(ChanField::RAW_HEADERS, ChanFieldType::UINT32);

    auto rh_ls_orig =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  rh_types.begin(), rh_types.end(), pw.columns_per_packet);

    PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
    ScanBatcher batcher(info);
    int n_packets = 0;
    while (pcap.next_packet())
        if (pcap.current_info().dst_port == 7502) {
            LidarPacket packet;
            packet.host_timestamp = 0;
            packet.buf.resize(pcap.current_length());
            memcpy(packet.buf.data(), pcap.current_data(),
                   pcap.current_length());
            ASSERT_FALSE(batcher(packet, rh_ls_orig));
            ++n_packets;
        }

    // produced and re-parsed RAW_HEADERS fields should match
    auto packets = std::vector<Packet>{};
    impl::scan_to_packets(rh_ls_orig, pw, std::back_inserter(packets), 0, 0);
    ASSERT_EQ(packets.size(), n_packets);

    auto rh_ls_repr =
        LidarScan(info.format.columns_per_frame, pw.pixels_per_column,
                  rh_types.begin(), rh_types.end(), pw.columns_per_packet);
    ScanBatcher repr_batcher(info);
    for (auto& p : packets) {
        ASSERT_FALSE(repr_batcher(p, rh_ls_repr));
    }

    auto rh_orig = rh_ls_orig.field<uint32_t>(ChanField::RAW_HEADERS);
    auto rh_repr = rh_ls_repr.field<uint32_t>(ChanField::RAW_HEADERS);
    EXPECT_TRUE((rh_orig == rh_repr).all());
}

TEST(PacketWriterImuTest, packet_writer_imu_test) {
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
    auto pw = PacketWriter(df);

    // TODO: unmagic once things are settled with the format
    size_t imu_packet_size = 65535;

    std::vector<uint8_t> packet_buf(imu_packet_size, 0);
    uint8_t* data = packet_buf.data();

    uint64_t timestamp = 133701337;
    std::string sentence = "Freddie Mercury";

    pw.set_imu_nmea_ts(data, timestamp);
    pw.set_imu_nmea_sentence(data, sentence);

    sentence.resize(NMEA_SENTENCE_LENGTH, '\0');
    EXPECT_EQ(pw.imu_nmea_ts(data), timestamp);
    EXPECT_EQ(sentence, pw.imu_nmea_sentence(data));

    EXPECT_THROW(pw.set_imu_nmea_sentence(
                     data, std::string(NMEA_SENTENCE_LENGTH + 1, 'a')),
                 std::invalid_argument);

    uint32_t interval = df.columns_per_frame / df.imu_measurements_per_packet;

    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        uint8_t* measurement = pw.imu_nth_measurement(i, data);
        pw.set_col_measurement_id(measurement, interval * i);
        pw.set_col_timestamp(measurement, 1000 + i);
        pw.set_col_status(measurement, 0x1);
        pw.set_imu_la_x(measurement, 100 + i);
        pw.set_imu_la_y(measurement, 200 + i);
        pw.set_imu_la_z(measurement, 300 + i);
        pw.set_imu_av_x(measurement, 400 + i);
        pw.set_imu_av_y(measurement, 500 + i);
        pw.set_imu_av_z(measurement, 600 + i);
    }

    for (size_t i = 0; i < df.imu_measurements_per_packet; ++i) {
        const uint8_t* measurement = pw.imu_nth_measurement(i, data);
        EXPECT_EQ(pw.col_measurement_id(measurement), interval * i);
        EXPECT_EQ(pw.col_timestamp(measurement), 1000 + i);
        EXPECT_EQ(pw.col_status(measurement), 0x1);
        EXPECT_EQ(pw.imu_la_x(measurement), 100 + i);
        EXPECT_EQ(pw.imu_la_y(measurement), 200 + i);
        EXPECT_EQ(pw.imu_la_z(measurement), 300 + i);
        EXPECT_EQ(pw.imu_av_x(measurement), 400 + i);
        EXPECT_EQ(pw.imu_av_y(measurement), 500 + i);
        EXPECT_EQ(pw.imu_av_z(measurement), 600 + i);
    }

    auto accel = Field(fd_array<float>(64, 3));
    auto gyro = Field(fd_array<float>(64, 3));

    pw.parse_accel(0, data, accel);
    pw.parse_gyro(0, data, gyro);

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

TEST(PacketWriterImuTest, scan_to_packets_imu_zm_test) {
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
    format.zone_monitoring_enabled = true;
    auto writer = PacketWriter{format};
    SensorInfo info{};
    info.format = format;

    uint32_t init_id = 1007;
    uint64_t prod_sn = 123456789;
    uint8_t alert_flags = 0x7F;

    auto scan = LidarScan(format);
    scan.status() = 0x1;
    std::iota(scan.measurement_id().data(),
              scan.measurement_id().data() + scan.measurement_id().size(), 0);
    scan.frame_id = 995;

    auto scan_alert_flags = scan.alert_flags();
    for (int i = 0; i < scan_alert_flags.size(); ++i) {
        scan_alert_flags.data()[i] = alert_flags;
    }

    using namespace ChanField;

    ArrayView1<uint64_t> ts = scan.field(IMU_TIMESTAMP);
    for (size_t i = 0; i < ts.shape[0]; ++i) {
        ts(i) = 10000 + i;
    }
    uint32_t interval =
        format.columns_per_frame / format.imu_measurements_per_packet;
    ArrayView1<uint16_t> m_id = scan.field(IMU_MEASUREMENT_ID);
    for (size_t i = 0; i < m_id.shape[0]; ++i) {
        m_id(i) = interval * i;
    }
    ArrayView1<uint16_t> status = scan.field(IMU_STATUS);
    for (size_t i = 0; i < status.shape[0]; ++i) {
        status(i) = 0x1;
    }

    ArrayView2<float> acc = scan.field(IMU_ACC);
    ArrayView2<float> gyro = scan.field(IMU_GYRO);
    for (size_t i = 0; i < acc.shape[0]; ++i) {
        acc(i, 0) = 100 + i;
        acc(i, 1) = 200 + i;
        acc(i, 2) = 300 + i;
        gyro(i, 0) = 400 + i;
        gyro(i, 1) = 500 + i;
        gyro(i, 2) = 600 + i;
    }

    ArrayView1<uint64_t> packet_ts = scan.field(IMU_PACKET_TIMESTAMP);
    for (size_t i = 0; i < packet_ts.shape[0]; ++i) {
        packet_ts(i) = 20000 + i;
    }

    ArrayView1<uint8_t> imu_alert_flags = scan.field(IMU_ALERT_FLAGS);
    for (size_t i = 0; i < imu_alert_flags.shape[0]; ++i) {
        imu_alert_flags(i) = alert_flags;
    }

    std::string s = "Freddie Mercury";
    ArrayView2<char> nmea_string = scan.field(POSITION_STRING);
    for (size_t i = 0; i < nmea_string.shape[0]; ++i) {
        char* ptr = nmea_string.subview(i).data();
        std::memcpy(ptr, s.data(), s.length());
    }
    ArrayView1<uint64_t> nmea_ts = scan.field(POSITION_TIMESTAMP);
    for (size_t i = 0; i < nmea_ts.shape[0]; ++i) {
        nmea_ts(i) = 30000 + i;
    }

    ArrayView1<uint64_t> zone_packet_ts = scan.field(ZONE_PACKET_TIMESTAMP);
    zone_packet_ts(0) = 40000;

    ArrayView1<uint64_t> zone_ts = scan.field(ZONE_TIMESTAMP);
    zone_ts(0) = 50000;

    ArrayView1<uint8_t> zone_hash = scan.field(LIVE_ZONESET_HASH);
    for (size_t i = 0; i < zone_hash.shape[0]; ++i) {
        zone_hash(i) = i;
    }

    ArrayView1<uint8_t> zone_alert_flags = scan.field(ZONE_ALERT_FLAGS);
    for (size_t i = 0; i < zone_alert_flags.shape[0]; ++i) {
        zone_alert_flags(i) = alert_flags;
    }

    ArrayView1<ZoneState> zone_states = scan.field(ZONE_STATES);
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
    impl::scan_to_packets(scan, writer, std::back_inserter(packets), init_id,
                          prod_sn);
    size_t total_packets_expected = format.lidar_packets_per_frame() +
                                    format.imu_packets_per_frame + 1 /*zm*/;
    EXPECT_EQ(packets.size(), total_packets_expected);

    size_t total_lidar = 0;
    size_t total_imu = 0;
    size_t total_zm = 0;

    for (auto&& p : packets) {
        const uint8_t* buf = p.buf.data();
        switch (p.type()) {
            case PacketType::Lidar:
                ++total_lidar;
                EXPECT_EQ(writer.packet_type(buf), 0x1);
                break;
            case PacketType::Imu:
                ++total_imu;
                EXPECT_EQ(writer.packet_type(buf), 0x2);
                break;
            case PacketType::Zone:
                ++total_zm;
                EXPECT_EQ(writer.packet_type(buf), 0x3);
                break;
            default:
                break;
        }
        EXPECT_EQ(writer.frame_id(buf), scan.frame_id);
        EXPECT_EQ(writer.init_id(buf), init_id);
        EXPECT_EQ(writer.prod_sn(buf), prod_sn);
        EXPECT_EQ(writer.alert_flags(buf), alert_flags);
        EXPECT_EQ(writer.calculate_crc(buf, p.buf.size()),
                  writer.crc(buf, p.buf.size()).value());
    }

    EXPECT_EQ(total_lidar, format.lidar_packets_per_frame());
    EXPECT_EQ(total_imu, format.imu_packets_per_frame);
    EXPECT_EQ(total_zm, 1);

    ScanBatcher batcher{info};
    LidarScan scan2{format};
    for (auto&& p : packets) {
        batcher(p, scan2);
    }

    // wipe out that one because our nmea field comes as garbage
    ArrayView2<double> lat_long = scan.field(POSITION_LAT_LONG);
    for (size_t i = 0; i < lat_long.shape[0]; ++i) {
        lat_long(i, 0) = std::numeric_limits<double>::quiet_NaN();
        lat_long(i, 1) = std::numeric_limits<double>::quiet_NaN();
    }

    for (auto& field_name_and_value : scan.fields()) {
        auto& field_name = field_name_and_value.first;
        auto& expected_value = field_name_and_value.second;
        EXPECT_EQ(scan2.field(field_name), expected_value)
            << "Field " << field_name << " does not match!";
    }

    // the second scan should have batched to be the same
    EXPECT_TRUE(scan == scan2);
}
