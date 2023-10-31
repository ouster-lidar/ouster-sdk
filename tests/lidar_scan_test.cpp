/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdlib>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/impl/lidar_scan_impl.h"
#include "ouster/types.h"

#define TEST_REPEAT 5

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;
using namespace ouster::sensor;

static const Table<ChanField, ChanFieldType, 0> empty_field_slots{};

static const Table<ChanField, ChanFieldType, 4> legacy_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT32},
     {ChanField::NEAR_IR, ChanFieldType::UINT32},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT32}}};

static const Table<ChanField, ChanFieldType, 7> dual_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT16},
     {ChanField::SIGNAL2, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

static const Table<ChanField, ChanFieldType, 4> contrived_slots{
    {{ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::SIGNAL2, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

static const Table<ChanField, ChanFieldType, 4> duplicated_slots{
    {{ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

struct set_field_data {
    template <typename T>
    void operator()(Eigen::Ref<ouster::img_t<T>> field, int data) {
        for (int x = 0; x < field.rows(); x++) {
            for (int y = 0; y < field.cols(); y++) {
                field(x, y) = data;
            }
        }
    }
};

struct check_field_data {
    template <typename T>
    void operator()(Eigen::Ref<ouster::img_t<T>> field, int data) {
        EXPECT_TRUE((field == data).all());
    }
};

void zero_check_fields(ouster::LidarScan& scan) {
    for (auto it = scan.begin(); it != scan.end(); it++) {
        ouster::impl::visit_field(scan, it->first, check_field_data(), 0);
    }
}

TEST(LidarScan, EmptyConstructorInit) {
    auto scan = ouster::LidarScan();
    EXPECT_EQ(scan.w, 0);
    EXPECT_EQ(scan.h, 0);

    EXPECT_EQ(scan.frame_id, -1);

    EXPECT_EQ(scan.end() - scan.begin(), 0);

    zero_check_fields(scan);
}

TEST(LidarScan, LegacyConstructorInit) {
    int w = 10;
    int h = 20;
    auto scan = ouster::LidarScan(w, h);
    EXPECT_EQ(scan.w, w);
    EXPECT_EQ(scan.h, h);
    EXPECT_EQ(scan.frame_id, -1);

    size_t count = 0;
    size_t hit_count = 0;
    std::vector<ChanField> field_copy;
    for (auto item : legacy_field_slots) {
        field_copy.push_back(std::get<0>(item));
    }
    for (auto it = scan.begin(); it != scan.end(); it++) {
        auto result =
            std::find(field_copy.begin(), field_copy.end(), it->first);
        if (result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0u);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(legacy_field_slots.size(), count);

    EXPECT_TRUE((scan.status() == 0).all());
    EXPECT_TRUE((scan.measurement_id() == 0).all());
    EXPECT_TRUE((scan.timestamp() == 0).all());

    zero_check_fields(scan);
}

TEST(LidarScan, DualReturnConstructorInit) {
    int w = 40;
    int h = 60;
    auto scan = ouster::LidarScan(
        w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
    EXPECT_EQ(scan.w, w);
    EXPECT_EQ(scan.h, h);
    EXPECT_EQ(scan.frame_id, -1);

    size_t count = 0;
    size_t hit_count = 0;
    std::vector<ChanField> field_copy;
    for (auto item : dual_field_slots) {
        field_copy.push_back(std::get<0>(item));
    }
    for (auto it = scan.begin(); it != scan.end(); it++) {
        auto result =
            std::find(field_copy.begin(), field_copy.end(), it->first);
        if (result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0u);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(dual_field_slots.size(), count);

    EXPECT_TRUE((scan.status() == 0).all());
    EXPECT_TRUE((scan.measurement_id() == 0).all());
    EXPECT_TRUE((scan.timestamp() == 0).all());

    zero_check_fields(scan);
}

TEST(LidarScan, CustomFieldConstructorInit) {
    int w = 80;
    int h = 100;
    auto scan =
        ouster::LidarScan(w, h, contrived_slots.begin(), contrived_slots.end());
    EXPECT_EQ(scan.w, w);
    EXPECT_EQ(scan.h, h);
    EXPECT_EQ(scan.frame_id, -1);

    size_t count = 0;
    size_t hit_count = 0;
    std::vector<ChanField> field_copy;
    for (auto item : contrived_slots) {
        field_copy.push_back(std::get<0>(item));
    }

    for (auto it = scan.begin(); it != scan.end(); it++) {
        auto result =
            std::find(field_copy.begin(), field_copy.end(), it->first);
        if (result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0u);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(contrived_slots.size(), count);

    EXPECT_TRUE((scan.status() == 0).all());
    EXPECT_TRUE((scan.measurement_id() == 0).all());
    EXPECT_TRUE((scan.timestamp() == 0).all());

    zero_check_fields(scan);
}

TEST(LidarScan, EmptyEquality) {
    auto scan1 = ouster::LidarScan();
    auto scan2 = ouster::LidarScan();
    auto scan3 = ouster::LidarScan();
    auto scan4 = ouster::LidarScan(10, 20);
    auto scan5 = ouster::LidarScan(0, 0);
    auto scan6 = ouster::LidarScan(0, 0, empty_field_slots.begin(),
                                   empty_field_slots.end());
    auto scan7 = ouster::LidarScan(
        0, 0, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
    scan3.frame_id = 1;

    EXPECT_TRUE(scan1 == scan2);
    EXPECT_TRUE(scan1 != scan3);
    EXPECT_TRUE(scan1 != scan4);
    EXPECT_TRUE(scan1 != scan5);
    EXPECT_TRUE(scan1 == scan6);
    EXPECT_TRUE(scan1 != scan7);
}

TEST(LidarScan, LegacyEquality) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        auto scan1 = ouster::LidarScan(w, h);
        auto scan2 = ouster::LidarScan(w, h);
        auto scan3 = ouster::LidarScan(w, h);
        auto scan4 = ouster::LidarScan(w, h, legacy_field_slots.begin(),
                                       legacy_field_slots.end());
        auto scan5 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        scan3.frame_id = 1;

        EXPECT_TRUE(scan1 == scan2);
        EXPECT_TRUE(scan1 != scan3);
        EXPECT_TRUE(scan1 == scan4);
        EXPECT_TRUE(scan1 != scan5);
    }
}

TEST(LidarScan, DualReturnsEquality) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        auto scan1 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan2 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan3 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan4 = ouster::LidarScan(w, h, dual_field_slots.begin(),
                                       dual_field_slots.end());
        scan3.frame_id = 1;

        EXPECT_TRUE(scan1 == scan2);
        EXPECT_TRUE(scan1 != scan3);
        EXPECT_TRUE(scan1 == scan4);
    }
}

TEST(LidarScan, CustomEquality) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        auto test_array1 = dual_field_slots;
        auto test_array2 = dual_field_slots;
        auto test_array3 = std::vector<std::pair<ChanField, ChanFieldType>>(
            dual_field_slots.begin() + 1, dual_field_slots.end());

        std::random_shuffle(test_array1.begin(), test_array1.end());
        std::random_shuffle(test_array3.begin(), test_array3.end());

        auto scan1 =
            ouster::LidarScan(w, h, test_array1.begin(), test_array1.end());
        auto scan2 =
            ouster::LidarScan(w, h, test_array1.begin(), test_array1.end());
        auto scan3 =
            ouster::LidarScan(w, h, test_array2.begin(), test_array2.end());
        auto scan4 =
            ouster::LidarScan(w, h, test_array2.begin(), test_array2.end());
        auto scan5 =
            ouster::LidarScan(w, h, test_array3.begin(), test_array3.end());

        scan4.frame_id = 1;

        EXPECT_TRUE(scan1 == scan2);
        EXPECT_TRUE(scan1 != scan3);
        EXPECT_TRUE(scan3 != scan4);
        EXPECT_TRUE(scan1 != scan5);
    }
    EXPECT_THROW(
        {
            auto scan6 = ouster::LidarScan(10, 10, duplicated_slots.begin(),
                                           duplicated_slots.end());
        },
        std::invalid_argument);
}

TEST(LidarScan, DataCheck) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        std::unordered_map<int, uint8_t> expected_data;

        auto scan1 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan2 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan3 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);

        for (auto it = scan1.begin(); it != scan1.end(); it++) {
            expected_data[it->first] = rand() % 254 + 1;
            ouster::impl::visit_field(scan1, it->first, set_field_data(),
                                      expected_data[it->first]);
            ouster::impl::visit_field(scan2, it->first, set_field_data(),
                                      expected_data[it->first]);
            ouster::impl::visit_field(scan1, it->first, check_field_data(),
                                      expected_data[it->first]);
        }

        EXPECT_TRUE(scan1 == scan2);
        EXPECT_TRUE(scan1 != scan3);
    }
}

TEST(LidarScan, CustomUserFields) {
    using LidarScanFieldTypes = std::vector<
        std::pair<ouster::sensor::ChanField, ouster::sensor::ChanFieldType>>;

    LidarScanFieldTypes user_fields{
        {ChanField::CUSTOM0, ChanFieldType::UINT8},
        {ChanField::CUSTOM3, ChanFieldType::UINT64},
        {ChanField::CUSTOM9, ChanFieldType::UINT16}};

    ouster::LidarScan user_scan(10, 10, user_fields.begin(), user_fields.end());

    EXPECT_EQ(3, std::distance(user_scan.begin(), user_scan.end()));

    zero_check_fields(user_scan);
}

TEST(LidarScan, packet_timestamp) {
    int w = 32;
    int h = 32;
    auto scan = ouster::LidarScan(w, h);
    // host timestamp header should have w/columns-per-packet entries
    // (default DEFAULT_COLUMNS_PER_PACKET is 16)
    EXPECT_EQ(scan.packet_timestamp().rows(), 2);
    EXPECT_TRUE((scan.packet_timestamp() == 0).all());

    LidarPacket packet;
    ouster::sensor::sensor_info info;
    info.format.columns_per_frame = w;
    info.format.columns_per_packet = 0;
    info.format.pixels_per_column = h;
    info.format.udp_profile_lidar = PROFILE_LIDAR_LEGACY;

    EXPECT_THROW(
        {
            try {
                ouster::ScanBatcher scan_batcher(info);
            } catch (std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), "unexpected columns_per_packet: 0");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarScan, packet_timestamp_2) {
    // ScanBatcher::operator() should throw
    // if the packet timestamp header index exceeds the header size in the
    // LidarScan
    int h = 32;
    auto scan = ouster::LidarScan(32, h);
    EXPECT_EQ(scan.packet_timestamp().rows(),
              scan.w / DEFAULT_COLUMNS_PER_PACKET);
    EXPECT_TRUE((scan.packet_timestamp() == 0).all());

    LidarPacket packet;
    packet.host_timestamp = 123;

    ouster::sensor::sensor_info info;
    info.format.udp_profile_lidar = PROFILE_LIDAR_LEGACY;
    info.format.pixels_per_column = h;
    // not enough columns per packet according to the measurement id
    info.format.columns_per_packet = 1;
    auto pf = ouster::sensor::get_format(info);

    uint8_t* col_buf = const_cast<uint8_t*>(pf.nth_col(0, packet.buf.data()));
    uint16_t bogus_measurement_id = 1234;
    std::memcpy(col_buf + 8, &bogus_measurement_id,
                sizeof(bogus_measurement_id));
    const uint16_t m_id = pf.col_measurement_id(col_buf);
    ASSERT_EQ(m_id, bogus_measurement_id);

    ouster::ScanBatcher scan_batcher(info.format.columns_per_frame, pf);
    EXPECT_THROW(
        {
            try {
                scan_batcher(packet, scan);
            } catch (std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), "unexpected scan dimensions");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarScan, packet_timestamp_3) {
    // ScanBatcher::operator() should update the packet timestamp header
    // in the LidarScan if a LidarPacket is provided
    int w = 32;
    int h = 32;
    auto scan = ouster::LidarScan(w, h);
    EXPECT_EQ(scan.packet_timestamp().rows(), w / DEFAULT_COLUMNS_PER_PACKET);
    EXPECT_TRUE((scan.packet_timestamp() == 0).all());

    LidarPacket packet;
    packet.host_timestamp = 123;

    ouster::sensor::sensor_info info;
    info.format.udp_profile_lidar = PROFILE_LIDAR_LEGACY;
    info.format.pixels_per_column = h;
    info.format.columns_per_packet =
        DEFAULT_COLUMNS_PER_PACKET;  // not enough columns per packet according
                                     // to the measurement id
    info.format.columns_per_frame = w;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    info.format.pixels_per_column = h;
    auto pf = ouster::sensor::get_format(info);

    uint8_t* col_buf = const_cast<uint8_t*>(pf.nth_col(0, packet.buf.data()));
    uint16_t bogus_measurement_id = 0;
    std::memcpy(col_buf + 8, &bogus_measurement_id,
                sizeof(bogus_measurement_id));
    const uint16_t m_id = pf.col_measurement_id(col_buf);
    ASSERT_EQ(m_id, bogus_measurement_id);

    ouster::ScanBatcher scan_batcher(info.format.columns_per_frame, pf);
    scan_batcher(packet, scan);
    EXPECT_EQ(scan.packet_timestamp()[0], packet.host_timestamp);
    EXPECT_EQ(scan.packet_timestamp()[1], 0);
}
