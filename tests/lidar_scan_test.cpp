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
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/impl/lidar_scan_impl.h"
#include "ouster/types.h"

#define TEST_REPEAT 5

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;
using namespace ouster::sensor;

static const std::vector<ouster::FieldType> empty_field_slots{};

static const std::vector<ouster::FieldType> legacy_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT16},
     {ChanField::NEAR_IR, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::FLAGS, ChanFieldType::UINT8}}};

static const std::vector<ouster::FieldType> dual_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT16},
     {ChanField::SIGNAL2, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
     {ChanField::FLAGS, ChanFieldType::UINT8},
     {ChanField::FLAGS2, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

static const std::vector<ouster::FieldType> contrived_slots{
    {{ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::SIGNAL2, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

static const std::vector<ouster::FieldType> duplicated_slots{
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
    for (const auto& field : scan.fields()) {
        ouster::impl::visit_field(scan, field.first, check_field_data(), 0);
    }
}

TEST(LidarScanSanityTests, get_field_types_test) {
    ouster::LidarScanFieldTypes ft;
    ft.emplace_back(ChanField::RANGE, ChanFieldType::UINT32);
    ft.emplace_back(ChanField::SIGNAL, ChanFieldType::UINT16);
    ft.emplace_back(ChanField::RANGE2, ChanFieldType::UINT32);
    ft.emplace_back(ChanField::SIGNAL2, ChanFieldType::UINT16);
    ft.emplace_back(ChanField::REFLECTIVITY, ChanFieldType::UINT8);
    ft.emplace_back(ChanField::NEAR_IR, ChanFieldType::UINT16);
    ft.emplace_back(ChanField::FLAGS, ChanFieldType::UINT8);
    ft.emplace_back(ChanField::FLAGS2, ChanFieldType::UINT8);
    ft.emplace_back("CUSTOM0", ChanFieldType::UINT64);
    ft.emplace_back("CUSTOM7", ChanFieldType::UINT16);
    std::sort(ft.begin(), ft.end());

    auto ls = ouster::LidarScan(1028, 128, ft.begin(), ft.end());
    EXPECT_EQ(ft, ls.field_types());
}

TEST(LidarScan, EmptyConstructorInit) {
    auto scan = ouster::LidarScan();
    EXPECT_EQ(scan.w, 0);
    EXPECT_EQ(scan.h, 0);

    EXPECT_EQ(scan.frame_id, -1);

    EXPECT_EQ(scan.fields().size(), 0);

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
    std::vector<std::string> field_copy;
    for (auto item : legacy_field_slots) {
        field_copy.push_back(item.name);
    }
    for (const auto& field : scan.fields()) {
        auto result =
            std::find(field_copy.begin(), field_copy.end(), field.first);
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
    std::vector<std::string> field_copy;
    for (auto item : dual_field_slots) {
        field_copy.push_back(item.name);
    }
    for (const auto& field : scan.fields()) {
        auto result =
            std::find(field_copy.begin(), field_copy.end(), field.first);
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
    std::vector<std::string> field_copy;
    for (auto item : contrived_slots) {
        field_copy.push_back(item.name);
    }

    for (const auto& field : scan.fields()) {
        auto result =
            std::find(field_copy.begin(), field_copy.end(), field.first);
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
    scan3.frame_id = 1;

    EXPECT_TRUE(scan1 == scan2);
    EXPECT_TRUE(scan1 != scan3);
    EXPECT_TRUE(scan1 != scan4);
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
        auto test_array3 = std::vector<ouster::FieldType>(
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
        EXPECT_TRUE(scan1 == scan3);
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

        std::unordered_map<std::string, uint8_t> expected_data;

        auto scan1 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan2 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
        auto scan3 = ouster::LidarScan(
            w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);

        for (const auto& field : scan1.fields()) {
            expected_data[field.first] = rand() % 254 + 1;
            ouster::impl::visit_field(scan1, field.first, set_field_data(),
                                      expected_data[field.first]);
            ouster::impl::visit_field(scan2, field.first, set_field_data(),
                                      expected_data[field.first]);
            ouster::impl::visit_field(scan1, field.first, check_field_data(),
                                      expected_data[field.first]);
        }

        EXPECT_TRUE(scan1 == scan2);
        EXPECT_TRUE(scan1 != scan3);
    }
}

TEST(LidarScan, CustomUserFields) {
    using LidarScanFieldTypes = std::vector<ouster::FieldType>;

    LidarScanFieldTypes user_fields{{"CUSTOM0", ChanFieldType::UINT8},
                                    {"CUSTOM3", ChanFieldType::UINT64},
                                    {"CUSTOM9", ChanFieldType::UINT16}};

    ouster::LidarScan user_scan(10, 10, user_fields.begin(), user_fields.end());

    EXPECT_EQ(3, user_scan.fields().size());

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

TEST(LidarScan, test_get_first_valid_packet_timestamp) {
    int w = 1024;
    int h = 32;
    auto scan = ouster::LidarScan(w, h);
    EXPECT_EQ(scan.packet_timestamp().rows(), w / DEFAULT_COLUMNS_PER_PACKET);
    ASSERT_TRUE((scan.packet_timestamp() == 0).all());

    auto packet_ts = scan.packet_timestamp();
    // fill in some default values
    std::iota(packet_ts.data(), packet_ts.data() + packet_ts.size(), 1);
    ASSERT_TRUE((packet_ts == scan.packet_timestamp()).all());

    // no packet found
    EXPECT_EQ(scan.get_first_valid_packet_timestamp(), 0);

    // first packet
    scan.status()[1] = 1;
    EXPECT_EQ(scan.get_first_valid_packet_timestamp(), 1);

    // fifth packet
    scan.status()[1] = 0;
    scan.status()[74] = 1;
    EXPECT_EQ(scan.get_first_valid_packet_timestamp(), 5);

    scan.status()[74] = 0;
    scan.status()[1023] = 1;
    EXPECT_EQ(scan.get_first_valid_packet_timestamp(), 64);
}

TEST(LidarScan, test_packet_timestamps_length) {
    auto ls10 = ouster::LidarScan(10, 64, PROFILE_RNG19_RFL8_SIG16_NIR16, 32);
    auto ls32 = ouster::LidarScan(32, 64, PROFILE_RNG19_RFL8_SIG16_NIR16, 32);
    auto ls33 = ouster::LidarScan(33, 64, PROFILE_RNG19_RFL8_SIG16_NIR16, 32);

    EXPECT_EQ(ls10.packet_timestamp().size(), 1);
    EXPECT_EQ(ls32.packet_timestamp().size(), 1);
    EXPECT_EQ(ls33.packet_timestamp().size(), 2);
}

TEST(LidarScan, destagger) {
    // It raises std::invalid_argument when the image height doesn't match the
    // shift rows
    int w = 32;
    int h = 32;
    auto scan = ouster::LidarScan(w, h);
    std::vector<int> shift_by_row;
    const auto& range = scan.field(ChanField::RANGE);
    EXPECT_THROW(
        {
            try {
                ouster::destagger<unsigned int>(range, shift_by_row);
            } catch (const std::invalid_argument& e) {
                ASSERT_STREQ(e.what(),
                             "image height does not match shifts size");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarScan, lidar_scan_to_string_test) {
    ouster::LidarScan ls(128, 1024, PROFILE_RNG19_RFL8_SIG16_NIR16);
    ls.add_field("custom_field", ouster::fd_array<double>(33, 44, 55), {});
    ls.field<uint32_t>(ChanField::RANGE) = 10;

    std::string s;
    EXPECT_NO_THROW({ s = to_string(ls); });
    std::cout << s << std::endl;
}

TEST(TransformTest, TransformN3) {
    // Define known input points of shape (10, 3)
    ouster::pose_util::Points points(10, 3);
    points << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
        25.0, 26.0, 27.0, 28.0, 29.0, 30.0;

    // Define a non-identity transformation matrix (4x4)
    ouster::pose_util::Pose transformation_matrix;
    transformation_matrix << 0.866, -0.5, 0.0, 1.0, 0.5, 0.866, 0.0, 2.0, 0.0,
        0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0;

    // Expected transformed points (manually calculated)
    ouster::pose_util::Points expected_points(10, 3);
    expected_points << 0.866, 4.232, 2, 1.964, 8.33, 5, 3.062, 12.428, 8, 4.16,
        16.526, 11, 5.258, 20.624, 14, 6.356, 24.722, 17, 7.454, 28.82, 20,
        8.552, 32.918, 23, 9.65, 37.016, 26, 10.748, 41.114, 29;

    // Call the transform function
    ouster::pose_util::Points result =
        ouster::pose_util::transform(points, transformation_matrix);

    // Compare the results
    for (int i = 0; i < result.rows(); ++i) {
        for (int j = 0; j < result.cols(); ++j) {
            EXPECT_NEAR(result(i, j), expected_points(i, j),
                        1e-3);  // Small tolerance for floating-point comparison
        }
    }
}
