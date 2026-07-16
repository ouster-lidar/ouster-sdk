/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/lidar_frame.h"

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdlib>
#include <map>
#include <numeric>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/core/impl/lidar_frame_impl.h"
#include "ouster/core/types.h"

#define TEST_REPEAT 5

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;
using namespace ouster::sdk::core;

static const std::vector<FieldType> empty_field_slots{};

static const std::vector<FieldType> legacy_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT16},
     {ChanField::NEAR_IR, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::FLAGS, ChanFieldType::UINT8}}};

static const std::vector<FieldType> dual_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT16},
     {ChanField::SIGNAL2, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
     {ChanField::FLAGS, ChanFieldType::UINT8},
     {ChanField::FLAGS2, ChanFieldType::UINT8},
     {ChanField::WINDOW, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

static const std::vector<FieldType> contrived_slots{
    {{ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::SIGNAL2, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

static const std::vector<FieldType> duplicated_slots{
    {{ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::RANGE2, ChanFieldType::UINT32},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::NEAR_IR, ChanFieldType::UINT16}}};

struct set_field_data {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, int data) {
        for (int x = 0; x < field.rows(); x++) {
            for (int y = 0; y < field.cols(); y++) {
                field(x, y) = data;
            }
        }
    }
};

struct check_field_data {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, int data) {
        EXPECT_TRUE((field == data).all());
    }
};

void zero_check_fields(LidarFrame& frame) {
    for (const auto& field : frame.fields()) {
        impl::visit_field(frame, field.first, check_field_data(), 0);
    }
}

std::shared_ptr<SensorInfo> fake_sensor_info(
    int h, int w,
    ouster::sdk::core::UDPProfileLidar profile = ouster::sdk::core::UDPProfileLidar::LEGACY) {
    SensorInfo info;
    info.format.columns_per_packet = 16;
    info.format.pixels_per_column = h;
    info.format.columns_per_frame = w;
    info.format.udp_profile_lidar = profile;
    info.format.udp_profile_imu = ouster::sdk::core::UDPProfileIMU::LEGACY;
    info.fw_rev = "3.2.1";
    info.image_rev = "3.2.1";
    return std::make_shared<SensorInfo>(std::move(info));
}

TEST(LidarFrameSanityTests, get_field_types_test) {
    LidarFrameFieldTypes ft;
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

    auto ls = LidarFrame(fake_sensor_info(128, 1028), ft);
    EXPECT_EQ(ft, ls.field_types());
}

TEST(LidarFrame, EmptyConstructorInit) {
    auto frame = LidarFrame();
    EXPECT_EQ(frame.w, 0);
    EXPECT_EQ(frame.h, 0);

    EXPECT_EQ(frame.frame_id, -1);

    EXPECT_EQ(frame.fields().size(), 0);

    zero_check_fields(frame);
}

TEST(LidarFrame, LegacyConstructorInit) {
    int w = 10;
    int h = 20;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    EXPECT_EQ(frame.w, w);
    EXPECT_EQ(frame.h, h);
    EXPECT_EQ(frame.frame_id, -1);

    size_t count = 0;
    size_t hit_count = 0;
    std::vector<std::string> field_copy;
    for (auto item : legacy_field_slots) {
        field_copy.push_back(item.name);
    }
    for (const auto& field : frame.fields()) {
        auto result = std::find(field_copy.begin(), field_copy.end(), field.first);
        if (result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0u);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(legacy_field_slots.size(), count);

    EXPECT_TRUE((frame.status() == 0).all());
    EXPECT_TRUE((frame.measurement_id() == 0).all());
    EXPECT_TRUE((frame.timestamp() == 0).all());

    zero_check_fields(frame);
}

TEST(LidarFrame, DualReturnConstructorInit) {
    int w = 40;
    int h = 60;
    auto frame = LidarFrame(fake_sensor_info(h, w, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL));
    EXPECT_EQ(frame.w, w);
    EXPECT_EQ(frame.h, h);
    EXPECT_EQ(frame.frame_id, -1);

    size_t count = 0;
    size_t hit_count = 0;
    std::vector<std::string> field_copy;
    for (auto item : dual_field_slots) {
        field_copy.push_back(item.name);
    }
    for (const auto& field : frame.fields()) {
        auto result = std::find(field_copy.begin(), field_copy.end(), field.first);
        if (result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0u);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(dual_field_slots.size(), count);

    EXPECT_TRUE((frame.status() == 0).all());
    EXPECT_TRUE((frame.measurement_id() == 0).all());
    EXPECT_TRUE((frame.timestamp() == 0).all());

    zero_check_fields(frame);
}

TEST(LidarFrame, CustomFieldConstructorInit) {
    int w = 80;
    int h = 100;
    auto frame = LidarFrame(fake_sensor_info(h, w), contrived_slots);
    EXPECT_EQ(frame.w, w);
    EXPECT_EQ(frame.h, h);
    EXPECT_EQ(frame.frame_id, -1);

    size_t count = 0;
    size_t hit_count = 0;
    std::vector<std::string> field_copy;
    for (auto item : contrived_slots) {
        field_copy.push_back(item.name);
    }

    for (const auto& field : frame.fields()) {
        auto result = std::find(field_copy.begin(), field_copy.end(), field.first);
        if (result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0u);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(contrived_slots.size(), count);

    EXPECT_TRUE((frame.status() == 0).all());
    EXPECT_TRUE((frame.measurement_id() == 0).all());
    EXPECT_TRUE((frame.timestamp() == 0).all());

    zero_check_fields(frame);
}

TEST(LidarFrame, EmptyEquality) {
    auto frame1 = LidarFrame();
    auto frame2 = LidarFrame();
    auto frame3 = LidarFrame();
    auto frame4 = LidarFrame(20, 10);
    frame3.frame_id = 1;

    EXPECT_TRUE(frame1 == frame2);
    EXPECT_TRUE(frame1 != frame3);
    EXPECT_TRUE(frame1 != frame4);
}

TEST(LidarFrame, LegacyEquality) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        auto info = fake_sensor_info(h, w);
        auto frame1 = LidarFrame(info);
        auto frame2 = LidarFrame(info);
        auto frame3 = LidarFrame(info);
        auto frame4 = LidarFrame(info, legacy_field_slots);
        auto frame5 =
            LidarFrame(fake_sensor_info(h, w, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL));
        frame3.frame_id = 1;

        EXPECT_TRUE(frame1 == frame2);
        EXPECT_TRUE(frame1 != frame3);
        EXPECT_TRUE(frame1 == frame4);
        EXPECT_TRUE(frame1 != frame5);
    }
}

TEST(LidarFrame, DualReturnsEquality) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        auto info = fake_sensor_info(h, w, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
        auto frame1 = LidarFrame(info);
        auto frame2 = LidarFrame(info);
        auto frame3 = LidarFrame(info);
        auto frame4 = LidarFrame(info, dual_field_slots);
        frame3.frame_id = 1;

        EXPECT_TRUE(frame1 == frame2);
        EXPECT_TRUE(frame1 != frame3);
        EXPECT_TRUE(frame1 == frame4);
    }
}

TEST(LidarFrame, CustomEquality) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        auto test_array1 = dual_field_slots;
        auto test_array2 = dual_field_slots;
        auto test_array3 =
            std::vector<FieldType>(dual_field_slots.begin() + 1, dual_field_slots.end());

        std::default_random_engine rand_gen;
        std::shuffle(test_array1.begin(), test_array1.end(), rand_gen);
        std::shuffle(test_array3.begin(), test_array3.end(), rand_gen);

        auto info = fake_sensor_info(h, w);
        auto frame1 = LidarFrame(info, test_array1);
        auto frame2 = LidarFrame(info, test_array1);
        auto frame3 = LidarFrame(info, test_array2);
        auto frame4 = LidarFrame(info, test_array2);
        auto frame5 = LidarFrame(info, test_array3);

        frame4.frame_id = 1;

        EXPECT_TRUE(frame1 == frame2);
        EXPECT_TRUE(frame1 == frame3);
        EXPECT_TRUE(frame3 != frame4);
        EXPECT_TRUE(frame1 != frame5);
    }
    EXPECT_THROW({ auto frame6 = LidarFrame(fake_sensor_info(10, 10), duplicated_slots); },
                 std::invalid_argument);
}

TEST(LidarFrame, DataCheck) {
    for (int i = 0; i < TEST_REPEAT; i++) {
        int w = rand() % 1000 + 1;
        int h = rand() % 1000 + 1;

        std::unordered_map<std::string, uint8_t> expected_data;

        auto info = fake_sensor_info(h, w, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
        auto frame1 = LidarFrame(info);
        auto frame2 = LidarFrame(info);
        auto frame3 = LidarFrame(info);

        for (const auto& field : frame1.fields()) {
            expected_data[field.first] = rand() % 254 + 1;
            impl::visit_field(frame1, field.first, set_field_data(), expected_data[field.first]);
            impl::visit_field(frame2, field.first, set_field_data(), expected_data[field.first]);
            impl::visit_field(frame1, field.first, check_field_data(), expected_data[field.first]);
        }

        EXPECT_TRUE(frame1 == frame2);
        EXPECT_TRUE(frame1 != frame3);
    }
}

TEST(LidarFrame, CustomUserFields) {
    using LidarFrameFieldTypes = std::vector<FieldType>;

    LidarFrameFieldTypes user_fields{{"CUSTOM0", ChanFieldType::UINT8},
                                     {"CUSTOM3", ChanFieldType::UINT64},
                                     {"CUSTOM9", ChanFieldType::UINT16}};

    LidarFrame user_frame(fake_sensor_info(10, 10), user_fields);

    EXPECT_EQ(3, user_frame.fields().size());

    zero_check_fields(user_frame);
}

TEST(LidarFrame, packet_timestamp) {
    int w = 32;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    // host timestamp header should have w/columns-per-packet entries
    // (default DEFAULT_COLUMNS_PER_PACKET is 16)
    EXPECT_EQ(frame.packet_timestamp().rows(), 2);
    EXPECT_TRUE((frame.packet_timestamp() == 0).all());

    LidarPacket packet;
    SensorInfo info;
    info.format.columns_per_frame = w;
    info.format.columns_per_packet = 0;
    info.format.pixels_per_column = h;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;

    EXPECT_THROW(
        {
            try {
                FrameBatcher frame_batcher(info);
            } catch (std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), "unexpected columns_per_packet: 0");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarFrame, packet_timestamp_2) {
    // FrameBatcher::operator() should throw
    // if the packet timestamp header index exceeds the header size in the
    // LidarFrame
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, 32));
    EXPECT_EQ(frame.packet_timestamp().rows(), frame.w / DEFAULT_COLUMNS_PER_PACKET);
    EXPECT_TRUE((frame.packet_timestamp() == 0).all());

    SensorInfo info;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info.format.pixels_per_column = h;
    // not enough columns per packet according to the measurement id
    info.format.columns_per_packet = 1;
    info.format.imu_measurements_per_packet = 1;
    auto pf = get_format(info);

    LidarPacket packet(pf.lidar_packet_size);
    packet.host_timestamp = 123;

    uint8_t* col_buf = const_cast<uint8_t*>(pf.nth_col(0, packet.buf.data()));
    uint16_t bogus_measurement_id = 1234;
    std::memcpy(col_buf + 8, &bogus_measurement_id, sizeof(bogus_measurement_id));
    const uint16_t m_id = pf.col_measurement_id(col_buf);
    ASSERT_EQ(m_id, bogus_measurement_id);

    FrameBatcher frame_batcher(info);
    EXPECT_THROW(
        {
            try {
                frame_batcher(packet, frame);
            } catch (std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), "unexpected frame dimensions");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarFrame, packet_timestamp_3) {
    // FrameBatcher::operator() should update the packet timestamp header
    // in the LidarFrame if a LidarPacket is provided
    int w = 32;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    EXPECT_EQ(frame.packet_timestamp().rows(), w / DEFAULT_COLUMNS_PER_PACKET);
    EXPECT_TRUE((frame.packet_timestamp() == 0).all());

    SensorInfo info;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info.format.pixels_per_column = h;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;  // not enough columns per packet
                                                                  // according to the measurement id
    info.format.columns_per_frame = w;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    info.format.pixels_per_column = h;
    info.format.imu_measurements_per_packet = 1;
    auto pf = get_format(info);

    LidarPacket packet(pf.lidar_packet_size);
    packet.host_timestamp = 123;

    uint8_t* col_buf = const_cast<uint8_t*>(pf.nth_col(0, packet.buf.data()));
    uint16_t bogus_measurement_id = 0;
    std::memcpy(col_buf + 8, &bogus_measurement_id, sizeof(bogus_measurement_id));
    const uint16_t m_id = pf.col_measurement_id(col_buf);
    ASSERT_EQ(m_id, bogus_measurement_id);

    FrameBatcher frame_batcher(info);
    frame_batcher(packet, frame);
    EXPECT_EQ(frame.packet_timestamp()[0], packet.host_timestamp);
    EXPECT_EQ(frame.packet_timestamp()[1], 0);
}

TEST(LidarFrame, test_get_first_valid_packet_timestamp) {
    int w = 1024;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    EXPECT_EQ(frame.packet_timestamp().rows(), w / DEFAULT_COLUMNS_PER_PACKET);
    ASSERT_TRUE((frame.packet_timestamp() == 0).all());

    auto packet_ts = frame.packet_timestamp();
    // fill in some default values
    std::iota(packet_ts.data(), packet_ts.data() + packet_ts.size(), 1);
    ASSERT_TRUE((packet_ts == frame.packet_timestamp()).all());

    // no packet found
    EXPECT_THROW(frame.get_first_valid_packet_timestamp(), std::runtime_error);

    // first packet
    frame.status()[1] = 1;
    EXPECT_EQ(frame.get_first_valid_packet_timestamp(), 1);

    // fifth packet
    frame.status()[1] = 0;
    frame.status()[74] = 1;
    EXPECT_EQ(frame.get_first_valid_packet_timestamp(), 5);

    frame.status()[74] = 0;
    frame.status()[1023] = 1;
    EXPECT_EQ(frame.get_first_valid_packet_timestamp(), 64);
}

TEST(LidarFrame, test_packet_timestamps_length) {
    auto ls10 = LidarFrame(64, 10, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 32);
    auto ls32 = LidarFrame(64, 32, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 32);
    auto ls33 = LidarFrame(64, 33, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 32);

    EXPECT_EQ(ls10.packet_timestamp().size(), 1);
    EXPECT_EQ(ls32.packet_timestamp().size(), 1);
    EXPECT_EQ(ls33.packet_timestamp().size(), 2);
}

TEST(LidarFrame, destagger) {
    // It raises std::invalid_argument when the image height doesn't match the
    // shift rows
    int w = 32;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    std::vector<int> shift_by_row;
    const auto& range = frame.field(ChanField::RANGE);
    EXPECT_THROW(
        {
            try {
                destagger<unsigned int>(range, shift_by_row);
            } catch (const std::invalid_argument& e) {
                ASSERT_STREQ(e.what(), "image height does not match shifts size");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarFrame, lidar_frame_to_string_test) {
    LidarFrame ls(fake_sensor_info(128, 1024, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16));
    ls.add_field("custom_field", fd_array<double>(33, 44, 55), FieldClass::FRAME_FIELD);
    ls.field<uint32_t>(ChanField::RANGE) = 10;

    std::string s;
    EXPECT_NO_THROW({ s = to_string(ls); });
    std::cout << s << std::endl;
}

TEST(LidarFrame, sensor_info_being_copied) {
    SensorInfo info;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info.format.columns_per_frame = 1024;
    info.format.pixels_per_column = 128;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;

    LidarFrame orig_frame(info);
    LidarFrame copy_frame(orig_frame);

    EXPECT_EQ(copy_frame.sensor_info, orig_frame.sensor_info);
}

TEST(LidarFrame, to_string_zone_states) {
    // It doesn't throw when ZoneState field is present
    LidarFrame ls(1, 1);
    ls.add_field(ChanField::ZONE_STATES, fd_array<ZoneState>(16), FieldClass::FRAME_FIELD);
    to_string(ls);
}

TEST(LidarFrame, object_lists_being_cloned) {
    LidarFrame frame(fake_sensor_info(10, 10));
    using namespace ouster::sdk;
    std::vector<core::Object> objects(2);
    objects[0].id = 1;
    objects[0].creation_ts = 99;
    objects[0].timestamp = 199;
    objects[0].class_id = 1;
    objects[0].class_confidence = 0.9;
    objects[0].object_to_body.set_position(Eigen::Vector3d{1, 2, 3});
    objects[0].object_to_body.set_rotation(Eigen::Vector3d{2, 2, 2});
    objects[0].body_to_world.set_position(Eigen::Vector3d{10, 20, 30});
    objects[0].body_to_world.set_rotation(Eigen::Vector3d{0.1, 0.2, 0.3});
    objects[0].velocity = core::UnalignedVector3f{2, 3, 4};
    objects[0].dimensions = core::UnalignedVector3f{1, 1, 1};
    objects[0].properties["num_points"] = "[100]";
    objects[0].properties["attributes"] = "\[\"eats_icecream\", \"carries_bag\"]";
    objects[1].id = 2;
    objects[1].creation_ts = 100;
    objects[1].timestamp = 200;
    objects[1].class_id = 2;
    objects[1].class_confidence = 0.8;
    objects[1].object_to_body.set_position(Eigen::Vector3d{3, 2, 1});
    objects[1].object_to_body.set_rotation(Eigen::Vector3d{1, 1, 1});
    objects[1].body_to_world.set_position(Eigen::Vector3d{4, 5, 6});
    objects[1].body_to_world.set_rotation(Eigen::Vector3d{0.4, 0.5, 0.6});
    objects[1].velocity = core::UnalignedVector3f{4, 3, 2};
    objects[1].dimensions = core::UnalignedVector3f{2, 2, 2};
    objects[1].properties["num_points"] = "\"[50]\"";
    objects[1].properties["attributes"] = "[\"parked_illegaly\"]";
    frame.objects()["test_objects"] = objects;

    // test copy constructor
    LidarFrame frame_copied{frame};
    // test copy is performed correctly
    EXPECT_EQ(frame_copied.objects(), frame.objects());
    // test copy is not shallow
    frame_copied.objects()["copy_obj_list"] = objects;
    EXPECT_NE(frame_copied.objects(), frame.objects());
    // test comparison
    EXPECT_NE(frame_copied, frame);

    // test copy assignment
    LidarFrame frame_copy_assigned = frame;
    // test copy is performed correctly
    EXPECT_EQ(frame_copy_assigned.objects(), frame.objects());
    // test copy is not shallow
    frame_copy_assigned.objects()["copy_obj_list"] = objects;
    EXPECT_NE(frame_copy_assigned.objects(), frame.objects());
    // test comparison
    EXPECT_NE(frame_copy_assigned, frame);
}

TEST(LidarFrame, move) {
    LidarFrame frame1(fake_sensor_info(10, 10));
    frame1.frame_id = 42;
    LidarFrame frame2 = std::move(frame1);
    EXPECT_EQ(frame2.frame_id, 42);
    EXPECT_EQ(frame1.frame_id, 42);
}

TEST(LidarFrame, no_valid_columns_throws) {
    LidarFrame frame(fake_sensor_info(10, 10));
    frame.status().setZero();
    EXPECT_THROW(frame.get_first_valid_column(), std::runtime_error);
    EXPECT_THROW(frame.get_last_valid_column(), std::runtime_error);
}
