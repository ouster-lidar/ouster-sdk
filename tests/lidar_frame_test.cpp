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
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "info_util.h"
#include "ouster/core/camera_frame.h"
#include "ouster/core/frame_set.h"
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
    auto frame4 = LidarFrame(fake_sensor_info(20, 10));
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

TEST(LidarFrame, receive_timestamp) {
    int w = 32;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    // host timestamp header should have w/columns-per-packet entries
    // (default DEFAULT_COLUMNS_PER_PACKET is 16)
    EXPECT_EQ(frame.receive_timestamp().rows(), 2);
    EXPECT_TRUE((frame.receive_timestamp() == 0).all());

    LidarInfo info;
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

TEST(LidarFrame, receive_timestamp_2) {
    // FrameBatcher::operator() should throw
    // if the packet timestamp header index exceeds the header size in the
    // LidarFrame
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, 32));
    EXPECT_EQ(frame.receive_timestamp().rows(), frame.w / DEFAULT_COLUMNS_PER_PACKET);
    EXPECT_TRUE((frame.receive_timestamp() == 0).all());

    LidarInfo info;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info.format.pixels_per_column = h;
    // not enough columns per packet according to the measurement id
    info.format.columns_per_packet = 1;
    info.format.imu_measurements_per_packet = 1;
    auto pf = std::make_shared<PacketFormat>(info);

    LidarPacket packet(pf);
    packet.host_timestamp = 123;

    uint8_t* col_buf = const_cast<uint8_t*>(pf->nth_col(0, packet.buf.data()));
    uint16_t bogus_measurement_id = 1234;
    std::memcpy(col_buf + 8, &bogus_measurement_id, sizeof(bogus_measurement_id));
    const uint16_t m_id = pf->col_measurement_id(col_buf);
    ASSERT_EQ(m_id, bogus_measurement_id);

    FrameBatcher frame_batcher(info);
    EXPECT_THROW(
        {
            try {
                frame_batcher.batch(packet, frame);
            } catch (std::invalid_argument& e) {
                EXPECT_STREQ(e.what(), "unexpected frame dimensions");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(LidarFrame, receive_timestamp_3) {
    // FrameBatcher::operator() should update the packet timestamp header
    // in the LidarFrame if a LidarPacket is provided
    int w = 32;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    EXPECT_EQ(frame.receive_timestamp().rows(), w / DEFAULT_COLUMNS_PER_PACKET);
    EXPECT_TRUE((frame.receive_timestamp() == 0).all());

    LidarInfo info;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info.format.pixels_per_column = h;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;  // not enough columns per packet
                                                                  // according to the measurement id
    info.format.columns_per_frame = w;
    info.format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    info.format.pixels_per_column = h;
    info.format.imu_measurements_per_packet = 1;
    auto pf = std::make_shared<PacketFormat>(info);

    LidarPacket packet(pf);
    packet.host_timestamp = 123;

    uint8_t* col_buf = const_cast<uint8_t*>(pf->nth_col(0, packet.buf.data()));
    uint16_t bogus_measurement_id = 0;
    std::memcpy(col_buf + 8, &bogus_measurement_id, sizeof(bogus_measurement_id));
    const uint16_t m_id = pf->col_measurement_id(col_buf);
    ASSERT_EQ(m_id, bogus_measurement_id);

    FrameBatcher frame_batcher(info);
    frame_batcher.batch(packet, frame);
    EXPECT_EQ(frame.receive_timestamp()[0], packet.host_timestamp);
    EXPECT_EQ(frame.receive_timestamp()[1], 0);
}

TEST(LidarFrame, test_get_first_valid_receive_timestamp) {
    int w = 1024;
    int h = 32;
    auto frame = LidarFrame(fake_sensor_info(h, w));
    EXPECT_EQ(frame.receive_timestamp().rows(), w / DEFAULT_COLUMNS_PER_PACKET);
    ASSERT_TRUE((frame.receive_timestamp() == 0).all());

    auto packet_ts = frame.receive_timestamp();
    // fill in some default values
    std::iota(packet_ts.data(), packet_ts.data() + packet_ts.size(), 1);
    ASSERT_TRUE((packet_ts == frame.receive_timestamp()).all());

    // no packet found
    EXPECT_THROW(frame.get_first_valid_receive_timestamp(), std::runtime_error);

    // first packet
    frame.status()[1] = 1;
    EXPECT_EQ(frame.get_first_valid_receive_timestamp(), 1);

    // fifth packet
    frame.status()[1] = 0;
    frame.status()[74] = 1;
    EXPECT_EQ(frame.get_first_valid_receive_timestamp(), 5);

    frame.status()[74] = 0;
    frame.status()[1023] = 1;
    EXPECT_EQ(frame.get_first_valid_receive_timestamp(), 64);
}

TEST(LidarFrame, test_receive_timestamps_length) {
    auto ls10 = LidarFrame(fake_sensor_info(64, 10, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 32));
    auto ls32 = LidarFrame(fake_sensor_info(64, 32, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 32));
    auto ls33 = LidarFrame(fake_sensor_info(64, 33, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 32));

    EXPECT_EQ(ls10.receive_timestamp().size(), 1);
    EXPECT_EQ(ls32.receive_timestamp().size(), 1);
    EXPECT_EQ(ls33.receive_timestamp().size(), 2);
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
    auto info = std::make_shared<LidarInfo>();
    info->format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info->format.columns_per_frame = 1024;
    info->format.pixels_per_column = 128;
    info->format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    info->format.columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;

    LidarFrame orig_frame(info);
    LidarFrame copy_frame(orig_frame);

    EXPECT_EQ(copy_frame.sensor_info(), orig_frame.sensor_info());
}

TEST(LidarFrame, typed_sensor_info_accessor_and_setter) {
    LidarFrame frame;
    EXPECT_EQ(frame.sensor_info(), nullptr);

    auto info = std::make_shared<LidarInfo>();
    info->sn = 12345;
    frame.set_sensor_info(info);

    // The accessor's result type follows the static type of the expression.
    static_assert(std::is_same<decltype(frame.sensor_info()), std::shared_ptr<LidarInfo>>::value,
                  "LidarFrame::sensor_info() must return a shared_ptr<LidarInfo>");
    EXPECT_EQ(frame.sensor_info(), info);

    // Viewed as a Frame, the same name yields the shared base type.
    const Frame& base = frame;
    static_assert(
        std::is_same<decltype(base.sensor_info()), const std::shared_ptr<SensorInfo>&>::value,
        "Frame::sensor_info() must return a shared_ptr<SensorInfo>");
    EXPECT_EQ(base.sensor_info(), info);
    EXPECT_EQ(base.sensor_info()->sn, 12345u);

    frame.set_sensor_info(nullptr);
    EXPECT_EQ(frame.sensor_info(), nullptr);
}

TEST(CameraFrame, typed_sensor_info_accessor_and_setter) {
    CameraFrame frame;
    EXPECT_EQ(frame.sensor_info(), nullptr);

    auto info = std::make_shared<CameraInfo>();
    info->sn = 54321;
    frame.set_sensor_info(info);

    static_assert(std::is_same<decltype(frame.sensor_info()), std::shared_ptr<CameraInfo>>::value,
                  "CameraFrame::sensor_info() must return a shared_ptr<CameraInfo>");
    EXPECT_EQ(frame.sensor_info(), info);

    const Frame& base = frame;
    EXPECT_EQ(base.sensor_info(), info);
}

TEST(Frame, assign_sensor_info_rejects_mismatched_kind) {
    // set_sensor_info() rejects the wrong metadata type at compile time, so the
    // only way to attempt a mismatch is through the generic write path - which
    // checks the kind at runtime instead.
    CameraFrame frame;
    auto lidar_info = std::make_shared<LidarInfo>();

    EXPECT_FALSE(frame.accepts_sensor_info(*lidar_info));
    EXPECT_THROW(impl::assign_sensor_info(frame, lidar_info), std::invalid_argument);
    EXPECT_EQ(frame.sensor_info(), nullptr);

    auto camera_info = std::make_shared<CameraInfo>();
    EXPECT_TRUE(frame.accepts_sensor_info(*camera_info));
    EXPECT_NO_THROW(impl::assign_sensor_info(frame, camera_info));
    EXPECT_EQ(frame.sensor_info(), camera_info);

    // Clearing is always allowed.
    EXPECT_NO_THROW(impl::assign_sensor_info(frame, nullptr));
    EXPECT_EQ(frame.sensor_info(), nullptr);
}

TEST(FrameSet, valid_frames_of_selects_by_concrete_type) {
    FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{std::make_shared<LidarFrame>(), nullptr,
                                                           std::make_shared<CameraFrame>(),
                                                           std::make_shared<LidarFrame>()});

    EXPECT_EQ(frame_set.valid_frames_of<LidarFrame>().size(), 2u);
    EXPECT_EQ(frame_set.valid_frames_of<CameraFrame>().size(), 1u);
    EXPECT_EQ(frame_set.valid_frames_of<Frame>().size(), 3u);

    // The named aliases stay in agreement with the template.
    EXPECT_EQ(frame_set.valid_lidar_frames().size(),
              frame_set.valid_frames_of<LidarFrame>().size());
    EXPECT_EQ(frame_set.valid_camera_frames().size(),
              frame_set.valid_frames_of<CameraFrame>().size());
}

// Every member of LidarFrame participates in equality. Each block below
// perturbs exactly one member of an otherwise identical pair.
TEST(LidarFrame, EqualityCoversEveryMember) {
    auto info = fake_sensor_info(4, 8);

    {  // status: a column marked valid is not the same data as an invalid one
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.status()[0] = 1;  // bit 0 = valid
        EXPECT_TRUE(a != b);
    }
    {  // alert flags, from the eUDP packet headers
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.alert_flags()[0] = 7;
        EXPECT_TRUE(a != b);
    }
    {  // thermal shutdown countdown
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.shutdown_countdown = 3;
        EXPECT_TRUE(a != b);
    }
    {  // shot limiting countdown
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.shot_limiting_countdown = 5;
        EXPECT_TRUE(a != b);
    }
    {  // measurement id header
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.measurement_id()[0] = 42;
        EXPECT_TRUE(a != b);
    }
    {  // frame id and frame status
        LidarFrame a(info), b(info);
        b.frame_id = 9;
        EXPECT_TRUE(a != b);

        LidarFrame c(info), d(info);
        d.frame_status = 9;
        EXPECT_TRUE(c != d);
    }
    {  // timestamp, receive timestamp, and pose headers
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.timestamp()[0] = 1234;
        EXPECT_TRUE(a != b);

        LidarFrame c(info), d(info);
        d.receive_timestamp()[0] = 1234;
        EXPECT_TRUE(c != d);

        LidarFrame e(info), f(info);
        Matrix4dR pose = Matrix4dR::Identity();
        pose(0, 3) = 1.0;
        f.set_column_pose(0, pose);
        EXPECT_TRUE(e != f);
    }
    {  // field contents
        LidarFrame a(info), b(info);
        ASSERT_TRUE(a == b);
        b.field<uint32_t>(ChanField::RANGE)(0, 0) = 1;
        EXPECT_TRUE(a != b);
    }
}

// A copy compares equal even after every member has been perturbed, which
// guards against equality accidentally ignoring a member wholesale.
TEST(LidarFrame, EqualityOfPerturbedCopy) {
    LidarFrame original(fake_sensor_info(4, 8));
    original.frame_id = 7;
    original.frame_status = 3;
    original.shutdown_countdown = 2;
    original.shot_limiting_countdown = 1;
    original.status()[0] = 1;
    original.alert_flags()[0] = 4;
    original.measurement_id()[1] = 5;
    original.timestamp()[1] = 99;
    original.field<uint32_t>(ChanField::RANGE)(1, 1) = 1234;

    LidarFrame copy(original);
    EXPECT_TRUE(original == copy);
    EXPECT_FALSE(original != copy);
}

TEST(CameraFrame, Equality) {
    auto info = std::make_shared<CameraInfo>();

    CameraFrame a(info), b(info);
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);

    {  // dimensions
        CameraFrame wide(info);
        wide.w = 640;
        wide.h = 480;
        EXPECT_TRUE(a != wide);

        CameraFrame also_wide(wide);
        EXPECT_TRUE(wide == also_wide);
    }
    {  // timestamp and pose headers
        CameraFrame other(info);
        other.timestamp()[0] = 1234;
        EXPECT_TRUE(a != other);

        CameraFrame posed(info);
        Matrix4dR pose = Matrix4dR::Identity();
        pose(1, 3) = 2.0;
        posed.set_column_pose(0, pose);
        EXPECT_TRUE(a != posed);
    }
    {  // field presence and contents
        CameraFrame with_field(info);
        with_field.w = 4;
        with_field.h = 2;
        with_field.add_field(FieldType(ChanField::RANGE, ChanFieldType::UINT32));

        CameraFrame same(with_field);
        EXPECT_TRUE(with_field == same);

        same.field<uint32_t>(ChanField::RANGE)(0, 0) = 1;
        EXPECT_TRUE(with_field != same);
    }
}

// Equality is not compared across concrete frame types, including when the
// frames are only known as base references.
TEST(Frame, EqualityDistinguishesConcreteTypes) {
    LidarFrame lidar;
    CameraFrame camera;

    const Frame& lidar_base = lidar;
    const Frame& camera_base = camera;

    EXPECT_TRUE(lidar_base != camera_base);
    EXPECT_TRUE(camera_base != lidar_base);
    EXPECT_FALSE(lidar_base == camera_base);

    // Comparing through the base still reaches the derived state, rather than
    // comparing only the members declared on Frame.
    LidarFrame other_lidar;
    other_lidar.frame_id = 1;
    const Frame& other_lidar_base = other_lidar;
    EXPECT_TRUE(lidar_base != other_lidar_base);
}

TEST(FrameSet, EqualityComparesFramesOfEveryType) {
    auto lidar_info = fake_sensor_info(4, 8);
    auto camera_info = std::make_shared<CameraInfo>();

    auto make_set = [&]() {
        auto camera = std::make_shared<CameraFrame>(camera_info);
        camera->w = 4;
        camera->h = 2;
        camera->add_field(FieldType(ChanField::RANGE, ChanFieldType::UINT32));
        return FrameSet(std::vector<std::shared_ptr<Frame>>{
            std::make_shared<LidarFrame>(lidar_info), nullptr, camera});
    };

    EXPECT_TRUE(make_set() == make_set());
    EXPECT_FALSE(make_set() != make_set());

    {  // a difference confined to the camera frame is detected
        auto a = make_set();
        auto b = make_set();
        b[2]->field<uint32_t>(ChanField::RANGE)(0, 0) = 1;
        EXPECT_TRUE(a != b);
    }
    {  // as is a difference confined to the lidar frame
        auto a = make_set();
        auto b = make_set();
        std::static_pointer_cast<LidarFrame>(b[0])->frame_id = 1;
        EXPECT_TRUE(a != b);
    }
    {  // swapping a camera frame for a lidar frame is not equal
        auto a = make_set();
        auto b = make_set();
        b[2] = std::make_shared<LidarFrame>(lidar_info);
        EXPECT_TRUE(a != b);
    }
    {  // a missing frame is not equal to a present one
        auto a = make_set();
        auto b = make_set();
        b[1] = std::make_shared<LidarFrame>(lidar_info);
        EXPECT_TRUE(a != b);
    }
}

TEST(LidarFrame, to_string_zone_states) {
    // It doesn't throw when ZoneState field is present
    LidarFrame ls(fake_sensor_info(1, 1));
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

TEST(LidarFrame, valid_columns_via_base_frame) {
    LidarFrame frame(fake_sensor_info(10, 10));
    frame.status().setZero();
    frame.status()[3] = 1u;
    frame.status()[7] = 1u;

    // Callable on the concrete type and polymorphically through Frame&, since
    // the accessors now live on the base Frame.
    EXPECT_EQ(frame.get_first_valid_column(), 3);
    EXPECT_EQ(frame.get_last_valid_column(), 7);

    Frame& base = frame;
    EXPECT_EQ(base.get_first_valid_column(), 3);
    EXPECT_EQ(base.get_last_valid_column(), 7);
}
