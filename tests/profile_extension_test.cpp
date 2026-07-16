/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/profile_extension.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"

using namespace ouster::sdk::core;
using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::FieldDecodeInfo;

std::shared_ptr<SensorInfo> fake_sensor_info(
    int h, int w,
    ouster::sdk::core::UDPProfileLidar profile = ouster::sdk::core::UDPProfileLidar::LEGACY) {
    SensorInfo info;
    info.format.columns_per_packet = 16;
    info.format.pixels_per_column = h;
    info.format.columns_per_frame = w;
    info.format.udp_profile_lidar = profile;
    info.format.udp_profile_imu = ouster::sdk::core::UDPProfileIMU::LEGACY;
    return std::make_shared<SensorInfo>(std::move(info));
}

TEST(ProfileExtension, ProfileExtensionTest) {
    std::string name = "DUAL_RETURNS_COPYCAT";
    // clang-format off
    std::vector<std::pair<std::string, FieldDecodeInfo>> fields{
        {ChanField::RANGE, {ChanFieldType::UINT32, 0, 0x0007ffff, 0}},
        {ChanField::FLAGS, {ChanFieldType::UINT8, 2, 0b11111000, 3}},
        {ChanField::REFLECTIVITY, {ChanFieldType::UINT8, 3, 0, 0}},
        {ChanField::RANGE2, {ChanFieldType::UINT32, 4, 0x0007ffff, 0}},
        {ChanField::FLAGS2, {ChanFieldType::UINT8, 6, 0b11111000, 3}},
        {ChanField::REFLECTIVITY2, {ChanFieldType::UINT8, 7, 0, 0}},
        {ChanField::SIGNAL, {ChanFieldType::UINT16, 8, 0, 0}},
        {ChanField::SIGNAL2, {ChanFieldType::UINT16, 10, 0, 0}},
        {ChanField::NEAR_IR, {ChanFieldType::UINT16, 12, 0, 0}},
        {ChanField::RAW32_WORD1, {ChanFieldType::UINT32, 0, 0, 0}},
        {ChanField::RAW32_WORD2, {ChanFieldType::UINT32, 4, 0, 0}},
        {ChanField::RAW32_WORD3, {ChanFieldType::UINT32, 8, 0, 0}},
        {ChanField::RAW32_WORD4, {ChanFieldType::UINT32, 12, 0, 0}}};
    size_t chan_data_size = 16;

    UDPProfileLidar profile_nr = static_cast<UDPProfileLidar>(-1);
    EXPECT_NO_THROW(profile_nr = add_custom_profile(name, fields, chan_data_size));
    UDPProfileLidar prof = static_cast<UDPProfileLidar>(profile_nr);
    EXPECT_EQ(udp_profile_lidar_of_string(name).value(), prof);

    auto frame = LidarFrame(fake_sensor_info(60, 40, prof));

    for (const auto& field: fields)
    {
        auto res = frame.fields().find(field.first);
        EXPECT_TRUE(res != frame.fields().end());
    }
    EXPECT_EQ(frame.fields().size(), fields.size());

    // TODO: would be good to check parsing here too -- Tim T.

    // profile already exists
    EXPECT_THROW(
        add_custom_profile(static_cast<int>(profile_nr), name, fields, chan_data_size),
        std::invalid_argument);
    EXPECT_THROW(
        add_custom_profile(static_cast<int>(UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL),
                           "NEW_NAME_DUALRETURNS", fields, chan_data_size),
        std::invalid_argument);
    EXPECT_THROW(
        add_custom_profile(110,
                           to_string(UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL),
                           fields, chan_data_size),
        std::invalid_argument);
    // nr 0 is prohibited
    EXPECT_THROW(
        add_custom_profile(0, name, fields, chan_data_size),
        std::invalid_argument);
}
