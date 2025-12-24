#include "zone_header.h"

#include <gtest/gtest.h>

#include <array>

#include "ouster/zrb.h"

using ouster::sdk::core::CacheRenderMetadata;
using ouster::sdk::core::SerialNumber;
using ouster::sdk::core::Sha256;
using ouster::sdk::core::Zrb;

TEST(CacheRenderMetadata, default_constructor_initializes_members) {
    ouster::sdk::core::CacheRenderMetadata metadata;

    // Check that all members are initialized to zero or equivalent
    EXPECT_FLOAT_EQ(metadata.m_per_zmbin, 0.0f);
    EXPECT_EQ(metadata.stl_hash, ouster::sdk::core::Sha256());
    auto zero_sn = ouster::sdk::core::SerialNumber{};
    EXPECT_EQ(metadata.serial_number, zero_sn);
    auto zero_transform = std::array<float, 16>{};
    EXPECT_EQ(metadata.beam_to_lidar, zero_transform);
    EXPECT_EQ(metadata.lidar_to_sensor, zero_transform);
    EXPECT_EQ(metadata.sensor_to_body, zero_transform);
}

TEST(CacheRenderMetadata, throw_if_transforms_not_set) {
    Zrb zrb;
    std::bitset<2048> valid_col_mask;
    EXPECT_THROW(
        {
            try {
                ouster::sdk::core::CacheRenderMetadata metadata(
                    zrb, valid_col_mask,
                    ouster::sdk::core::serial_number_from_int(123456789));
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(
                    "CacheRenderMetadata: beam_to_lidar_transform not set",
                    e.what());
                throw;
            }
        },
        std::logic_error);
    zrb.beam_to_lidar_transform = ouster::sdk::core::mat4d::Identity();
    EXPECT_THROW(
        {
            try {
                ouster::sdk::core::CacheRenderMetadata metadata(
                    zrb, valid_col_mask,
                    ouster::sdk::core::serial_number_from_int(123456789));
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(
                    "CacheRenderMetadata: lidar_to_sensor_transform not set",
                    e.what());
                throw;
            }
        },
        std::logic_error);
    zrb.lidar_to_sensor_transform = ouster::sdk::core::mat4d::Identity();
    EXPECT_THROW(
        {
            try {
                ouster::sdk::core::CacheRenderMetadata metadata(
                    zrb, valid_col_mask,
                    ouster::sdk::core::serial_number_from_int(123456789));
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(
                    "CacheRenderMetadata: sensor_to_body_transform not set",
                    e.what());
                throw;
            }
        },
        std::logic_error);
}
