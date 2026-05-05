#include "ouster/beam_config.h"

#include <gtest/gtest.h>

#include "ouster/typedefs.h"

using ouster::sdk::core::BeamConfig;
using ouster::sdk::core::mat4d;

TEST(BeamConfig, it_throws_if_transforms_are_not_set) {
    EXPECT_THROW(
        {
            try {
                BeamConfig bc_invalid(1024, {0.0}, {0.0}, mat4d::Zero(),
                                      mat4d::Identity(), mat4d::Identity(),
                                      0.0025f, 123456789);
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("BeamConfig: beam_to_lidar_transform not set",
                             e.what());
                throw;
            }
        },
        std::logic_error);
    EXPECT_THROW(
        {
            try {
                BeamConfig bc_invalid(1024, {0.0}, {0.0}, mat4d::Identity(),
                                      mat4d::Zero(), mat4d::Identity(), 0.0025f,
                                      123456789);
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("BeamConfig: lidar_to_sensor_transform not set",
                             e.what());
                throw;
            }
        },
        std::logic_error);
    EXPECT_THROW(
        {
            try {
                BeamConfig bc_invalid(1024, {0.0}, {0.0}, mat4d::Identity(),
                                      mat4d::Identity(), mat4d::Zero(), 0.0025f,
                                      123456789);
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("BeamConfig: sensor_to_body_transform not set",
                             e.what());
                throw;
            }
        },
        std::logic_error);
}

TEST(BeamConfig, construct_valid) {
    // It scales the sensor_to_body_transform translation by 1000 (meters to mm)
    // because the XYZLut expects it that way
    mat4d sensor_to_body = mat4d::Identity();
    sensor_to_body(0, 3) = 1.0;
    sensor_to_body(1, 3) = 2.0;
    sensor_to_body(2, 3) = 3.0;
    BeamConfig bc_valid(1024, {0.0, 1.0, 2.0, 3.0}, {0.0, 1.0, 2.0, 3.0},
                        mat4d::Identity(), mat4d::Identity(), sensor_to_body,
                        0.0025f, 123456789);
    EXPECT_EQ(bc_valid.sensor_to_body_transform, sensor_to_body);
    EXPECT_EQ(bc_valid.n_cols, 1024);
    EXPECT_EQ(bc_valid.n_rows, bc_valid.px_altitudes.size());
}

TEST(BeamConfig, equality) {
    auto n_cols = 1024;
    auto m_per_zmbin = 0.0025f;
    auto serial_number = 123456789;
    mat4d beam_to_lidar = mat4d::Identity();
    beam_to_lidar(0, 3) = 0.2;
    mat4d lidar_to_sensor = mat4d::Identity();
    lidar_to_sensor(0, 3) = 0.1;
    const BeamConfig bc1(n_cols, {0.0}, {0.0}, beam_to_lidar, lidar_to_sensor,
                         mat4d::Identity(), m_per_zmbin, serial_number);
    BeamConfig bc2(n_cols, {0.0}, {0.0}, beam_to_lidar, lidar_to_sensor,
                   mat4d::Identity(), m_per_zmbin, serial_number);
    EXPECT_EQ(bc1, bc2);
    bc2.m_per_zmbin = 0.003f;
    EXPECT_NE(bc1, bc2);
    bc2.m_per_zmbin = m_per_zmbin;
    bc2.serial_number = 987654321;
    EXPECT_NE(bc1, bc2);
    bc2.serial_number = serial_number;
    bc2.n_cols = 512;
    EXPECT_NE(bc1, bc2);
    bc2.n_cols = n_cols;
    bc2.beam_to_lidar_transform(0, 3) = 0.3;
    EXPECT_NE(bc1, bc2);
    bc2.beam_to_lidar_transform = beam_to_lidar;
    bc2.lidar_to_sensor_transform(0, 3) = 0.4;
    EXPECT_NE(bc1, bc2);
    bc2.lidar_to_sensor_transform = lidar_to_sensor;
    bc2.px_altitudes = {1.0};
    EXPECT_NE(bc1, bc2);
    bc2.px_altitudes = {0.0};
    bc2.px_azimuths = {1.0};
    EXPECT_NE(bc1, bc2);
    bc2.px_azimuths = {0.0};
    bc2.sensor_to_body_transform(0, 3) = 0.5;
    EXPECT_NE(bc1, bc2);
    bc2.n_rows = 2;
    EXPECT_NE(bc1, bc2);
}
