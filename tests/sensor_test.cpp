#include <gtest/gtest.h>

#include <string>

#include "ouster/core/types.h"
#include "ouster/sensor/sensor_frame_set_source.h"

const std::string test_string = "TEST STRING HERE";

TEST(Sensor, SmokeTests) {
    ouster::sdk::core::SensorConfig config;
    config.udp_dest = test_string;
    ouster::sdk::sensor::Sensor test(test_string, config);

    EXPECT_EQ(test.hostname(), test_string);
    EXPECT_EQ(test.desired_config().udp_dest, test_string);
}
