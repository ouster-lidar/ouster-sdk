#include <gtest/gtest.h>

#include <string>

#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

const std::string test_string = "TEST STRING HERE";

TEST(Sensor, SmokeTests) {
    ouster::sdk::core::SensorConfig config;
    config.udp_dest = test_string;
    ouster::sdk::sensor::Sensor test(test_string, config);

    EXPECT_EQ(test.hostname(), test_string);
    EXPECT_EQ(test.desired_config().udp_dest, test_string);
}
