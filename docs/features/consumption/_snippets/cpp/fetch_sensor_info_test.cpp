#include <gtest/gtest.h>

#include <iostream>
#include <string>

#define main fetch_sensor_info_example_main
#include "fetch_sensor_info.cpp"
#undef main

#include "../../../cpp_test_utils/test_utils.h"

using namespace ouster::sdk;
using docs_test_utils::env_or_empty;
using docs_test_utils::file_exists;

TEST(FetchSensorInfoSnippet, FetchesAndWritesSensorInfo) {
    const auto hostname = env_or_empty("SENSOR_HOSTNAME");
    if (hostname.empty()) {
        GTEST_SKIP() << "SENSOR_HOSTNAME not set; skipping live sensor test";
    }

    const std::string json_file = hostname + ".json";

    // Remove file if it exists from previous run
    if (file_exists(json_file)) {
        std::remove(json_file.c_str());
    }

    ASSERT_NO_THROW(fetch_sensor_info(hostname));

    // Verify JSON file was created
    ASSERT_TRUE(file_exists(json_file)) << "Expected JSON file to be created: " << json_file;

    // Clean up
    std::remove(json_file.c_str());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
