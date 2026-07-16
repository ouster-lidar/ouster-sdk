#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>

#define main record_packet_example_main
#include "read_packet.cpp"
#undef main

TEST(ReadPacketSnippet, PrintsPacketSummary) {
    const char* sensor_hostname = std::getenv("SENSOR_HOSTNAME");
    if (!sensor_hostname || std::string(sensor_hostname).empty()) {
        GTEST_SKIP() << "SENSOR_HOSTNAME environment variable not set";
    }

    std::cout << "[DEBUG] Using sensor hostname: " << sensor_hostname << std::endl;
    ASSERT_NO_THROW({
        ouster::sdk::docs::record_sensor_session(sensor_hostname, 7502, 7503,
                                                 /*n_seconds=*/2);
    });
    // No output capture; just ensure it runs without throwing.
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
