#include "pcap_packet_source.cpp"

#include <gtest/gtest.h>

#include <string>

#include "../../../cpp_test_utils/test_utils.h"

using docs_test_utils::capture_stdout;
using docs_test_utils::file_exists;
using docs_test_utils::join_path;

namespace {

std::string repo_tests_dir() {
#ifdef OUSTER_SDK_SOURCE_DIR
    return join_path(OUSTER_SDK_SOURCE_DIR, "tests");
#else
    return {};
#endif
}

}  // namespace

TEST(PcapPacketSourceSnippet, CountsAndPrintsTypedPackets) {
    const auto tests_root = repo_tests_dir();
    if (tests_root.empty()) {
        GTEST_SKIP() << "OUSTER_SDK_SOURCE_DIR not defined";
    }

    const auto pcap_path =
        join_path(join_path(tests_root, "pcaps"), "OS-0-32-U1_v2.2.0_1024x10.pcap");

    ASSERT_TRUE(file_exists(pcap_path)) << "Missing PCAP fixture: " << pcap_path;

    ouster::sdk::docs::PacketCounts counts;
    const auto output =
        capture_stdout([&]() { counts = ouster::sdk::docs::read_pcap_packets(pcap_path); });

    EXPECT_EQ(counts.lidar, 64u);
    EXPECT_EQ(counts.imu, 10u);

    EXPECT_NE(output.find("sensor=0 lidar frame_id=1453 bytes=8448"), std::string::npos)
        << "Missing first lidar packet summary. Output: '" << output << "'";
    EXPECT_NE(output.find("sensor=0 imu host_ts=1635444808691218000"), std::string::npos)
        << "Missing first imu packet summary. Output: '" << output << "'";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
