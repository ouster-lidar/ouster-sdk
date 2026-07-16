#include <gtest/gtest.h>

#include <functional>
#include <iostream>
#include <string>
#include <vector>

#define main stream_single_source_example_main
#include "stream_single_source.cpp"
#undef main

#include "../../../cpp_test_utils/test_utils.h"

using namespace ouster::sdk::core;
using docs_test_utils::capture_stdout;
using docs_test_utils::data_root;
using docs_test_utils::env_or_empty;
using docs_test_utils::file_exists;
using docs_test_utils::join_path;
using ouster::sdk::open_source;

namespace {

struct FixturePaths {
    std::string pcap;
    std::string json;
    std::string osf;
};

FixturePaths fixtures() {
    const auto root = data_root();
    std::cout << "DEBUG: data_root = " << root << std::endl;
    return {
        join_path(join_path(root, "pcaps"), "OS-1-128_v2.3.0_1024x10_lb_n3.pcap"),
        join_path(join_path(root, "pcaps"), "OS-1-128_v2.3.0_1024x10.json"),
        join_path(join_path(root, "osfs"), "single_scan_016.osf"),
    };
}

}  // namespace

TEST(StreamSingleSourceSnippet, DebugFixturePaths) {
    const auto paths = fixtures();
    std::cout << "PCAP path: " << paths.pcap << std::endl;
    std::cout << "JSON path: " << paths.json << std::endl;
    std::cout << "OSF path: " << paths.osf << std::endl;
    std::cout << "PCAP exists: " << file_exists(paths.pcap) << std::endl;
    std::cout << "JSON exists: " << file_exists(paths.json) << std::endl;
    std::cout << "OSF exists: " << file_exists(paths.osf) << std::endl;
}

TEST(StreamSingleSourceSnippet, ReplayDataPrintsFrame) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.pcap)) << "Required test fixture not found: " << paths.pcap;
    ASSERT_TRUE(file_exists(paths.json)) << "Required test fixture not found: " << paths.json;

    const auto output = capture_stdout([&]() { replay_data(paths.pcap, paths.json); });
    EXPECT_NE(output.find("frame"), std::string::npos) << "Output: '" << output << "'";
}

TEST(StreamSingleSourceSnippet, ReplayPcapMetadataPrintsFrame) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.pcap)) << "Required test fixture not found: " << paths.pcap;
    ASSERT_TRUE(file_exists(paths.json)) << "Required test fixture not found: " << paths.json;

    const auto output = capture_stdout([&]() { replay_pcap_metadata(paths.pcap, paths.json); });

    std::cout << "ReplayPcapMetadata output: " << output << std::endl;
    EXPECT_NE(output.find("frame 0"), std::string::npos) << "Output: '" << output << "'";
}

TEST(StreamSingleSourceSnippet, ReplayOpenSourceMetadataPrintsFrame) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.pcap)) << "Required test fixture not found: " << paths.pcap;
    ASSERT_TRUE(file_exists(paths.json)) << "Required test fixture not found: " << paths.json;

    const auto output =
        capture_stdout([&]() { replay_open_source_metadata(paths.pcap, paths.json); });
    std::cout << "ReplayOpenSourceMetadata output: " << output << std::endl;
    EXPECT_NE(output.find("frame 0"), std::string::npos) << "Output: '" << output << "'";
}

TEST(StreamSingleSourceSnippet, ReplayOsfPrintsFrame) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.osf)) << "Required test fixture not found: " << paths.osf;

    std::string output;
    try {
        output = capture_stdout([&]() {
            auto source = open_source(paths.osf);
            print_frames(source, 1);
        });
    } catch (const std::runtime_error& err) {
        FAIL() << "Failed to open OSF fixture: " << err.what();
        return;
    }

    EXPECT_NE(output.find("frame"), std::string::npos) << "Output: '" << output << "'";
}

TEST(StreamSingleSourceSnippet, ReadOsfFramesPrintFrameInfo) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.osf)) << "Required test fixture not found: " << paths.osf;

    const auto output = capture_stdout([&]() { read_osf_frames(paths.osf, /*limit=*/1); });

    EXPECT_NE(output.find("frame "), std::string::npos) << "Output: '" << output << "'";
}

TEST(StreamSingleSourceSnippet, SelectSingleSensorPrintsFrameId) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.osf)) << "Required test fixture not found: " << paths.osf;

    std::string output;
    try {
        output = capture_stdout([&]() { select_single_sensor(paths.osf); });
    } catch (const std::runtime_error& err) {
        FAIL() << "select_single_sensor threw: " << err.what();
        return;
    }
    EXPECT_NE(output.find("via sensor_idx: frame_id="), std::string::npos)
        << "Output: '" << output << "'";
    EXPECT_NE(output.find("via single():   frame_id="), std::string::npos)
        << "Output: '" << output << "'";
}

TEST(StreamSingleSourceSnippet, StreamLiveSensorSkipsIfNoHostname) {
    const auto hostname = env_or_empty("SENSOR_HOSTNAME");
    if (hostname.empty()) {
        GTEST_SKIP() << "SENSOR_HOSTNAME not set; skipping live sensor test";
    }
    EXPECT_NO_THROW(stream_live_sensor(hostname));
}

TEST(StreamSingleSourceSnippet, StreamLiveViaOpenSourceCollateSkipsIfNoHostname) {
    const auto hostname = env_or_empty("SENSOR_HOSTNAME");
    if (hostname.empty()) {
        GTEST_SKIP() << "SENSOR_HOSTNAME not set; skipping live collated test";
    }
    EXPECT_NO_THROW(stream_live_via_open_source_collate(hostname));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
