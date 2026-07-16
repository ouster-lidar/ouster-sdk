#include "stream_multi_source.cpp"

#include <gtest/gtest.h>

#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "../../../cpp_test_utils/test_utils.h"
#include "ouster/core/impl/open_source_impl.h"
#include "ouster/core/io_type.h"
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/pcap/pcap_frame_set_source.h"

using namespace ouster::sdk::core;
using docs_test_utils::capture_stdout;
using docs_test_utils::data_root;
using docs_test_utils::file_exists;
using docs_test_utils::join_path;
using ouster::sdk::FrameSetSourceOptions;
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

    return {join_path(join_path(root, "pcaps"), "OS-0-32-U1_v2.2.0_1024x10-single-packet.pcap"),
            join_path(join_path(root, "pcaps"), "OS-0-32-U1_v2.2.0_1024x10.json"),
            join_path(join_path(root, "osfs"), "single_scan_016.osf")};
}

}  // namespace

TEST(StreamMultiSourceSnippet, DebugFixturePaths) {
    const auto paths = fixtures();
    std::cout << "PCAP path: " << paths.pcap << std::endl;
    std::cout << "JSON path: " << paths.json << std::endl;
    std::cout << "OSF path: " << paths.osf << std::endl;

    std::cout << "PCAP exists: " << file_exists(paths.pcap) << std::endl;
    std::cout << "JSON exists: " << file_exists(paths.json) << std::endl;
    std::cout << "OSF exists: " << file_exists(paths.osf) << std::endl;
}

TEST(StreamMultiSourceSnippet, IterateSourcePrintsFrame) {
    const auto paths = fixtures();

    ASSERT_TRUE(file_exists(paths.pcap)) << "Required test fixture not found: " << paths.pcap;
    ASSERT_TRUE(file_exists(paths.json)) << "Required test fixture not found: " << paths.json;

    std::string output;
    ASSERT_NO_THROW({
        output = capture_stdout([&]() {
            auto source = open_source(
                std::vector<std::string>{paths.pcap},
                [&](FrameSetSourceOptions& opt) {
                    opt.meta = std::vector<std::string>{paths.json};
                },
                /*collate=*/true,
                /*sensor_idx=*/-1);
            iterate_source(source, 1);
        });
    });

    std::cout << "Captured output: '" << output << "'" << std::endl;
    EXPECT_NE(output.find("frame: 0"), std::string::npos);
}

TEST(StreamMultiSourceSnippet, StreamMultiOpenSourcePrintsFrame) {
    const auto paths = fixtures();

    ASSERT_TRUE(file_exists(paths.pcap)) << "Required test fixture not found: " << paths.pcap;
    ASSERT_TRUE(file_exists(paths.json)) << "Required test fixture not found: " << paths.json;

    std::string output;
    ASSERT_NO_THROW({
        output = capture_stdout([&]() {
            stream_multi_open_source(std::vector<std::string>{paths.pcap},
                                     std::vector<std::string>{paths.json}, 1);
        });
    });

    std::cout << "Captured output: '" << output << "'" << std::endl;
    EXPECT_NE(output.find("frame: 0"), std::string::npos)
        << "Expected to find 'frame: 0' in output: '" << output << "'";
}

TEST(StreamMultiSourceSnippet, ReplayMultiRecordingPrintsFrame) {
    const auto paths = fixtures();

    ASSERT_TRUE(file_exists(paths.osf)) << "Required test fixture not found: " << paths.osf;

    std::string output;
    ASSERT_NO_THROW({
        output = capture_stdout(
            [&]() { replay_multi_recording(std::vector<std::string>{paths.osf}, 1); });
    });

    std::cout << "Captured output: '" << output << "'" << std::endl;
    EXPECT_NE(output.find("frame: 0"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
