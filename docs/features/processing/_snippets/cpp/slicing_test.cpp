#include "slicing.cpp"

#include <gtest/gtest.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "../../../cpp_test_utils/test_utils.h"
#include "ouster/core/open_source.h"
#include "ouster/osf/osf_frame_set_source.h"

using namespace ouster::sdk;
using docs_test_utils::capture_stdout;
using docs_test_utils::file_exists;
using docs_test_utils::join_path;

namespace {
std::string fixture_path() {
    return join_path(join_path("tests", "osfs"), "osf_fw_3.2_10scans.osf");
}

std::string parent_path(const std::string& path) {
    const auto pos = path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return {};
    }
    return path.substr(0, pos);
}

std::string resolved_fixture_path() {
    const std::string rel = fixture_path();
    if (file_exists(rel)) {
        return rel;
    }

    std::string probe = parent_path(__FILE__);
    for (int i = 0; i < 8; ++i) {
        const std::string candidate = join_path(probe, rel);
        if (file_exists(candidate)) {
            return candidate;
        }
        std::string next = parent_path(probe);
        if (next.empty() || next == probe) {
            break;
        }
        probe = next;
    }
    throw std::runtime_error("Required test fixture not found: " + rel);
}

core::AnyFrameSetSource osf_fixture() {
    const auto osf_path = resolved_fixture_path();
    if (!file_exists(osf_path)) {
        throw std::runtime_error("Required test fixture not found: " + osf_path);
    }
    auto source = open_source(osf_path, [](FrameSetSourceOptions& opts) { opts.index = true; });
    return source;
}

const std::vector<int> expected_frames = {58684, 58685, 58686, 58687, 58688,
                                          58689, 58690, 58691, 58692, 58693};

std::string frames_to_string(const std::vector<int>& frames) {
    std::ostringstream oss;
    for (int frame : frames) {
        oss << frame << '\n';
    }
    return oss.str();
}

}  // namespace

TEST(ProcessingSlicingCpp, PrintTenthFrameExamplePrintsExpectedFrames) {
    const auto source = osf_fixture();
    const auto output =
        capture_stdout([&]() { ouster::docs::print_nth_frame_example_cpp(source); });

    std::vector<int> expected = {expected_frames.at(9), expected_frames.at(9)};
    EXPECT_EQ(output, frames_to_string(expected));
}

TEST(ProcessingSlicingCpp, PrintLastFrameExamplePrintsExpectedFrame) {
    const auto source = osf_fixture();
    const auto output =
        capture_stdout([&]() { ouster::docs::print_last_frame_example_cpp(source); });

    EXPECT_EQ(output, frames_to_string(std::vector<int>{expected_frames.back()}));
}

TEST(ProcessingSlicingCpp, PrintFirstTenFramesExamplePrintsExpectedFrames) {
    const auto source = osf_fixture();
    const auto output =
        capture_stdout([&]() { ouster::docs::print_first_n_frames_example_cpp(source); });

    std::vector<int> expected(expected_frames.begin(), expected_frames.begin() + 9);
    EXPECT_EQ(output, frames_to_string(expected));
}

TEST(ProcessingSlicingCpp, PrintStepSlicedFramesExamplePrintsExpectedFrames) {
    const auto source = osf_fixture();
    const auto output =
        capture_stdout([&]() { ouster::docs::print_step_sliced_frames_example_cpp(source); });

    std::vector<int> expected;
    for (size_t i = 0; i < 10; i += 2) {
        expected.push_back(expected_frames.at(i));
    }
    EXPECT_EQ(output, frames_to_string(expected));
}

TEST(ProcessingSlicingCpp, FirstValidColumnExamplePrintsColumnIndex) {
    const auto source = osf_fixture();
    const auto output =
        capture_stdout([&]() { ouster::docs::first_valid_column_example_cpp(source); });

    EXPECT_EQ(output, "0\n");
}

TEST(ProcessingSlicingCpp, LastValidColumnExamplePrintsColumnIndex) {
    const auto source = osf_fixture();
    const auto output =
        capture_stdout([&]() { ouster::docs::last_valid_column_example_cpp(source); });

    EXPECT_EQ(output, "2047\n");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
