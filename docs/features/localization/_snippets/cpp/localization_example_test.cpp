#define main localization_example_main
#include "localization_example.cpp"
#undef main

#include <gtest/gtest.h>

#include <regex>
#include <sstream>
#include <string>

#include "../../../cpp_test_utils/test_utils.h"

using docs_test_utils::capture_stdout;
using docs_test_utils::file_exists;
using docs_test_utils::join_path;

namespace {

std::string osf_fixture_path() {
#ifdef OUSTER_SDK_SOURCE_DIR
    return join_path(join_path(OUSTER_SDK_SOURCE_DIR, "tests"),
                     "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
#else
    return "tests/osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf";
#endif
}

std::string map_fixture_path() {
#ifdef OUSTER_SDK_SOURCE_DIR
    return join_path(join_path(OUSTER_SDK_SOURCE_DIR, "tests"),
                     "osfs/OS-1-128_v2.3.0_1024x10_lb_n3_map-000.ply");
#else
    return "tests/osfs/OS-1-128_v2.3.0_1024x10_lb_n3_map-000.ply";
#endif
}

}  // namespace

// ---------------------------------------------------------------------------
// End-to-end: pose output format and numeric ranges
// ---------------------------------------------------------------------------

TEST(LocalizationExample, PrintsPoseForEachFrame) {
    const auto osf_path = osf_fixture_path();
    const auto map_path = map_fixture_path();

    ASSERT_TRUE(file_exists(osf_path)) << "OSF fixture not found: " << osf_path;
    ASSERT_TRUE(file_exists(map_path)) << "Map fixture not found: " << map_path;

    const char* argv[] = {"localization_example", osf_path.c_str(), map_path.c_str()};
    std::string output;
    ASSERT_NO_THROW(
        output = capture_stdout([&] { localization_example_main(3, const_cast<char**>(argv)); }));

    ASSERT_FALSE(output.empty()) << "Expected pose output but got nothing";

    // Each line must match:
    //   idx = <int>; ts = <int>; XYZ: <f>, <f>, <f> (R: <f>, P: <f>, Y: <f>)
    static const std::regex kLineRe(
        R"(idx = (\d+); ts = (\d+); XYZ: ([-\d.]+), ([-\d.]+), ([-\d.]+) \(R: ([-\d.]+), P: ([-\d.]+), Y: ([-\d.]+)\))");

    std::istringstream ss(output);
    std::string line;
    int line_count = 0;

    while (std::getline(ss, line)) {
        if (line.empty()) continue;
        std::smatch m;
        ASSERT_TRUE(std::regex_search(line, m, kLineRe))
            << "Line did not match expected format: " << line;

        const int idx = std::stoi(m[1]);
        const long ts = std::stol(m[2]);
        const double x = std::stod(m[3]);
        const double y = std::stod(m[4]);
        const double z = std::stod(m[5]);
        const double roll = std::stod(m[6]);
        const double pitch = std::stod(m[7]);
        const double yaw = std::stod(m[8]);

        EXPECT_GE(idx, 1795) << "frame_id below expected range";
        EXPECT_LE(idx, 1797) << "frame_id above expected range";
        EXPECT_GT(ts, 0) << "timestamp must be positive";
        EXPECT_LT(std::abs(x), 1.0) << "X=" << x << " m unexpectedly far from map origin";
        EXPECT_LT(std::abs(y), 1.0) << "Y=" << y << " m unexpectedly far from map origin";
        EXPECT_LT(std::abs(z), 0.1) << "Z=" << z << " m unexpectedly far from map origin";
        EXPECT_LT(std::abs(roll), 5.0) << "Roll=" << roll << "° unexpectedly large";
        EXPECT_LT(std::abs(pitch), 5.0) << "Pitch=" << pitch << "° unexpectedly large";
        EXPECT_LT(std::abs(yaw), 5.0) << "Yaw=" << yaw << "° unexpectedly large";

        ++line_count;
    }

    EXPECT_EQ(line_count, 3) << "Expected 3 pose lines from the test dataset";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
