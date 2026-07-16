#include "slam_dewarp_example.cpp"

#include <gtest/gtest.h>

#include <string>

#include "../../../cpp_test_utils/test_utils.h"

using docs_test_utils::file_exists;
using docs_test_utils::join_path;
using ouster::sdk::docs::slam_dewarp_once;

namespace {
std::string osf_fixture_path() {
#ifdef OUSTER_SDK_SOURCE_DIR
    return join_path(join_path(OUSTER_SDK_SOURCE_DIR, "tests"), "osfs/single_scan_016.osf");
#else
    return "tests/osfs/single_scan_016.osf";
#endif
}
}  // namespace

TEST(SlamDewarpExample, ProducesPointsFromOsf) {
    const auto osf_path = osf_fixture_path();
    ASSERT_TRUE(file_exists(osf_path)) << "OSF sample not available: " << osf_path;

    auto points = slam_dewarp_once(osf_path);
    ASSERT_GT(points.size(), 0);

    const bool has_non_zero = (points.array().abs() > 1e-6f).any();
    EXPECT_TRUE(has_non_zero);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
