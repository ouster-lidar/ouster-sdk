#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

#define main po_example_main
#include "po_example.cpp"
#undef main

using namespace ouster::sdk::docs;

namespace {

bool has_test_data_dir() {
    return !env_or_empty("TEST_DATA_DIR").empty();
}
bool is_slow_enabled() {
    return env_or_empty("ENABLE_SLOW_DOC_TESTS") == "1";
}

}  // namespace

TEST(PoExampleSnippet, RunsWithoutErrorAndGeneratesFiles) {
    if (!has_test_data_dir()) {
        GTEST_SKIP() << "TEST_DATA_DIR not set, skipping test";
    }
    if (!is_slow_enabled()) {
        GTEST_SKIP() << "ENABLE_SLOW_DOC_TESTS not set, skipping slow test";
    }

    // Run the pose optimizer example and check it doesn't throw
    ASSERT_NO_THROW(run_pose_optimizer_example());

    // Check that the trajectory file was created
    EXPECT_TRUE(file_exists("loop_test_traj.csv"))
        << "Expected trajectory file 'loop_test_traj.csv' was not created";

    // Check that the OSF output file was created
    EXPECT_TRUE(file_exists("po_output.osf"))
        << "Expected OSF output file 'po_output.osf' was not created";

    // Clean up
    std::remove("loop_test_traj.csv");
    std::remove("po_output.osf");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
