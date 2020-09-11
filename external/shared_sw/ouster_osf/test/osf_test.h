#pragma once

#include <gtest/gtest.h>

#include "common.h"

namespace ouster {
namespace OSF {

// Base class for all osf util tests
// TEST_FULL_SET env var is used to enforce all tests and FAIL if
// TEST_DATA_DIR is not available.
//
// Usually TEST_DATA_DIR defined in 'warden-dev app test' and in CI setup
// routines
class OsfTest : public ::testing::Test {
   protected:
    static void SetUpTestCase() {
        char* test_full_set_var;
        if ((test_full_set_var = std::getenv("TEST_FULL_SET")) != nullptr) {
            if (strcmp(test_full_set_var, "true") == 0) {
                test_full_set_ = true;
            }
        }
    }

    // Should we do tests that rely on test_data existence
    // which should be provided via TEST_DATA_DIR env var.
    bool test_full_set() { return test_full_set_; }

   private:
    static bool test_full_set_;
};

// By default we are not requiring TEST_DATA_DIR so the
// tests that require test_data can be skipped.
bool OsfTest::test_full_set_ = false;

// Test fixture to get and check test data dir
// use it for tests that needed test data
class OsfTestWithData : public OsfTest {
   protected:
    virtual void SetUp() {
        if (!get_test_data_dir(&test_data_dir_)) {
            // TODO: Fix this behavour later when we have a uniform way of
            // getting test data for everyone
            // FAIL() should be a correct behaviour always and everywhere
            if (test_full_set()) {
                FAIL() << "Can't get TEST_DATA_DIR";
                return;
            }
            // TODO[pb]: Use GTEST_SKIP() when it's available
            skipped_ = true;
            return;
        }
        skipped_ = false;
    }

    std::string test_data_dir() { return test_data_dir_; }
    bool skipped() { return skipped_; }

   private:
    std::string test_data_dir_;
    bool skipped_;
};

#define TEST_DATA_SKIP()                \
    if (skipped()) {                    \
        std::cout << "TEST SKIPPED!\n"; \
        return;                         \
    }

}  // namespace OSF
}  // namespace ouster
