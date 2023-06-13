/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <gtest/gtest.h>

#include "common.h"

namespace ouster {
namespace osf {

// Base class for all osf util tests
class OsfTest : public ::testing::Test {};

// Test fixture to get and check test data dir
// use it for tests that needed test data
class OsfTestWithData : public OsfTest {
   protected:
    virtual void SetUp() {
        if (!get_test_data_dir(test_data_dir_)) {
            FAIL() << "Can't get DATA_DIR";
            return;
        }
    }

    std::string test_data_dir() { return test_data_dir_; }

   private:
    std::string test_data_dir_;
};

class OsfTestWithDataAndFiles : public osf::OsfTestWithData {
   public:
    static std::string output_dir;
    static std::vector<std::string> files;
    std::string tmp_file(const std::string& basename) {
        std::string res = path_concat(output_dir, basename);
        // TODO[pb]: Switch to map? to avoid overlaps/double delete
        files.push_back(res);
        return res;
    }
    static void SetUpTestCase() {
        if (!make_tmp_dir(output_dir)) FAIL();
    }

    // clean up temp files
    static void TearDownTestCase() {
        for (const auto& path : files) unlink_path(path);
        remove_dir(output_dir);
    }
};
std::string OsfTestWithDataAndFiles::output_dir = {};
std::vector<std::string> OsfTestWithDataAndFiles::files = {};

}  // namespace osf
}  // namespace ouster
