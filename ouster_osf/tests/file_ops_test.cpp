/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "common.h"
#include "osf_test.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {
namespace {

class FileOpsTest : public OsfTestWithDataAndFiles {};

TEST_F(FileOpsTest, TempDir) {
    std::string tmp_dir;
    EXPECT_TRUE(make_tmp_dir(tmp_dir));
    EXPECT_TRUE(path_exists(tmp_dir));
    EXPECT_TRUE(is_dir(tmp_dir));

    EXPECT_TRUE(remove_dir(tmp_dir));

    EXPECT_FALSE(path_exists(tmp_dir));
    EXPECT_FALSE(is_dir(tmp_dir));

    EXPECT_FALSE(unlink_path(tmp_dir));
    EXPECT_FALSE(remove_dir(tmp_dir));
}

TEST_F(FileOpsTest, TempAndMakeDir) {
    std::string tmp_dir;
    EXPECT_TRUE(make_tmp_dir(tmp_dir));
    std::string tmp_dir_new = path_concat(tmp_dir, "new_dir");
    EXPECT_FALSE(path_exists(tmp_dir_new));
    EXPECT_FALSE(is_dir(tmp_dir_new));

    EXPECT_TRUE(make_dir(tmp_dir_new));

    EXPECT_TRUE(path_exists(tmp_dir_new));
    EXPECT_TRUE(is_dir(tmp_dir_new));

    // Can't remove non-empty dir
    EXPECT_FALSE(remove_dir(tmp_dir));
    EXPECT_FALSE(unlink_path(tmp_dir));

    EXPECT_TRUE(remove_dir(tmp_dir_new));
    EXPECT_FALSE(path_exists(tmp_dir_new));
    EXPECT_FALSE(is_dir(tmp_dir_new));

    EXPECT_TRUE(remove_dir(tmp_dir));
}

TEST_F(FileOpsTest, IsDirGeneral) {
    EXPECT_FALSE(is_dir(""));
    EXPECT_TRUE(is_dir("."));
}

TEST_F(FileOpsTest, PathConcats) {
    EXPECT_EQ("", path_concat("", ""));
    EXPECT_EQ("hello", path_concat("", "hello"));
    EXPECT_EQ("hello", path_concat("hello", ""));
#ifdef _WIN32
    EXPECT_EQ("c:\\b", path_concat("c:", "b"));
    EXPECT_EQ("/a/b\\f/g/", path_concat("/a/b//", "f/g/"));
    EXPECT_EQ("/f/g/", path_concat("/a/b//", "/f/g/"));
    EXPECT_EQ("f:/g/", path_concat("/a/b//", "f:/g/"));
#else
    EXPECT_EQ("/", path_concat("/", ""));
    EXPECT_EQ("////", path_concat("////", ""));
    EXPECT_EQ("//", path_concat("//", ""));
    EXPECT_EQ("/a", path_concat("//", "a"));
    EXPECT_EQ("/a", path_concat("//", "a"));
    EXPECT_EQ("/a", path_concat("//b", "/a"));
    EXPECT_EQ("/", path_concat("", "/"));
    EXPECT_EQ("//", path_concat("", "//"));
    EXPECT_EQ("////", path_concat("/", "////"));
    EXPECT_EQ("/a/b", path_concat("/a\\", "b"));
    EXPECT_EQ("/a/b", path_concat("/a\\//", "b"));
    EXPECT_EQ("c:/b", path_concat("c:", "b"));
    EXPECT_EQ("/a/b/f", path_concat("/a/b/", "f"));
    EXPECT_EQ("/a/b/f/g", path_concat("/a/b/", "f/g"));
    EXPECT_EQ("/a/b/f/g/", path_concat("/a/b/", "f/g/"));
    EXPECT_EQ("/a/b/f/g/", path_concat("/a/b//", "f/g/"));
    EXPECT_EQ("/f/g/", path_concat("/a/b//", "/f/g/"));
    // Yes its weird and this function just can't be ideal ...
    EXPECT_EQ("/a/b/f:/g/", path_concat("/a/b//", "f:/g/"));
#endif
}

TEST_F(FileOpsTest, TestDataDirCheck) { EXPECT_TRUE(is_dir(test_data_dir())); }

TEST_F(FileOpsTest, TestFileSize) {
    // TODO[pb]: Change to file creation later ...
    const std::string test_file_name =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    int64_t fsize = file_size(test_file_name);
    EXPECT_EQ(1021684, fsize);
    std::string not_a_file = path_concat(test_data_dir(), "not_a_file");
    EXPECT_TRUE(file_size(not_a_file) < 0);
    EXPECT_TRUE(file_size(test_data_dir()) < 0);
}

TEST_F(FileOpsTest, TestFileMapping) {
    // TODO[pb]: Change to file creation later ...
    const std::string test_file_name =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");

    uint8_t* file_buf = mmap_open(test_file_name);
    EXPECT_TRUE(file_buf != nullptr);

    int64_t fsize = file_size(test_file_name);
    EXPECT_EQ(1021684, fsize);

    if (file_buf != nullptr) {
        std::cout << "bytes = " << to_string(file_buf, 64) << std::endl;
        std::cout << "bytes = " << to_string(file_buf + 4, 64) << std::endl;
        std::cout << "bytes = " << to_string(file_buf + 4 + 4, 64) << std::endl;
        std::cout << "bytes = " << to_string(file_buf + fsize - 64, 64)
                  << std::endl;
    }

    EXPECT_TRUE(mmap_close(file_buf, fsize));
}

}  // namespace
}  // namespace osf
}  // namespace ouster
