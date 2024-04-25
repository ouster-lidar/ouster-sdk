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

TEST_F(FileOpsTest, TruncateFile) {
    const int fsize = 1000;
    const int trunc_size = 450;

    std::string temp_dir;
    EXPECT_TRUE(make_tmp_dir(temp_dir));
    std::string temp_file = path_concat(temp_dir, "test_file");

    std::fstream test_file_out;
    test_file_out.open(temp_file, std::fstream::out | std::fstream::trunc |
                                      std::fstream::binary);
    for (int i = 0; i < fsize; i++) {
        test_file_out << (uint8_t)i;
    }
    test_file_out.close();

    EXPECT_EQ(file_size(temp_file), fsize);
    truncate_file(temp_file, trunc_size);
    EXPECT_EQ(file_size(temp_file), trunc_size);
    unlink_path(temp_file);
    remove_dir(temp_dir);
}

TEST_F(FileOpsTest, AppendBinaryFileBlank) {
    const int fsize = 10;

    std::string temp_dir;
    EXPECT_TRUE(make_tmp_dir(temp_dir));
    std::string temp_file = path_concat(temp_dir, "test_file");
    std::string temp_file2 = path_concat(temp_dir, "test_file2");

    std::fstream test_file;
    test_file.open(temp_file2, std::fstream::out | std::fstream::trunc |
                                   std::fstream::binary);
    for (int i = 0; i < fsize; i++) {
        test_file << (uint8_t)i;
    }
    test_file.close();

    EXPECT_EQ(file_size(temp_file2), fsize);
    EXPECT_EQ(append_binary_file(temp_file, temp_file2), fsize);
    EXPECT_EQ(file_size(temp_file), fsize);

    test_file.open(temp_file2, std::fstream::in | std::fstream::binary);
    for (int i = 0; i < fsize; i++) {
        char temp;
        test_file.read(&temp, 1);
        EXPECT_EQ(temp, (char)i);
    }
    test_file.close();

    unlink_path(temp_file);
    unlink_path(temp_file2);
    remove_dir(temp_dir);
}

TEST_F(FileOpsTest, AppendBinaryFile) {
    const int fsize1 = 100;
    const int fsize2 = 50;

    std::string temp_dir;
    EXPECT_TRUE(make_tmp_dir(temp_dir));
    std::string temp_file = path_concat(temp_dir, "test_file");
    std::string temp_file2 = path_concat(temp_dir, "test_file2");

    std::fstream test_file;
    test_file.open(temp_file, std::fstream::out | std::fstream::trunc |
                                  std::fstream::binary);
    for (int i = 0; i < fsize1; i++) {
        test_file << (uint8_t)i;
    }
    test_file.close();

    test_file.open(temp_file2, std::fstream::out | std::fstream::trunc |
                                   std::fstream::binary);
    for (int i = 0; i < fsize2; i++) {
        test_file << (uint8_t)(i + fsize1);
    }
    test_file.close();

    EXPECT_EQ(file_size(temp_file), fsize1);
    EXPECT_EQ(file_size(temp_file2), fsize2);
    EXPECT_EQ(append_binary_file(temp_file, temp_file2), (fsize1 + fsize2));
    EXPECT_EQ(file_size(temp_file), (fsize1 + fsize2));

    test_file.open(temp_file, std::fstream::in | std::fstream::binary);
    for (int i = 0; i < (fsize1 + fsize2); i++) {
        char temp;
        test_file.read(&temp, 1);
        EXPECT_EQ(temp, (char)i);
    }
    test_file.close();

    unlink_path(temp_file);
    unlink_path(temp_file2);
    remove_dir(temp_dir);
}

TEST_F(FileOpsTest, CopyTrailingBytes) {
    const int fsize1 = 200;
    const int offset = 150;

    std::string temp_dir;
    EXPECT_TRUE(make_tmp_dir(temp_dir));
    std::string temp_file = path_concat(temp_dir, "test_file");
    std::string temp_file2 = path_concat(temp_dir, "test_file2");

    std::fstream test_file;
    test_file.open(temp_file, std::fstream::out | std::fstream::trunc |
                                  std::fstream::binary);
    for (int i = 0; i < fsize1; i++) {
        test_file << (uint8_t)i;
    }
    test_file.close();

    EXPECT_EQ(file_size(temp_file), fsize1);
    EXPECT_EQ(copy_file_trailing_bytes(temp_file, temp_file2, offset),
              (fsize1 - offset));
    EXPECT_EQ(file_size(temp_file2), (fsize1 - offset));

    std::fstream test_file2;
    test_file2.open(temp_file2, std::fstream::in | std::fstream::binary);
    for (int i = 0; i < (fsize1 - offset); i++) {
        char temp;
        test_file2.read(&temp, 1);
        EXPECT_EQ(temp, (char)(i + offset));
    }
    test_file2.close();

    unlink_path(temp_file);
    unlink_path(temp_file2);
    remove_dir(temp_dir);
}

}  // namespace
}  // namespace osf
}  // namespace ouster
