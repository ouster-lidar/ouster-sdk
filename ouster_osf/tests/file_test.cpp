/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/file.h"

#include <gtest/gtest.h>

#include "fb_utils.h"
#include "osf_test.h"
#include "ouster/osf/basics.h"

namespace ouster {
namespace osf {
namespace {

class OsfFileTest : public OsfTestWithData {};

TEST_F(OsfFileTest, OpensOsfFileDefaultAsBadState) {
    // This opens nothing and produces the file in a !good() state
    OsfFile osf_file;

    EXPECT_FALSE(osf_file.good());

    // Check operator!
    if (!osf_file) {
        SUCCEED();
    }

    // Check bool operator
    bool ok = osf_file.good();
    if (ok) FAIL();
}

TEST_F(OsfFileTest, OpenOsfFileNominally) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));
    EXPECT_TRUE(osf_file);
    EXPECT_EQ(osf_file.version(), OSF_VERSION::V_2_0);
    EXPECT_EQ(osf_file.size(), 1021684);
    EXPECT_EQ(osf_file.offset(), 0);

    EXPECT_EQ(osf_file.metadata_offset(), 1013976);
    std::cout << "file = " << osf_file.to_string() << std::endl;

    EXPECT_EQ(osf_file.seek(100).offset(), 100);

    // Out of range seek should throw
    EXPECT_THROW(osf_file.seek(50000000), std::out_of_range);

    // OSF v2 OsfFile should be OK
    EXPECT_TRUE(osf_file.valid());

    // Test move semantics
    OsfFile osff(std::move(osf_file));
    EXPECT_FALSE(osf_file.good());
    EXPECT_FALSE(osf_file.valid());

    EXPECT_EQ(osff.offset(), 100);
    EXPECT_TRUE(osff.good());

    OsfFile osf_new;
    EXPECT_FALSE(osf_new.good());
    osf_new = std::move(osff);
    EXPECT_TRUE(osf_new.good());
    EXPECT_FALSE(osff);

    EXPECT_EQ(osf_new.seek(1001).offset(), 1001);
    EXPECT_TRUE(osf_new.valid());

    if (osf_new.is_memory_mapped()) {
        const uint8_t* b = osf_new.buf();
        EXPECT_TRUE(b != nullptr);
    }

    // Read header size from the beginning of the file
    uint8_t size_buf[4];
    osf_new.seek(0).read(size_buf, 4);
    EXPECT_EQ(osf_new.offset(), 4);

    size_t header_size = get_prefixed_size(size_buf);

    // Header length is always 52 bytes (0x34)
    EXPECT_EQ(header_size, 52);

    // Close copied out
    osff.close();
    EXPECT_FALSE(osff.good());
    EXPECT_FALSE(osff);

    // Close osf dest istance
    osf_new.close();
    EXPECT_FALSE(osf_new.good());
    EXPECT_FALSE(osf_new);
}

TEST_F(OsfFileTest, OpenOsfFileWithStandardRead) {
    OsfFile osf_file(
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf"));
    EXPECT_TRUE(osf_file);
    EXPECT_EQ(osf_file.version(), OSF_VERSION::V_2_0);
    EXPECT_EQ(osf_file.size(), 1021684);
    EXPECT_EQ(osf_file.offset(), 0);

    EXPECT_EQ(osf_file.metadata_offset(), 1013976);
    std::cout << "file = " << osf_file.to_string() << std::endl;

    EXPECT_TRUE(osf_file.valid());
}

TEST_F(OsfFileTest, OpenOsfFileHandleNonExistent) {
    const std::string test_file_name =
        path_concat(test_data_dir(), "non-file-thing");

    OsfFile osf_file(test_file_name);
    EXPECT_FALSE(osf_file);

    EXPECT_EQ(0, osf_file.size());
    EXPECT_EQ(test_file_name, osf_file.filename());
    EXPECT_EQ(OSF_VERSION::V_INVALID, osf_file.version());
    EXPECT_EQ(0, osf_file.offset());

    // Access to a bad file is an error
    ASSERT_THROW(osf_file.buf(), std::logic_error);
    ASSERT_THROW(osf_file.buf(1), std::logic_error);
    ASSERT_THROW(osf_file.seek(10), std::logic_error);

    uint8_t buf[10];
    ASSERT_THROW(osf_file.read(buf, 10), std::logic_error);
}

TEST_F(OsfFileTest, OsfFileDontOpenDir) {
    const std::string test_file_dir = test_data_dir();

    OsfFile osf_file(test_file_dir);
    EXPECT_FALSE(osf_file);

    ASSERT_THROW(osf_file.buf(), std::logic_error);
}

TEST_F(OsfFileTest, OsfFileCheckOutOfRangeAccess) {
    const std::string test_file_name =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");

    OsfFile osf_file(test_file_name);
    EXPECT_TRUE(osf_file);

    // Read buffer for read()
    constexpr int kBufSize = 10;
    uint8_t buf[kBufSize];

    // In range file access
    if (osf_file.is_memory_mapped()) {
        ASSERT_NO_THROW(osf_file.buf(100));
    }
    ASSERT_NO_THROW(osf_file.seek(100).seek(0));
    ASSERT_NO_THROW(osf_file.read(buf, kBufSize));

    // Out of range file access
    if (osf_file.is_memory_mapped()) {
        ASSERT_THROW(osf_file.buf(100000000), std::out_of_range);
    }
    ASSERT_THROW(osf_file.seek(100000000), std::out_of_range);
    ASSERT_THROW(osf_file.read(buf, 100000000), std::out_of_range);
}

}  // namespace
}  // namespace osf
}  // namespace ouster