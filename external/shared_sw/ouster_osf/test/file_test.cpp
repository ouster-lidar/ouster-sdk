#include "ouster/osf/file.h"

#include <gtest/gtest.h>

// #include "../src/util_impl.h"
#include "osf_test.h"

namespace ouster {
namespace OSF {
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
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() + "/lib-osf/fake_data_v12.osf");
    EXPECT_TRUE(osf_file);
    EXPECT_EQ(osf_file.version(), OSF_VERSION::V_1_2);
    EXPECT_EQ(osf_file.size(), 183512);
    EXPECT_EQ(osf_file.offset(), 0);

    EXPECT_EQ(osf_file.seek(100).offset(), 100);

    // Out of range seek should throw
    EXPECT_THROW(osf_file.seek(200000), std::out_of_range);

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

    const uint8_t* b = osf_new.buf();
    EXPECT_TRUE(b != nullptr);

    // Read header size from the beginning of the file
    uint8_t size_buf[4];
    osf_new.seek(0).read(size_buf, 4);
    EXPECT_EQ(osf_new.offset(), 4);

    size_t header_size = readPrefixedSizeFromOffset(size_buf);

    // Header length is always 52 bytes (0x34)
    EXPECT_EQ(header_size, 52);

    // TODO: Test destruction and move
}

TEST_F(OsfFileTest, OpenOsfFileHandleNonExistent) {
    TEST_DATA_SKIP();

    const std::string test_file_name = test_data_dir() + "/non-file-thing";

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
    TEST_DATA_SKIP();

    const std::string test_file_dir = test_data_dir();

    OsfFile osf_file(test_file_dir);
    EXPECT_FALSE(osf_file);

    ASSERT_THROW(osf_file.buf(), std::logic_error);
}

TEST_F(OsfFileTest, OsfFileCheckOutOfRangeAccess) {
    TEST_DATA_SKIP();

    const std::string test_file_name =
        test_data_dir() + "/lib-osf/fake_data_v12.osf";

    OsfFile osf_file(test_file_name);
    EXPECT_TRUE(osf_file);

    // Read buffer for read()
    constexpr int kBufSize = 10;
    uint8_t buf[kBufSize];

    // In range file access
    ASSERT_NO_THROW(osf_file.buf(100));
    ASSERT_NO_THROW(osf_file.seek(100).seek(0));
    ASSERT_NO_THROW(osf_file.read(buf, kBufSize));

    // Out of range file access
    ASSERT_THROW(osf_file.buf(1000000), std::out_of_range);
    ASSERT_THROW(osf_file.seek(1000000), std::out_of_range);
    ASSERT_THROW(osf_file.read(buf, 1000000), std::out_of_range);
}

}  // namespace
}  // namespace OSF
}  // namespace ouster