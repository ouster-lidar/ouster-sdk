/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/file.h"

#include <gtest/gtest.h>

#include <vector>

#include "osf_test.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/memory_mapped_osf_file.h"
#include "ouster/osf/stream_osf_file.h"

namespace {
bool has_substr(const char* search_string, const char* substr) {
    if (strstr(search_string, substr) != NULL) {
        return true;
    }
    return false;
}
}  // namespace

namespace ouster {
namespace sdk {
namespace osf {
namespace {

class OsfFileTest : public OsfTestWithData {
   public:
    std::vector<std::unique_ptr<OsfFile>> get_files(const std::string& path) {
        std::vector<std::unique_ptr<OsfFile>> v;
        v.emplace_back(std::make_unique<StreamOsfFile>(path));
        v.emplace_back(std::make_unique<MemoryMappedOsfFile>(path));
        return v;
    }
};

TEST_F(OsfFileTest, OpeningANonExistentFileThrows) {
    // It should throw a runtime error if provided with a path that doesn't
    // exist. Note - happens in the abstract base, so we don't need to test all
    // impls.
    auto path = "doesntexist";
    EXPECT_THROW(
        {
            try {
                StreamOsfFile f(path);
            } catch (const std::runtime_error& e) {
                // exact error message is different between Linux and Windows,
                // of course
                EXPECT_TRUE(has_substr(e.what(), "Read failure"));
                throw;
            }
        },
        std::runtime_error);
}

TEST_F(OsfFileTest, OpenOsfFileNominally) {
    auto path =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    for (auto& osf_file_ptr : get_files(path)) {
        auto& osf_file = *osf_file_ptr;
        EXPECT_TRUE(osf_file);

        ouster::sdk::core::Version expected_version{2, 1, 0};
        EXPECT_EQ(osf_file.version(), expected_version);

        EXPECT_EQ(osf_file.metadata_offset().offset(), 1015824);
        std::cout << "file = " << osf_file.to_string() << std::endl;

        // OSF v2 OsfFile should be OK
        EXPECT_TRUE(osf_file.good());
    }
}

template <typename OsfFileT>
void testMoveSemantics(const std::string& path) {
    OsfFileT osf_file(path);
    EXPECT_TRUE(osf_file);

    ouster::sdk::core::Version expected_version{2, 1, 0};
    EXPECT_EQ(osf_file.version(), expected_version);

    EXPECT_EQ(osf_file.metadata_offset().offset(), 1015824);
    std::cout << "file = " << osf_file.to_string() << std::endl;

    // OSF v2 OsfFile should be OK
    EXPECT_TRUE(osf_file.good());

    // Test move semantics
    OsfFileT osff(std::move(osf_file));
    EXPECT_FALSE(osf_file.good());
    EXPECT_EQ(osff.version(), expected_version);

    EXPECT_EQ(osff.metadata_offset().offset(), 1015824);
    EXPECT_TRUE(osff.good());

    // Close copied out
    osff.close();
    EXPECT_FALSE(osff.good());
    EXPECT_FALSE(osff);
}

TEST_F(OsfFileTest, OsfFileFullMoveSemantics) {
    auto path =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    testMoveSemantics<StreamOsfFile>(path);
    testMoveSemantics<MemoryMappedOsfFile>(path);
}

TEST_F(OsfFileTest, OpenOsfFileWithStandardRead) {
    auto path =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    for (auto& osf_file_ptr : get_files(path)) {
        auto& osf_file = *osf_file_ptr;
        EXPECT_TRUE(osf_file);

        ouster::sdk::core::Version expected_version{2, 1, 0};
        EXPECT_EQ(osf_file.version(), expected_version);

        EXPECT_EQ(osf_file.metadata_offset().offset(), 1015824);
        std::cout << "file = " << osf_file.to_string() << std::endl;

        EXPECT_TRUE(osf_file.good());
    }
}

TEST_F(OsfFileTest, OsfFileDontOpenDir) {
    const std::string test_file_dir = test_data_dir();
    // It should throw a runtime error if provided with a path that is a
    // directory. Note - happens in the abstract base, so we don't need to test
    // all impls.
    EXPECT_THROW({ StreamOsfFile f(test_file_dir); }, std::runtime_error);
}

TEST_F(OsfFileTest, OsfFileCheckOutOfRangeAccess) {
    const std::string test_file_name =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    for (auto& osf_file_ptr : get_files(test_file_name)) {
        auto& osf_file = *osf_file_ptr;
        EXPECT_TRUE(osf_file);

        OsfOffset offset = {100, 10};

        ASSERT_NO_THROW(osf_file.read(offset));

        // Out of range file access
        offset = {100000000, 10};
        ASSERT_THROW(
            {
                try {
                    osf_file.read(offset);
                } catch (const std::runtime_error& e) {
                    ASSERT_STREQ(e.what(), "EOF reached");
                    throw;
                }
            },
            std::runtime_error);
    }
}

}  // namespace
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
