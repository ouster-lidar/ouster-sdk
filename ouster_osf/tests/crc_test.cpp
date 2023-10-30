/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>
#include <zlib.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "ouster/osf/crc32.h"

namespace ouster {
namespace osf {
namespace {

class CrcTest : public ::testing::Test {};

TEST_F(CrcTest, SmokeSanityCheck) {
    const std::vector<uint8_t> data = {0, 1, 2, 3, 4, 5, 6, 7};
    const uint32_t crc = osf::crc32(data.data(), data.size());
    EXPECT_EQ(0x88aa689f, crc);

    const std::vector<uint8_t> data_rev(data.rbegin(), data.rend());
    const uint32_t crc_rev = osf::crc32(data_rev.data(), data_rev.size());
    EXPECT_EQ(0xa1509ef8, crc_rev);
}

}  // namespace
}  // namespace osf
}  // namespace ouster
