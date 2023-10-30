/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "png_tools.h"

#include <gtest/gtest.h>
#include <sys/stat.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>

#include "common.h"
#include "osf_test.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {
namespace {

class OsfPngToolsTest : public OsfTestWithDataAndFiles {};

using ouster::sensor::lidar_mode;
using ouster::sensor::sensor_info;

size_t field_size(LidarScan& ls, sensor::ChanField f) {
    switch (ls.field_type(f)) {
        case sensor::ChanFieldType::UINT8:
            return ls.field<uint8_t>(f).size();
            break;
        case sensor::ChanFieldType::UINT16:
            return ls.field<uint16_t>(f).size();
            break;
        case sensor::ChanFieldType::UINT32:
            return ls.field<uint32_t>(f).size();
            break;
        case sensor::ChanFieldType::UINT64:
            return ls.field<uint64_t>(f).size();
            break;
        default:
            return 0;
            break;
    }
}

// Check that we can make lidar scan and have fields with expected value inside
TEST_F(OsfPngToolsTest, MakesLidarScan) {
    const sensor_info si = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    LidarScan ls = get_random_lidar_scan(si);

    const auto n = si.format.columns_per_frame * si.format.pixels_per_column;

    EXPECT_EQ(ls.w, si.format.columns_per_frame);
    EXPECT_EQ(ls.h, si.format.pixels_per_column);
    for (const auto& f : ls) {
        EXPECT_EQ(field_size(ls, f.first), n);
    }
    EXPECT_EQ(ls.status().size(), si.format.columns_per_frame);
}

#define ENCODE_IMAGE_TEST(TEST_NAME, ENCODE_FUNC, DECODE_FUNC)                 \
    template <typename Ti>                                                     \
    struct TEST_NAME {                                                         \
        template <typename To>                                                 \
        bool to(const LidarScan& ls, const std::vector<int>& px_offset,        \
                Ti mask_bits = 0) {                                            \
            ScanChannelData encoded_channel;                                   \
            img_t<Ti> key_orig(ls.h, ls.w);                                    \
            key_orig = key_orig.unaryExpr([=](Ti) {                            \
                double sr = static_cast<double>(std::rand()) / RAND_MAX;       \
                return static_cast<Ti>(                                        \
                    sr * static_cast<double>(std::numeric_limits<Ti>::max())); \
            });                                                                \
            if (mask_bits && sizeof(Ti) * 8 > mask_bits) {                     \
                key_orig = key_orig.unaryExpr([=](Ti a) {                      \
                    return static_cast<Ti>(a & ((1LL << mask_bits) - 1));      \
                });                                                            \
            }                                                                  \
            bool res_enc =                                                     \
                ENCODE_FUNC<Ti>(encoded_channel, key_orig, px_offset);         \
            EXPECT_FALSE(res_enc);                                             \
            std::cout << #ENCODE_FUNC                                          \
                      << ": encoded bytes = " << encoded_channel.size()        \
                      << " ================= " << std::endl;                   \
            EXPECT_TRUE(!encoded_channel.empty());                             \
            img_t<To> decoded_img{ls.h, ls.w};                                 \
            bool res_dec =                                                     \
                DECODE_FUNC<To>(decoded_img, encoded_channel, px_offset);      \
            EXPECT_FALSE(res_dec);                                             \
            bool round_trip = (key_orig.template cast<uint64_t>() ==           \
                               decoded_img.template cast<uint64_t>())          \
                                  .all();                                      \
            auto round_trip_cnt = (key_orig.template cast<uint64_t>() ==       \
                                   decoded_img.template cast<uint64_t>())      \
                                      .count();                                \
            std::cout << "cnt = " << round_trip_cnt << std::endl;              \
            return round_trip;                                                 \
        }                                                                      \
    };

ENCODE_IMAGE_TEST(test8bitImageCoders, encode8bitImage, decode8bitImage)
ENCODE_IMAGE_TEST(test16bitImageCoders, encode16bitImage, decode16bitImage)
ENCODE_IMAGE_TEST(test24bitImageCoders, encode24bitImage, decode24bitImage)
ENCODE_IMAGE_TEST(test32bitImageCoders, encode32bitImage, decode32bitImage)
ENCODE_IMAGE_TEST(test64bitImageCoders, encode64bitImage, decode64bitImage)

// Check encodeXXbitImage functions on RANGE fields of LidarScan
// converted to various img_t<T> of different unsigned int sizes.
TEST_F(OsfPngToolsTest, ImageCoders) {
    const sensor_info si = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    LidarScan ls = get_random_lidar_scan(si);
    auto px_offset = si.format.pixel_shift_by_row;

    // ======== 8bit ==========

    EXPECT_TRUE(test8bitImageCoders<uint8_t>().to<uint8_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint8_t>().to<uint16_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint8_t>().to<uint32_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint8_t>().to<uint64_t>(ls, px_offset, 8));

    EXPECT_TRUE(test8bitImageCoders<uint16_t>().to<uint8_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint16_t>().to<uint16_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint16_t>().to<uint32_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint16_t>().to<uint64_t>(ls, px_offset, 8));

    EXPECT_TRUE(test8bitImageCoders<uint32_t>().to<uint8_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint32_t>().to<uint16_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint32_t>().to<uint32_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint32_t>().to<uint64_t>(ls, px_offset, 8));

    EXPECT_TRUE(test8bitImageCoders<uint64_t>().to<uint8_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint64_t>().to<uint16_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint64_t>().to<uint32_t>(ls, px_offset, 8));
    EXPECT_TRUE(test8bitImageCoders<uint64_t>().to<uint64_t>(ls, px_offset, 8));

    // ======== 16bit ======
    // clang-format off
    EXPECT_TRUE(test16bitImageCoders<uint8_t>().to<uint8_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint8_t>().to<uint16_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint8_t>().to<uint32_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint8_t>().to<uint64_t>(ls, px_offset, 16));

    EXPECT_FALSE(test16bitImageCoders<uint16_t>().to<uint8_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint16_t>().to<uint16_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint16_t>().to<uint32_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint16_t>().to<uint64_t>(ls, px_offset, 16));

    EXPECT_FALSE(test16bitImageCoders<uint32_t>().to<uint8_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint32_t>().to<uint16_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint32_t>().to<uint32_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint32_t>().to<uint64_t>(ls, px_offset, 16));

    EXPECT_FALSE(test16bitImageCoders<uint64_t>().to<uint8_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint64_t>().to<uint16_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint64_t>().to<uint32_t>(ls, px_offset, 16));
    EXPECT_TRUE(test16bitImageCoders<uint64_t>().to<uint64_t>(ls, px_offset, 16));
    // clang-format on

    // ======== 24bit ======

    EXPECT_TRUE(test24bitImageCoders<uint8_t>().to<uint8_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint8_t>().to<uint16_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint8_t>().to<uint32_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint8_t>().to<uint64_t>(ls, px_offset, 24));

    EXPECT_FALSE(
        test24bitImageCoders<uint16_t>().to<uint8_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint16_t>().to<uint16_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint16_t>().to<uint32_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint16_t>().to<uint64_t>(ls, px_offset, 24));

    EXPECT_FALSE(
        test24bitImageCoders<uint32_t>().to<uint8_t>(ls, px_offset, 24));
    EXPECT_FALSE(
        test24bitImageCoders<uint32_t>().to<uint16_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint32_t>().to<uint32_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint32_t>().to<uint64_t>(ls, px_offset, 24));

    EXPECT_FALSE(
        test24bitImageCoders<uint64_t>().to<uint8_t>(ls, px_offset, 24));
    EXPECT_FALSE(
        test24bitImageCoders<uint64_t>().to<uint16_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint64_t>().to<uint32_t>(ls, px_offset, 24));
    EXPECT_TRUE(
        test24bitImageCoders<uint64_t>().to<uint64_t>(ls, px_offset, 24));

    // ======== 32bit ======

    EXPECT_TRUE(test32bitImageCoders<uint8_t>().to<uint8_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint8_t>().to<uint16_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint8_t>().to<uint32_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint8_t>().to<uint64_t>(ls, px_offset, 32));

    EXPECT_FALSE(
        test32bitImageCoders<uint16_t>().to<uint8_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint16_t>().to<uint16_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint16_t>().to<uint32_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint16_t>().to<uint64_t>(ls, px_offset, 32));

    EXPECT_FALSE(
        test32bitImageCoders<uint32_t>().to<uint8_t>(ls, px_offset, 32));
    EXPECT_FALSE(
        test32bitImageCoders<uint32_t>().to<uint16_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint32_t>().to<uint32_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint32_t>().to<uint64_t>(ls, px_offset, 32));

    EXPECT_FALSE(
        test32bitImageCoders<uint64_t>().to<uint8_t>(ls, px_offset, 32));
    EXPECT_FALSE(
        test32bitImageCoders<uint64_t>().to<uint16_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint64_t>().to<uint32_t>(ls, px_offset, 32));
    EXPECT_TRUE(
        test32bitImageCoders<uint64_t>().to<uint64_t>(ls, px_offset, 32));

    // ======== 64bit ======

    EXPECT_TRUE(test64bitImageCoders<uint8_t>().to<uint8_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint8_t>().to<uint16_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint8_t>().to<uint32_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint8_t>().to<uint64_t>(ls, px_offset, 64));

    EXPECT_FALSE(
        test64bitImageCoders<uint16_t>().to<uint8_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint16_t>().to<uint16_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint16_t>().to<uint32_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint16_t>().to<uint64_t>(ls, px_offset, 64));

    EXPECT_FALSE(
        test64bitImageCoders<uint32_t>().to<uint8_t>(ls, px_offset, 64));
    EXPECT_FALSE(
        test64bitImageCoders<uint32_t>().to<uint16_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint32_t>().to<uint32_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint32_t>().to<uint64_t>(ls, px_offset, 64));

    EXPECT_FALSE(
        test64bitImageCoders<uint64_t>().to<uint8_t>(ls, px_offset, 64));
    EXPECT_FALSE(
        test64bitImageCoders<uint64_t>().to<uint16_t>(ls, px_offset, 64));
    EXPECT_FALSE(
        test64bitImageCoders<uint64_t>().to<uint32_t>(ls, px_offset, 64));
    EXPECT_TRUE(
        test64bitImageCoders<uint64_t>().to<uint64_t>(ls, px_offset, 64));
}

}  // namespace
}  // namespace osf
}  // namespace ouster
