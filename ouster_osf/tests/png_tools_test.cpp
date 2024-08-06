/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "png_tools.h"

#include <gtest/gtest.h>
#include <png.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/ostream_sink.h>
#include <sys/stat.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>

#include "common.h"
#include "osf_test.h"
#include "ouster/impl/logging.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

// Internals to test
void png_osf_flush_data(png_structp);
void png_osf_error(png_structp png_ptr, png_const_charp msg);

namespace {

class OsfPngToolsTest : public OsfTestWithDataAndFiles {};

using ouster::sensor::lidar_mode;
using ouster::sensor::sensor_info;

size_t field_size(LidarScan& ls, const std::string& f) {
    switch (ls.field_type(f).element_type) {
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
    for (const auto& f : ls.field_types()) {
        EXPECT_EQ(field_size(ls, f.name), n);
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

TEST_F(OsfPngToolsTest, InternalsTest) {
    // This is unused but is still required, test calling it
    // Not expecting any returns
    png_structp foo =
        png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_osf_flush_data(foo);

    bool error_caught = false;
    std::stringstream output_stream;
    auto ostream_sink = std::make_shared<
        spdlog::sinks::ostream_sink<spdlog::details::null_mutex>>(
        output_stream);
    ouster::sensor::impl::Logger::instance().configure_generic_sink(
        ostream_sink, "info");

    if (setjmp(png_jmpbuf(foo))) {
        error_caught = true;
    } else {
        png_osf_error(
            foo,
            "Also Checkout Porcupine Tree - Arriving Somewhere But Not Here");
    }

    std::string output_error = output_stream.str();
    auto error_loc = output_error.find("[error]");
    EXPECT_NE(error_loc, std::string::npos);
    output_error = output_error.substr(error_loc);
    EXPECT_TRUE(error_caught);
#ifdef _WIN32
    EXPECT_EQ(output_error,
              "[error] ERROR libpng osf: Also Checkout Porcupine Tree"
              " - Arriving Somewhere But Not Here\r\n");
#else
    EXPECT_EQ(output_error,
              "[error] ERROR libpng osf: Also Checkout Porcupine Tree"
              " - Arriving Somewhere But Not Here\n");
#endif
}

#ifndef OUSTER_OSF_NO_THREADING
TEST_F(OsfPngToolsTest, scanDecodeFields) {
    // it should propagate the exception
    // if destagger throws std::invalid_argument
    int w = 32;
    int h = 32;
    auto scan = ouster::LidarScan(w, h);
    auto field_types = scan.field_types();
    std::vector<std::pair<std::string, ouster::sensor::ChanFieldType>> fields;
    for (const auto& field : field_types) {
        fields.push_back({field.name, field.element_type});
    }
    std::vector<int> shift_by_row;
    EXPECT_THROW(
        {
            try {
                scanEncodeFields(scan, shift_by_row, fields);
            } catch (const std::invalid_argument& e) {
                ASSERT_STREQ(e.what(),
                             "image height does not match shifts size");
                throw;
            }
        },
        std::invalid_argument);
}
#endif

TEST(OsfFieldEncodeTest, field_encode_decode_test) {
    auto test_field_encoding = [](const ouster::Field& f) {
        ScanChannelData compressed;
        EXPECT_NO_THROW({ compressed = encodeField(f); });
        Field decoded(f.desc());
        EXPECT_NO_THROW({ decodeField(decoded, compressed); });
        EXPECT_EQ(f, decoded);
    };

    std::random_device rd;
    std::mt19937 gen{rd()};

    std::normal_distribution<float> nd_f{100.f, 10.f};
    test_field_encoding(randomized_field<float>(gen, nd_f, {128, 1024, 3}));
    test_field_encoding(randomized_field<float>(gen, nd_f, {128, 1024}));
    test_field_encoding(randomized_field<float>(gen, nd_f, {4096}));

    std::normal_distribution<double> nd_d{0.0, 1000.0};
    test_field_encoding(randomized_field<double>(gen, nd_d, {128, 1024, 3}));
    test_field_encoding(randomized_field<double>(gen, nd_d, {128, 1024}));
    test_field_encoding(randomized_field<double>(gen, nd_d, {4096}));

    std::uniform_int_distribution<uint32_t> ud_u32{0, 4096};
    test_field_encoding(
        randomized_field<uint32_t>(gen, ud_u32, {128, 1024, 3}));
    test_field_encoding(randomized_field<uint32_t>(gen, ud_u32, {128, 1024}));
    test_field_encoding(randomized_field<uint32_t>(gen, ud_u32, {4096}));

    std::uniform_int_distribution<int64_t> ud_i64{0, 1024 * 1024};
    test_field_encoding(randomized_field<int64_t>(gen, ud_i64, {128, 1024, 3}));
    test_field_encoding(randomized_field<int64_t>(gen, ud_i64, {128, 1024}));
    test_field_encoding(randomized_field<int64_t>(gen, ud_i64, {4096}));
}

}  // namespace
}  // namespace osf
}  // namespace ouster
