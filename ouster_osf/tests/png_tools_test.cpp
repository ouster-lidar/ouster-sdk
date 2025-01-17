/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "png_tools.h"

#include <gtest/gtest.h>
#include <png.h>
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
#include "ouster/osf/png_lidarscan_encoder.h"
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

#ifndef OUSTER_OSF_NO_THREADING

TEST_F(OsfPngToolsTest, scanDecodeFields) {
    // it should propagate the exception
    // if destagger throws std::invalid_argument

    // create a writer with a sensor info from DATA_DIR
    const sensor_info si = sensor::metadata_from_json(
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    // assert precondition
    ASSERT_EQ(si.format.pixel_shift_by_row.size(), 128);
    std::string output_osf_filename = tmp_file("scan_decode_fields_test.osf");

    // create a lidar scan that's the wrong size
    int w = 32;
    int h = 32;
    auto scan = ouster::LidarScan(w, h);
    auto field_types = scan.field_types();
    std::vector<int> shift_by_row;
    EXPECT_THROW(
        {
            try {
                Writer writer(output_osf_filename, si);
                writer.save(0, scan);
                writer.close();
            } catch (std::invalid_argument& e) {
                ASSERT_STREQ(e.what(),
                             "lidar scan size (32, 32) does not match the "
                             "sensor info resolution (1024, 128)");
                throw;
            }
        },
        std::invalid_argument);
}

#endif

TEST(OsfFieldEncodeTest, field_encode_decode_test) {
    auto test_field_encoding = [](const ouster::Field& f) {
        ScanChannelData compressed;
        PngLidarScanEncoder encoder(4);
        EXPECT_NO_THROW({ compressed = encoder.encodeField(f); });
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
