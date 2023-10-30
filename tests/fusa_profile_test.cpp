/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <iostream>

#include "ouster/os_pcap.h"
#include "ouster/pcap.h"

using namespace ouster;

namespace sensor {

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}

/// A packet with FuSa UDP profile and the default number of columns is 16640
/// bytes.
TEST(FusaProfileTest, packet_size) {
    auto data_dir = getenvs("DATA_DIR");
    std::string dataset = "OS-1-128_767798045_1024x10_20230712_120049";
    sensor_utils::PcapReader pcap(data_dir + "/" + dataset + ".pcap");
    std::string metadata_path = data_dir + "/" + dataset + ".json";
    auto info = ouster::sensor::metadata_from_json(metadata_path);

    // The profile in the corresponding metadata should be
    // identified correctly as PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL
    ASSERT_EQ(info.format.udp_profile_lidar,
              ouster::sensor::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL);

    size_t packet_size = pcap.next_packet();
    ASSERT_EQ(packet_size, 16640L);
}

/// The fields of a packet with FuSa UDP profile are parsed properly.
TEST(FusaProfileTest, fields) {
    // Ref:
    // https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html#configurable-data-packet-format-v2-x
    // Note - I painstakingly validated the constants using a hex editor:
    // https://imhex.werwolv.net/
    auto data_dir = getenvs("DATA_DIR");
    std::string dataset = "OS-1-128_767798045_1024x10_20230712_120049";
    sensor_utils::PcapReader pcap(data_dir + "/" + dataset + ".pcap");
    std::string metadata_path = data_dir + "/" + dataset + ".json";
    auto info = ouster::sensor::metadata_from_json(metadata_path);
    auto pf = ouster::sensor::get_format(info);

    // check preconditions for the test
    constexpr int pixels_per_column = 128u;
    ASSERT_EQ(info.mode, ouster::sensor::MODE_1024x10);
    ASSERT_EQ(info.format.udp_profile_lidar,
              ouster::sensor::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL);
    ASSERT_EQ(pf.pixels_per_column, pixels_per_column);

    // check field widths
    int expected_cols = 16;
    ASSERT_EQ(pf.field_type(ouster::sensor::RANGE), ouster::sensor::UINT32);
    ASSERT_EQ(pf.field_type(ouster::sensor::RANGE2), ouster::sensor::UINT32);
    ASSERT_EQ(pf.columns_per_packet, expected_cols);

    // check packet header values
    pcap.next_packet();
    ASSERT_EQ(pf.packet_type(pcap.current_data()), 1u);
    ASSERT_EQ(pf.frame_id(pcap.current_data()), 229u);
    ASSERT_EQ(pf.init_id(pcap.current_data()), info.init_id);
    ASSERT_EQ(std::to_string(pf.prod_sn(pcap.current_data())), info.sn);
    ASSERT_EQ(pf.countdown_thermal_shutdown(pcap.current_data()), 0u);
    ASSERT_EQ(pf.countdown_shot_limiting(pcap.current_data()), 0u);
    ASSERT_EQ(pf.thermal_shutdown(pcap.current_data()), 0u);
    ASSERT_EQ(pf.shot_limiting(pcap.current_data()), 0u);

    // check column header values
    uint64_t col_timestamps[] = {
        647839983424, 647840089656, 647840187456, 647840275096,
        647840379896, 647840469616, 647840567576, 647840675576,
        647840768056, 647840861408, 647840965048, 647841059888,
        647841161128, 647841259968, 647841351568, 647841450008};

    for (int col = 0; col < expected_cols; col++) {
        EXPECT_EQ(pf.col_timestamp(pf.nth_col(col, pcap.current_data())),
                  col_timestamps[col]);
        // note - measurement id isn't necessarily the same as the column number
        // just a coincidence here
        EXPECT_EQ(pf.col_measurement_id(pf.nth_col(col, pcap.current_data())),
                  col);
        EXPECT_EQ(pf.col_status(pf.nth_col(col, pcap.current_data())),
                  1u  // per eUDP docs, the column status of 0x01 indicates a
                      // valid column
        );
    }

    // check some pixel values
    int test_col_idx = 4;
    int pixels_to_test = 16;
    const uint8_t* col = pf.nth_col(test_col_idx, pcap.current_data());
    uint32_t expected_range[] = {0,    2328, 2328, 2320, 3096, 2312,
                                 2304, 2280, 3056, 2296, 2264, 2256,
                                 3000, 2272, 2240, 2240};
    uint32_t expected_range2[] = {0,   0, 0, 0, 0,   0, 0, 0,
                                  480, 0, 0, 0, 448, 0, 0, 0};

    uint32_t range_array[pixels_per_column];
    pf.col_field(col, ouster::sensor::RANGE, range_array);
    for (int range_idx = 0; range_idx < pixels_to_test; range_idx++) {
        EXPECT_EQ(range_array[range_idx], expected_range[range_idx]);
    }
    pf.col_field(col, ouster::sensor::RANGE2, range_array);
    for (int range_idx = 0; range_idx < pixels_to_test; range_idx++) {
        EXPECT_EQ(range_array[range_idx], expected_range2[range_idx]);
    }

    uint16_t expected_nearir[] = {320, 544, 528, 496, 400, 464, 496, 560,
                                  368, 512, 640, 640, 400, 672, 688, 608};
    uint16_t nearir_array[pixels_per_column];
    pf.col_field(col, ouster::sensor::NEAR_IR, nearir_array);
    for (int nearir_idx = 0; nearir_idx < pixels_to_test; nearir_idx++) {
        EXPECT_EQ(nearir_array[nearir_idx], expected_nearir[nearir_idx]);
    }

    uint8_t expected_refl[] = {3, 26, 31, 32, 13, 38, 44, 45,
                               1, 58, 60, 61, 1,  67, 68, 72};
    uint8_t expected_refl2[] = {0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 3, 0, 0, 0};
    uint8_t refl_array[pixels_per_column];
    pf.col_field(col, ouster::sensor::REFLECTIVITY, refl_array);
    for (int refl_idx = 0; refl_idx < pixels_to_test; refl_idx++) {
        EXPECT_EQ(refl_array[refl_idx], expected_refl[refl_idx]);
    }
    pf.col_field(col, ouster::sensor::REFLECTIVITY2, refl_array);
    for (int refl_idx = 0; refl_idx < pixels_to_test; refl_idx++) {
        EXPECT_EQ(refl_array[refl_idx], expected_refl2[refl_idx]);
    }

    uint32_t raw_words[pixels_per_column];
    uint32_t raw_words2[pixels_per_column];
    pf.col_field(col, ouster::sensor::RAW32_WORD1, raw_words);
    pf.col_field(col, ouster::sensor::RAW32_WORD2, raw_words2);

    // check raw words against known range values
    // to confirm raw word offsets are correct
    for (int idx = 0; idx < pixels_to_test; idx++) {
        ASSERT_EQ(raw_words[idx] & 0x7fff, expected_range[idx] >> 3);
        ASSERT_EQ(raw_words2[idx] & 0x7fff, expected_range2[idx] >> 3);

        // The test data was produced with an early snapshot
        // for FuSa that only include MVP fields.
        // Check that non-MVP bits (reserved, blocked pixel, and pixel error)
        // are zero.
        ASSERT_EQ(raw_words2[idx] & 0xff000000, 0);
    }
}

}  // namespace sensor
