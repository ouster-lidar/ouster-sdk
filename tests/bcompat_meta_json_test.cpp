#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstdlib>
#include <map>

#include "bcompat_sensor_info_data.h"  // test data
#include "ouster/types.h"

using namespace ouster::testing;

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}

class MetaJsonTest : public testing::TestWithParam<const char*> {};

// Add test data here: these are the base names for input .json files and
// pre-parsed structs in bcompat_sensor_info_data.h
//
// Expected output files will need to be re-generated whenever sensor_info
// or the parsing output changes
// clang-format off
INSTANTIATE_TEST_CASE_P(BCompat, MetaJsonTest,
testing::Values(
  "1_12_os1-991937000062-16A0",
  "1_12_os1-991937000062-64",
  "1_13_os1-991937000062-16A0",
  "1_13_os1-991937000062-32A02",
  "1_13_os1-991937000062-64",
  "1_14_6cccd_os-882002000138-128",
  "1_14_6cccd_os-882002000138-32U0",
  "1_14_6cccd_os-882002000138-64U02",
  "1_14_beta_os1-991937000062-16A0",
  "1_14_beta_os1-991937000062-32A02",
  "1_14_beta_os1-991937000062-64",
  "1_14_beta_os-882004000055-128",
  "2_0_rc2_os-992011000121-32U0",
  "2_0_0_os1-992008000494-128_col_win",
  "ouster-studio-reduced-config-v1"
));
// clang-format on

std::string data_dir = ".";

// Backwards-compatibility test for meta json parsing: compare previously parsed
// sensor_info structs to the output of metadata_from_json
TEST_P(MetaJsonTest, MetadataFromJson) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");

    // parse json file
    const sensor_info si = metadata_from_json(data_dir + "/" + param + ".json");

    // compare with previously parsed struct from bcompat_sensor_info_data.h
    const sensor_info& si_expected = expected_sensor_infos.at(param);

    EXPECT_EQ(si.name, si_expected.name);
    EXPECT_EQ(si.sn, si_expected.sn);
    EXPECT_EQ(si.fw_rev, si_expected.fw_rev);
    EXPECT_EQ(si.mode, si_expected.mode);
    EXPECT_EQ(si.prod_line, si_expected.prod_line);

    EXPECT_EQ(si.format.pixels_per_column,
              si_expected.format.pixels_per_column);
    EXPECT_EQ(si.format.columns_per_packet,
              si_expected.format.columns_per_packet);
    EXPECT_EQ(si.format.columns_per_frame,
              si_expected.format.columns_per_frame);
    EXPECT_EQ(si.format.pixel_shift_by_row,
              si_expected.format.pixel_shift_by_row);
    EXPECT_EQ(si.format.column_window, si_expected.format.column_window);

    EXPECT_EQ(si.beam_azimuth_angles, si_expected.beam_azimuth_angles);
    EXPECT_EQ(si.beam_altitude_angles, si_expected.beam_altitude_angles);
    EXPECT_EQ(si.lidar_origin_to_beam_origin_mm,
              si_expected.lidar_origin_to_beam_origin_mm);

    EXPECT_EQ(si.imu_to_sensor_transform, si_expected.imu_to_sensor_transform);
    EXPECT_EQ(si.lidar_to_sensor_transform,
              si_expected.lidar_to_sensor_transform);
    EXPECT_EQ(si.extrinsic, si_expected.extrinsic);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
