/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>
#include <json/json.h>

#include <fstream>

#include "ouster/types.h"
#include "ouster/util.h"

class MetaFiles : public testing::TestWithParam<const char*> {};

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}

TEST(Util, combinedTestEmpty) {
    Json::Value root_orig{};
    Json::Value root_new{};
    Json::Value combined_true{};

    std::vector<std::string> changed_true;
    std::vector<std::string> changed_test;

    auto result = ouster::combined(root_orig, root_new, changed_test);

    EXPECT_EQ(result, combined_true);
    EXPECT_EQ(changed_test, changed_true);
}

TEST(Util, combinedTestStrings) {
    Json::Value root_orig{};
    Json::Value root_new{};
    Json::Value combined_true{};

    root_orig["aaa"] = 1;
    root_orig["bbb"] = "222";
    root_orig["ccc"]["fff"] = "333";
    root_orig["ccc"]["ooo"] = "444";
    root_orig["ttt"]["ggg"]["jjj"] = "101010";
    root_orig["ttt"]["ggg"]["lll"] = 2;

    root_new["aaa"] = 1;
    root_new["bbb"] = "555";                   // change bbb
    root_new["ccc"]["fff"] = "$$$";            // change ccc[fff]
    root_new["ccc"]["hhh"] = "777";            // add ccc[hhh]
    root_new["ddd"] = "666";                   // add ddd
    root_new["eee"]["iii"] = "999";            // add eee and eee[iii]
    root_new["ttt"]["ggg"]["lll"] = 3;         // change ttt[ggg][lll]
    root_new["ttt"]["ggg"]["mmm"] = "333333";  // add ttt[ggg][mmm]

    combined_true = root_orig;
    combined_true["bbb"] = "555";
    combined_true["ccc"]["fff"] = "$$$";
    combined_true["ccc"]["hhh"] = "777";
    combined_true["ddd"] = "666";
    combined_true["eee"]["iii"] = "999";
    combined_true["ttt"]["ggg"]["lll"] = 3;
    combined_true["ttt"]["ggg"]["mmm"] = "333333";

    std::vector<std::string> changed_true{
        "bbb", "ccc.fff",     "ccc.hhh",    "ddd",
        "eee", "ttt.ggg.lll", "ttt.ggg.mmm"};
    std::vector<std::string> changed_test;

    auto result = ouster::combined(root_orig, root_new, changed_test);

    EXPECT_EQ(result, combined_true);
    EXPECT_EQ(changed_test, changed_true);
}

// clang-format off
INSTANTIATE_TEST_CASE_P(
    TestMetas, MetaFiles,
    testing::Values(
        "1_12_os1-991913000010-64",
        "1_12_os1-991937000062-16A0_legacy",
        "1_12_os1-991937000062-64_legacy",
        "1_13_os1-991913000010-64",
        "1_13_os1-991937000062-16A0_legacy",
        "1_13_os1-991937000062-32A02_legacy",
        "1_14_6cccd_os-882002000138-128_legacy",
        "1_14_beta_os1-991937000062-16A0_legacy",
        "1_14_6cccd_os-882002000138-32U0_legacy",
        "1_14_beta_os1-991937000062-64_legacy",
        "2_0_0_os1-991913000010-64",
        "2_0_0_os1-992008000494-128_col_win_legacy",
        "2_0_rc2_os-992011000121-32U0_legacy",
        "2_1_2_os1-991913000010-64",
        "2_1_2_os1-991913000010-64_legacy",
        "2_2_os-992119000444-128",
        "2_2_os-992119000444-128_legacy",
        "2_3_1_os-992146000760-128_legacy",
        "2_3_1_os-992146000760-128",
        "2_4_0_os-992146000760-128",
        "2_5_0_os-992146000760-128",
        "2_4_0_os-992146000760-128_legacy",
        "2_5_0_os-992146000760-128_legacy",
        "3_0_1_os-122246000293-128",
        "3_0_1_os-122246000293-128_legacy",
        "ouster-studio-reduced-config-v1"));
// clang-format on

TEST_P(MetaFiles, origStringTestMetadata) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");
    auto json_file = data_dir + param + ".json";

    ouster::sensor::sensor_info si =
        ouster::sensor::metadata_from_json(json_file);

    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(json_file);
    buf << ifs.rdbuf();
    ifs.close();

    si.mode = ouster::sensor::lidar_mode::MODE_4096x5;
    si.build_date = "FAKEBUILDDATE";

    EXPECT_EQ(si.original_string(), buf.str());
}

TEST_P(MetaFiles, combinedTestMetadata) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");

    const ouster::sensor::sensor_info si_orig =
        ouster::sensor::metadata_from_json(data_dir + param + ".json");

    // Make si new -- change a few values
    auto si_new = si_orig;
    si_new.init_id = 5;
    si_new.mode = ouster::sensor::lidar_mode::MODE_4096x5;
    si_new.format.fps = 47289;  // fps is an addition instead of a replacement
    si_new.beam_altitude_angles[5] = 0.01;

    // Make sure they aren't somehow the same
    EXPECT_NE(si_new.mode, si_orig.mode);
    EXPECT_NE(si_new.init_id, si_orig.init_id);
    EXPECT_NE(si_new.format.fps, si_orig.format.fps);
    EXPECT_NE(si_new.beam_altitude_angles, si_orig.beam_altitude_angles);

    auto si_new_updated_string = si_new.updated_metadata_string();
    auto si_roundtrip = ouster::sensor::parse_metadata(si_new_updated_string);

    EXPECT_EQ(si_new.mode, si_roundtrip.mode);
    EXPECT_EQ(si_new.init_id, si_roundtrip.init_id);
    EXPECT_EQ(si_new.format.fps, si_roundtrip.format.fps);
    EXPECT_EQ(si_new.beam_altitude_angles, si_roundtrip.beam_altitude_angles);

    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{si_new_updated_string};

    Json::parseFromStream(builder, ss, &root, &errors);

    auto changed_json = root["ouster-sdk"]["changed_fields"];

    auto find_changed = [&changed_json](auto str1, auto str2) {
        return (std::find(changed_json.begin(), changed_json.end(), str1) !=
                    changed_json.end() ||
                std::find(changed_json.begin(), changed_json.end(), str2) !=
                    changed_json.end());
    };

    EXPECT_TRUE(
        find_changed("sensor_info.initialization_id", "initialization_id"));

    if (param.substr(0, 4) == "1_12" || param.substr(0, 4) == "1_13" ||
        param == "ouster-studio-reduced-config-v1") {
        // 1_12 and 1_13 did not have data_format at all
        EXPECT_TRUE(find_changed("data_format", "lidar_data_format"));
    } else {
        // 1.14+ have data_format so only fps changes
        EXPECT_TRUE(find_changed("data_format.fps", "lidar_data_format.fps"));
    }
    EXPECT_TRUE(find_changed("lidar_mode", "config_params.lidar_mode"));
    EXPECT_TRUE(find_changed("beam_intrinsics.beam_altitude_angles",
                             "beam_altitude_angles"));

    if (param == "ouster-studio-reduced-config-v1") {
        EXPECT_TRUE(changed_json.size() == 12);
    } else {
        if (std::find(changed_json.begin(), changed_json.end(), "ouster-sdk") !=
            changed_json.end()) {
            // check indicates non-legacy format
            if (param.substr(0, 4) == "1_12" || param.substr(0, 4) == "1_13") {
                // udp_dest and operating_mode are new
                EXPECT_TRUE(changed_json.size() == 7);
            } else {
                // expect ouster-sdk along with the main 4
                EXPECT_TRUE(changed_json.size() == 5);
            }

        } else {
            // mode, init_id, data_format.fps (or data_format itself),
            // beam_altitude_angles
            EXPECT_TRUE(changed_json.size() == 4);
        }
    }
}
