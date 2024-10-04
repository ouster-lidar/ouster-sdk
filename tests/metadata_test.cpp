/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>
#include <json/json.h>

#include <fstream>
#include <string>
#include <utility>
#include <vector>

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
        "1_12_os1-991937000062-64_legacy",
        "1_13_os1-991913000010-64",
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
        "ouster-studio-reduced-config-v1"
));
// clang-format on

TEST_P(MetaFiles, combinedTestMetadata) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");

    const ouster::sensor::sensor_info si_orig =
        ouster::sensor::metadata_from_json(data_dir + param + ".json");

    // Make si new -- change a few values
    auto si_new = si_orig;
    si_new.init_id = 5;
    si_new.config.lidar_mode = ouster::sensor::lidar_mode::MODE_4096x5;
    si_new.format.fps = 47289;  // fps is an addition instead of a replacement
    si_new.beam_altitude_angles[5] = 0.01;

    // Make sure they aren't somehow the same
    EXPECT_NE(si_new.config.lidar_mode, si_orig.config.lidar_mode);
    EXPECT_NE(si_new.init_id, si_orig.init_id);
    EXPECT_NE(si_new.format.fps, si_orig.format.fps);
    EXPECT_NE(si_new.beam_altitude_angles, si_orig.beam_altitude_angles);

    auto si_new_updated_string = si_new.to_json_string();
    auto si_roundtrip = ouster::sensor::sensor_info(si_new_updated_string);

    EXPECT_EQ(si_new.config.lidar_mode, si_roundtrip.config.lidar_mode);
    EXPECT_EQ(si_new.init_id, si_roundtrip.init_id);
    EXPECT_EQ(si_new.format.fps, si_roundtrip.format.fps);
    EXPECT_EQ(si_new.beam_altitude_angles, si_roundtrip.beam_altitude_angles);
}

class product_info_test : public ouster::sensor::product_info {
   public:
    product_info_test(std::string product_info_string, std::string form_factor,
                      bool short_range, std::string beam_config, int beam_count)
        : ouster::sensor::product_info(product_info_string, form_factor,
                                       short_range, beam_config, beam_count){};
};

TEST(Util, TestProdlineDecoder) {
    EXPECT_EQ(ouster::sensor::product_info(), ouster::sensor::product_info());
    EXPECT_EQ(
        ouster::sensor::product_info::create_product_info("OS-0-128-BH02-SR"),
        ouster::sensor::product_info::create_product_info("OS-0-128-BH02-SR"));
    EXPECT_NE(
        ouster::sensor::product_info::create_product_info("OS-0-128-BH02-SR"),
        ouster::sensor::product_info());
    EXPECT_NE(
        ouster::sensor::product_info::create_product_info("OS-0-128"),
        ouster::sensor::product_info::create_product_info("OS-0-128-BH02-SR"));
    EXPECT_NE(
        ouster::sensor::product_info::create_product_info("OS-0-128-BH02"),
        ouster::sensor::product_info::create_product_info("OS-0-128-BH02-SR"));

    bool error_recieved = false;
    try {
        ouster::sensor::product_info::create_product_info("DEADBEEF");
    } catch (const std::runtime_error& e) {
        EXPECT_EQ(std::string(e.what()),
                  "Product Info \"DEADBEEF\" is not a recognized product info");
        error_recieved = true;
    }
    EXPECT_TRUE(error_recieved);

    auto bad_count = ouster::sensor::product_info::create_product_info(
        "OS-0-STUFF HERE-BH02-SR");
    EXPECT_EQ(bad_count.beam_count, 0);

    std::vector<std::pair<std::string, product_info_test>> test_product_infos =
        {std::make_pair(
             "FOOBAR-1234",
             product_info_test("FOOBAR-1234", "FOOBAR1234", false, "U", 0)),
         std::make_pair("OS-0-128",
                        product_info_test("OS-0-128", "OS0", false, "U", 128)),
         std::make_pair(
             "OS-0-128-BH02-SR",
             product_info_test("OS-0-128-BH02-SR", "OS0", true, "BH02", 128)),
         std::make_pair("OS-0-32-AH02", product_info_test("OS-0-32-AH02", "OS0",
                                                          false, "AH02", 32)),
         std::make_pair("OS-0-32-BH02", product_info_test("OS-0-32-BH02", "OS0",
                                                          false, "BH02", 32)),
         std::make_pair("OS-0-32-G",
                        product_info_test("OS-0-32-G", "OS0", false, "G", 32)),
         std::make_pair("OS-0-32-U0", product_info_test("OS-0-32-U0", "OS0",
                                                        false, "U0", 32)),
         std::make_pair("OS-0-32-U1", product_info_test("OS-0-32-U1", "OS0",
                                                        false, "U1", 32)),
         std::make_pair("OS-0-32-U2", product_info_test("OS-0-32-U2", "OS0",
                                                        false, "U2", 32)),
         std::make_pair("OS-0-32-U3", product_info_test("OS-0-32-U3", "OS0",
                                                        false, "U3", 32)),
         std::make_pair("OS-0-64-AH", product_info_test("OS-0-64-AH", "OS0",
                                                        false, "AH", 64)),
         std::make_pair("OS-0-64-BH", product_info_test("OS-0-64-BH", "OS0",
                                                        false, "BH", 64)),
         std::make_pair("OS-0-64-G",
                        product_info_test("OS-0-64-G", "OS0", false, "G", 64)),
         std::make_pair("OS-0-64-U02", product_info_test("OS-0-64-U02", "OS0",
                                                         false, "U02", 64)),
         std::make_pair("OS-0-64-U13", product_info_test("OS-0-64-U13", "OS0",
                                                         false, "U13", 64)),
         std::make_pair("OS-1-128",
                        product_info_test("OS-1-128", "OS1", false, "U", 128)),
         std::make_pair("OS-1-128-SR", product_info_test("OS-1-128-SR", "OS1",
                                                         true, "U", 128)),
         std::make_pair("OS-1-16-A1", product_info_test("OS-1-16-A1", "OS1",
                                                        false, "A1", 16)),
         std::make_pair("OS-1-16-U0", product_info_test("OS-1-16-U0", "OS1",
                                                        false, "U0", 16)),
         std::make_pair("OS-1-32-A02", product_info_test("OS-1-32-A02", "OS1",
                                                         false, "A02", 32)),
         std::make_pair("OS-1-32-BH02", product_info_test("OS-1-32-BH02", "OS1",
                                                          false, "BH02", 32)),
         std::make_pair("OS-1-32-BH13", product_info_test("OS-1-32-BH13", "OS1",
                                                          false, "BH13", 32)),
         std::make_pair("OS-1-32-C",
                        product_info_test("OS-1-32-C", "OS1", false, "C", 32)),
         std::make_pair("OS-1-32-G",
                        product_info_test("OS-1-32-G", "OS1", false, "G", 32)),
         std::make_pair("OS-1-32-U0", product_info_test("OS-1-32-U0", "OS1",
                                                        false, "U0", 32)),
         std::make_pair("OS-1-32-U1", product_info_test("OS-1-32-U1", "OS1",
                                                        false, "U1", 32)),
         std::make_pair("OS-1-32-U2", product_info_test("OS-1-32-U2", "OS1",
                                                        false, "U2", 32)),
         std::make_pair("OS-1-32-U3", product_info_test("OS-1-32-U3", "OS1",
                                                        false, "U3", 32)),
         std::make_pair("OS-1-64",
                        product_info_test("OS-1-64", "OS1", false, "U", 64)),
         std::make_pair("OS-1-64-AH", product_info_test("OS-1-64-AH", "OS1",
                                                        false, "AH", 64)),
         std::make_pair("OS-1-64-BH", product_info_test("OS-1-64-BH", "OS1",
                                                        false, "BH", 64)),
         std::make_pair("OS-1-64-G",
                        product_info_test("OS-1-64-G", "OS1", false, "G", 64)),
         std::make_pair("OS-1-64-U02", product_info_test("OS-1-64-U02", "OS1",
                                                         false, "U02", 64)),
         std::make_pair("OS-1-64-U13", product_info_test("OS-1-64-U13", "OS1",
                                                         false, "U13", 64)),
         std::make_pair("OS-2-128",
                        product_info_test("OS-2-128", "OS2", false, "U", 128)),
         std::make_pair("OS-2-32-BH02", product_info_test("OS-2-32-BH02", "OS2",
                                                          false, "BH02", 32)),
         std::make_pair("OS-2-32-G",
                        product_info_test("OS-2-32-G", "OS2", false, "G", 32)),
         std::make_pair("OS-2-32-U0", product_info_test("OS-2-32-U0", "OS2",
                                                        false, "U0", 32)),
         std::make_pair("OS-2-32-U2", product_info_test("OS-2-32-U2", "OS2",
                                                        false, "U2", 32)),
         std::make_pair("OS-2-64-BH", product_info_test("OS-2-64-BH", "OS2",
                                                        false, "BH", 64)),
         std::make_pair("OS-2-64-G",
                        product_info_test("OS-2-64-G", "OS2", false, "G", 64)),
         std::make_pair("OS-2-64-U02", product_info_test("OS-2-64-U02", "OS2",
                                                         false, "U02", 64)),
         std::make_pair(
             "OS-DOME-128",
             product_info_test("OS-DOME-128", "OSDOME", false, "U", 128)),
         std::make_pair(
             "OS-DOME-32-AH02",
             product_info_test("OS-DOME-32-AH02", "OSDOME", false, "AH02", 32)),
         std::make_pair(
             "OS-DOME-32-AH13",
             product_info_test("OS-DOME-32-AH13", "OSDOME", false, "AH13", 32)),
         std::make_pair(
             "OS-DOME-32-BH02",
             product_info_test("OS-DOME-32-BH02", "OSDOME", false, "BH02", 32)),
         std::make_pair(
             "OS-DOME-32-BH13",
             product_info_test("OS-DOME-32-BH13", "OSDOME", false, "BH13", 32)),
         std::make_pair(
             "OS-DOME-32-G",
             product_info_test("OS-DOME-32-G", "OSDOME", false, "G", 32)),
         std::make_pair(
             "OS-DOME-32-U0",
             product_info_test("OS-DOME-32-U0", "OSDOME", false, "U0", 32)),
         std::make_pair(
             "OS-DOME-32-U1",
             product_info_test("OS-DOME-32-U1", "OSDOME", false, "U1", 32)),
         std::make_pair(
             "OS-DOME-32-U2",
             product_info_test("OS-DOME-32-U2", "OSDOME", false, "U2", 32)),
         std::make_pair(
             "OS-DOME-32-U3",
             product_info_test("OS-DOME-32-U3", "OSDOME", false, "U3", 32)),
         std::make_pair(
             "OS-DOME-64-AH",
             product_info_test("OS-DOME-64-AH", "OSDOME", false, "AH", 64)),
         std::make_pair(
             "OS-DOME-64-BH",
             product_info_test("OS-DOME-64-BH", "OSDOME", false, "BH", 64)),
         std::make_pair(
             "OS-DOME-64-G",
             product_info_test("OS-DOME-64-G", "OSDOME", false, "G", 64)),
         std::make_pair(
             "OS-DOME-64-U02",
             product_info_test("OS-DOME-64-U02", "OSDOME", false, "U02", 64)),
         std::make_pair(
             "OS-DOME-64-U13",
             product_info_test("OS-DOME-64-U13", "OSDOME", false, "U13", 64))};

    for (auto it : test_product_infos) {
        auto actual =
            ouster::sensor::product_info::create_product_info(it.first);
        std::cout << "Comparing:" << std::endl;
        std::cout << to_string(actual);
        std::cout << "To:" << std::endl;
        std::cout << to_string((ouster::sensor::product_info)it.second);
        EXPECT_EQ(actual, (ouster::sensor::product_info)it.second);
    }
}
