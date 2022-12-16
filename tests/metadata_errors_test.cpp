/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include "ouster/types.h"

class MetaErrorsFiles : public testing::TestWithParam<const char*> {};

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}

INSTANTIATE_TEST_CASE_P(
    ErrorMetas, MetaErrorsFiles,
    testing::Values(
        "complete_but_all_zeros_legacy",     // error out instead of passing to
                                             // xyzlut
        "incomplete_data_format_legacy",     // missing columns_per_frame
        "incomplete_data_format_nonlegacy",  // missing pixels per column
        "incomplete_no_sensor_info_nonlegacy",  // nonlegacy can't be missing
                                                // sensor info unlike legacy
        "incomplete_no_calref_nonlegacy",       // ditto calref
        "garbled_legacy_and_nonlegacy",    // sensor_info items on top level.
                                           // should read as nonlegacy and fail
        "legacy_with_calibration_status",  // has calibration_status but nothing
                                           // else. should read as nonlegacy and
                                           // fail
        "incorrect_nbeam_angles_legacy_113"  // missing one from beam altitude
                                             // angles
        ));

// Backwards-compatibility test for meta json parsing: compare previously parsed
// sensor_info structs to the output of metadata_from_json
TEST_P(MetaErrorsFiles, MetadataParsingExceptions) {
    std::string param = GetParam();

    auto data_dir = getenvs("DATA_DIR");

    EXPECT_ANY_THROW({
        // parse json file
        const ouster::sensor::sensor_info si =
            ouster::sensor::metadata_from_json(data_dir + "/malformed/" +
                                               param + ".json");
    });
}
