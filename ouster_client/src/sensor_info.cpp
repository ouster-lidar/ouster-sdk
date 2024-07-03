/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <json/json.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/types.h"
#include "ouster/util.h"
#include "ouster/version.h"

namespace ouster {

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

/* version field required by ouster studio */
enum configuration_version { FW_2_0 = 3, FW_2_2 = 4 };

namespace sensor {

extern ColumnWindow default_column_window(uint32_t columns_per_frame);
extern data_format default_data_format(lidar_mode mode);
extern double default_lidar_origin_to_beam_origin(std::string prod_line);
extern mat4d default_beam_to_lidar_transform(std::string prod_line);
extern calibration_status default_calibration_status();
extern sensor_config parse_config(const Json::Value& root);
extern data_format parse_data_format(const Json::Value& root);
extern Json::Value cal_to_json(const calibration_status& cal);
extern Json::Value config_to_json(const sensor_config& config);

/* Equality operators and functions */

bool operator==(const sensor_info& lhs, const sensor_info& rhs) {
    return lhs.has_fields_equal(rhs);
}

bool operator!=(const sensor_info& lhs, const sensor_info& rhs) {
    return !(lhs == rhs);
}

bool sensor_info::has_fields_equal(const sensor_info& other) const {
    return (
        this->sn == other.sn && this->fw_rev == other.fw_rev &&
        this->prod_line == other.prod_line && this->format == other.format &&
        this->beam_azimuth_angles == other.beam_azimuth_angles &&
        this->beam_altitude_angles == other.beam_altitude_angles &&
        this->lidar_origin_to_beam_origin_mm ==
            other.lidar_origin_to_beam_origin_mm &&
        this->beam_to_lidar_transform == other.beam_to_lidar_transform &&
        this->imu_to_sensor_transform == other.imu_to_sensor_transform &&
        this->lidar_to_sensor_transform == other.lidar_to_sensor_transform &&
        this->extrinsic == other.extrinsic && this->init_id == other.init_id &&
        this->build_date == other.build_date &&
        this->image_rev == other.image_rev && this->prod_pn == other.prod_pn &&
        this->status == other.status && this->cal == other.cal &&
        this->config == other.config && this->user_data == other.user_data);
}

/* Default values */

sensor_info default_sensor_info(lidar_mode mode) {
    auto info = sensor_info();
    info.sn = "000000000000";
    info.fw_rev = "UNKNOWN";

    info.prod_line = "OS-1-64";

    info.format = default_data_format(mode);
    info.beam_azimuth_angles = gen1_azimuth_angles;
    info.beam_altitude_angles = gen1_altitude_angles;
    info.lidar_origin_to_beam_origin_mm =
        default_lidar_origin_to_beam_origin(info.prod_line);
    info.beam_to_lidar_transform = default_imu_to_sensor_transform;
    info.imu_to_sensor_transform =
        default_beam_to_lidar_transform(info.prod_line);
    info.lidar_to_sensor_transform = default_lidar_to_sensor_transform;
    info.extrinsic = mat4d::Identity();
    info.init_id = 0;
    info.build_date = "";
    info.image_rev = "";
    info.prod_pn = "";
    info.status = "";
    info.user_data = "";
    info.cal = default_calibration_status();
    info.config = sensor_config{};
    info.config.lidar_mode = mode;
    info.config.udp_port_lidar = 0;
    info.config.udp_port_imu = 0;

    return info;
}

// clang-format off
extern const std::vector<double> gen1_altitude_angles = {
    16.611, 16.084, 15.557, 15.029, 14.502, 13.975, 13.447, 12.920,
    12.393, 11.865, 11.338, 10.811, 10.283, 9.756, 9.229, 8.701, 
    8.174, 7.646, 7.119, 6.592, 6.064, 5.537, 5.010, 4.482, 
    3.955, 3.428, 2.900, 2.373, 1.846, 1.318, 0.791, 0.264,
    -0.264, -0.791, -1.318, -1.846, -2.373, -2.900, -3.428, -3.955,
    -4.482, -5.010, -5.537, -6.064, -6.592, -7.119, -7.646, -8.174,
    -8.701, -9.229, -9.756, -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

extern const std::vector<double> gen1_azimuth_angles = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055,
    -3.164,
};
// clang-format on

extern const mat4d default_imu_to_sensor_transform =
    (mat4d() << 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1)
        .finished();

extern const mat4d default_lidar_to_sensor_transform =
    (mat4d() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1)
        .finished();

/* String conversion */

// bool represents whether it is an object (true) or just a member (false)
// NOTE: lidar_data_format and calibration_status should be objects but as they
// were introduced earlier, non-legacy formats for FW version do not include
// them
// TODO parse metadata by FW version specified ?
// clang-format off
const std::map<std::string, bool> nonlegacy_metadata_fields = {
    {"sensor_info", true}, {"beam_intrinsics", true}, 
    {"imu_intrinsics", true}, {"lidar_intrinsics", true}, 
    {"config_params", true}, {"lidar_data_format", false}, 
    {"calibration_status", false}
};

// clang-format on

static bool is_new_format(const Json::Value& root) {
    size_t nonlegacy_fields_present = 0;
    std::string missing_fields = "";
    for (const auto& field_pair : nonlegacy_metadata_fields) {
        auto field = field_pair.first;
        auto is_obj = field_pair.second;
        if (root.isMember(field)) {
            nonlegacy_fields_present++;
            if (is_obj && !root[field].isObject()) {
                throw std::runtime_error{"Non-legacy metadata field " + field +
                                         " must have child fields"};
            }
        } else {
            missing_fields += field + " ";
        }
    }

    if (nonlegacy_fields_present > 0 &&
        nonlegacy_fields_present < nonlegacy_metadata_fields.size()) {
        throw std::runtime_error{"Non-legacy metadata must include fields: " +
                                 missing_fields};
    }

    return nonlegacy_fields_present == nonlegacy_metadata_fields.size();
}

static void parse_metadata(sensor_info& info, const Json::Value& root,
                           bool skip_beam_validation) {
    const std::vector<std::string> minimum_metadata_fields{"config_params",
                                                           "beam_intrinsics"};
    for (auto field : minimum_metadata_fields) {
        if (!root.isMember(field)) {
            throw std::runtime_error{"Metadata must contain: " + field};
        }
    }

    // nice to have fields which we will use defaults for if they don't exist
    const std::vector<std::string> desired_metadata_fields{"imu_intrinsics",
                                                           "lidar_intrinsics"};
    for (auto field : desired_metadata_fields) {
        if (!root.isMember(field)) {
            logger().warn("No " + field +
                          " found in metadata. Will be left blank or filled in "
                          "with default legacy values");
        }
    }

    // if these are not present they are also empty strings
    auto sensor_info = root["sensor_info"];
    info.build_date = sensor_info["build_date"].asString();
    info.fw_rev = sensor_info["build_rev"].asString();
    info.image_rev = sensor_info["image_rev"].asString();
    info.prod_line = sensor_info["prod_line"].asString();
    info.prod_pn = sensor_info["prod_pn"].asString();
    info.sn = sensor_info["prod_sn"].asString();
    info.status = sensor_info["status"].asString();

    // default to 0 if init_id key not present
    info.init_id = sensor_info["initialization_id"].asInt();

    // checked that lidar_mode is present already - never empty string
    auto mode =
        lidar_mode_of_string(root["config_params"]["lidar_mode"].asString());

    // "data_format" introduced in fw 2.0. Fall back to 1.13
    if (root.isMember("lidar_data_format") &&
        root["lidar_data_format"].isObject()) {
        info.format = parse_data_format(root["lidar_data_format"]);
        // data_format.fps was added for DF sensors, so we are backfilling
        // fps value for OS sensors here if it's not present in metadata
        if (info.format.fps == 0) {
            info.format.fps = frequency_of_lidar_mode(mode);
        }
    } else {
        logger().warn(
            "No lidar_data_format found. Using default legacy data format");
        info.format = default_data_format(mode);
    }

    // "lidar_origin_to_beam_origin_mm" introduced in fw 2.0 BUT missing
    // on OS-DOME. Handle falling back to FW 1.13 or setting to 0
    // according to prod-line
    auto beam_intrinsics = root["beam_intrinsics"];
    if (beam_intrinsics.isMember("lidar_origin_to_beam_origin_mm")) {
        info.lidar_origin_to_beam_origin_mm =
            beam_intrinsics["lidar_origin_to_beam_origin_mm"].asDouble();
    } else {
        if (info.prod_line.find("OS-DOME-") ==
            0) {  // is an OS-DOME - fill with 0
            info.lidar_origin_to_beam_origin_mm = 0;
        } else {  // not an OS-DOME
            logger().warn(
                "No lidar_origin_to_beam_origin_mm found. Using default "
                "value for the specified prod_line or default gen 1 values "
                "if prod_line is missing");
            info.lidar_origin_to_beam_origin_mm =
                default_lidar_origin_to_beam_origin(
                    info.prod_line);  // note it is possible that
                                      // info.prod_line is ""
        }
    }

    // beam_to_lidar_transform" introduced in fw 2.5/fw 3.0
    if (beam_intrinsics.isMember("beam_to_lidar_transform")) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.beam_to_lidar_transform(i, j) =
                    beam_intrinsics["beam_to_lidar_transform"][ind].asDouble();
            }
        }
    } else {
        // fw is < 2.5/3.0 and we need to manually fill it in
        info.beam_to_lidar_transform = mat4d::Identity();
        info.beam_to_lidar_transform(0, 3) =
            info.lidar_origin_to_beam_origin_mm;
    }

    if (beam_intrinsics["beam_altitude_angles"].size() != 0 &&
        beam_intrinsics["beam_altitude_angles"].size() !=
            info.format.pixels_per_column)
        throw std::runtime_error{"Unexpected number of beam_altitude_angles"};

    if (beam_intrinsics["beam_azimuth_angles"].size() != 0 &&
        beam_intrinsics["beam_azimuth_angles"].size() !=
            info.format.pixels_per_column)
        throw std::runtime_error{"Unexpected number of beam_azimuth_angles"};

    if (beam_intrinsics["beam_altitude_angles"].size() ==
        info.format.pixels_per_column) {
        if (beam_intrinsics["beam_altitude_angles"][0].isArray()) {
            // DF sensor path
            for (const auto& row : beam_intrinsics["beam_altitude_angles"])
                for (const auto& v : row)
                    info.beam_altitude_angles.push_back(v.asDouble());

            if (info.beam_altitude_angles.size() !=
                info.format.pixels_per_column * info.format.columns_per_frame) {
                throw std::runtime_error{
                    "Unexpected number of total beam_altitude_angles"};
            }
        } else {
            // OS sensor path
            for (const auto& v : beam_intrinsics["beam_altitude_angles"])
                info.beam_altitude_angles.push_back(v.asDouble());
        }
    }

    if (beam_intrinsics["beam_azimuth_angles"].size() ==
        info.format.pixels_per_column) {
        if (beam_intrinsics["beam_azimuth_angles"][0].isArray()) {
            // DF sensor path
            for (const auto& row : beam_intrinsics["beam_azimuth_angles"]) {
                for (const auto& v : row)
                    info.beam_azimuth_angles.push_back(v.asDouble());
            }

            if (info.beam_azimuth_angles.size() !=
                info.format.pixels_per_column * info.format.columns_per_frame) {
                throw std::runtime_error{
                    "Unexpected number of total beam_azimuth_angles"};
            }
        } else {
            // OS sensor path
            for (const auto& v : beam_intrinsics["beam_azimuth_angles"])
                info.beam_azimuth_angles.push_back(v.asDouble());
        }
    }

    // "imu_to_sensor_transform" may be absent in sensor config
    // produced by Ouster Studio, so we backfill it with default value
    auto imu_intrinsics = root["imu_intrinsics"];
    if (imu_intrinsics["imu_to_sensor_transform"].size() == 16) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.imu_to_sensor_transform(i, j) =
                    imu_intrinsics["imu_to_sensor_transform"][ind].asDouble();
            }
        }
    } else {
        logger().warn(
            "No valid imu_to_sensor_transform found. Using default for gen "
            "1");
        info.imu_to_sensor_transform = default_imu_to_sensor_transform;
    }

    // "lidar_to_sensor_transform" may be absent in sensor config
    // produced by Ouster Studio, so we backfill it with default value
    auto lidar_intrinsics = root["lidar_intrinsics"];
    if (lidar_intrinsics["lidar_to_sensor_transform"].size() == 16) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.lidar_to_sensor_transform(i, j) =
                    lidar_intrinsics["lidar_to_sensor_transform"][ind]
                        .asDouble();
            }
        }
    } else {
        logger().warn(
            "No valid lidar_to_sensor_transform found. Using default for "
            "gen "
            "1");
        info.lidar_to_sensor_transform = default_lidar_to_sensor_transform;
    }

    auto zero_check = [](auto el, std::string name) {
        if (el.size() == 0) return;
        bool all_zeros = std::all_of(el.cbegin(), el.cend(),
                                     [](double k) { return k == 0.0; });
        if (all_zeros) {
            throw std::runtime_error{"Field " + name +
                                     " in the metadata cannot all be zeros."};
        }
    };

    if (!skip_beam_validation) {
        zero_check(info.beam_altitude_angles, "beam_altitude_angles");
        zero_check(info.beam_azimuth_angles, "beam_azimuth_angles");
    } else {
        logger().warn("Skipping all 0 beam angle check");
    }

    info.extrinsic = mat4d::Identity();

    if (root.isMember("ouster-sdk")) {
        auto sdk_group = root["ouster-sdk"];
        if (sdk_group["extrinsic"].size() == 16) {
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    const Json::Value::ArrayIndex ind = i * 4 + j;
                    info.extrinsic(i, j) =
                        sdk_group["extrinsic"][ind].asDouble();
                }
            }
        } else {
            logger().info("No valid extrinsics found. Using identity.");
        }
    }

    // we are guaranteed calibration_status as a key exists so don't need to
    // check again
    if (root["calibration_status"].isObject()) {
        if (root["calibration_status"]["reflectivity"]["valid"].isBool()) {
            info.cal.reflectivity_status =
                root["calibration_status"]["reflectivity"]["valid"].asBool();
        } else {
            logger().warn(
                "metadata field calibration_status.reflectivity.valid is "
                "not Bool value, but: {}. Using False instead.",
                root["calibration_status"]["reflectivity"]["valid"].asString());
        }

        if (info.cal.reflectivity_status) {
            info.cal.reflectivity_timestamp =
                root["calibration_status"]["reflectivity"]["timestamp"]
                    .asString();
        }
    }

    info.config = parse_config(root["config_params"]);
    info.user_data = root["user_data"].asString();
}

static void parse_legacy(sensor_info& info, const Json::Value& root,
                         bool skip_beam_validation) {
    // just convert to non-legacy and run the non-legacy parse
    const std::vector<std::string> config_fields{
        "udp_port_imu",
        "udp_port_lidar",
        "lidar_mode",
    };

    const std::vector<std::string> beam_intrinsics_fields{
        "lidar_origin_to_beam_origin_mm", "beam_altitude_angles",
        "beam_azimuth_angles", "beam_to_lidar_transform"};

    const std::vector<std::string> sensor_info_fields{
        "prod_line",         "status",    "prod_pn",    "prod_sn",
        "initialization_id", "build_rev", "build_date", "image_rev",
    };

    // Error if we dont have required fields
    const std::vector<std::string> minimum_metadata_fields{"lidar_mode"};

    for (auto field : minimum_metadata_fields) {
        if (!root.isMember(field)) {
            throw std::runtime_error{"Metadata must contain: " + field};
        }
    }

    Json::Value result;
    if (root.isMember("lidar_to_sensor_transform")) {
        result["lidar_intrinsics"]["lidar_to_sensor_transform"] =
            root["lidar_to_sensor_transform"];
    }
    if (root.isMember("imu_to_sensor_transform")) {
        result["imu_intrinsics"]["imu_to_sensor_transform"] =
            root["imu_to_sensor_transform"];
    }
    if (root.isMember("data_format")) {
        result["lidar_data_format"] = root["data_format"];
    }
    if (root.isMember("client_version")) {
        result["ouster-sdk"]["client_version"] = root["client_version"];
    }
    for (const auto& field : config_fields) {
        if (root.isMember(field)) {
            result["config_params"][field] = root[field];
        }
    }

    for (const auto& field : beam_intrinsics_fields) {
        if (root.isMember(field)) {
            result["beam_intrinsics"][field] = root[field];
        }
    }

    for (const auto& field : sensor_info_fields) {
        if (root.isMember(field)) {
            result["sensor_info"][field] = root[field];
        }
    }

    parse_metadata(info, result, skip_beam_validation);
}

sensor_info::sensor_info() {
    // TODO - understand why this seg faults in CI when uncommented
    // logger().warn("Initializing sensor_info without original metadata
    // string");
}

sensor_info::sensor_info(const std::string& metadata,
                         bool skip_beam_validation) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::runtime_error{"Errors parsing metadata string: " +
                                     errors};
    }

    if (is_new_format(root)) {
        was_legacy_ = false;
        logger().info("parsing non-legacy metadata format");
        parse_metadata(*this, root, skip_beam_validation);
    } else {
        was_legacy_ = true;
        logger().info("parsing legacy metadata format");
        parse_legacy(*this, root, skip_beam_validation);
    }
}

void mat4d_to_json(Json::Value& val, mat4d mat) {
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            val.append(mat(i, j));
        }
    }
}

/* DO NOT make public - internal logic use only
 * Powers outputting a sensor_info to a nested json resembling non-legacy
 * metadata
 */
Json::Value info_to_nested_json(const sensor_info& info) {
    Json::Value result{};

    result["sensor_info"]["build_date"] = info.build_date;
    result["sensor_info"]["build_rev"] = info.fw_rev;
    result["sensor_info"]["image_rev"] = info.image_rev;
    result["sensor_info"]["initialization_id"] = info.init_id;
    result["sensor_info"]["prod_line"] = info.prod_line;
    result["sensor_info"]["prod_pn"] = info.prod_pn;
    result["sensor_info"]["prod_sn"] = info.sn;
    result["sensor_info"]["status"] = info.status;

    // data_format
    result["lidar_data_format"]["pixels_per_column"] =
        info.format.pixels_per_column;
    result["lidar_data_format"]["columns_per_packet"] =
        info.format.columns_per_packet;
    result["lidar_data_format"]["columns_per_frame"] =
        info.format.columns_per_frame;
    result["lidar_data_format"]["fps"] = info.format.fps;
    result["lidar_data_format"]["column_window"].append(
        info.format.column_window.first);
    result["lidar_data_format"]["column_window"].append(
        info.format.column_window.second);
    result["lidar_data_format"]["udp_profile_lidar"] =
        to_string(info.format.udp_profile_lidar);
    result["lidar_data_format"]["udp_profile_imu"] =
        to_string(info.format.udp_profile_imu);

    for (auto i : info.format.pixel_shift_by_row)
        result["lidar_data_format"]["pixel_shift_by_row"].append(i);

    // beam intrinsics
    //
    mat4d_to_json(result["beam_intrinsics"]["beam_to_lidar_transform"],
                  info.beam_to_lidar_transform);
    result["beam_intrinsics"]["lidar_origin_to_beam_origin_mm"] =
        info.lidar_origin_to_beam_origin_mm;

    if (info.beam_azimuth_angles.size() == info.format.pixels_per_column) {
        // OS sensor path
        for (auto angle : info.beam_azimuth_angles)
            result["beam_intrinsics"]["beam_azimuth_angles"].append(angle);
        for (auto angle : info.beam_altitude_angles)
            result["beam_intrinsics"]["beam_altitude_angles"].append(angle);
    } else {
        // DF sensor path
        int j = 0;
        for (size_t i = 0; i < info.beam_azimuth_angles.size(); i++) {
            int col_index_within_row = i % info.format.columns_per_frame;
            if (col_index_within_row == 0) {  // start new array
                result["beam_intrinsics"]["beam_azimuth_angles"].append(
                    Json::Value(Json::arrayValue));
                j++;
            }
            result["beam_intrinsics"]["beam_azimuth_angles"][j - 1].append(
                info.beam_azimuth_angles[i]);
        }

        j = 0;
        for (size_t i = 0; i < info.beam_altitude_angles.size(); i++) {
            int col_index_within_row = i % info.format.columns_per_frame;
            if (col_index_within_row == 0) {  // start new array
                result["beam_intrinsics"]["beam_altitude_angles"].append(
                    Json::Value(Json::arrayValue));
                j++;
            }
            result["beam_intrinsics"]["beam_altitude_angles"][j - 1].append(
                info.beam_altitude_angles[i]);
        }
    }

    result["calibration_status"] = cal_to_json(info.cal);

    result["config_params"] = config_to_json(info.config);

    result["user_data"] = info.user_data;

    mat4d_to_json(result["imu_intrinsics"]["imu_to_sensor_transform"],
                  info.imu_to_sensor_transform);

    mat4d_to_json(result["lidar_intrinsics"]["lidar_to_sensor_transform"],
                  info.lidar_to_sensor_transform);

    mat4d_to_json(result["ouster-sdk"]["extrinsic"], info.extrinsic);

    return result;
}

std::string sensor_info::to_json_string() const {
    Json::Value result = info_to_nested_json(*this);

    result["ouster-sdk"]["output_source"] = "sensor_info_to_string";
    result["ouster-sdk"]["client_version"] = client_version();

    Json::StreamWriterBuilder write_builder;
    write_builder["enableYAMLCompatibility"] = true;
    write_builder["precision"] = 6;
    write_builder["indentation"] = "    ";
    return Json::writeString(write_builder, result);
}

ouster::util::version sensor_info::get_version() const {
    return ouster::util::version_from_string(image_rev);
}

product_info sensor_info::get_product_info() const {
    return product_info::create_product_info(prod_line);
}

sensor_info metadata_from_json(const std::string& json_file,
                               bool skip_beam_validation) {
    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(json_file);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs) {
        std::stringstream ss;
        ss << "Failed to read metadata file: " << json_file;
        throw std::runtime_error{ss.str()};
    }

    return sensor_info(buf.str(), skip_beam_validation);
}

std::string to_string(const sensor_info& info) { return info.to_json_string(); }

std::string get_firmware_version(const Json::Value& metadata_root) {
    auto fw_ver = std::string{};
    if (metadata_root["sensor_info"].isObject()) {
        if (metadata_root["sensor_info"].isMember("image_rev")) {
            // image_rev is preferred over build_rev
            fw_ver = metadata_root["sensor_info"]["image_rev"].asString();
        }
    }
    return fw_ver;
}

ouster::util::version firmware_version_from_metadata(
    const std::string& metadata) {
    if (metadata.empty()) {
        throw std::invalid_argument(
            "firmware_version_from_metadata metadata empty!");
    }

    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (!Json::parseFromStream(builder, ss, &root, &errors))
        throw std::runtime_error{
            "Errors parsing metadata for parse_metadata: " + errors};

    auto fw_ver = get_firmware_version(root);
    if (fw_ver.empty()) {
        throw std::runtime_error(
            "firmware_version_from_metadata failed to deduce version info from "
            "metadata!");
    }

    return ouster::util::version_from_string(fw_ver);
}

}  // namespace sensor
}  // namespace ouster
