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

#include "logging.h"
#include "ouster/impl/build.h"
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
    return (lhs.has_fields_equal(rhs) &&
            lhs.original_string() == rhs.original_string());
}

bool operator!=(const sensor_info& lhs, const sensor_info& rhs) {
    return !(lhs == rhs);
}

bool sensor_info::has_fields_equal(const sensor_info& other) const {
    return (
        this->name == other.name && this->sn == other.sn &&
        this->fw_rev == other.fw_rev && this->mode == other.mode &&
        this->prod_line == other.prod_line && this->format == other.format &&
        this->beam_azimuth_angles == other.beam_azimuth_angles &&
        this->beam_altitude_angles == other.beam_altitude_angles &&
        this->lidar_origin_to_beam_origin_mm ==
            other.lidar_origin_to_beam_origin_mm &&
        this->beam_to_lidar_transform == other.beam_to_lidar_transform &&
        this->imu_to_sensor_transform == other.imu_to_sensor_transform &&
        this->lidar_to_sensor_transform == other.lidar_to_sensor_transform &&
        this->extrinsic == other.extrinsic && this->init_id == other.init_id &&
        this->udp_port_lidar == other.udp_port_lidar &&
        this->udp_port_imu == other.udp_port_imu &&
        this->build_date == other.build_date &&
        this->image_rev == other.image_rev && this->prod_pn == other.prod_pn &&
        this->status == other.status && this->cal == other.cal &&
        this->config == other.config);
}

/* Default values */

sensor_info default_sensor_info(lidar_mode mode) {
    auto info = sensor_info();

    info.name = "UNKNOWN";
    info.sn = "000000000000";
    info.fw_rev = "UNKNOWN";
    info.mode = mode;
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
    info.udp_port_lidar = 0;
    info.udp_port_imu = 0;
    info.build_date = "";
    info.image_rev = "";
    info.prod_pn = "";
    info.status = "";
    info.cal = default_calibration_status();
    info.config = sensor_config{};

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

static bool is_new_format(const std::string& metadata) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::runtime_error{
                "Error parsing metadata when checking format: " + errors};
    }

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

void parse_legacy(sensor_info& info, const std::string& metadata,
                  bool skip_beam_validation, bool suppress_legacy_warnings) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::runtime_error{
                "Errors parsing metadata for parse_metadata: " + errors};
    }

    // NOTE[pb]: DF development metadata.json doesn't have beam_altitude_angles
    // and beam_azimuth_angles and instead provides beam_xyz. However
    // final implementation should have azimuth/altitude angles and
    // we may uncomment the validation back closer to the release.
    // const std::vector<std::string> minimum_legacy_metadata_fields{
    //     "beam_altitude_angles", "beam_azimuth_angles", "lidar_mode"};

    // NOTE[pb]: DF metadata doesn't have lidar_mode, but because it's
    // going through convert_to_legacy() function it get's the empty
    // string lidar_mode ... hmmm, fine for now...
    const std::vector<std::string> minimum_legacy_metadata_fields{"lidar_mode"};

    for (auto field : minimum_legacy_metadata_fields) {
        if (!root.isMember(field)) {
            throw std::runtime_error{"Metadata must contain: " + field};
        }
    }

    // nice to have fields which we will use defaults for if they don't
    // exist
    const std::vector<std::string> desired_legacy_metadata_fields{
        "imu_to_sensor_transform",
        "lidar_to_sensor_transform",
        "prod_line",
        "prod_sn",
        "build_rev",
        "config_params"};
    for (auto field : desired_legacy_metadata_fields) {
        if (!root.isMember(field)) {
            if (suppress_legacy_warnings && field != "config_params") {
                logger().warn(
                    "No " + field +
                    " found in metadata. Will be left blank or filled in "
                    "with default legacy values");
            }
        }
    }

    // will be empty string if not present
    info.name = root["hostname"].asString();

    // if these are not present they are also empty strings
    info.build_date = root["build_date"].asString();
    info.fw_rev = root["build_rev"].asString();
    info.image_rev = root["image_rev"].asString();
    info.prod_line = root["prod_line"].asString();
    info.prod_pn = root["prod_pn"].asString();
    info.sn = root["prod_sn"].asString();
    info.status = root["status"].asString();

    // default to 0 if init_id key not present
    info.init_id = root["initialization_id"].asInt();

    // checked that lidar_mode is present already - never empty string
    info.mode = lidar_mode_of_string(root["lidar_mode"].asString());

    // "data_format" introduced in fw 2.0. Fall back to 1.13
    if (root.isMember("data_format")) {
        info.format = parse_data_format(root["data_format"]);
        // data_format.fps was added for DF sensors, so we are backfilling
        // fps value for OS sensors here if it's not present in metadata
        if (info.format.fps == 0) {
            info.format.fps = frequency_of_lidar_mode(info.mode);
        }
    } else {
        logger().warn("No data_format found. Using default legacy data format");
        info.format = default_data_format(info.mode);
    }

    // "lidar_origin_to_beam_origin_mm" introduced in fw 2.0 BUT missing
    // on OS-DOME. Handle falling back to FW 1.13 or setting to 0
    // according to prod-line
    if (root.isMember("lidar_origin_to_beam_origin_mm")) {
        info.lidar_origin_to_beam_origin_mm =
            root["lidar_origin_to_beam_origin_mm"].asDouble();
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
    if (root.isMember("beam_to_lidar_transform")) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.beam_to_lidar_transform(i, j) =
                    root["beam_to_lidar_transform"][ind].asDouble();
            }
        }
    } else {
        // fw is < 2.5/3.0 and we need to manually fill it in
        info.beam_to_lidar_transform = mat4d::Identity();
        info.beam_to_lidar_transform(0, 3) =
            info.lidar_origin_to_beam_origin_mm;
    }

    if (root["beam_altitude_angles"].size() != 0 &&
        root["beam_altitude_angles"].size() != info.format.pixels_per_column)
        throw std::runtime_error{"Unexpected number of beam_altitude_angles"};

    if (root["beam_azimuth_angles"].size() != 0 &&
        root["beam_azimuth_angles"].size() != info.format.pixels_per_column)
        throw std::runtime_error{"Unexpected number of beam_azimuth_angles"};

    if (root["beam_altitude_angles"].size() == info.format.pixels_per_column) {
        if (root["beam_altitude_angles"][0].isArray()) {
            // DF sensor path
            for (const auto& row : root["beam_altitude_angles"])
                for (const auto& v : row)
                    info.beam_altitude_angles.push_back(v.asDouble());

            if (info.beam_altitude_angles.size() !=
                info.format.pixels_per_column * info.format.columns_per_frame) {
                throw std::runtime_error{
                    "Unexpected number of total beam_altitude_angles"};
            }
        } else {
            // OS sensor path
            for (const auto& v : root["beam_altitude_angles"])
                info.beam_altitude_angles.push_back(v.asDouble());
        }
    }

    if (root["beam_azimuth_angles"].size() == info.format.pixels_per_column) {
        if (root["beam_azimuth_angles"][0].isArray()) {
            // DF sensor path
            for (const auto& row : root["beam_azimuth_angles"]) {
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
            for (const auto& v : root["beam_azimuth_angles"])
                info.beam_azimuth_angles.push_back(v.asDouble());
        }
    }

    // NOTE[pb]: this block that handles beam_xyz shouldn't survive past
    // the DF development phase and we need to swith to azimuth/altitude
    // angles in the metadata, because they take less space and they
    // are less redundant configuration of intrinsics than unit xyz vectors
    if (info.beam_altitude_angles.empty() && info.beam_azimuth_angles.empty()) {
        if (root["beam_xyz"].size() !=
            3 * info.format.pixels_per_column * info.format.columns_per_frame) {
            throw std::runtime_error{"Unexpected number of beam_xyz"};
        }

        // DF sensor path
        auto& xyz = root["beam_xyz"];
        for (Json::Value::ArrayIndex idx = 0; idx < xyz.size(); idx += 3) {
            auto x = xyz[idx + 0].asDouble();
            auto y = xyz[idx + 1].asDouble();
            auto z = xyz[idx + 2].asDouble();
            auto al = std::atan2(z, sqrt(x * x + y * y)) * 180.0 / M_PI;
            auto az = std::atan2(y, x) * 180.0 / M_PI;
            info.beam_altitude_angles.push_back(al);
            info.beam_azimuth_angles.push_back(az);
        }
    }

    // "imu_to_sensor_transform" may be absent in sensor config
    // produced by Ouster Studio, so we backfill it with default value
    if (root["imu_to_sensor_transform"].size() == 16) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.imu_to_sensor_transform(i, j) =
                    root["imu_to_sensor_transform"][ind].asDouble();
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
    if (root["lidar_to_sensor_transform"].size() == 16) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.lidar_to_sensor_transform(i, j) =
                    root["lidar_to_sensor_transform"][ind].asDouble();
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

    // default to 0 if keys are not present
    info.udp_port_lidar = root["udp_port_lidar"].asInt();
    info.udp_port_imu = root["udp_port_imu"].asInt();
}

static void update_json_obj(Json::Value& dst, const Json::Value& src) {
    const std::vector<std::string>& members = src.getMemberNames();
    for (const auto& key : members) {
        dst[key] = src[key];
    }
}

sensor_info::sensor_info() {
    // TODO - understand why this seg faults in CI when uncommented
    // logger().warn("Initializing sensor_info without original metadata
    // string");
    original_metadata_string = "";
}

sensor_info::sensor_info(const std::string& metadata,
                         bool skip_beam_validation) {
    original_metadata_string = metadata;

    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::runtime_error{"Errors parsing metadata string: " +
                                     errors};
    }

    if (is_new_format(metadata)) {
        logger().info("parsing non-legacy metadata format");
        parse_legacy(*this, convert_to_legacy(metadata), skip_beam_validation,
                     true);
        // also parse the sensor_config

        // we are guaranteed calibration_status as a key exists so don't need to
        // check again
        if (root["calibration_status"].isObject()) {
            if (root["calibration_status"]["reflectivity"]["valid"].isBool()) {
                this->cal.reflectivity_status =
                    root["calibration_status"]["reflectivity"]["valid"]
                        .asBool();
            } else {
                logger().warn(
                    "metadata field calibration_status.reflectivity.valid is "
                    "not Bool value, but: {}. Using False instead.",
                    root["calibration_status"]["reflectivity"]["valid"]
                        .asString());
            }

            if (this->cal.reflectivity_status) {
                this->cal.reflectivity_timestamp =
                    root["calibration_status"]["reflectivity"]["timestamp"]
                        .asString();
            }
        }

        this->config = parse_config(root["config_params"]);

    } else {
        logger().info("parsing legacy metadata format");
        parse_legacy(*this, metadata, skip_beam_validation, false);
    }
}

std::string sensor_info::original_string() const {
    return original_metadata_string;
}

void mat4d_to_json(Json::Value& val, mat4d mat) {
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            val.append(mat(i, j));
        }
    }
}

/* DO NOT make public - internal logic use only
 * Powers outputting a sensor_info to a flat JSON resembling legacy metadata
 */
Json::Value info_to_flat_json(const sensor_info& info) {
    Json::Value result{};

    result["hostname"] = info.name;
    result["prod_sn"] = info.sn;
    result["build_rev"] = info.fw_rev;
    result["lidar_mode"] = to_string(info.mode);
    result["prod_line"] = info.prod_line;

    // data_format
    result["data_format"]["pixels_per_column"] = info.format.pixels_per_column;
    result["data_format"]["columns_per_packet"] =
        info.format.columns_per_packet;
    result["data_format"]["columns_per_frame"] = info.format.columns_per_frame;
    result["data_format"]["fps"] = info.format.fps;
    result["data_format"]["column_window"].append(
        info.format.column_window.first);
    result["data_format"]["column_window"].append(
        info.format.column_window.second);
    result["data_format"]["udp_profile_lidar"] =
        to_string(info.format.udp_profile_lidar);
    result["data_format"]["udp_profile_imu"] =
        to_string(info.format.udp_profile_imu);
    for (auto i : info.format.pixel_shift_by_row)
        result["data_format"]["pixel_shift_by_row"].append(i);

    result["lidar_origin_to_beam_origin_mm"] =
        info.lidar_origin_to_beam_origin_mm;

    if (info.beam_azimuth_angles.size() ==
        info.format.pixels_per_column * info.format.columns_per_frame) {
        // Don't output for DF for now
        ;
    } else {
        for (auto i : info.beam_azimuth_angles)
            result["beam_azimuth_angles"].append(i);

        for (auto i : info.beam_altitude_angles)
            result["beam_altitude_angles"].append(i);
    }

    mat4d_to_json(result["beam_to_lidar_transform"],
                  info.beam_to_lidar_transform);
    mat4d_to_json(result["imu_to_sensor_transform"],
                  info.imu_to_sensor_transform);
    mat4d_to_json(result["lidar_to_sensor_transform"],
                  info.lidar_to_sensor_transform);
    mat4d_to_json(result["extrinsic"], info.extrinsic);

    result["initialization_id"] = info.init_id;
    result["udp_port_lidar"] = info.udp_port_lidar;
    result["udp_port_imu"] = info.udp_port_imu;

    result["build_date"] = info.build_date;
    result["image_rev"] = info.image_rev;
    result["prod_pn"] = info.prod_pn;
    result["status"] = info.status;

    result["calibration_status"] = cal_to_json(info.cal);
    result["config_params"] = config_to_json(info.config);

    return result;
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
        ;

        // TODO: debug DF beam_altitude/beam_azimuthoutpu
        /*
        int j = 0;
        for (size_t i=0; i<info.beam_azimuth_angles.size(); i++ ){
            int col_index_within_row = i % info.format.columns_per_frame;
            if (col_index_within_row == 0) { // make new array
                result["beam_intrinsics"]["beam_azimuth_angles"].append(Json::Value(Json::arrayValue));
                j++;
            }
            result["beam_intrinsics"]["beam_azimuth_angles"][j-1].append(info.beam_azimuth_angles[i]);
        }

        // reset j
        j = 0;
        for (size_t i=0; i<info.beam_altitude_angles.size(); i++ ){
            int col_index_within_row = i % info.format.columns_per_frame;
            if (col_index_within_row == 0) { // make new array
                result["beam_intrinsics"]["beam_altitude_angles"].append(Json::Value(Json::arrayValue));
                j++;
            }
            result["beam_intrinsics"]["beam_altitude_angles"][j-1].append(info.beam_altitude_angles[i]);
        }
        */
    }

    result["calibration_status"] = cal_to_json(info.cal);

    result["config_params"] = config_to_json(info.config);
    result["config_params"]["lidar_mode"] = to_string(info.mode);
    result["config_params"]["udp_port_lidar"] = info.udp_port_lidar;
    result["config_params"]["udp_port_imu"] = info.udp_port_imu;

    mat4d_to_json(result["imu_intrinsics"]["imu_to_sensor_transform"],
                  info.imu_to_sensor_transform);

    mat4d_to_json(result["lidar_intrinsics"]["lidar_to_sensor_transform"],
                  info.lidar_to_sensor_transform);

    result["ouster-sdk"]["hostname"] = info.name;
    mat4d_to_json(result["ouster-sdk"]["extrinsic"], info.extrinsic);

    return result;
}

// TODO refactor for performance since we're parsing
std::string sensor_info::updated_metadata_string() {
    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";
    Json::Value result{};
    Json::Value root_orig{};
    Json::Value root_new{};

    if (original_metadata_string.empty()) {
        logger().warn(
            "No original metadata string - will output a complete non-legacy "
            "metadata");
        root_new = info_to_nested_json(*this);
    } else {
        Json::CharReaderBuilder builder{};
        std::string errors{};
        std::stringstream ss{original_metadata_string};

        if (!Json::parseFromStream(builder, ss, &root_orig, &errors))
            throw std::runtime_error{
                "Error parsing original metadata string when checking "
                "format: " +
                errors};

        auto orig_info = sensor_info(original_metadata_string);

        if (is_new_format(original_metadata_string)) {
            logger().info(
                "Outputting updated metadata string based on non-legacy format "
                "of original metadata");
            if (this->fw_rev.substr(0, 2) == "v1") {
                // NOTE: currently updated_metatadata_string does not handle
                // outputting udp_dest and operating_mode back into udp_ip and
                // auto_start_flag for FW 1.12, 1.13, 1.14 in the config_params
                // for non-legacy
                logger().warn(
                    "Outputting an updated non-legacy metadata format from FWs "
                    "below 2.2 is not recommend");
            }

            root_new = info_to_nested_json(*this);

            // check if format was auto-populated - we know it's new format so
            // can skip the isMember check
            if (!root_orig["lidar_data_format"].isObject()) {
                if (this->format == orig_info.format)
                    root_new.removeMember("lidar_data_format");
            } else {
                // format was not auto-populated so now check fps,
                // pixel_shift_by_row, column_window, udp_profile_lidar,
                // udp_profile_imu
                if (!root_orig["lidar_data_format"].isMember("fps") &&
                    this->format.fps == orig_info.format.fps)
                    root_new["lidar_data_format"].removeMember("fps");
                if (!root_orig["lidar_data_format"].isMember("column_window") &&
                    this->format.column_window ==
                        orig_info.format.column_window)
                    root_new["lidar_data_format"].removeMember("column_window");
                if (!root_orig["lidar_data_format"].isMember(
                        "pixel_shift_by_row") &&
                    this->format.pixel_shift_by_row ==
                        orig_info.format.pixel_shift_by_row)
                    root_new["lidar_data_format"].removeMember(
                        "pixel_shift_by_row");
                if (!root_orig["lidar_data_format"].isMember(
                        "udp_profile_lidar") &&
                    this->format.udp_profile_lidar ==
                        orig_info.format.udp_profile_lidar)
                    root_new["lidar_data_format"].removeMember(
                        "udp_profile_lidar");
                if (!root_orig["lidar_data_format"].isMember(
                        "udp_profile_imu") &&
                    this->format.udp_profile_imu ==
                        orig_info.format.udp_profile_imu)
                    root_new["lidar_data_format"].removeMember(
                        "udp_profile_imu");
            }

            // check beam_intrinsics.imu_to_sensor_transform -
            //     NO NEED - FW 1.12 already had this - skip check
            // check lidar_intrinsics.lidar_to_sensor_transform -
            //     NO NEED - FW 1.12 already had this - skip check

            // check lidar_origin_to_beam_origin_mm
            if (!root_orig["beam_intrinsics"].isMember(
                    "lidar_origin_to_beam_origin_mm") &&
                this->lidar_origin_to_beam_origin_mm ==
                    orig_info.lidar_origin_to_beam_origin_mm)
                root_new["beam_intrinsics"].removeMember(
                    "lidar_origin_to_beam_origin_mm");

            // check beam_intinrics.beam_to_lidar_transform
            if (!root_orig["beam_intrinsics"].isMember(
                    "beam_to_lidar_transform") &&
                this->beam_to_lidar_transform ==
                    orig_info.beam_to_lidar_transform)
                root_new["beam_intrinsics"].removeMember(
                    "beam_to_lidar_transform");

            if (!root_orig["sensor_info"].isMember("initialization_id") &&
                this->init_id == orig_info.init_id)
                root_new["sensor_info"].removeMember("initialization_id");

            if (root_orig["calibration_status"] ==
                    "error: Command not recognized." &&
                this->cal == orig_info.cal) {
                root_new["calibration_status"] =
                    "error: Command not recognized.";
            }

            if (root_orig["lidar_data_format"] ==
                    "error: Command not recognized." &&
                this->format == orig_info.format) {
                root_new["lidar_data_format"] =
                    "error: Command not recognized.";
            }

        } else {
            // have original metadata string that is legacy format -  warn users
            // what they will lose Users who initialize with legacy metadata but
            // want to change these values ...
            // ... should upgrade to non-legacy format
            if (this->config != sensor_config() ||
                this->cal != calibration_status()) {
                logger().warn(
                    "Your sensor_info has set sensor_config and/or "
                    "calibration_status "
                    "items despite starting with legacy metadata. These will "
                    "be "
                    "disregarded in your output (which will be of legacy "
                    "format.");
            }
            root_new = info_to_flat_json(*this);

            root_new.removeMember("config_params");
            root_new.removeMember("calibration_status");
            if (this->udp_port_imu == orig_info.udp_port_imu) {
                root_new.removeMember("udp_port_imu");
            }
            if (this->udp_port_lidar == orig_info.udp_port_lidar) {
                root_new.removeMember("udp_port_lidar");
            }
            if (this->extrinsic == orig_info.extrinsic) {
                root_new.removeMember("extrinsic");
            }

            // check format
            if (!root_orig["data_format"].isObject()) {
                if (this->format == orig_info.format)
                    root_new.removeMember("data_format");
            } else {
                // format was not auto-populated so now check fps,
                // pixel_shift_by_row, column_window, udp_profile_lidar,
                // udp_profile_imu
                if (!root_orig["data_format"].isMember("fps") &&
                    this->format.fps == orig_info.format.fps)
                    root_new["data_format"].removeMember("fps");
                if (!root_orig["data_format"].isMember("pixel_shift_by_row") &&
                    this->format.pixel_shift_by_row ==
                        orig_info.format.pixel_shift_by_row)
                    root_new["data_format"].removeMember("pixel_shift_by_row");
                if (!root_orig["data_format"].isMember("column_window") &&
                    this->format.column_window ==
                        orig_info.format.column_window)
                    root_new["data_format"].removeMember("column_window");
                if (!root_orig["data_format"].isMember("udp_profile_lidar") &&
                    this->format.udp_profile_lidar ==
                        orig_info.format.udp_profile_lidar)
                    root_new["data_format"].removeMember("udp_profile_lidar");
                if (!root_orig["data_format"].isMember("udp_profile_imu") &&
                    this->format.udp_profile_imu ==
                        orig_info.format.udp_profile_imu)
                    root_new["data_format"].removeMember("udp_profile_imu");
            }
            // check imu_to_sensor_transform
            if (!root_orig.isMember("imu_to_sensor_transform") &&
                this->imu_to_sensor_transform ==
                    orig_info.imu_to_sensor_transform)
                root_new.removeMember("imu_to_sensor_transform");

            // check lidar_to_sensor_transform
            if (!root_orig.isMember("lidar_to_sensor_transform") &&
                this->lidar_to_sensor_transform ==
                    orig_info.lidar_to_sensor_transform)
                root_new.removeMember("lidar_to_sensor_transform");
            // check lidar_origin_to_beam_origin_mm
            if (!root_orig.isMember("lidar_origin_to_beam_origin_mm") &&
                this->lidar_origin_to_beam_origin_mm ==
                    orig_info.lidar_origin_to_beam_origin_mm)
                root_new.removeMember("lidar_origin_to_beam_origin_mm");
            // check beam_to_lidar_transform
            if (!root_orig.isMember("beam_to_lidar_transform") &&
                this->beam_to_lidar_transform ==
                    orig_info.beam_to_lidar_transform)
                root_new.removeMember("beam_to_lidar_transform");
        }
    }

    std::vector<std::string> changed;
    result = ouster::combined(root_orig, root_new, changed);

    // Relevant for both non-legacy and legacy
    result["ouster-sdk"]["output_source"] = "updated_metadata_string";
    result["ouster-sdk"]["client_version"] = client_version();
    for (auto& changed_str : changed)
        result["ouster-sdk"]["changed_fields"].append(changed_str);

    Json::StreamWriterBuilder write_builder;
    write_builder["enableYAMLCompatibility"] = true;
    write_builder["precision"] = 6;
    write_builder["indentation"] = "    ";
    return Json::writeString(write_builder, result);
}

std::string convert_to_legacy(const std::string& metadata) {
    if (!is_new_format(metadata))
        throw std::invalid_argument(
            "Invalid non-legacy metadata format provided");

    Json::Value root{};
    Json::CharReaderBuilder read_builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(read_builder, ss, &root, &errors)) {
            throw std::runtime_error{
                "Errors parsing metadata for convert_to_legacy: " + errors};
        }
    }
    Json::Value result{};

    if (root.isMember("config_params")) {
        result["lidar_mode"] = root["config_params"]["lidar_mode"];
        result["udp_port_lidar"] = root["config_params"]["udp_port_lidar"];
        result["udp_port_imu"] = root["config_params"]["udp_port_imu"];
    }

    if (root.isMember("client_version"))
        result["client_version"] = root["client_version"];

    if (root.isMember("ouster-sdk")) result["ouster-sdk"] = root["ouster-sdk"];

    // TODO eventually remove
    // NOTE: DO NOT REMOVE until mid 2024
    // json-calibration-version powers any legacy conversion being done for
    // users still on Kitware Ouster Studio probably best to announce removal
    // "breakage" by Beginning 2024
    result["json_calibration_version"] = FW_2_2;

    result["hostname"] = root["hostname"].asString();

    update_json_obj(result, root["sensor_info"]);
    update_json_obj(result, root["beam_intrinsics"]);
    update_json_obj(result, root["imu_intrinsics"]);
    update_json_obj(result, root["lidar_intrinsics"]);

    if (root.isMember("lidar_data_format") &&
        root["lidar_data_format"].isObject()) {
        result["data_format"] = Json::Value{};
        update_json_obj(result["data_format"], root["lidar_data_format"]);
    }

    Json::StreamWriterBuilder write_builder;
    write_builder["enableYAMLCompatibility"] = true;
    write_builder["precision"] = 6;
    write_builder["indentation"] = "    ";
    return Json::writeString(write_builder, result);
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

    return parse_metadata(buf.str(), skip_beam_validation);
}

// TODO - fix up according to debug output desires
std::string to_string(const sensor_info& info) {
    logger().warn(
        "Calling debug to_string on sensor_info. Does NOT produce valid "
        "metadata.json");
    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";

    auto root = info_to_flat_json(info);
    root["ouster-sdk"]["output_source"] = "DEBUG:to_string";

    return Json::writeString(builder, root);
}

sensor_info parse_metadata(const std::string& metadata,
                           bool skip_beam_validation) {
    return sensor_info(metadata, skip_beam_validation);
}

}  // namespace sensor
}  // namespace ouster
