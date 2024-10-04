/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <json/json.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/metadata.h"
#include "ouster/types.h"
#include "ouster/util.h"
#include "ouster/version.h"

namespace ouster {

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[out] sensor_info The sensor_info to populate.
 * @param[out] issues The issues that occured during parsing.
 * @return if there are any critical issues or not.
 */
extern bool parse_and_validate_metadata(
    const std::string& json_data, ouster::sensor::sensor_info& sensor_info,
    ValidatorIssues& issues);

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

auto sensor_info::w() const -> decltype(format.columns_per_frame) {
    return format.columns_per_frame;
}

auto sensor_info::h() const -> decltype(format.pixels_per_column) {
    return format.pixels_per_column;
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

sensor_info::sensor_info() {
    // TODO - understand why this seg faults in CI when uncommented
    // logger().warn("Initializing sensor_info without original metadata
    // string");
}

sensor_info::sensor_info(const std::string& metadata) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    ValidatorIssues issues;

    if (metadata.size() > 0) {
        parse_and_validate_metadata(metadata, *this, issues);
        if (issues.critical.size() > 0) {
            std::stringstream error_string;
            error_string << "ERROR: Critical Metadata Issues Exist: "
                         << std::endl;
            for (auto it : issues.critical) {
                error_string << it.to_string() << std::endl;
            }
            throw std::runtime_error(error_string.str());
        }
    } else {
        throw std::runtime_error("ERROR: empty metadata passed in");
    }
}
sensor_info::sensor_info(const std::string& metadata,
                         bool /*skip_beam_validation*/)
    : sensor_info(metadata) {}

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
                               bool /*skip_beam_validation*/) {
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

    return sensor_info(buf.str());
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
