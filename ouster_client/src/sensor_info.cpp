/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <map>
#include <nonstd/optional.hpp>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/defaults.h"
#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/metadata.h"
#include "ouster/types.h"
#include "ouster/version.h"

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;
using ouster::sdk::core::mat4d_to_array;

namespace ouster {
namespace sdk {
namespace core {

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[out] sensor_info The sensor_info to populate.
 * @param[out] issues The issues that occured during parsing.
 * @return if there are any critical issues or not.
 */
extern bool parse_and_validate_metadata(const std::string& json_data,
                                        SensorInfo& sensor_info,
                                        ValidatorIssues& issues);

/* version field required by ouster studio */
enum ConfigurationVersion { FW_2_0 = 3, FW_2_2 = 4 };

static ColumnWindow default_column_window(uint32_t columns_per_frame) {
    return {0, columns_per_frame - 1};
}

double default_lidar_origin_to_beam_origin(std::string prod_line) {
    double lidar_origin_to_beam_origin_mm = 12.163;  // default for gen 1
    if (prod_line.find("OS-0-") == 0) {
        lidar_origin_to_beam_origin_mm = 27.67;
    } else if (prod_line.find("OS-1-") == 0) {
        lidar_origin_to_beam_origin_mm = 15.806;
    } else if (prod_line.find("OS-2-") == 0) {
        lidar_origin_to_beam_origin_mm = 13.762;
    }
    return lidar_origin_to_beam_origin_mm;
}

mat4d default_beam_to_lidar_transform(std::string prod_line) {
    mat4d beam_to_lidar_transform = mat4d::Identity();
    beam_to_lidar_transform(0, 3) =
        default_lidar_origin_to_beam_origin(prod_line);
    return beam_to_lidar_transform;
}

CalibrationStatus default_calibration_status() { return CalibrationStatus{}; }

extern jsoncons::json cal_to_json(const CalibrationStatus& cal);
extern jsoncons::json config_to_json(const SensorConfig& config);

/* Equality operators and functions */

bool operator==(const DataFormat& lhs, const DataFormat& rhs) {
    return (lhs.pixels_per_column == rhs.pixels_per_column &&
            lhs.columns_per_packet == rhs.columns_per_packet &&
            lhs.columns_per_frame == rhs.columns_per_frame &&
            lhs.imu_measurements_per_packet ==
                rhs.imu_measurements_per_packet &&
            lhs.pixel_shift_by_row == rhs.pixel_shift_by_row &&
            lhs.column_window == rhs.column_window &&
            lhs.udp_profile_lidar == rhs.udp_profile_lidar &&
            lhs.header_type == rhs.header_type &&
            lhs.udp_profile_imu == rhs.udp_profile_imu && lhs.fps == rhs.fps &&
            lhs.zone_monitoring_enabled == rhs.zone_monitoring_enabled);
}

bool operator!=(const DataFormat& lhs, const DataFormat& rhs) {
    return !(lhs == rhs);
}

bool operator==(const SensorInfo& lhs, const SensorInfo& rhs) {
    return lhs.has_fields_equal(rhs);
}

bool operator!=(const SensorInfo& lhs, const SensorInfo& rhs) {
    return !(lhs == rhs);
}

DataFormat default_data_format(LidarMode mode) {
    auto repeat = [](int name, const std::vector<int>& value) {
        std::vector<int> res{};
        for (int i = 0; i < name; i++) {
            res.insert(res.end(), value.begin(), value.end());
        }
        return res;
    };

    uint32_t pixels_per_column = 64;
    uint32_t columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    uint32_t columns_per_frame = n_cols_of_lidar_mode(mode);
    ColumnWindow column_window = default_column_window(columns_per_frame);
    uint32_t imu_measurements_per_packet = 0;
    uint32_t imu_packets_per_frame = 0;

    std::vector<int> offset;
    switch (columns_per_frame) {
        case 512:
            offset = repeat(16, {9, 6, 3, 0});
            break;
        case 1024:
            offset = repeat(16, {18, 12, 6, 0});
            break;
        case 2048:
            offset = repeat(16, {36, 24, 12, 0});
            break;
        case 4096:
            offset = repeat(16, {72, 48, 24, 0});
            break;
        default:
            throw std::invalid_argument{"default_data_format"};
    }

    return {pixels_per_column,
            columns_per_packet,
            columns_per_frame,
            imu_measurements_per_packet,
            imu_packets_per_frame,
            offset,
            column_window,
            UDPProfileLidar::LEGACY,
            UDPProfileIMU::LEGACY,
            HeaderType::STANDARD,
            static_cast<uint16_t>(frequency_of_lidar_mode(mode)),
            false};
}

bool SensorInfo::has_fields_equal(const SensorInfo& other) const {
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
        this->config == other.config && this->user_data == other.user_data &&
        this->zone_set == other.zone_set);
}

auto SensorInfo::w() const -> decltype(format.columns_per_frame) {
    return format.columns_per_frame;
}

auto SensorInfo::h() const -> decltype(format.pixels_per_column) {
    return format.pixels_per_column;
}

/* Default values */

SensorInfo default_sensor_info(LidarMode mode) {
    auto info = SensorInfo();
    info.sn = 0;
    info.fw_rev = "UNKNOWN";

    info.prod_line = "OS-1-64";

    info.format = default_data_format(mode);
    info.beam_azimuth_angles = GEN1_AZIMUTH_ANGLES;
    info.beam_altitude_angles = GEN1_ALTITUDE_ANGLES;
    info.lidar_origin_to_beam_origin_mm =
        default_lidar_origin_to_beam_origin(info.prod_line);
    info.beam_to_lidar_transform =
        default_beam_to_lidar_transform(info.prod_line);
    info.imu_to_sensor_transform = DEFAULT_IMU_TO_SENSOR_TRANSFORM;
    info.lidar_to_sensor_transform = DEFAULT_LIDAR_TO_SENSOR_TRANSFORM;
    info.extrinsic = mat4d::Identity();
    info.init_id = 0;
    info.build_date = "";
    info.image_rev = "";
    info.prod_pn = "";
    info.status = "";
    info.user_data = "";
    info.cal = default_calibration_status();
    info.config = SensorConfig{};
    info.config.lidar_mode = mode;
    info.config.udp_port_lidar = 0;
    info.config.udp_port_imu = 0;

    return info;
}

// clang-format off
extern const std::vector<double> GEN1_ALTITUDE_ANGLES = {
    16.611, 16.084, 15.557, 15.029, 14.502, 13.975, 13.447, 12.920,
    12.393, 11.865, 11.338, 10.811, 10.283, 9.756, 9.229, 8.701,
    8.174, 7.646, 7.119, 6.592, 6.064, 5.537, 5.010, 4.482,
    3.955, 3.428, 2.900, 2.373, 1.846, 1.318, 0.791, 0.264,
    -0.264, -0.791, -1.318, -1.846, -2.373, -2.900, -3.428, -3.955,
    -4.482, -5.010, -5.537, -6.064, -6.592, -7.119, -7.646, -8.174,
    -8.701, -9.229, -9.756, -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

extern const std::vector<double> GEN1_AZIMUTH_ANGLES = {
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

extern const mat4d DEFAULT_IMU_TO_SENSOR_TRANSFORM =
    (mat4d() << 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1)
        .finished();

extern const mat4d DEFAULT_LIDAR_TO_SENSOR_TRANSFORM =
    (mat4d() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1)
        .finished();

/* String conversion */

// bool represents whether it is an object (true) or just a member (false)
// NOTE: lidar_data_format and calibration_status should be objects but as they
// were introduced earlier, non-legacy formats for FW version do not include
// them
// TODO parse metadata by FW version specified ?
// clang-format off
const std::map<std::string, bool> NONLEGACY_METADATA_FIELDS = {
    {"sensor_info", true}, {"beam_intrinsics", true},
    {"imu_intrinsics", true}, {"lidar_intrinsics", true},
    {"config_params", true}, {"lidar_data_format", false},
    {"calibration_status", false}
};

// clang-format on

SensorInfo::SensorInfo() {
    // TODO - understand why this seg faults in CI when uncommented
    // logger().warn("Initializing SensorInfo without original metadata
    // string");
}

SensorInfo::SensorInfo(const std::string& metadata) {
    ValidatorIssues issues;

    if (!metadata.empty()) {
        parse_and_validate_metadata(metadata, *this, issues);
        if (!issues.critical.empty()) {
            std::stringstream error_string;
            error_string << "ERROR: Critical Metadata Issues Exist: "
                         << std::endl;
            for (const auto& it : issues.critical) {
                error_string << it.to_string() << std::endl;
            }
            throw std::runtime_error(error_string.str());
        }
    } else {
        throw std::runtime_error("ERROR: empty metadata passed in");
    }
}

std::string SensorInfo::to_json_string() const {
    jsoncons::json result;

    result["sensor_info"]["build_date"] = build_date;
    result["sensor_info"]["build_rev"] = fw_rev;
    result["sensor_info"]["image_rev"] = image_rev;
    result["sensor_info"]["initialization_id"] = init_id;
    result["sensor_info"]["prod_line"] = prod_line;
    result["sensor_info"]["prod_pn"] = prod_pn;
    result["sensor_info"]["prod_sn"] = std::to_string(sn);
    result["sensor_info"]["status"] = status;

    // data_format
    result["lidar_data_format"]["pixels_per_column"] = format.pixels_per_column;
    result["lidar_data_format"]["columns_per_packet"] =
        format.columns_per_packet;
    result["lidar_data_format"]["columns_per_frame"] = format.columns_per_frame;
    result["lidar_data_format"]["fps"] = format.fps;
    result["lidar_data_format"]["column_window"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["lidar_data_format"]["column_window"].emplace_back(
        format.column_window.first);
    result["lidar_data_format"]["column_window"].emplace_back(
        format.column_window.second);
    result["lidar_data_format"]["udp_profile_lidar"] =
        to_string(format.udp_profile_lidar);
    result["lidar_data_format"]["udp_profile_imu"] =
        to_string(format.udp_profile_imu);
    result["lidar_data_format"]["header_type"] = to_string(format.header_type);
    result["imu_data_format"]["imu_measurements_per_packet"] =
        format.imu_measurements_per_packet;
    result["imu_data_format"]["imu_packets_per_frame"] =
        format.imu_packets_per_frame;

    result["lidar_data_format"]["pixel_shift_by_row"] =
        jsoncons::json(jsoncons::json_array_arg);
    for (auto i : format.pixel_shift_by_row) {
        result["lidar_data_format"]["pixel_shift_by_row"].emplace_back(i);
    }

    // beam intrinsics
    //
    result["beam_intrinsics"] = jsoncons::json();
    result["beam_intrinsics"]["beam_to_lidar_transform"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["beam_intrinsics"]["beam_to_lidar_transform"] =
        mat4d_to_array(beam_to_lidar_transform);
    result["beam_intrinsics"]["lidar_origin_to_beam_origin_mm"] =
        lidar_origin_to_beam_origin_mm;

    result["beam_intrinsics"]["beam_azimuth_angles"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["beam_intrinsics"]["beam_altitude_angles"] =
        jsoncons::json(jsoncons::json_array_arg);
    if (beam_azimuth_angles.size() == format.pixels_per_column) {
        // OS sensor path
        for (auto angle : beam_azimuth_angles) {
            result["beam_intrinsics"]["beam_azimuth_angles"].emplace_back(
                angle);
        }
        for (auto angle : beam_altitude_angles) {
            result["beam_intrinsics"]["beam_altitude_angles"].emplace_back(
                angle);
        }
    } else {
        // DF sensor path
        int j = 0;
        for (size_t i = 0; i < beam_azimuth_angles.size(); i++) {
            int col_index_within_row = i % format.columns_per_frame;
            if (col_index_within_row == 0) {
                result["beam_intrinsics"]["beam_azimuth_angles"].emplace_back(
                    jsoncons::json(jsoncons::json_array_arg));
                j++;
            }
            result["beam_intrinsics"]["beam_azimuth_angles"][j - 1]
                .emplace_back(beam_azimuth_angles[i]);
        }

        j = 0;
        for (size_t i = 0; i < beam_altitude_angles.size(); i++) {
            int col_index_within_row = i % format.columns_per_frame;
            if (col_index_within_row == 0) {
                result["beam_intrinsics"]["beam_altitude_angles"].emplace_back(
                    jsoncons::json(jsoncons::json_array_arg));
                j++;
            }
            result["beam_intrinsics"]["beam_altitude_angles"][j - 1]
                .emplace_back(beam_altitude_angles[i]);
        }
    }
    result["calibration_status"] = cal_to_json(cal);

    result["config_params"] = config_to_json(config);

    result["user_data"] = user_data;

    if (zone_set) {
        auto blob = zone_set->to_zip_blob(ZoneSetOutputFilter::STL_AND_ZRB);
        std::string out;
        jsoncons::encode_base64(blob.begin(), blob.end(), out);
        result["zone_set"] = out;
    }

    result["imu_intrinsics"] = jsoncons::json();
    result["imu_intrinsics"]["imu_to_sensor_transform"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["imu_intrinsics"]["imu_to_sensor_transform"] =
        mat4d_to_array(imu_to_sensor_transform);

    result["lidar_intrinsics"] = jsoncons::json();
    result["lidar_intrinsics"]["lidar_to_sensor_transform"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["lidar_intrinsics"]["lidar_to_sensor_transform"] =
        mat4d_to_array(lidar_to_sensor_transform);

    result["ouster-sdk"] = jsoncons::json();
    result["ouster-sdk"]["extrinsic"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["ouster-sdk"]["extrinsic"] = mat4d_to_array(extrinsic);

    result["ouster-sdk"]["output_source"] = "sensor_info_to_string";
    result["ouster-sdk"]["client_version"] = client_version();
    std::string out;
    result.dump(out);
    return out;
}

Version SensorInfo::get_version() const {
    return version_from_string(image_rev);
}

ProductInfo SensorInfo::get_product_info() const {
    return ProductInfo::create_product_info(prod_line);
}

int SensorInfo::num_returns() const {
    if (format.udp_profile_lidar ==
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL ||
        format.udp_profile_lidar == UDPProfileLidar::RNG15_RFL8_NIR8_DUAL ||
        format.udp_profile_lidar ==
            UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL) {
        return 2;
    }
    return 1;
}

SensorInfo metadata_from_json(const std::string& json_file,
                              bool /*skip_beam_validation*/) {
    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(json_file);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs) {
        std::stringstream string_stream;
        string_stream << "Failed to read metadata file: " << json_file;
        throw std::runtime_error{string_stream.str()};
    }

    return SensorInfo(buf.str());
}

std::string to_string(const SensorInfo& info) { return info.to_json_string(); }

Version firmware_version_from_metadata(const std::string& metadata) {
    if (metadata.empty()) {
        throw std::invalid_argument(
            "firmware_version_from_metadata metadata empty!");
    }

    jsoncons::json value_array = jsoncons::jsonpath::json_query(
        jsoncons::json::parse(metadata), "$.sensor_info.image_rev");
    if (value_array.size() == 1) {
        return version_from_string(value_array[0].as<std::string>());
    } else {
        throw std::invalid_argument(
            "firmware_version_from_metadata failed to deduce version info from "
            "metadata!");
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
