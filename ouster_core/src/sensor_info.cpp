/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/sensor_info.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <map>
#include <mutex>
#include <nonstd/optional.hpp>
#include <ostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/data_format.h"
#include "ouster/core/defaults.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/impl/table.h"
#include "ouster/core/metadata.h"
#include "ouster/core/sensor_config.h"
#include "ouster/core/version.h"
#include "ouster/core/xyzlut.h"

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;
using ouster::impl::lookup;
using ouster::impl::rlookup;
using ouster::impl::Table;
using std::stoul;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {
struct SensorInfoCache {
    std::mutex mutex;
    std::unique_ptr<XYZLutT<float>> lutf;
    std::unique_ptr<XYZLutT<double>> lutd;
};

OUSTER_API_VAR Table<ShotLimitingStatus, const char*, 10> shot_limiting_status_strings{{
    {ShotLimitingStatus::NORMAL, "NORMAL"},
    {ShotLimitingStatus::IMMINENT, "IMMINENT"},
    {ShotLimitingStatus::REDUCTION_0_10, "REDUCTION_0_10"},
    {ShotLimitingStatus::REDUCTION_10_20, "REDUCTION_10_20"},
    {ShotLimitingStatus::REDUCTION_20_30, "REDUCTION_20_30"},
    {ShotLimitingStatus::REDUCTION_30_40, "REDUCTION_30_40"},
    {ShotLimitingStatus::REDUCTION_40_50, "REDUCTION_40_50"},
    {ShotLimitingStatus::REDUCTION_50_60, "REDUCTION_50_60"},
    {ShotLimitingStatus::REDUCTION_60_70, "REDUCTION_60_70"},
    {ShotLimitingStatus::REDUCTION_70_75, "REDUCTION_70_75"},
}};

OUSTER_API_VAR Table<ThermalShutdownStatus, const char*, 2> thermal_shutdown_status_strings{{
    {ThermalShutdownStatus::NORMAL, "NORMAL"},
    {ThermalShutdownStatus::IMMINENT, "IMMINENT"},
}};

}  // namespace impl

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[out] sensor_info The sensor_info to populate.
 * @param[out] issues The issues that occured during parsing.
 * @return if there are any critical issues or not.
 */
extern bool parse_and_validate_metadata(const std::string& json_data, SensorInfo& sensor_info,
                                        ValidatorIssues& issues);

/* version field required by ouster studio */
enum ConfigurationVersion { FW_2_0 = 3, FW_2_2 = 4 };

double default_lidar_origin_to_beam_origin(const std::string& prod_line) {
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

mat4d default_beam_to_lidar_transform(const std::string& prod_line) {
    mat4d beam_to_lidar_transform = mat4d::Identity();
    beam_to_lidar_transform(0, 3) = default_lidar_origin_to_beam_origin(prod_line);
    return beam_to_lidar_transform;
}

CalibrationStatus default_calibration_status() {
    return CalibrationStatus{};
}

jsoncons::json cal_to_json(const CalibrationStatus& cal) {
    jsoncons::json root;

    if (cal.reflectivity_status) {
        root["reflectivity"]["valid"] = cal.reflectivity_status.value();
    }
    if (cal.reflectivity_timestamp) {
        root["reflectivity"]["timestamp"] = cal.reflectivity_timestamp.value();
    }

    return root;
}
extern jsoncons::json config_to_json(const SensorConfig& config);
extern DataFormat default_data_format(LidarMode mode);

/* Equality operators and functions */

bool operator==(const SensorInfo& lhs, const SensorInfo& rhs) {
    return lhs.has_fields_equal(rhs);
}

bool operator!=(const SensorInfo& lhs, const SensorInfo& rhs) {
    return !(lhs == rhs);
}

bool SensorInfo::has_fields_equal(const SensorInfo& other) const {
    return (this->sn == other.sn && this->fw_rev == other.fw_rev &&
            this->prod_line == other.prod_line && this->format == other.format &&
            this->beam_azimuth_angles == other.beam_azimuth_angles &&
            this->beam_altitude_angles == other.beam_altitude_angles &&
            this->lidar_origin_to_beam_origin_mm == other.lidar_origin_to_beam_origin_mm &&
            this->beam_to_lidar_transform == other.beam_to_lidar_transform &&
            this->imu_to_sensor_transform == other.imu_to_sensor_transform &&
            this->lidar_to_sensor_transform == other.lidar_to_sensor_transform &&
            this->sensor_to_body == other.sensor_to_body && this->init_id == other.init_id &&
            this->build_date == other.build_date && this->image_rev == other.image_rev &&
            this->prod_pn == other.prod_pn && this->status == other.status &&
            this->cal == other.cal && this->config == other.config &&
            this->user_data == other.user_data && this->client_version == other.client_version &&
            this->zone_set == other.zone_set);
}

auto SensorInfo::w() const -> decltype(format.columns_per_frame) {
    return format.columns_per_frame;
}

auto SensorInfo::h() const -> decltype(format.pixels_per_column) {
    return format.pixels_per_column;
}

/* Default values */

std::shared_ptr<SensorInfo> SensorInfo::from_default(LidarMode mode) {
    auto info = std::make_shared<SensorInfo>();
    info->sn = 0;
    info->fw_rev = "UNKNOWN";

    info->prod_line = "OS-1-64";

    info->format = default_data_format(mode);
    info->beam_azimuth_angles = GEN1_AZIMUTH_ANGLES;
    info->beam_altitude_angles = GEN1_ALTITUDE_ANGLES;
    info->lidar_origin_to_beam_origin_mm = default_lidar_origin_to_beam_origin(info->prod_line);
    info->beam_to_lidar_transform = default_beam_to_lidar_transform(info->prod_line);
    info->imu_to_sensor_transform = DEFAULT_IMU_TO_SENSOR_TRANSFORM;
    info->lidar_to_sensor_transform = DEFAULT_LIDAR_TO_SENSOR_TRANSFORM;
    info->sensor_to_body = mat4d::Identity();
    info->init_id = 0;
    info->build_date = "";
    info->image_rev = "";
    info->prod_pn = "";
    info->status = "";
    info->user_data = "";
    info->cal = default_calibration_status();
    info->config = SensorConfig{};
    info->config.lidar_mode = mode;
    info->config.udp_port_lidar = 0;
    info->config.udp_port_imu = 0;

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
    (mat4d() << 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1).finished();

extern const mat4d DEFAULT_LIDAR_TO_SENSOR_TRANSFORM =
    (mat4d() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1).finished();

/* String conversion */

// TODO - understand why this seg faults in CI when uncommented
// inside of sensor info constructor
//
// logger().warn("Initializing SensorInfo without original metadata
// string");
SensorInfo::SensorInfo() = default;

SensorInfo::SensorInfo(const std::string& metadata) {
    ValidatorIssues issues;

    if (!metadata.empty()) {
        parse_and_validate_metadata(metadata, *this, issues);
        if (!issues.critical.empty()) {
            std::stringstream error_string;
            error_string << "ERROR: Critical Metadata Issues Exist: " << std::endl;
            for (const auto& it : issues.critical) {
                error_string << it.to_string() << std::endl;
            }
            throw std::runtime_error(error_string.str());
        }
    } else {
        throw std::runtime_error("ERROR: empty metadata passed in");
    }
}

SensorInfo::SensorInfo(const SensorInfo&) = default;

SensorInfo::SensorInfo(SensorInfo&&) = default;

SensorInfo& SensorInfo::operator=(const SensorInfo&) = default;

SensorInfo::~SensorInfo() = default;

template <>
const XYZLutT<float>& SensorInfo::xyzlut() const {
    std::unique_lock<std::mutex> lock(cache_->mutex);
    if (!cache_->lutf) {
        cache_->lutf = std::make_unique<XYZLutT<float>>(*this, true);
    }
    return *cache_->lutf;
}

template <>
const XYZLutT<double>& SensorInfo::xyzlut() const {
    std::unique_lock<std::mutex> lock(cache_->mutex);
    if (!cache_->lutd) {
        cache_->lutd = std::make_unique<XYZLutT<double>>(*this, true);
    }
    return *cache_->lutd;
}

void SensorInfo::clear_cache() {
    std::unique_lock<std::mutex> lock(cache_->mutex);
    cache_->lutf = {};
    cache_->lutd = {};
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
    result["lidar_data_format"]["columns_per_packet"] = format.columns_per_packet;
    result["lidar_data_format"]["columns_per_frame"] = format.columns_per_frame;
    result["lidar_data_format"]["fps"] = format.fps;
    result["lidar_data_format"]["column_window"] = jsoncons::json(jsoncons::json_array_arg);
    result["lidar_data_format"]["column_window"].emplace_back(format.column_window.first);
    result["lidar_data_format"]["column_window"].emplace_back(format.column_window.second);
    result["lidar_data_format"]["udp_profile_lidar"] = to_string(format.udp_profile_lidar);
    result["lidar_data_format"]["udp_profile_imu"] = to_string(format.udp_profile_imu);
    result["lidar_data_format"]["header_type"] = to_string(format.header_type);
    result["imu_data_format"]["imu_measurements_per_packet"] = format.imu_measurements_per_packet;
    result["imu_data_format"]["imu_packets_per_frame"] = format.imu_packets_per_frame;

    result["lidar_data_format"]["pixel_shift_by_row"] = jsoncons::json(jsoncons::json_array_arg);
    for (auto i : format.pixel_shift_by_row) {
        result["lidar_data_format"]["pixel_shift_by_row"].emplace_back(i);
    }

    // beam intrinsics
    //
    result["beam_intrinsics"] = jsoncons::json();
    result["beam_intrinsics"]["beam_to_lidar_transform"] = jsoncons::json(jsoncons::json_array_arg);
    result["beam_intrinsics"]["beam_to_lidar_transform"] = mat4d_to_array(beam_to_lidar_transform);
    result["beam_intrinsics"]["lidar_origin_to_beam_origin_mm"] = lidar_origin_to_beam_origin_mm;

    result["beam_intrinsics"]["beam_azimuth_angles"] = jsoncons::json(jsoncons::json_array_arg);
    result["beam_intrinsics"]["beam_altitude_angles"] = jsoncons::json(jsoncons::json_array_arg);
    if (beam_azimuth_angles.size() == format.pixels_per_column) {
        // OS sensor path
        for (auto angle : beam_azimuth_angles) {
            result["beam_intrinsics"]["beam_azimuth_angles"].emplace_back(angle);
        }
        for (auto angle : beam_altitude_angles) {
            result["beam_intrinsics"]["beam_altitude_angles"].emplace_back(angle);
        }
    } else {
        // DF sensor path
        int j = 0;
        for (size_t i = 0; i < beam_azimuth_angles.size(); i++) {
            int col_index_within_row = static_cast<int>(i % format.columns_per_frame);
            if (col_index_within_row == 0) {
                result["beam_intrinsics"]["beam_azimuth_angles"].emplace_back(
                    jsoncons::json(jsoncons::json_array_arg));
                j++;
            }
            result["beam_intrinsics"]["beam_azimuth_angles"][j - 1].emplace_back(
                beam_azimuth_angles[i]);
        }

        j = 0;
        for (size_t i = 0; i < beam_altitude_angles.size(); i++) {
            int col_index_within_row = static_cast<int>(i % format.columns_per_frame);
            if (col_index_within_row == 0) {
                result["beam_intrinsics"]["beam_altitude_angles"].emplace_back(
                    jsoncons::json(jsoncons::json_array_arg));
                j++;
            }
            result["beam_intrinsics"]["beam_altitude_angles"][j - 1].emplace_back(
                beam_altitude_angles[i]);
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
    result["imu_intrinsics"]["imu_to_sensor_transform"] = jsoncons::json(jsoncons::json_array_arg);
    result["imu_intrinsics"]["imu_to_sensor_transform"] = mat4d_to_array(imu_to_sensor_transform);

    result["lidar_intrinsics"] = jsoncons::json();
    result["lidar_intrinsics"]["lidar_to_sensor_transform"] =
        jsoncons::json(jsoncons::json_array_arg);
    result["lidar_intrinsics"]["lidar_to_sensor_transform"] =
        mat4d_to_array(lidar_to_sensor_transform);

    result["ouster-sdk"] = jsoncons::json();
    result["ouster-sdk"]["extrinsic"] = jsoncons::json(jsoncons::json_array_arg);
    result["ouster-sdk"]["extrinsic"] = mat4d_to_array(sensor_to_body);

    result["ouster-sdk"]["output_source"] = "sensor_info_to_string";
    result["ouster-sdk"]["client_version"] = ouster::sdk::core::client_version();
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
    if (format.udp_profile_lidar == UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL ||
        format.udp_profile_lidar == UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL ||
        format.udp_profile_lidar == UDPProfileLidar::RNG15_RFL8_NIR8_DUAL ||
        format.udp_profile_lidar == UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL ||
        format.udp_profile_lidar == UDPProfileLidar::RNG19_RFL8_SIG16_ZONE16_DUAL) {
        return 2;
    }
    return 1;
}

SensorInfo metadata_from_json(const std::string& json_file, bool /*skip_beam_validation*/) {
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

bool operator==(const CalibrationStatus& lhs, const CalibrationStatus& rhs) {
    return (lhs.reflectivity_status == rhs.reflectivity_status &&
            lhs.reflectivity_timestamp == rhs.reflectivity_timestamp);
}

bool operator!=(const CalibrationStatus& lhs, const CalibrationStatus& rhs) {
    return !(lhs == rhs);
}

std::string to_string(const CalibrationStatus& cal) {
    auto root = cal_to_json(cal);
    std::string out;
    root.dump(out);
    return out;
}

ProductInfo ProductInfo::create_product_info(const std::string& product_info_string) {
    std::regex product_regex(
        R"(^(\w+)-(\d+|DOME)?(?:-(MAX))?(?:-(\d+))?(?:-(RGB))?(?:-((?!SR)\w+))?-?(SR)?)");
    std::smatch matches;
    if (!product_info_string.empty()) {
        if (regex_search(product_info_string, matches, product_regex)) {
            std::string form_factor = matches.str(1) + matches.str(2) + matches.str(3);
            bool short_range = (!matches.str(7).empty());
            auto beam_config = matches.str(6);
            if (beam_config.empty()) {
                beam_config = "U";
            }
            bool rgb = matches.str(5) == "RGB";
            int beam_count;
            try {
                beam_count = stoi(matches.str(4));
            } catch (const std::exception& e) {
                beam_count = 0;
            }

            return ProductInfo(product_info_string, form_factor, short_range, beam_config,
                               beam_count, rgb);
        } else {
            throw std::runtime_error("Product Info \"" + product_info_string +
                                     "\" is not a recognized product info");
        }
    }
    return ProductInfo();
}

ProductInfo::ProductInfo() : ProductInfo("", "", false, "", 0){};

ProductInfo::ProductInfo(std::string product_info_string, std::string form_factor, bool short_range,
                         std::string beam_config, int beam_count, bool rgb)
    : full_product_info(std::move(product_info_string)),
      form_factor(std::move(form_factor)),
      short_range(short_range),
      beam_config(std::move(beam_config)),
      beam_count(beam_count),
      rgb(rgb) {}

bool operator==(const ProductInfo& lhs, const ProductInfo& rhs) {
    return lhs.full_product_info == rhs.full_product_info && lhs.form_factor == rhs.form_factor &&
           lhs.short_range == rhs.short_range && lhs.beam_config == rhs.beam_config &&
           lhs.beam_count == rhs.beam_count && lhs.rgb == rhs.rgb;
}

bool operator!=(const ProductInfo& lhs, const ProductInfo& rhs) {
    return !(lhs == rhs);
}

std::string to_string(const ProductInfo& info) {
    std::stringstream output;
    output << "Product Info: " << std::endl;
    output << "\tFull Product Info: \"" << info.full_product_info << "\"" << std::endl;
    output << "\tForm Factor: \"" << info.form_factor << "\"" << std::endl;
    output << "\tShort Range: \"" << info.short_range << "\"" << std::endl;
    output << "\tBeam Config: \"" << info.beam_config << "\"" << std::endl;
    output << "\tBeam Count: \"" << info.beam_count << "\"" << std::endl;
    output << "\tRGB: \"" << info.rgb << "\"" << std::endl;
    return output.str();
}

std::string to_string(ShotLimitingStatus shot_limiting_status) {
    auto res = lookup(ouster::sdk::core::impl::shot_limiting_status_strings, shot_limiting_status);
    return res ? res.value() : "UNKNOWN";
}

std::string to_string(ThermalShutdownStatus thermal_shutdown_status) {
    auto res =
        lookup(ouster::sdk::core::impl::thermal_shutdown_status_strings, thermal_shutdown_status);
    return res ? res.value() : "UNKNOWN";
}

std::string client_version() {
    return std::string("ouster_core ").append(ouster::sdk::SDK_VERSION);
}

Version version_from_string(const std::string& version_string) {
    auto rgx =
        std::regex(R"((([\w\d]*)-([\w\d]*)-)?v?(\d*)\.(\d*)\.(\d*)-?([\d\w.]*)?\+?([\d\w.]*)?)");
    std::smatch matches;
    std::regex_search(version_string, matches, rgx);

    if (matches.size() < 9) {
        return INVALID_VERSION;
    }

    try {
        Version version_val;
        version_val.major = static_cast<uint16_t>(stoul(matches[4]));
        version_val.minor = static_cast<uint16_t>(stoul(matches[5]));
        version_val.patch = static_cast<uint16_t>(stoul(matches[6]));
        version_val.stage = matches[2];
        version_val.machine = matches[3];
        version_val.prerelease = matches[7];
        version_val.build = matches[8];
        return version_val;
    } catch (const std::exception&) {
        return INVALID_VERSION;
    }
}

const Version& invalid_version = INVALID_VERSION;

}  // namespace core
}  // namespace sdk
}  // namespace ouster
