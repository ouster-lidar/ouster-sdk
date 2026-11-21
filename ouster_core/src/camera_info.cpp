/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/camera_info.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <jsoncons/json.hpp>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "metadata_json.h"
#include "ouster/core/camera_model.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/typedefs.h"

namespace ouster {
namespace sdk {
namespace core {

namespace {
float read_float_or_default(const jsoncons::json& obj, const std::string& key) {
    if (!obj.contains(key)) {
        return 0.0f;
    }
    const auto& value = obj.at(key);
    if (!value.is_number()) {
        return 0.0f;
    }
    return value.as<float>();
}

uint64_t read_uint64_or_default(const jsoncons::json& obj, const std::string& key) {
    if (!obj.contains(key)) {
        return 0;
    }
    const auto& value = obj.at(key);
    if (value.is_number()) {
        return value.as<uint64_t>();
    }
    if (value.is_string()) {
        try {
            return std::stoull(value.as<std::string>());
        } catch (const std::exception&) {
            return 0;
        }
    }
    return 0;
}

// Read a 4x4 row-major transform from a 16-element JSON array. Returns the
// zero matrix when the key is absent or malformed.
ouster::sdk::core::mat4d read_mat4d_or_default(const jsoncons::json& obj, const std::string& key) {
    if (!obj.contains(key)) {
        return ouster::sdk::core::mat4d::Zero();
    }
    const auto& value = obj.at(key);
    if (!value.is_array() || value.size() != ouster::sdk::core::mat4d::SizeAtCompileTime) {
        return ouster::sdk::core::mat4d::Zero();
    }
    std::array<double, ouster::sdk::core::mat4d::SizeAtCompileTime> arr{};
    size_t i = 0;
    for (const auto& element : value.array_range()) {
        arr[i++] = element.is_number() ? element.as<double>() : 0.0;
    }
    return ouster::sdk::core::mat4d_from_array(arr);
}

}  // namespace

CameraInfo::~CameraInfo() = default;

CameraInfo::CameraInfo() {
    // A camera's IMU (when present, e.g. a ZED) is treated as coincident with
    // the camera/sensor frame unless calibration says otherwise, so default the
    // inherited imu_to_sensor_transform to identity rather than the base zero.
    imu_to_sensor_transform = ouster::sdk::core::mat4d::Identity();
}

CameraInfo::CameraInfo(const CameraInfo& other) = default;

// Defaulted out-of-line; the SensorInfo base's move is not noexcept, so the compiler
// must deduce the spec. See camera_info.h.
// NOLINTBEGIN(cppcoreguidelines-noexcept-move-operations)
CameraInfo::CameraInfo(CameraInfo&& other) = default;

CameraInfo& CameraInfo::operator=(CameraInfo&& other) = default;
// NOLINTEND(cppcoreguidelines-noexcept-move-operations)

CameraInfo& CameraInfo::operator=(const CameraInfo& other) {
    if (this != &other) {
        ouster::sdk::core::SensorInfo::operator=(other);
        format = other.format;
        model = other.model;
    }
    return *this;
}

namespace {

// Populate a default-constructed CameraInfo from parsed camera metadata.
void init_camera_info_from_json(const jsoncons::json& root, CameraInfo& info) {
    // Camera metadata must wrap its fields in a top-level "camera_info" object:
    // { "camera_info": { "model": {...}, ... } }.
    const jsoncons::json* camera_info_section = nullptr;
    if (root.contains("camera_info") && root.at("camera_info").is_object()) {
        camera_info_section = &root.at("camera_info");
    } else {
        // No "camera_info" object: leave the model absent.
        return;
    }

    info.sn = read_uint64_or_default(*camera_info_section, "sn");
    // Preserve the identity defaults when a key is absent;
    // only override when the metadata actually provides the extrinsic.
    if (camera_info_section->contains("sensor_to_body")) {
        info.sensor_to_body = read_mat4d_or_default(*camera_info_section, "sensor_to_body");
    }
    if (camera_info_section->contains("imu_to_sensor_transform")) {
        info.imu_to_sensor_transform =
            read_mat4d_or_default(*camera_info_section, "imu_to_sensor_transform");
    }
    const jsoncons::json* format_section = nullptr;
    if (camera_info_section->contains("format") && camera_info_section->at("format").is_object()) {
        format_section = &camera_info_section->at("format");
    } else {
        throw std::invalid_argument("ERROR: camera metadata must contain a 'format' object");
    }

    info.format.columns_per_frame =
        static_cast<uint32_t>(read_uint64_or_default(*format_section, "columns_per_frame"));
    info.format.pixels_per_column =
        static_cast<uint32_t>(read_uint64_or_default(*format_section, "pixels_per_column"));
    info.format.fps = static_cast<uint16_t>(read_uint64_or_default(*format_section, "fps"));

    // An absent "model" is meaningful: the camera is uncalibrated, so its
    // imagery can be viewed but not projected.
    if (!camera_info_section->contains("model")) {
        return;
    }
    const auto& model_json = camera_info_section->at("model");
    if (model_json.is_null()) {
        return;
    }
    if (!model_json.is_object()) {
        throw std::runtime_error("ERROR: 'model' in camera metadata must be an object");
    }
    // The focal length is validated by the constructor; surface its complaint
    // as a metadata parse error like the checks above.
    try {
        info.model = CameraModel(
            read_float_or_default(model_json, "fx"), read_float_or_default(model_json, "fy"),
            read_float_or_default(model_json, "cx"), read_float_or_default(model_json, "cy"));
    } catch (const std::invalid_argument& e) {
        std::stringstream error_stream;
        error_stream << "ERROR: bad 'model' in camera metadata: " << e.what();
        throw std::runtime_error(error_stream.str());
    }
}

}  // namespace

CameraInfo::CameraInfo(const std::string& json_string) : CameraInfo() {
    if (json_string.empty()) {
        throw std::runtime_error("ERROR: empty camera metadata passed in");
    }

    jsoncons::json root;
    try {
        root = jsoncons::json::parse(json_string);
    } catch (const std::exception& e) {
        std::stringstream error_stream;
        error_stream << "ERROR: failed to parse camera metadata JSON: " << e.what();
        throw std::runtime_error(error_stream.str());
    }
    init_camera_info_from_json(root, *this);
}

std::string CameraInfo::to_json_string() const {
    jsoncons::json sensor_to_body_out(jsoncons::json_array_arg);
    for (double element : ouster::sdk::core::mat4d_to_array(sensor_to_body)) {
        sensor_to_body_out.push_back(element);
    }

    jsoncons::json imu_to_sensor_out(jsoncons::json_array_arg);
    for (double element : ouster::sdk::core::mat4d_to_array(imu_to_sensor_transform)) {
        imu_to_sensor_out.push_back(element);
    }

    jsoncons::json format_section;
    format_section["pixels_per_column"] = format.pixels_per_column;
    format_section["columns_per_frame"] = format.columns_per_frame;
    format_section["fps"] = format.fps;
    jsoncons::json camera_info_section;
    camera_info_section["sn"] = sn;
    camera_info_section["format"] = format_section;
    camera_info_section["sensor_to_body"] = std::move(sensor_to_body_out);
    camera_info_section["imu_to_sensor_transform"] = std::move(imu_to_sensor_out);
    // Omit "model" entirely when absent, so an uncalibrated camera round-trips
    // as uncalibrated rather than as a zero-valued model.
    if (model) {
        jsoncons::json model_out;
        model_out["fx"] = model->fx;
        model_out["fy"] = model->fy;
        model_out["cx"] = model->cx;
        model_out["cy"] = model->cy;
        camera_info_section["model"] = std::move(model_out);
    }

    jsoncons::json result;
    result["camera_info"] = std::move(camera_info_section);

    std::string out;
    result.dump(out);
    return out;
}

std::shared_ptr<SensorInfo> SensorInfo::from_json_string(const std::string& json_string) {
    // Infer the sensor kind from the JSON schema. Camera and lidar metadata use
    // mutually-exclusive discriminator keys, so a top-level check is
    // unambiguous: camera JSON carries a `camera_info` object; lidar JSON does
    // not. The parsed document is reused to populate the result.
    jsoncons::json root;
    try {
        root = jsoncons::json::parse(json_string);
    } catch (const std::exception&) {
        // Let the LidarInfo parser report a precise error.
        return std::make_shared<LidarInfo>(json_string);
    }
    if (root.contains("camera_info") && root.at("camera_info").is_object()) {
        auto camera = std::make_shared<CameraInfo>();
        init_camera_info_from_json(root, *camera);
        return camera;
    }
    auto lidar = std::make_shared<LidarInfo>();
    init_lidar_info_from_json(root, *lidar);
    return lidar;
}

CameraInfo camera_metadata_from_json(const std::string& json_file) {
    std::ifstream ifs(json_file);
    if (!ifs) {
        std::stringstream error_stream;
        error_stream << "Failed to read camera metadata file: " << json_file;
        throw std::runtime_error(error_stream.str());
    }
    std::stringstream buf;
    buf << ifs.rdbuf();
    return CameraInfo(buf.str());
}

size_t CameraInfo::w() const {
    return format.columns_per_frame;
}

size_t CameraInfo::h() const {
    return format.pixels_per_column;
}

bool operator==(const CameraInfo& lhs, const CameraInfo& rhs) {
    return lhs.sn == rhs.sn && lhs.model == rhs.model && lhs.sensor_to_body == rhs.sensor_to_body &&
           lhs.imu_to_sensor_transform == rhs.imu_to_sensor_transform && lhs.format == rhs.format;
}

bool operator!=(const CameraInfo& lhs, const CameraInfo& rhs) {
    return !(lhs == rhs);
}

int CameraInfo::fps() const {
    return format.fps;
}

bool CameraInfo::could_be_synchronized() const {
    return false;
}

double CameraInfo::latency() const {
    return 0.0;
}

std::shared_ptr<SensorInfo> CameraInfo::clone() const {
    return std::make_shared<CameraInfo>(*this);
}

bool operator==(const CameraDataFormat& lhs, const CameraDataFormat& rhs) {
    return lhs.pixels_per_column == rhs.pixels_per_column &&
           lhs.columns_per_frame == rhs.columns_per_frame && lhs.fps == rhs.fps;
}

bool operator!=(const CameraDataFormat& lhs, const CameraDataFormat& rhs) {
    return !(lhs == rhs);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
