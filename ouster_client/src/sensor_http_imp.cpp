#include "sensor_http_imp.h"

#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>

#include "curl_client.h"
#include "ouster/types.h"

using ouster::sensor::util::UserDataAndPolicy;

using namespace ouster::sensor::impl;

SensorHttpImp::SensorHttpImp(const std::string& hostname)
    : http_client(std::make_unique<CurlClient>(hostname)) {}

SensorHttpImp::~SensorHttpImp() = default;

std::string SensorHttpImp::metadata(int timeout_sec) const {
    return get("api/v1/sensor/metadata", timeout_sec);
}

std::string SensorHttpImp::sensor_info(int timeout_sec) const {
    return get("api/v1/sensor/metadata/sensor_info", timeout_sec);
}

std::string SensorHttpImp::get_config_params(bool active,
                                             int timeout_sec) const {
    auto config_type = active ? "active" : "staged";
    return get(
        std::string("api/v1/sensor/cmd/get_config_param?args=") + config_type,
        timeout_sec);
}

void SensorHttpImp::set_config_param(const std::string& key,
                                     const std::string& value,
                                     int timeout_sec) const {
    auto encoded_value = http_client->encode(value);  // encode config params
    auto url =
        "api/v1/sensor/cmd/set_config_param?args=" + key + "+" + encoded_value;
    execute(url, "\"set_config_param\"", timeout_sec);
}

std::string SensorHttpImp::active_config_params(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_config_param?args=active", timeout_sec);
}

std::string SensorHttpImp::staged_config_params(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_config_param?args=staged", timeout_sec);
}

void SensorHttpImp::set_udp_dest_auto(int timeout_sec) const {
    execute("api/v1/sensor/cmd/set_udp_dest_auto", "{}", timeout_sec);
}

std::string SensorHttpImp::beam_intrinsics(int timeout_sec) const {
    return get("api/v1/sensor/metadata/beam_intrinsics", timeout_sec);
}

std::string SensorHttpImp::imu_intrinsics(int timeout_sec) const {
    return get("api/v1/sensor/metadata/imu_intrinsics", timeout_sec);
}

std::string SensorHttpImp::lidar_intrinsics(int timeout_sec) const {
    return get("api/v1/sensor/metadata/lidar_intrinsics", timeout_sec);
}

std::string SensorHttpImp::lidar_data_format(int timeout_sec) const {
    return get("api/v1/sensor/metadata/lidar_data_format", timeout_sec);
}

std::string SensorHttpImp::calibration_status(int timeout_sec) const {
    return get("api/v1/sensor/metadata/calibration_status", timeout_sec);
}

std::string SensorHttpImp::network(int timeout_sec) const {
    return get("api/v1/system/network", timeout_sec);
}

// reinitialize to activate new settings
void SensorHttpImp::reinitialize(int timeout_sec) const {
    execute("api/v1/sensor/cmd/reinitialize", "{}", timeout_sec);
}

void SensorHttpImp::save_config_params(int timeout_sec) const {
    execute("api/v1/sensor/cmd/save_config_params", "{}", timeout_sec);
}

std::string SensorHttpImp::get_user_data(int timeout_sec) const {
    // the API returns a jsonified string (quoted and escaped), parse it and
    // return the actual string value to the user
    return jsoncons::json::parse(get("api/v1/user/data", timeout_sec))
        .as<std::string>();
}

UserDataAndPolicy SensorHttpImp::get_user_data_and_policy(
    int timeout_sec) const {
    std::string url = "api/v1/user/data?include_metadata=true";
    auto json_data = get(url, timeout_sec);
    jsoncons::json root;

    try {
        root = jsoncons::json::parse(json_data);
        return {root["policy"].as<std::string>() != "clear_on_config_delete",
                root["value"].as<std::string>()};
    } catch (const jsoncons::ser_error& e) {
        throw std::runtime_error("Http response parse failed! url: " + url +
                                 " Error: " + e.what());
    }
}

void SensorHttpImp::set_user_data(const std::string& data,
                                  bool keep_on_config_delete,
                                  int timeout_sec) const {
    std::string json;
    auto temp_data = jsoncons::json(data);
    temp_data.dump(json);
    http_client->put(
        std::string("api/v1/user/data") +
            (keep_on_config_delete ? "?policy=keep_on_config_delete" : ""),
        json, timeout_sec);
}

void SensorHttpImp::delete_user_data(int timeout_sec) const {
    http_client->del("api/v1/user/data", timeout_sec);
}

std::string SensorHttpImp::get(const std::string& url, int timeout_sec) const {
    return http_client->get(url, timeout_sec);
}

void SensorHttpImp::execute(const std::string& url,
                            const std::string& validation,
                            int timeout_sec) const {
    auto result = get(url, timeout_sec);
    if (result != validation)
        throw std::runtime_error("SensorHttpImp::execute failed! url: " + url +
                                 " returned [" + result + "], expected [" +
                                 validation + "]");
}

void SensorHttpImp::set_static_ip(const std::string& ip_address,
                                  int timeout_sec) const {
    std::string json;
    auto temp_data = jsoncons::json(ip_address);
    temp_data.dump(json);
    http_client->put("api/v1/system/network/ipv4/override", json, timeout_sec);
}

void SensorHttpImp::delete_static_ip(int timeout_sec) const {
    http_client->del("api/v1/system/network/ipv4/override", timeout_sec);
}

std::vector<uint8_t> SensorHttpImp::diagnostics_dump(int timeout_sec) const {
    std::string str = http_client->get("/api/v1/diagnostics/dump", timeout_sec);
    return std::vector<uint8_t>(str.begin(), str.end());
}

SensorHttpImp_2_2::SensorHttpImp_2_2(const std::string& hostname)
    : SensorHttpImp_2_4_or_3(hostname) {}

void SensorHttpImp_2_2::set_udp_dest_auto(int timeout_sec) const {
    return execute("api/v1/sensor/cmd/set_udp_dest_auto",
                   "\"set_config_param\"", timeout_sec);
}

SensorHttpImp_2_1::SensorHttpImp_2_1(const std::string& hostname)
    : SensorHttpImp_2_2(hostname) {}

std::string SensorHttpImp_2_1::metadata(int timeout_sec) const {
    jsoncons::json root;
    try {
        root["sensor_info"] = jsoncons::json::parse(sensor_info(timeout_sec));
        root["beam_intrinsics"] =
            jsoncons::json::parse(beam_intrinsics(timeout_sec));
        root["imu_intrinsics"] =
            jsoncons::json::parse(imu_intrinsics(timeout_sec));
        root["lidar_intrinsics"] =
            jsoncons::json::parse(lidar_intrinsics(timeout_sec));
        root["lidar_data_format"] =
            jsoncons::json::parse(lidar_data_format(timeout_sec));
        root["calibration_status"] =
            jsoncons::json::parse(calibration_status(timeout_sec));
        root["config_params"] =
            jsoncons::json::parse(get_config_params(true, timeout_sec));
    } catch (const jsoncons::ser_error& e) {
        throw std::runtime_error(e.what());
    }

    std::string result;
    root.dump(result);
    return result;
}

std::string SensorHttpImp_2_1::sensor_info(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_sensor_info", timeout_sec);
}

std::string SensorHttpImp_2_1::beam_intrinsics(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_beam_intrinsics", timeout_sec);
}

std::string SensorHttpImp_2_1::imu_intrinsics(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_imu_intrinsics", timeout_sec);
}

std::string SensorHttpImp_2_1::lidar_intrinsics(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_lidar_intrinsics", timeout_sec);
}

std::string SensorHttpImp_2_1::lidar_data_format(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_lidar_data_format", timeout_sec);
}

std::string SensorHttpImp_2_1::calibration_status(int timeout_sec) const {
    return get("api/v1/sensor/cmd/get_calibration_status", timeout_sec);
}

SensorHttpImp_2_4_or_3::SensorHttpImp_2_4_or_3(const std::string& hostname)
    : SensorHttpImp(hostname) {}

std::string SensorHttpImp_2_4_or_3::get_user_data(int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

UserDataAndPolicy SensorHttpImp_2_4_or_3::get_user_data_and_policy(
    int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

void SensorHttpImp_2_4_or_3::set_user_data(const std::string& /*data*/,
                                           bool /*keep_on_config_delete*/,
                                           int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

void SensorHttpImp_2_4_or_3::delete_user_data(int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}
