#include "sensor_http_imp.h"

#include "curl_client.h"

using ouster::sensor::util::UserDataAndPolicy;
using std::string;
using namespace ouster::sensor::impl;

SensorHttpImp::SensorHttpImp(const string& hostname)
    : http_client(std::make_unique<CurlClient>(hostname)) {}

SensorHttpImp::~SensorHttpImp() = default;

Json::Value SensorHttpImp::metadata(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata", timeout_sec);
}

Json::Value SensorHttpImp::sensor_info(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata/sensor_info", timeout_sec);
}

string SensorHttpImp::get_config_params(bool active, int timeout_sec) const {
    auto config_type = active ? "active" : "staged";
    return get(string("api/v1/sensor/cmd/get_config_param?args=") + config_type,
               timeout_sec);
}

void SensorHttpImp::set_config_param(const string& key, const string& value,
                                     int timeout_sec) const {
    auto encoded_value = http_client->encode(value);  // encode config params
    auto url =
        "api/v1/sensor/cmd/set_config_param?args=" + key + "+" + encoded_value;
    execute(url, "\"set_config_param\"", timeout_sec);
}

Json::Value SensorHttpImp::active_config_params(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_config_param?args=active",
                    timeout_sec);
}

Json::Value SensorHttpImp::staged_config_params(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_config_param?args=staged",
                    timeout_sec);
}

void SensorHttpImp::set_udp_dest_auto(int timeout_sec) const {
    execute("api/v1/sensor/cmd/set_udp_dest_auto", "{}", timeout_sec);
}

Json::Value SensorHttpImp::beam_intrinsics(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata/beam_intrinsics", timeout_sec);
}

Json::Value SensorHttpImp::imu_intrinsics(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata/imu_intrinsics", timeout_sec);
}

Json::Value SensorHttpImp::lidar_intrinsics(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata/lidar_intrinsics", timeout_sec);
}

Json::Value SensorHttpImp::lidar_data_format(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata/lidar_data_format", timeout_sec);
}

Json::Value SensorHttpImp::calibration_status(int timeout_sec) const {
    return get_json("api/v1/sensor/metadata/calibration_status", timeout_sec);
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
    return get_json("api/v1/user/data", timeout_sec).asString();
}

UserDataAndPolicy SensorHttpImp::get_user_data_and_policy(
    int timeout_sec) const {
    auto json = get_json("api/v1/user/data?include_metadata=true", timeout_sec);
    return {json["policy"].asString() != "clear_on_config_delete",
            json["value"].asString()};
}

void SensorHttpImp::set_user_data(const std::string& data,
                                  bool keep_on_config_delete,
                                  int timeout_sec) const {
    Json::StreamWriterBuilder wbuilder;
    std::string json = Json::writeString(wbuilder, Json::Value(data));
    http_client->put(
        string("api/v1/user/data") +
            (keep_on_config_delete ? "?policy=keep_on_config_delete" : ""),
        json, timeout_sec);
}

void SensorHttpImp::delete_user_data(int timeout_sec) const {
    http_client->del("api/v1/user/data", timeout_sec);
}

string SensorHttpImp::get(const string& url, int timeout_sec) const {
    return http_client->get(url, timeout_sec);
}

Json::Value SensorHttpImp::get_json(const string& url, int timeout_sec) const {
    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root;
    auto result = get(url, timeout_sec);
    if (!reader->parse(result.c_str(), result.c_str() + result.size(), &root,
                       nullptr))
        throw std::runtime_error("SensorHttpImp::get_json failed! url: " + url);
    return root;
}

void SensorHttpImp::execute(const string& url, const string& validation,
                            int timeout_sec) const {
    auto result = get(url, timeout_sec);
    if (result != validation)
        throw std::runtime_error("SensorHttpImp::execute failed! url: " + url +
                                 " returned [" + result + "], expected [" +
                                 validation + "]");
}

SensorHttpImp_2_2::SensorHttpImp_2_2(const string& hostname)
    : SensorHttpImp_2_4_or_3(hostname) {}

void SensorHttpImp_2_2::set_udp_dest_auto(int timeout_sec) const {
    return execute("api/v1/sensor/cmd/set_udp_dest_auto",
                   "\"set_config_param\"", timeout_sec);
}

SensorHttpImp_2_1::SensorHttpImp_2_1(const string& hostname)
    : SensorHttpImp_2_2(hostname) {}

Json::Value SensorHttpImp_2_1::metadata(int timeout_sec) const {
    Json::Value root;
    root["sensor_info"] = sensor_info(timeout_sec);
    root["beam_intrinsics"] = beam_intrinsics(timeout_sec);
    root["imu_intrinsics"] = imu_intrinsics(timeout_sec);
    root["lidar_intrinsics"] = lidar_intrinsics(timeout_sec);
    root["lidar_data_format"] = lidar_data_format(timeout_sec);
    root["calibration_status"] = calibration_status(timeout_sec);

    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value node;
    auto res = get_config_params(true, timeout_sec);
    auto parse_success =
        reader->parse(res.c_str(), res.c_str() + res.size(), &node, nullptr);
    root["config_params"] = parse_success ? node : res;
    return root;
}

Json::Value SensorHttpImp_2_1::sensor_info(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_sensor_info", timeout_sec);
}

Json::Value SensorHttpImp_2_1::beam_intrinsics(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_beam_intrinsics", timeout_sec);
}

Json::Value SensorHttpImp_2_1::imu_intrinsics(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_imu_intrinsics", timeout_sec);
}

Json::Value SensorHttpImp_2_1::lidar_intrinsics(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_lidar_intrinsics", timeout_sec);
}

Json::Value SensorHttpImp_2_1::lidar_data_format(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_lidar_data_format", timeout_sec);
}

Json::Value SensorHttpImp_2_1::calibration_status(int timeout_sec) const {
    return get_json("api/v1/sensor/cmd/get_calibration_status", timeout_sec);
}

SensorHttpImp_2_4_or_3::SensorHttpImp_2_4_or_3(const string& hostname)
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
