#include "sensor_http_imp.h"

#include <cstdint>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

#include "curl_client.h"
#include "ouster/types.h"

using ouster::sdk::sensor::UserDataAndPolicy;

using namespace ouster::sdk::sensor::impl;
using namespace ouster::sdk::core;

const static Version MINIMUM_STAGING_VERSION = Version(3, 1, 0);

const static Version MINIMUM_ZM_VERSION = Version(3, 2, 0);

SensorHttpImp::SensorHttpImp(const std::string& hostname)
    : http_client_(std::make_unique<CurlClient>(hostname)) {}

SensorHttpImp::~SensorHttpImp() = default;

void SensorHttpImp::set_additional_headers(
    const std::vector<std::string>& headers) {
    http_client_->set_additional_headers(headers);
}

void SensorHttpImp::set_http_api_prefix(const std::string& prefix) {
    api_prefix_ = prefix;

    // add slash if it is missing
    if ((!api_prefix_.empty()) && api_prefix_.back() != '/') {
        api_prefix_.push_back('/');
    }
}

std::string SensorHttpImp::metadata(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata", timeout_sec);
}

std::string SensorHttpImp::sensor_info(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata/sensor_info",
               timeout_sec);
}

std::string SensorHttpImp::get_config_params(bool active,
                                             int timeout_sec) const {
    if (firmware_version() < MINIMUM_STAGING_VERSION) {
        auto config_type = active ? "active" : "staged";
        return get(std::string("api/v1/" + api_prefix_ +
                               "sensor/cmd/get_config_param?args=") +
                       config_type,
                   timeout_sec);
    }
    auto config_type = active ? "false" : "true";
    return get(std::string("api/v1/" + api_prefix_ + "sensor/config?staging=") +
                   config_type,
               timeout_sec);
}

void SensorHttpImp::set_config_param(const std::string& key,
                                     const std::string& value,
                                     int timeout_sec) const {
    if (firmware_version() < MINIMUM_STAGING_VERSION) {
        auto encoded_value =
            http_client_->encode(value);  // encode config params
        auto url = "api/v1/" + api_prefix_ +
                   "sensor/cmd/set_config_param?args=" + key + "+" +
                   encoded_value;
        execute(url, "\"set_config_param\"", timeout_sec);
        return;
    }
    std::string json;
    if (key == ".") {
        // set everything
        json = value;
    } else {
        // set only this parameter
        json = "{\"" + key + "\": " + value + "}";
    }
    http_client_->post(
        "api/v1/" + api_prefix_ +
            "sensor/config?staging=true&reinit=false&persist=false",
        json, timeout_sec);
}

std::string SensorHttpImp::active_config_params(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/config", timeout_sec);
}

std::string SensorHttpImp::staged_config_params(int timeout_sec) const {
    if (firmware_version() < MINIMUM_STAGING_VERSION) {
        return get(
            "api/v1/" + api_prefix_ + "sensor/cmd/get_config_param?args=staged",
            timeout_sec);
    }
    return get("api/v1/" + api_prefix_ + "sensor/config?staging=true",
               timeout_sec);
}

void SensorHttpImp::set_udp_dest_auto(int timeout_sec) const {
    if (firmware_version() < MINIMUM_STAGING_VERSION) {
        execute("api/v1/" + api_prefix_ + "sensor/cmd/set_udp_dest_auto", "{}",
                timeout_sec);
        return;
    }

    std::string json = R"({"udp_dest":"@auto"})";
    http_client_->post(
        "api/v1/" + api_prefix_ +
            "sensor/config?staging=true&reinit=false&persist=false",
        json, timeout_sec);
}

void SensorHttpImp::restart(int timeout_sec) const {
    std::string json;
    http_client_->post("api/v1/" + api_prefix_ + "system/restart", json,
                       timeout_sec);
}

std::string SensorHttpImp::beam_intrinsics(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata/beam_intrinsics",
               timeout_sec);
}

std::string SensorHttpImp::imu_intrinsics(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata/imu_intrinsics",
               timeout_sec);
}

std::string SensorHttpImp::lidar_intrinsics(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata/lidar_intrinsics",
               timeout_sec);
}

std::string SensorHttpImp::lidar_data_format(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata/lidar_data_format",
               timeout_sec);
}

std::string SensorHttpImp::calibration_status(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "sensor/metadata/calibration_status",
               timeout_sec);
}

std::string SensorHttpImp::network(int timeout_sec) const {
    return get("api/v1/" + api_prefix_ + "system/network", timeout_sec);
}

// reinitialize to activate new settings
void SensorHttpImp::reinitialize(int timeout_sec) const {
    if (firmware_version() < MINIMUM_STAGING_VERSION) {
        execute("api/v1/" + api_prefix_ + "sensor/cmd/reinitialize", "{}",
                timeout_sec);
        return;
    }

    std::string json = "{}";
    http_client_->post(
        "api/v1/" + api_prefix_ +
            "sensor/config?staging=true&reinit=true&persist=false",
        json, timeout_sec);
}

void SensorHttpImp::save_config_params(int timeout_sec) const {
    if (firmware_version() < MINIMUM_STAGING_VERSION) {
        execute("api/v1/" + api_prefix_ + "sensor/cmd/save_config_params", "{}",
                timeout_sec);
        return;
    }
    std::string json = "{}";
    http_client_->post(
        "api/v1/" + api_prefix_ + "sensor/config?persist=true&reinit=false",
        json, timeout_sec);
}

std::string SensorHttpImp::auto_detected_udp_dest(
    int timeout_sec,
    nonstd::optional<std::string> original_destination_in) const {
    if (firmware_version() < version_from_string("2.3.1")) {
        throw std::runtime_error(
            "retrieving auto detected udp destination not supported on this "
            "firmware version");
    }

    std::string original_destination;
    if (original_destination_in.has_value()) {
        original_destination = original_destination_in.value();
    } else {
        original_destination =
            SensorConfig(get_config_params(true, timeout_sec))
                .udp_dest.value_or("");
    }

    std::string json;
    jsoncons::json root;
    root["udp_dest"] = "@auto";
    root.dump(json);
    http_client_->post(
        "api/v1/" + api_prefix_ + "sensor/config?reinit=False&persist=False",
        json, timeout_sec);

    // todo maybe speed in future by only parsing udp_dest
    auto udp_auto = SensorConfig(get_config_params(false, timeout_sec));

    json = "";
    root["udp_dest"] = original_destination;
    root.dump(json);
    http_client_->post(
        "api/v1/" + api_prefix_ + "sensor/config?reinit=False&persist=False",
        json, timeout_sec);

    return udp_auto.udp_dest.value_or("");
}

std::string SensorHttpImp::get_user_data(int timeout_sec) const {
    if (is_vlp()) {
        throw std::runtime_error(
            "user data API not supported on this FW version");
    }
    // the API returns a jsonified string (quoted and escaped), parse it and
    // return the actual string value to the user
    return jsoncons::json::parse(get("api/v1/user/data", timeout_sec))
        .as<std::string>();
}

UserDataAndPolicy SensorHttpImp::get_user_data_and_policy(
    int timeout_sec) const {
    if (is_vlp()) {
        throw std::runtime_error(
            "user data API not supported on this FW version");
    }
    std::string url =
        "api/v1/" + api_prefix_ + "user/data?include_metadata=true";
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
    if (is_vlp()) {
        throw std::runtime_error(
            "user data API not supported on this FW version");
    }
    std::string json;
    auto temp_data = jsoncons::json(data);
    temp_data.dump(json);
    http_client_->put(
        std::string("api/v1/" + api_prefix_ + "user/data") +
            (keep_on_config_delete ? "?policy=keep_on_config_delete" : ""),
        json, timeout_sec);
}

void SensorHttpImp::delete_user_data(int timeout_sec) const {
    http_client_->del("api/v1/" + api_prefix_ + "user/data", timeout_sec);
}

std::vector<uint8_t> SensorHttpImp::get_zone_monitor_config_zip(
    bool staged, int timeout_sec) const {
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "zone monitor only available on fw versions 3.2+");
    }

    std::string str = http_client_->get(
        std::string("api/v1/" + api_prefix_ + "zone_monitor/") +
            (staged ? "staged" : "active") + "/zip",
        timeout_sec);
    return std::vector<uint8_t>(str.begin(), str.end());
}

void SensorHttpImp::set_zone_monitor_config_zip(
    const std::vector<uint8_t>& zip_archive, int timeout_sec) const {
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "zone monitor only available on fw versions 3.2+");
    }

    http_client_->post("api/v1/" + api_prefix_ + "zone_monitor/staged/zip",
                       zip_archive, timeout_sec);
    /** currently is applied automatically, may be a bug on the fw
    http_client_->post("api/v1/" + api_prefix_ + "zone_monitor/apply",
                       std::string{}, timeout_sec);
    */
}

void SensorHttpImp::delete_zone_monitor_staged_config(int timeout_sec) const {
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "zone monitor only available on fw versions 3.2+");
    }
    http_client_->del("api/v1/" + api_prefix_ + "zone_monitor/staged",
                      timeout_sec);
}

void SensorHttpImp::apply_zone_monitor_staged_config_to_active(
    int timeout_sec) const {
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "zone monitor only available on fw versions 3.2+");
    }
    http_client_->post("api/v1/" + api_prefix_ + "zone_monitor/apply",
                       std::string{}, timeout_sec);
}

std::vector<uint8_t> SensorHttpImp::get_zone_monitor_live_ids(
    int timeout_sec) const {
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "zone monitor only available on fw versions 3.2+");
    }
    auto resp = jsoncons::json::parse(http_client_->get(
        "api/v1/" + api_prefix_ + "zone_monitor/live_ids", timeout_sec));
    return resp.as<std::vector<uint8_t>>();
}

void SensorHttpImp::set_zone_monitor_live_ids(
    const std::vector<uint8_t>& zone_ids, int timeout_sec) const {
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "zone monitor only available on fw versions 3.2+");
    }
    http_client_->post("api/v1/" + api_prefix_ + "zone_monitor/live_ids",
                       jsoncons::json(zone_ids).to_string(), timeout_sec);
}

std::string SensorHttpImp::get(const std::string& url, int timeout_sec) const {
    return http_client_->get(url, timeout_sec);
}

void SensorHttpImp::execute(const std::string& url,
                            const std::string& validation,
                            int timeout_sec) const {
    auto result = get(url, timeout_sec);
    if (result != validation) {
        throw std::runtime_error("SensorHttpImp::execute failed! url: " + url +
                                 " returned [" + result + "], expected [" +
                                 validation + "]");
    }
}

void SensorHttpImp::set_static_ip(const std::string& ip_address,
                                  int timeout_sec) const {
    std::string json;
    auto temp_data = jsoncons::json(ip_address);
    temp_data.dump(json);
    http_client_->put("api/v1/" + api_prefix_ + "system/network/ipv4/override",
                      json, timeout_sec);
}

void SensorHttpImp::set_static_ip(const std::string& ip_address,
                                  const std::string& gateway_address,
                                  int timeout_sec) const {
    if (gateway_address.empty()) {
        return set_static_ip(ip_address, timeout_sec);
    }

    // throw if unsupported
    if (firmware_version() < MINIMUM_ZM_VERSION) {
        throw std::runtime_error(
            "setting gateway address not supported on this FW version");
    }

    std::string json;
    jsoncons::json root;
    root["addr"] = ip_address;
    root["gateway"] = gateway_address;
    root.dump(json);
    http_client_->put("api/v1/" + api_prefix_ + "system/network/ipv4/override",
                      json, timeout_sec);
}

void SensorHttpImp::delete_static_ip(int timeout_sec) const {
    http_client_->del("api/v1/" + api_prefix_ + "system/network/ipv4/override",
                      timeout_sec);
}

std::vector<uint8_t> SensorHttpImp::diagnostics_dump(int timeout_sec) const {
    std::string str = http_client_->get(
        "/api/v1/" + api_prefix_ + "diagnostics/dump", timeout_sec);
    return std::vector<uint8_t>(str.begin(), str.end());
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
