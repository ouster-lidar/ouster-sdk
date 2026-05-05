#include "ouster/sensor_http.h"

#include <jsoncons/json.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "curl_client.h"
#include "sensor_http_imp.h"

using namespace ouster::sdk::core;
using namespace ouster::sdk::sensor;

static std::mutex header_mutex;
static std::vector<std::string> additional_headers;
static std::string api_prefix;

std::string SensorHttp::firmware_version_string(const std::string& hostname,
                                                int timeout_sec) {
    auto http_client = std::make_unique<CurlClient>(hostname);
    auto fwjson = http_client->get("api/v1/system/firmware", timeout_sec);

    // This will exception out on bad parse
    return jsoncons::json::parse(fwjson)["fw"].as<std::string>();
}

Version SensorHttp::firmware_version(const std::string& hostname,
                                     int timeout_sec) {
    auto result = firmware_version_string(hostname, timeout_sec);
    return ouster::sdk::core::version_from_string(result);
}

void ouster::sdk::sensor::set_http_api_headers(
    const std::vector<std::string>& headers) {
    std::lock_guard<std::mutex> guard(header_mutex);
    additional_headers = headers;
}

void ouster::sdk::sensor::set_http_api_prefix(const std::string& prefix) {
    std::lock_guard<std::mutex> guard(header_mutex);
    api_prefix = prefix;
}

std::unique_ptr<SensorHttp> SensorHttp::create(const std::string& hostname,
                                               int timeout_sec) {
    auto http_client = std::make_unique<CurlClient>(hostname);
    auto fwjson =
        http_client->get("api/v1/sensor/metadata/sensor_info", timeout_sec);
    auto parsedjson = jsoncons::json::parse(fwjson);
    // This will exception out on bad parse
    bool is_vlp = parsedjson["prod_line"].as<std::string>().find("VLP") !=
                  std::string::npos;
    auto fw = ouster::sdk::core::version_from_string(
        parsedjson["image_rev"].as<std::string>());

    if (fw == INVALID_VERSION || fw.major < 2 ||
        (fw.major == 2 && fw.minor <= 3)) {
        throw std::runtime_error(
            "SensorHttp:: firmware version information unavailable or "
            "not version not supported. Please upgrade your sensor to FW "
            "2.4 or later.");
    }

    std::vector<std::string> headers;
    std::string api_prefix_copy;
    {
        std::lock_guard<std::mutex> guard(header_mutex);
        headers = additional_headers;
        api_prefix_copy = api_prefix;
    }

    if ((fw.major == 2 && fw.minor == 4) || (fw.major == 3 && fw.minor == 0)) {
        auto instance =
            std::make_unique<impl::SensorHttpImp_2_4_or_3>(hostname);
        instance->version_ = fw;
        instance->hostname_ = hostname;
        instance->set_additional_headers(headers);
        instance->set_http_api_prefix(api_prefix_copy);
        return instance;
    }

    auto instance = std::make_unique<impl::SensorHttpImp>(hostname);
    instance->version_ = fw;
    instance->hostname_ = hostname;
    instance->vlp_prod_ = is_vlp;
    instance->set_additional_headers(headers);
    instance->set_http_api_prefix(api_prefix_copy);
    return instance;
}
