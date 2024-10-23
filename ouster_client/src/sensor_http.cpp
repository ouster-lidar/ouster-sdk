#include "ouster/sensor_http.h"

#include <sstream>

#include "curl_client.h"
#include "sensor_http_imp.h"
#include "sensor_tcp_imp.h"

using std::string;

using namespace ouster::util;
using namespace ouster::sensor;
using namespace ouster::sensor::util;
using namespace ouster::sensor::impl;

string SensorHttp::firmware_version_string(const string& hostname,
                                           int timeout_sec) {
    auto http_client = std::make_unique<CurlClient>(hostname);
    auto fwjson = http_client->get("api/v1/system/firmware", timeout_sec);

    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{fwjson};

    if (!Json::parseFromStream(builder, ss, &root, &errors))
        throw std::runtime_error{
            "Errors parsing firmware for firmware_version_string: " + errors};

    return root["fw"].asString();
}

version SensorHttp::firmware_version(const string& hostname, int timeout_sec) {
    auto result = firmware_version_string(hostname, timeout_sec);
    return ouster::util::version_from_string(result);
}

std::unique_ptr<SensorHttp> SensorHttp::create(const string& hostname,
                                               int timeout_sec) {
    auto fw = firmware_version(hostname, timeout_sec);

    if (fw == invalid_version || fw.major < 2) {
        throw std::runtime_error(
            "SensorHttp:: create firmware version information unavailable or "
            "not fully supported version. Please upgrade your sensor to FW "
            "2.0 or later.");
    }

    if (fw.major == 2) {
        switch (fw.minor) {
            case 0: {
                // FW 2.0 doesn't work properly with http
                auto instance = std::make_unique<SensorTcpImp>(hostname);
                instance->version_ = fw;
                instance->hostname_ = hostname;
                return instance;
            }
            case 1: {
                auto instance = std::make_unique<SensorHttpImp_2_1>(hostname);
                instance->version_ = fw;
                instance->hostname_ = hostname;
                return instance;
            }
            case 2: {
                auto instance = std::make_unique<SensorHttpImp_2_2>(hostname);
                instance->version_ = fw;
                instance->hostname_ = hostname;
                return instance;
            }
        }
    }
    if ((fw.major == 2 && (fw.minor == 4 || fw.minor == 3)) ||
        (fw.major == 3 && fw.minor == 0)) {
        auto instance = std::make_unique<SensorHttpImp_2_4_or_3>(hostname);
        instance->version_ = fw;
        instance->hostname_ = hostname;
        return instance;
    }

    auto instance = std::make_unique<SensorHttpImp>(hostname);
    instance->version_ = fw;
    instance->hostname_ = hostname;
    return instance;
}
