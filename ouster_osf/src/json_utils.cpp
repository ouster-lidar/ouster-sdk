/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "json_utils.h"

#include <sstream>

namespace ouster {
namespace osf {

bool parse_json(const std::string json_str, Json::Value& output) {
    // Parse Json
    Json::CharReaderBuilder jbuilder{};
    jbuilder["collectComments"] = false;
    std::stringstream source{json_str};
    std::string jerrs;
    return Json::parseFromStream(jbuilder, source, &output, &jerrs);
}

std::string json_string(const Json::Value& root) {
    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "  ";
    return Json::writeString(builder, root);
}

}  // namespace osf
}  // namespace ouster
