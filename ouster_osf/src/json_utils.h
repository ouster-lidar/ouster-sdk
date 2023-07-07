/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include "json/json.h"

namespace ouster {
namespace osf {

bool parse_json(const std::string json_str, Json::Value& output);

std::string json_string(const Json::Value& root);

}  // namespace osf
}  // namespace ouster