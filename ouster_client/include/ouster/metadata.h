/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster metadata processing
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/json_tools.h"
#include "ouster/types.h"
#include "ouster/visibility.h"
#include "ouster/zone_monitor.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[out] issues The issues that occured during parsing.
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_metadata(const std::string& json_data,
                                 ValidatorIssues& issues);

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[in] sensor_info The optional SensorInfo to populate.
 * @param[in] issues The issues that occurred during parsing.
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_metadata(const std::string& json_data,
                                 nonstd::optional<SensorInfo>& sensor_info,
                                 ValidatorIssues& issues);

/**
 * Parse config text blob from the sensor into a SensorConfig struct.
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] config a text blob given by get_config from client.h.
 * @param[out] sensor_config The optional sensor config object, if parsing
 *                           issues occur, will be empty.
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_config(const std::string& config,
                               SensorConfig& sensor_config);

/**
 * Parse config text blob from the sensor into a SensorConfig struct.
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] config a text blob given by get_config from client.h.
 * @param[out] sensor_config The optional sensor config object, if parsing
 *                           issues occur, will be empty.
 * @param[out] issues The specific issues parsing the sensor config
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_config(const std::string& config,
                               SensorConfig& sensor_config,
                               ValidatorIssues& issues);
}  // namespace core
}  // namespace sdk
}  // namespace ouster
