#pragma once

#include <jsoncons/json.hpp>

#include "ouster/core/json_tools.h"
#include "ouster/core/lidar_info.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Parse and validate parsed metadata.
 *
 * @param[in] root The parsed metadata.
 * @param[out] sensor_info The sensor_info to populate.
 * @param[out] issues The issues that occurred during parsing.
 * @return if there are any critical issues or not.
 */
bool parse_and_validate_metadata(const jsoncons::json& root, LidarInfo& sensor_info,
                                 ValidatorIssues& issues);

/**
 * Populate a LidarInfo from parsed metadata.
 *
 * @param[in] root The parsed metadata.
 * @param[out] sensor_info The sensor_info to populate.
 * @throws std::runtime_error if the metadata has critical issues.
 */
void init_lidar_info_from_json(const jsoncons::json& root, LidarInfo& sensor_info);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
