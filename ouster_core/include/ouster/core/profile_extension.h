/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "field_decode_info.h"
#include "ouster/core/deprecation.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Register a custom lidar profile with a caller-supplied profile number.
 *
 * @param[in] profile_nr profile enum value to register
 * @param[in] name profile name
 * @param[in] fields field definitions for the profile
 * @param[in] chan_data_size channel data size in bytes
 *
 * @throws std::invalid_argument if profile_nr is 0
 */
OUSTER_DEPRECATED_MSG("UDPProfileLidar add_custom_profile(name, fields, chan_data_size)",
                      OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_API_FUNCTION
void add_custom_profile(int profile_nr, const std::string& name,
                        const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
                        size_t chan_data_size);

/**
 * Register a custom lidar profile, allocating a new profile enum value.
 *
 * @param[in] name profile name
 * @param[in] fields field definitions for the profile
 * @param[in] chan_data_size channel data size in bytes
 *
 * @throws std::runtime_error when no profile slots are available
 * ("Limit of lidar profiles has been reached")
 *
 * @return allocated profile enum value
 */
OUSTER_API_FUNCTION
UDPProfileLidar add_custom_profile(
    const std::string& name, const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
    size_t chan_data_size);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
