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
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Register a custom lidar profile
 *
 * @param[in] name profile name
 * @param[in] fields field definitions for the profile
 * @param[in] chan_data_size channel data size in bytes
 *
 * @throws std::invalid_argument if the profile is already registered
 *
 * @return profile name
 */
OUSTER_API_FUNCTION
std::string add_custom_profile(const std::string& name,
                               const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
                               size_t chan_data_size);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
