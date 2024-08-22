/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ouster/ouster_client_export.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {
namespace impl {

struct FieldInfo {
    ChanFieldType ty_tag;
    size_t offset;
    uint64_t mask;
    int shift;
};

}  // namespace impl

OUSTER_CLIENT_EXPORT void add_custom_profile(
    int profile_nr, const std::string& name,
    const std::vector<std::pair<std::string, impl::FieldInfo>>& fields,
    size_t chan_data_size);

}  // namespace sensor
}  // namespace ouster
