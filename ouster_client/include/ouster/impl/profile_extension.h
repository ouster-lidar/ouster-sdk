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

#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

struct OUSTER_API_CLASS FieldInfo {
    ChanFieldType ty_tag;
    size_t offset;
    uint64_t mask;
    int shift;
};

}  // namespace impl

OUSTER_API_FUNCTION
void add_custom_profile(
    int profile_nr, const std::string& name,
    const std::vector<std::pair<std::string, impl::FieldInfo>>& fields,
    size_t chan_data_size);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
