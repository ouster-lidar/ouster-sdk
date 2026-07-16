/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/types.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <nonstd/optional.hpp>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "ouster/core/data_format.h"
#include "ouster/core/defaults.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/impl/table.h"

using nonstd::make_optional;
using nonstd::nullopt;

using nonstd::optional;
using ouster::impl::lookup;
using ouster::impl::rlookup;
using ouster::impl::Table;
using std::stoul;

namespace ouster {
namespace sdk {
namespace core {

/* Misc operations */

uint32_t n_cols_of_lidar_mode(LidarMode mode) {
    return mode.columns;
}

unsigned int frequency_of_lidar_mode(LidarMode mode) {
    return mode.fps;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
