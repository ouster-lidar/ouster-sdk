/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <string>

#include "nonstd/optional.hpp"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

enum class IoType {
    OSF = 0,
    PCAP = 1,
    SENSOR = 2,
    BAG = 3,
    CSV = 4,
    PLY = 5,
    PCD = 6,
    LAS = 7,
    MCAP = 8,
    PNG = 9,
    STL = 10
};

/// Return a IoType given a source arg str
/// @throws std::runtime_error if IoType is unknown
OUSTER_API_FUNCTION
IoType io_type(const std::string& source);

/// Convert an IoType to human readable string
/// @throws std::runtime_error if IoType is invalid
OUSTER_API_FUNCTION
std::string to_string(IoType type);

/// Return an IoType given the file extension for the provided file path
/// @throws std::runtime_error if IoType is unknown
OUSTER_API_FUNCTION
IoType io_type_from_extension(const std::string& filename);

/// Return a file extension for the given source type, if it's a file-based
/// source.
OUSTER_API_FUNCTION
nonstd::optional<std::string> extension_from_io_type(IoType type);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
