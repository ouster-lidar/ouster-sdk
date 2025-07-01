/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/visibility.h"

namespace ouster {
namespace core {
/// Determine if a path is a directory. Returns optional if the path does not
/// exist. False if the path is a file.
/// @return if path is a directory
OUSTER_API_FUNCTION
nonstd::optional<bool> is_directory(
    const std::string& path  ///< [in] path to check
);

/// Returns if something is a network host (resolvable network address)
/// @return if is network host
OUSTER_API_FUNCTION
bool is_host(const std::string& name  ///< [in] host to check
);

/// List the files in a given directory
/// @return list of files in directory
OUSTER_API_FUNCTION
std::vector<std::string> files_in_directory(
    const std::string& path  ///< [in] path to check
);

}  // namespace core
}  // namespace ouster
