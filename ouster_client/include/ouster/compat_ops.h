/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
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

/**
 * Get the size of the given file in bytes.
 * @param[in] path The path to the file.
 * @return The size of the file in bytes.
 */
OUSTER_API_FUNCTION
size_t get_file_size(const std::string& path);

/**
 * Get the contents of the file as a vector of bytes.
 * @param[in] path The path to the file.
 * @return A vector containing the contents of the file.
 */
OUSTER_API_FUNCTION
std::vector<uint8_t> get_file_as_bytes(const std::string& path);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
