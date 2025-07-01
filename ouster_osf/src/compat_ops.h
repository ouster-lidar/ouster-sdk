/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>

#include "ouster/visibility.h"

namespace ouster {
namespace osf {

/// @todo Fix the api comments in this file
#ifdef _WIN32
constexpr char FILE_SEP = '\\';
#else
constexpr char FILE_SEP = '/';
#endif

/// Checking the the path is it directory or not.
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool is_dir(const std::string& path);

/// Check path existence on the system
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool path_exists(const std::string& path);

/// Path concatenation with OS specific path separator
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
std::string path_concat(const std::string& path1, const std::string& path2);

/// Get the path to unique temp directory and create it.
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool make_tmp_dir(std::string& tmp_path);

/// Make directory
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool make_dir(const std::string& path);

/// Get environment variable
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool get_env_var(const std::string& name, std::string& value);

// Unlink path
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool unlink_path(const std::string& path);

// Remove directory
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool remove_dir(const std::string& path);

// Get file size
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
int64_t file_size(const std::string& path);

// File mapping open
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
uint8_t* mmap_open(const std::string& path, uintptr_t& memmap_handle);

// File mapping close
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
bool mmap_close(uint8_t* file_buf, uint64_t file_size, uintptr_t memmap_handle);

/// Get the last system error and return it in a string (not wide string)
/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
std::string get_last_error();

/**
 * Truncate a file to a certain length
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] path The file to truncate.
 * @param[in] filesize The final size of the file.
 *
 * @return The number of bytes of the final file.
 */
OUSTER_API_FUNCTION
int64_t truncate_file(const std::string& path, uint64_t filesize);

/**
 * Appends one file to another
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] append_to_file_name The file to append to.
 * @param[in] append_from_file_name The file to append from.
 *
 * @return The number of bytes of the final file.
 */
OUSTER_API_FUNCTION
int64_t append_binary_file(const std::string& append_to_file_name,
                           const std::string& append_from_file_name);

/**
 * Copies trailing bytes from a file
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] source_file The file to copy from.
 * @param[in] target_file The file to copy to.
 * @param[in] offset The offset in the source_file to start copying from.
 *
 * @return The number of bytes of the target file.
 */
OUSTER_API_FUNCTION
int64_t copy_file_trailing_bytes(const std::string& source_file,
                                 const std::string& target_file,
                                 uint64_t offset);
}  // namespace osf
}  // namespace ouster
