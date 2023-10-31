/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <string>

namespace ouster {
namespace osf {

#ifdef _WIN32
constexpr char FILE_SEP = '\\';
#else
constexpr char FILE_SEP = '/';
#endif

/// Checking the the path is it directory or not.
bool is_dir(const std::string& path);

/// Check path existence on the system
bool path_exists(const std::string& path);

/// Path concatenation with OS specific path separator
std::string path_concat(const std::string& path1, const std::string& path2);

/// Get the path to unique temp directory and create it.
bool make_tmp_dir(std::string& tmp_path);

/// Make directory
bool make_dir(const std::string& path);

/// Get environment variable
bool get_env_var(const std::string& name, std::string& value);

// Unlink path
bool unlink_path(const std::string& path);

// Remove directory
bool remove_dir(const std::string& path);

// Get file size
int64_t file_size(const std::string& path);

// File mapping open
uint8_t* mmap_open(const std::string& path);

// File mapping close
bool mmap_close(uint8_t* file_buf, const uint64_t file_size);

/// Get the last system error and return it in a string (not wide string)
std::string get_last_error();

}  // namespace osf
}  // namespace ouster