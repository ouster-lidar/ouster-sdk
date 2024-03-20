/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>

namespace ouster {
namespace osf {

/// @todo Fix the api comments in this file
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

/**
 * Truncate a file to a certain length
 *
 * @param[in] path The file to truncate.
 * @param[in] filesize The final size of the file.
 *
 * @return The number of bytes of the final file.
 */
int64_t truncate_file(const std::string& path, uint64_t filesize);

/**
 * Appends one file to another
 *
 * @param[in] append_to_file_name The file to append to.
 * @param[in] append_from_file_name The file to append from.
 *
 * @return The number of bytes of the final file.
 */
int64_t append_binary_file(const std::string& append_to_file_name,
                           const std::string& append_from_file_name);

/**
 * Copies trailing bytes from a file
 *
 * @param[in] source_file The file to copy from.
 * @param[in] target_file The file to copy to.
 * @param[in] offset The offset in the source_file to start copying from.
 *
 * @return The number of bytes of the target file.
 */
int64_t copy_file_trailing_bytes(const std::string& source_file,
                                 const std::string& target_file,
                                 uint64_t offset);
}  // namespace osf
}  // namespace ouster
