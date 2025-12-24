#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// TODO[tws] dedupe (compat-ops)
size_t get_file_size(const std::string& path);

// TODO[tws] dedupe
std::vector<uint8_t> get_file_as_bytes(const std::string& path);
