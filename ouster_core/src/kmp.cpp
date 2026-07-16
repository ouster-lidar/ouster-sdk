#include "ouster/core/impl/kmp.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <istream>
#include <string>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

std::vector<int> compute_lps(const std::string& pattern) {
    auto pattern_len = pattern.length();
    std::vector<int> lps(pattern_len, 0);
    int len = 0;
    std::string::size_type char_idx = 1;

    while (char_idx < pattern_len) {
        if (pattern[char_idx] == pattern[len]) {
            len++;
            lps[char_idx] = len;
            char_idx++;
        } else {
            if (len != 0) {
                len = lps[len - 1];
            } else {
                lps[char_idx] = 0;
                char_idx++;
            }
        }
    }
    return lps;
}

std::string::size_type kmp_search(std::istream& input, const std::string& pattern_in) {
    input.seekg(0, std::ios::beg);  // Ensure we start from the beginning of the stream
    if (input.fail()) {
        throw std::runtime_error("Failed to seek to the beginning of the stream.");
    }

    input.seekg(0, std::ios::end);
    std::streamsize file_size = input.tellg();
    if (file_size < 0) {
        throw std::runtime_error("Failed to determine the size of the stream.");
    }
    input.seekg(0, std::ios::beg);  // Ensure we start from the beginning of the stream
    if (input.fail()) {
        throw std::runtime_error("Failed to seek to the beginning of the stream.");
    }

    // make pattern lowercase for case-insensitive search
    std::string pattern = pattern_in;
    std::transform(pattern.begin(), pattern.end(), pattern.begin(),
                   [](unsigned char character) { return std::tolower(character); });

    if (pattern.empty()) {
        return 0;
    }

    auto pattern_len = pattern.length();
    std::vector<int> lps = compute_lps(pattern);

    constexpr std::streamsize buffer_size = 1024;
    std::array<char, buffer_size> buffer{};       // Buffer for reading from the stream
    std::string::size_type pattern_char_idx = 0;  // Current index in pattern
    int64_t char_idx = 0;                         // Current index in input stream

    // Read the stream character by character
    while (char_idx < file_size) {
        std::streamsize num_read =
            std::min(buffer_size, static_cast<std::streamsize>(file_size - char_idx));
        input.read(buffer.data(), num_read);
        if (!input.good()) {
            throw std::runtime_error("Read failure");
        }
        for (std::streamsize i = 0; i < num_read; ++i) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            char character = buffer[i];
            character = static_cast<char>(std::tolower(character));  // Case-insensitive search
            char_idx++;
            while (pattern_char_idx > 0 && character != pattern[pattern_char_idx]) {
                // Mismatch: skip using the LPS table
                pattern_char_idx = lps[pattern_char_idx - 1];
            }

            if (character == pattern[pattern_char_idx]) {
                pattern_char_idx++;
            }

            if (pattern_char_idx == pattern_len) {
                return char_idx - pattern_len;  // Match found, return the starting
                                                // index of the match in the stream
            }
        }
        if (num_read < buffer_size) {
            break;  // End of stream reached
        }
    }

    return std::string::npos;  // No match found
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
