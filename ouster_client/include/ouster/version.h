/**
 * @file
 * @brief Simple version struct
 */

#include <cstdint>
#include <string>

#pragma once

namespace ouster {
namespace util {

struct version {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};

const version invalid_version = {0, 0, 0};

inline bool operator==(const version& u, const version& v) {
    return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

inline bool operator<(const version& u, const version& v) {
    return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
           (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

inline bool operator<=(const version& u, const version& v) {
    return u < v || u == v;
}

inline bool operator!=(const version& u, const version& v) { return !(u == v); }

inline bool operator>=(const version& u, const version& v) { return !(u < v); }

inline bool operator>(const version& u, const version& v) { return !(u <= v); }

/**
 * Get string representation of a version.
 *
 * @param version
 * @return string representation of the version
 */
std::string to_string(const version& v);

/**
 * Get lidar mode from string.
 *
 * @param string
 * @return lidar mode corresponding to the string, or invalid_version on error
 */
version version_of_string(const std::string& s);

}  // namespace util
}  // namespace ouster
