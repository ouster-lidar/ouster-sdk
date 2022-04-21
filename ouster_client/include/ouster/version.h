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
    uint16_t major; ///< @todo fill me in
    uint16_t minor; ///< @todo fill me in
    uint16_t patch; ///< @todo fill me in
};

const version invalid_version = {0, 0, 0};

/** \defgroup ouster_client_version_operators Ouster Client version.h Operators
 * @{
 */
/** @todo document me */
inline bool operator==(const version& u, const version& v) {
    return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

/** @todo document me */
inline bool operator<(const version& u, const version& v) {
    return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
           (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

/** @todo document me */
inline bool operator<=(const version& u, const version& v) {
    return u < v || u == v;
}

/** @todo document me */
inline bool operator!=(const version& u, const version& v) { return !(u == v); }

/** @todo document me */
inline bool operator>=(const version& u, const version& v) { return !(u < v); }

/** @todo document me */
inline bool operator>(const version& u, const version& v) { return !(u <= v); }
/** @}*/

/**
 * Get string representation of a version.
 *
 * @param version
 * @return string representation of the version
 */
std::string to_string(const version& v);

/**
 * Get version from string.
 *
 * @param string
 * @return version corresponding to the string, or invalid_version on error
 */
version version_of_string(const std::string& s);

}  // namespace util
}  // namespace ouster
