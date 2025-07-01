/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Simple version struct
 */
#pragma once

#include <cstdint>
#include <sstream>
#include <string>

#include "ouster/visibility.h"

namespace ouster {
namespace util {

struct OUSTER_API_CLASS version {
    uint16_t major{};  ///< Major version number
    uint16_t minor{};  ///< Minor version number
    uint16_t patch{};  ///< Patch(or revision) version number

    std::string stage{};       ///< Release stage name, if present.
    std::string machine{};     ///< Machine name, if present.
    std::string prerelease{};  ///< Prerelease name (e.g. rc1), if present.
    std::string build{};       ///< Build info, if present. Often a date string.

    /**
     * Return the version as a string formatted as
     * [major].[minor].[patch]
     * or
     * [major].[minor].[patch]-[prerelease]
     * depending on whether there is a prerelease value.
     *
     * @return the version formatted as a string.
     */
    OUSTER_API_FUNCTION
    std::string simple_version_string() const {
        std::stringstream sstream;
        sstream << major << "." << minor << "." << patch;
        if (!prerelease.empty()) {
            sstream << "-" << prerelease;
        }
        return sstream.str();
    }
};

const version invalid_version = {0, 0, 0, "", "", "", ""};

/** \defgroup ouster_client_version_operators Ouster Client version.h Operators
 * @{
 */
/**
 * Equality operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the versions are the same.
 */
OUSTER_API_FUNCTION
inline bool operator==(const version& u, const version& v) {
    return u.major == v.major && u.minor == v.minor && u.patch == v.patch &&
           u.stage == v.stage && u.machine == v.machine && u.build == v.build &&
           u.prerelease == v.prerelease;
}

/**
 * Less than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is less than the second version.
 */
OUSTER_API_FUNCTION
inline bool operator<(const version& u, const version& v) {
    return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
           (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

/**
 * Less than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is less than or equal to the second version.
 */
OUSTER_API_FUNCTION
inline bool operator<=(const version& u, const version& v) {
    return u < v || u == v;
}

/**
 * In-equality operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the versions are not the same.
 */
OUSTER_API_FUNCTION
inline bool operator!=(const version& u, const version& v) { return !(u == v); }

/**
 * Greater than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than or equal to the second version.
 */
OUSTER_API_FUNCTION
inline bool operator>=(const version& u, const version& v) { return !(u < v); }

/**
 * Greater than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than the second version.
 */
OUSTER_API_FUNCTION
inline bool operator>(const version& u, const version& v) { return !(u <= v); }
/** @}*/

/**
 * Get version from string. Parses strings of the format:
 * STAGE-MACHINE-vMAJOR.MINOR.PATCH-PRERELEASE+BUILD
 * Requires at least major.minor.patch to return a valid version.
 *
 * @param[in] ver string.
 *
 * @return version corresponding to the string, or invalid_version on error.
 */
OUSTER_API_FUNCTION
version version_from_string(const std::string& ver);

}  // namespace util
}  // namespace ouster
