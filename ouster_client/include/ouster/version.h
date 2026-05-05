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

#include "ouster/deprecation.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Represents a semantic version such as major.minor.patch
 * with optional metadata fields.
 */
struct OUSTER_API_CLASS Version {
    uint16_t major{};  ///< Major version number
    uint16_t minor{};  ///< Minor version number
    uint16_t patch{};  ///< Patch(or revision) version number

    std::string stage{};       ///< Release stage name, if present.
    std::string machine{};     ///< Machine name, if present.
    std::string prerelease{};  ///< Prerelease name (e.g. rc1), if present.
    std::string build{};       ///< Build info, if present. Often a date string.

    /**
     * Construct a default version object.
     */
    OUSTER_API_FUNCTION Version() = default;

    /**
     * Construct a default version object with the specified version.
     * @param[in] maj Major version number.
     * @param[in] min Minor version number.
     * @param[in] pat Patch version number.
     */
    OUSTER_API_FUNCTION Version(uint16_t maj, uint16_t min, uint16_t pat)
        : major(maj), minor(min), patch(pat) {}

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

/**
 * @deprecated Use Version instead.
 */
OUSTER_DEPRECATED_TYPE(version, Version,                       // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)  // NOLINT

/**
 * Represents an invalid or uninitialized version.
 * Used when version parsing fails.
 */
const Version INVALID_VERSION = Version(0, 0, 0);

/**
 * @deprecated Use INVALID_VERSION
 *
 * Using an extern here due to duplicate definition errors popping up.
 */
OUSTER_DIAGNOSTIC_PUSH
OUSTER_DIAGNOSTIC_IGNORE_UNUSED
OUSTER_DEPRECATED_MSG(INVALID_VERSION, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
extern const Version& invalid_version;
OUSTER_DIAGNOSTIC_POP

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
inline bool operator==(const Version& u, const Version& v) {
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
inline bool operator<(const Version& u, const Version& v) {
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
inline bool operator<=(const Version& u, const Version& v) {
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
inline bool operator!=(const Version& u, const Version& v) { return !(u == v); }

/**
 * Greater than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than or equal to the second version.
 */
OUSTER_API_FUNCTION
inline bool operator>=(const Version& u, const Version& v) { return !(u < v); }

/**
 * Greater than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than the second version.
 */
OUSTER_API_FUNCTION
inline bool operator>(const Version& u, const Version& v) { return !(u <= v); }
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
Version version_from_string(const std::string& ver);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
