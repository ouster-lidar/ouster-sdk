/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster metadata processing
 */
#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "nonstd/optional.hpp"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {

/**
 * Class for representing metadata issues.
 */
struct OUSTER_API_CLASS ValidatorIssues {
   public:
    /**
     * Subclass for recording validator issues
     */
    class OUSTER_API_CLASS ValidatorEntry {
       public:
        /**
         * Construct a validator issue entry.
         *
         * @param[in] path The json path associated with the issue.
         * @param[in] msg The specific issue.
         */
        OUSTER_API_FUNCTION
        ValidatorEntry(const std::string& path, const std::string& msg);

        /**
         * Construct a validator issue entry from another validator
         * issue entry.
         *
         * @param[in] other The other validator entry to copy from.
         */
        OUSTER_API_FUNCTION
        ValidatorEntry(const ValidatorEntry& other) = default;

        // Move assignment constructor not available due to const members.
        ValidatorEntry& operator=(const ValidatorEntry& other) = delete;

        /**
         * Move constructor to move from another validator
         * issue entry.
         *
         * @param[in] other The other validator entry to move from.
         */
        OUSTER_API_FUNCTION
        ValidatorEntry(ValidatorEntry&& other) = default;

        // Move assignment constructor not available due to const members.
        ValidatorEntry& operator=(ValidatorEntry&& other) = delete;

        /**
         * Destructor.
         */
        OUSTER_API_FUNCTION
        ~ValidatorEntry() = default;

        /**
         * Return the string representation of the validation issue.
         *
         * @return the string representation of the validation issue.
         */
        OUSTER_API_FUNCTION
        std::string to_string() const;

        /**
         * Return the json path associated with the issue.
         *
         * @return the json path associated with the issue.
         */
        OUSTER_API_FUNCTION
        const std::string& get_path() const;

        /**
         * Return the specific issue.
         *
         * @return the specific issue.
         */
        OUSTER_API_FUNCTION
        const std::string& get_msg() const;

       protected:
        const std::string path_;  ///< The json path for the issue
        const std::string msg_;   ///< The specific issue
    };

    /**
     * Convenience alias for the issue list
     */
    using EntryList = std::vector<ValidatorEntry>;

    /**
     * Return the string representation of all of the validation issues.
     *
     * @return the string representation of all of the validation issues.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

    EntryList information;  ///< Validation issues at the information level
    EntryList warning;      ///< Validation issues at the warning level
    EntryList critical;     ///< Validation issues at the critical level
};

/**
 * Return the string representation of all of the validation issues in an
 * EntryList.
 *
 * @param[in] list The EntryList to get the string representation for.
 *
 * @return the string representation of all of the validation issues in an
 *         EntryList.
 */
OUSTER_API_FUNCTION
std::string to_string(const ValidatorIssues::EntryList& list);

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[out] issues The issues that occured during parsing.
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_metadata(const std::string& json_data,
                                 ValidatorIssues& issues);

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[in] sensor_info The optional sensor_info to populate.
 * @param[in] issues The issues that occurred during parsing.
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_metadata(
    const std::string& json_data,
    nonstd::optional<ouster::sensor::sensor_info>& sensor_info,
    ValidatorIssues& issues);

/**
 * Parse config text blob from the sensor into a sensor_config struct.
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] config a text blob given by get_config from client.h.
 * @param[out] sensor_config The optional sensor config object, if parsing
 *                           issues occur, will be empty.
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_config(const std::string& config,
                               ouster::sensor::sensor_config& sensor_config);

/**
 * Parse config text blob from the sensor into a sensor_config struct.
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] config a text blob given by get_config from client.h.
 * @param[out] sensor_config The optional sensor config object, if parsing
 *                           issues occur, will be empty.
 * @param[out] issues The specific issues parsing the sensor config
 *
 * @return If parsing was successful(no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_config(const std::string& config,
                               ouster::sensor::sensor_config& sensor_config,
                               ValidatorIssues& issues);
};  // namespace ouster
