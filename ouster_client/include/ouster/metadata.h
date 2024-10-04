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

namespace ouster {

/**
 * Class for representing metadata issues.
 */
struct ValidatorIssues {
   public:
    /**
     * Subclass for recording validator issues
     */
    class ValidatorEntry {
       public:
        /**
         * Construct a validator issue entry.
         *
         * @param[in] path The json path associated with the issue.
         * @param[in] msg The specific issue.
         */
        ValidatorEntry(const std::string& path, const std::string& msg);

        /**
         * Return the string representation of the validation issue.
         *
         * @return the string representation of the validation issue.
         */
        std::string to_string() const;

        /**
         * Return the json path associated with the issue.
         *
         * @return the json path associated with the issue.
         */
        const std::string& get_path() const;

        /**
         * Return the specific issue.
         *
         * @return the specific issue.
         */
        const std::string& get_msg() const;

       protected:
        const std::string path;  ///< The json path for the issue
        const std::string msg;   ///< The specific issue
    };

    /**
     * Convenience alias for the issue list
     */
    using EntryList = std::vector<ValidatorEntry>;
    EntryList information;  ///< Validation issues at the information level
    EntryList warning;      ///< Validation issues at the warning level
    EntryList critical;     ///< Validation issues at the critical level
};

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[in] issues The issues that occured during parsing.
 * @return If parsing was successful(no critical issues)
 */
bool parse_and_validate_metadata(const std::string& json_data,
                                 ValidatorIssues& issues);

/**
 * Parse and validate a metadata stream.
 *
 * @param[in] json_data The metadata data.
 * @param[in] sensor_info The optional sensor_info to populate.
 * @param[in] issues The issues that occurred during parsing.
 * @return If parsing was successful(no critical issues)
 */
bool parse_and_validate_metadata(
    const std::string& json_data,
    nonstd::optional<ouster::sensor::sensor_info>& sensor_info,
    ValidatorIssues& issues);

};  // namespace ouster
