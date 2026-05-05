/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Json parsing and validation tools.
 */
#pragma once

#include <string>
#include <vector>

#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

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

}  // namespace core
}  // namespace sdk
};  // namespace ouster
