/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Json parsing and validation tools.
 */

#include "ouster/json_tools.h"

#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "ouster/impl/json_tools.h"

using ouster::sdk::core::mat4d_from_array;

namespace ouster {
namespace sdk {
namespace core {

// NOLINTBEGIN(modernize-pass-by-value)
ValidatorIssues::ValidatorEntry::ValidatorEntry(const std::string& path,
                                                const std::string& msg)
    : path_(path), msg_(msg) {}
// NOLINTEND(modernize-pass-by-value)

std::string ValidatorIssues::ValidatorEntry::to_string() const {
    std::stringstream error_message;
    error_message << path_ << ": ";
    error_message << msg_;

    return error_message.str();
}

const std::string& ValidatorIssues::ValidatorEntry::get_path() const {
    return path_;
}

const std::string& ValidatorIssues::ValidatorEntry::get_msg() const {
    return msg_;
}

std::string to_string(const ValidatorIssues::EntryList& list) {
    std::stringstream output_string;
    for (auto const& it : list) {
        output_string << it.to_string() << std::endl;
    }
    return output_string.str();
}

std::string ValidatorIssues::to_string() const {
    std::stringstream output_string;
    if (!critical.empty()) {
        output_string << "Critical Issues:" << std::endl;
        output_string << ouster::sdk::core::to_string(critical);
    }
    if (!warning.empty()) {
        output_string << "Warning Issues:" << std::endl;
        output_string << ouster::sdk::core::to_string(warning);
    }
    if (!information.empty()) {
        output_string << "Information Issues:" << std::endl;
        output_string << ouster::sdk::core::to_string(information);
    }
    return output_string.str();
}

namespace impl {

JsonTools::JsonTools(const jsoncons::json& root, ValidatorIssues& issues)
    : root_(root), issues_(issues) {}

void JsonTools::skipped_due_to_item(ValidatorIssues::EntryList& severity,
                                    const std::string& item_skipped,
                                    const std::string& cause_item,
                                    const std::string& explanation) {
    std::stringstream error_message;
    error_message << "Validation step for path: \"" << item_skipped
                  << "\" skipped"
                  << " due to failures validating path: \"" << cause_item
                  << "\"." << explanation;

    auto entry =
        ValidatorIssues::ValidatorEntry(item_skipped, error_message.str());
    severity.push_back(entry);
}

bool JsonTools::path_exists(const std::string& path) {
    return (!jsoncons::jsonpath::json_query(root_, path).empty());
}

// TODO[tws] this method should accept an array instead of a vector
void JsonTools::decode_transform_array(mat4d& output,
                                       const std::vector<double>& data) {
    if (data.size() != mat4d::SizeAtCompileTime) {
        throw std::invalid_argument("Transform array must have exactly " +
                                    std::to_string(mat4d::SizeAtCompileTime) +
                                    " elements.");
    }
    std::array<double, mat4d::SizeAtCompileTime> arr;
    std::copy_n(data.begin(), data.size(), arr.begin());
    output = mat4d_from_array(arr);
}

void JsonTools::default_message(const std::string& path) {
    auto entry = ValidatorIssues::ValidatorEntry(
        path, "Metadata entry not found (" + path + "), using defaults");
    issues_.information.push_back(entry);
}

};  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
