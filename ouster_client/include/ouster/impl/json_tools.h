/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Json parsing and validation tools.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <functional>
#include <iomanip>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/json_tools.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

#ifdef __GNUG__
#include <cxxabi.h>
#endif

namespace ouster {
namespace sdk {
namespace core {
namespace impl {
#ifdef __GNUG__
/**
 * Function for unmangling cpp types.
 *
 * @param[in] name The mangled cpp type string.
 * @return The unmangled cpp type string.
 */
inline std::string fix_typename(const char* name) {
    std::size_t length = 0;
    int status = 0;
    std::unique_ptr<char, decltype(&std::free)> pointer(
        __cxxabiv1::__cxa_demangle(name, nullptr, &length, &status),
        &std::free);
    return pointer.get();
}
#else
/**
 * Function for unmangling cpp types.
 *
 * @param[in] name The mangled cpp type string.
 * @return The unmangled cpp type string.
 */
inline std::string fix_typename(const char* name) { return name; }
#endif

class JsonTools {
   protected:
    /**
     * Internal class for parsing and validating json files.
     *
     * @param[in] root The root of the json object to parse and validate.
     * @param[out] issues The issues that occurred during
     * parsing and validation.
     */
    JsonTools(const jsoncons::json& root, ValidatorIssues& issues);

    // ===============    Data    ===============
    const jsoncons::json& root_;  ///< The json root
    ValidatorIssues& issues_;     ///< The validation output

    // =============== Utilities  ===============
    /**
     * Utility function to emit a validation issue due to missing
     * prerequisite parse.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] item_skipped The item that was unable to be run due
     *                         to missing prerequisite.
     * @param[in] cause_item The prerequisite that caused the skip.
     * @param[in] explanation Additional information around the issue.
     */
    static void skipped_due_to_item(ValidatorIssues::EntryList& severity,
                                    const std::string& item_skipped,
                                    const std::string& cause_item,
                                    const std::string& explanation = "");

    /**
     * Utility function to test if a json path exists in the json data.
     *
     * @param[in] path The path to test.
     *
     * @return If the json path exists in the json data.
     */
    bool path_exists(const std::string& path);

    /**
     * Utility function to extract a mat4d dataset from a single
     * dimensional vector.
     *
     * @param[out] output The mat4d output to extract to.
     * @param[in] data The single dimensional vector to extract from.
     */
    static void decode_transform_array(mat4d& output,
                                       const std::vector<double>& data);

    /**
     * Utility function to emit a validation issue on using a default value.
     *
     * @param[in] path The path to use for emitting the validation issue.
     */
    void default_message(const std::string& path);

    // =============== Validators ===============
    /**
     * Post parsing validator to validate that there are at least
     * some non-zero entries. Path is only used for the validation event,
     * the data is coming in via the data parameter.
     *
     * @tparam T The type of data to verify, should be something
     *           that can be turned into a number.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for emitting the validation issue.
     * @param[in] data The data to validate.
     *
     * @return There are at least some non-zero entries in the data.
     */
    template <typename T,
              typename = std::enable_if<std::is_arithmetic<T>::value, T>>
    static bool verify_all_not_zero(ValidatorIssues::EntryList& severity,
                                    const std::string& path, T& data) {
        uint64_t zeros = 0;
        for (auto it : data) {
            if (it == 0) {
                zeros++;
            }
        }

        if (zeros == data.size()) {
            std::stringstream error_message;
            error_message
                << "Expected at least some non-zero values in metadata array";

            auto entry =
                ValidatorIssues::ValidatorEntry(path, error_message.str());
            severity.push_back(entry);
        } else {
            return true;
        }

        return false;
    }

    /**
     * Callback validator to use in a parse_and_validate_item call to verify
     * that a single resulting string is not empty.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for emitting the validation issue.
     * @param[in] data The data to validate.
     *
     * @return The string in data is not empty.
     */
    static bool verify_string_not_empty(ValidatorIssues::EntryList& severity,
                                        const std::string& path,
                                        const std::string& data) {
        if (!data.empty()) {
            return true;
        } else {
            std::stringstream error_message;
            error_message << "String that was expected to contain data"
                          << " was empty";

            auto entry =
                ValidatorIssues::ValidatorEntry(path, error_message.str());
            severity.push_back(entry);
        }
        return false;
    }

    /**
     * Method used to create a Callback validator to use
     * in a parse_and_validate_item call to verify
     * that a single resulting numeric value is inbetween
     * the lower and upper bounds. This specifically returns a
     * callback rather than being the callback due to the need
     * to specificy the bounds at the time of calling.
     *
     * @tparam T The type of data to verify, should be something
     *           that can be turned into a number.
     *
     * @param[in] lower The lower bound to validate against.
     * @param[in] upper The upper bound to validate against.
     *
     * @return A callback to use in a parse_and_validate_item call.
     */
    template <typename T,
              typename = std::enable_if<std::is_arithmetic<T>::value, T>>
    static std::function<bool(ValidatorIssues::EntryList&, const std::string&,
                              T)>
    make_verify_in_bounds(T lower, T upper) {
        return [lower, upper](ValidatorIssues::EntryList& severity,
                              const std::string& path, T data) {
            bool result = true;
            if (data < lower) {
                std::stringstream error_message;
                error_message << "Item value " << data
                              << " is lower than the lower bound " << lower;

                auto entry =
                    ValidatorIssues::ValidatorEntry(path, error_message.str());
                severity.push_back(entry);
                result = false;
            }

            if (data > upper) {
                std::stringstream error_message;
                error_message << "Item value " << data
                              << " is greater than the upper bound " << upper;

                auto entry =
                    ValidatorIssues::ValidatorEntry(path, error_message.str());
                severity.push_back(entry);
                result = false;
            }

            return result;
        };
    }

    // =============== Processors ===============
    /**
     * The main method for parsing and validating a single item in
     * a json dataset.
     *
     * @tparam T The type of data to validate
     * @tparam F Function type for the single item validation callback.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The variable to store the parsed results in.
     * @param[in] verification_callback The callback to call on each singular
     *     item to validate them.
     * @param[in] relaxed_number_verification For numeric types, setting this to
     *     true will accept any numeric value as the type specified.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_item(ValidatorIssues::EntryList& severity,
                                 const std::string& path, T& output,
                                 F verification_callback,
                                 bool relaxed_number_verification = false) {
        jsoncons::json value_array =
            jsoncons::jsonpath::json_query(root_, path);
        if (value_array.size() == 1) {
            jsoncons::json value = value_array[0];
            if (value.is<T>() ||
                (relaxed_number_verification && value.is_number() &&
                 std::is_arithmetic<T>::value)) {
                // Warning: ran into a really weird argument swapping issue here
                // we think it was related to ABI issues.
                output = value.as<T>();
                bool temp_result =
                    verification_callback(severity, path, value.as<T>());
                return temp_result;
            } else {
                try {
                    output = value.as<T>();
                } catch (...) {
                }
                std::stringstream error_message;
                error_message << "Type Expected: \""
                              << impl::fix_typename(typeid(T).name())
                              << "\" Actual Type: " << value.type()
                              << "\" Value: \"" << value << "\"";
                auto entry =
                    ValidatorIssues::ValidatorEntry(path, error_message.str());
                severity.push_back(entry);
            }
        } else {
            std::stringstream error_message;
            error_message << "Expected One Item In Data, "
                          << "Number Of Items: " << value_array.size()
                          << " Values: \"" << value_array << "\"";
            auto entry =
                ValidatorIssues::ValidatorEntry(path, error_message.str());
            severity.push_back(entry);
        }
        return false;
    }

    /**
     * Method for parsing and validating a single item in a json dataset.
     *
     * @tparam T The type of data to validate
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The variable to store the parsed results in.
     * @param[in] relaxed_number_verification For numeric types, setting this to
     *     true will accept any numeric value as the type specified.
     *
     * @return If the data was successfully validated.
     */
    template <typename T>
    bool parse_and_validate_item(ValidatorIssues::EntryList& severity,
                                 const std::string& path, T& output,
                                 bool relaxed_number_verification = false) {
        return parse_and_validate_item<T>(
            severity, path, output,
            [&](ValidatorIssues::EntryList& /*severity*/,
                const std::string& /*path*/, T /*data*/) { return true; },
            relaxed_number_verification);
    }

    /**
     * Method for parsing and validating a single optional item in a
     * json dataset.
     *
     * @tparam T The type of data to validate
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The variable to store the optional parsed results in.
     * @param[in] relaxed_number_verification For numeric types, setting this to
     *     true will accept any numeric value as the type specified.
     *
     * @return If the data was successfully validated.
     */
    template <typename T>
    bool parse_and_validate_item(ValidatorIssues::EntryList& severity,
                                 const std::string& path,
                                 nonstd::optional<T>& output,
                                 bool relaxed_number_verification = false) {
        T data;
        if (parse_and_validate_item<T>(severity, path, data,
                                       relaxed_number_verification)) {
            output = data;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method for parsing and validating a single optional item in a
     * json dataset while providing a validation callback.
     *
     * @tparam T The type of data to validate
     * @tparam F Function type for the single optional item validation callback.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The variable to store the parsed results in.
     * @param[in] verification_callback The callback to call on each singular
     *     optional item to validate them.
     * @param[in] relaxed_number_verification For numeric types, setting this to
     *     true will accept any numeric value as the type specified.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_item(ValidatorIssues::EntryList& severity,
                                 const std::string& path,
                                 nonstd::optional<T>& output,
                                 F verification_callback,
                                 bool relaxed_number_verification = false) {
        T data;
        bool temp_result = parse_and_validate_item<T, F>(
            severity, path, data, verification_callback,
            relaxed_number_verification);
        if (temp_result) {
            output = data;
        }

        return temp_result;
    }

    /**
     * Method for parsing and validating an array of items in a
     * json dataset.
     *
     * @tparam T The type of data to validate
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The vector variable to store the parsed results in.
     * @param[in] verify_count The size of the array that is expected.
     * @param[in] relaxed_number_verification For numeric types, setting this to
     *     true will accept any numeric value as the type specified.
     *
     * @return If the data was successfully validated.
     */
    template <typename T>
    bool parse_and_validate_item(ValidatorIssues::EntryList& severity,
                                 const std::string& path,
                                 std::vector<T>& output, size_t verify_count,
                                 bool relaxed_number_verification = false) {
        size_t index = 0;
        size_t matches = 0;
        std::vector<T> shadow_output;
        auto parse_callback = [&](const std::string& path,
                                  const jsoncons::json& /*val*/) {
            T data;
            if (parse_and_validate_item<T>(severity, path, data,
                                           relaxed_number_verification)) {
                matches++;
            }
            shadow_output.push_back(data);
            index++;
        };
        jsoncons::jsonpath::json_query(root_, path, parse_callback);
        bool result = (index == matches && matches > 0);
        if (verify_count > 0 && matches != verify_count) {
            std::stringstream error_message;
            error_message << "Invalid metadata array, got " << index
                          << " items, " << matches << " matching items,"
                          << " was expecting " << verify_count
                          << " matching items";
            severity.emplace_back(path, error_message.str());
            result = false;
        }

        if (!shadow_output.empty()) {
            output = shadow_output;
        }

        return result;
    }

    /**
     * Method for parsing and validating an array of items in a
     * json dataset  while providing a validation callback.
     *
     * @tparam T The type of data to validate
     * @tparam F Function type for the single item validation callback.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The vector variable to store the parsed results in.
     * @param[in] verify_count The size of the array that is expected.
     * @param[in] verification_callback The callback to call on each singular
     *     optional item to validate them.
     * @param[in] relaxed_number_verification For numeric types, setting this to
     *     true will accept any numeric value as the type specified.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_item(ValidatorIssues::EntryList& severity,
                                 const std::string& path,
                                 std::vector<T>& output, size_t verify_count,
                                 F verification_callback,
                                 bool relaxed_number_verification = false) {
        size_t index = 0;
        size_t matches = 0;
        std::vector<T> shadow_output;
        auto parse_callback = [&](const std::string& path,
                                  const jsoncons::json& /*val*/) {
            T data;
            if (parse_and_validate_item<T, F>(severity, path, data,
                                              verification_callback,
                                              relaxed_number_verification)) {
                matches++;
            }
            shadow_output.push_back(data);
            index++;
        };
        jsoncons::jsonpath::json_query(root_, path, parse_callback);
        bool result = (index == matches && matches > 0);
        if (verify_count > 0 && matches != verify_count) {
            std::stringstream error_message;
            error_message << "Invalid metadata array, got " << index
                          << " items, " << matches << " matching items,"
                          << " was expecting " << verify_count
                          << " matching items";
            severity.emplace_back(path, error_message.str());
            result = false;
        }

        if (!shadow_output.empty()) {
            output = shadow_output;
        }

        return result;
    }

    /**
     * Method for parsing and validating an optional enum value.
     *
     * @tparam T The type of data the enum is stored in the json dataset.
     * @tparam U The type that the enum should be stored in.
     * @tparam F Function type to turn the json data into the cpp enum.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The variable to store the optional parsed results in.
     * @param[in] func Function to turn the json data into the cpp enum.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename U, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_enum(ValidatorIssues::EntryList& severity,
                                 const std::string& path,
                                 nonstd::optional<U>& output, F func) {
        T data;
        if (parse_and_validate_item<T>(severity, path, data)) {
            try {
                output = func(data);
            } catch (std::exception& e) {
                std::stringstream error_message;
                error_message << "Failed To Parse Enum: " << data
                              << " Error Message: \"" << e.what() << "\"";
                severity.emplace_back(path, error_message.str());
                return false;
            }
            if (output.has_value()) {
                return true;
            } else {
                std::stringstream error_message;
                error_message << "Invalid Entry: " << data;
                severity.emplace_back(path, error_message.str());
                output.reset();
                return false;
            }
        }
        return false;
    }

    /**
     * Method for parsing and validating an enum value.
     *
     * @tparam T The type of data the enum is stored in the json dataset.
     * @tparam U The type that the enum should be stored in.
     * @tparam F Function type to turn the json data into the cpp enum.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[out] output The variable to store the parsed results in.
     * @param[in] func Function to turn the json data into the cpp enum.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename U, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_enum(ValidatorIssues::EntryList& severity,
                                 const std::string& path, U& output, F func) {
        nonstd::optional<U> temp_output;
        bool result =
            parse_and_validate_enum<T, U, F>(severity, path, temp_output, func);
        if (result) {
            output = *temp_output;
        }
        return result;
    }

    /**
     * Method for parsing and validating an optional datetime.
     *
     * @tparam T The type of data the datetime should be stored in.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[in] date_format The get_time format to try and decode using.
     * @param[out] output The variable to store the optional parsed results in.
     *
     * @return If the data was successfully validated.
     */
    template <typename T>
    bool parse_and_validate_datetime(ValidatorIssues::EntryList& severity,
                                     const std::string& path,
                                     const std::string& date_format,
                                     nonstd::optional<T>& output) {
        T data;
        if (parse_and_validate_item(severity, path, data,
                                    verify_string_not_empty)) {
            std::istringstream date_data(data);
            std::tm time = {};
            date_data.imbue(std::locale("C"));
            date_data >> std::get_time(&time, date_format.c_str());
            if (date_data.fail()) {
                std::stringstream error_message;
                error_message
                    << "Build date not a properly formatted DateTime: \"";
                error_message << data << "\"";

                auto entry =
                    ValidatorIssues::ValidatorEntry(path, error_message.str());
                severity.push_back(entry);

                return false;
            } else {
                output = data;
                return true;
            }
        }

        return false;
    }

    /**
     * Method for parsing and validating a datetime.
     *
     * @tparam T The type of data the datetime should be stored in.
     *
     * @param[out] severity The severity list to log the issue under.
     * @param[in] path The path to use for parsing and verification.
     * @param[in] date_format The get_time format to try and decode using.
     * @param[out] output The variable to store the parsed results in.
     *
     * @return If the data was successfully validated.
     */
    template <typename T>
    bool parse_and_validate_datetime(ValidatorIssues::EntryList& severity,
                                     const std::string& path,
                                     const std::string& date_format,
                                     T& output) {
        nonstd::optional<T> data;
        auto result =
            parse_and_validate_datetime(severity, path, date_format, data);
        if (data.has_value()) {
            output = *data;
        }

        return result;
    }
};

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
