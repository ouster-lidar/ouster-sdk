/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster metadata processing
 */

#include "ouster/metadata.h"

#include <exception>
#include <functional>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <string>
#include <type_traits>

#include "ouster/impl/logging.h"

#ifdef __GNUG__
#include <cxxabi.h>
/**
 * Function for unmangling cpp types.
 *
 * @param[in] name The mangled cpp type string.
 * @return The unmangled cpp type string.
 */
std::string fix_typename(const char* name) {
    std::size_t _length = 0;
    int _status = 0;
    std::unique_ptr<char, decltype(&std::free)> pointer(
        __cxxabiv1::__cxa_demangle(name, nullptr, &_length, &_status),
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
std::string fix_typename(const char* name) { return name; }
#endif

namespace ouster {

namespace sensor {
extern data_format default_data_format(lidar_mode mode);
extern double default_lidar_origin_to_beam_origin(std::string prod_line);
extern mat4d default_beam_to_lidar_transform(std::string prod_line);
extern const mat4d default_imu_to_sensor =
    (mat4d() << 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1)
        .finished();

extern const mat4d default_lidar_to_sensor =
    (mat4d() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1)
        .finished();
};  // namespace sensor

ValidatorIssues::ValidatorEntry::ValidatorEntry(const std::string& path,
                                                const std::string& msg)
    : path(path), msg(msg) {}

std::string ValidatorIssues::ValidatorEntry::to_string() const {
    std::stringstream errorMessage;
    errorMessage << path << ": ";
    errorMessage << msg;

    return errorMessage.str();
}

const std::string& ValidatorIssues::ValidatorEntry::get_path() const {
    return path;
}
const std::string& ValidatorIssues::ValidatorEntry::get_msg() const {
    return msg;
}

class MetadataImpl {
   public:
    /**
     * Internal class for parsing and validating metadata.
     *
     * @param[in] root The root of the json object to parse and validate.
     * @param[out] result The resulting metadata parsed and validated.
     */
    MetadataImpl(const jsoncons::json& root,
                 ouster::sensor::sensor_info& sensor_info,
                 ValidatorIssues& issues)
        : root(root),
          sensor_info(sensor_info),
          issues(issues),
          have_prod_line(false),
          prod_line_string("$.sensor_info.prod_line"),
          have_lidar_mode(false),
          lidar_mode_string("$.config_params.lidar_mode"),
          have_pixels_per_column(false),
          pixels_per_column_string("$.lidar_data_format.pixels_per_column") {
        parse_and_validate_sensor_info();
        parse_and_validate_config_params();
        // parse_and_validate_sensor_info must be run before
        // parse_and_validate_data_format
        //  due to requirements on prod_line
        // parse_and_validate_config_params must be run before
        // parse_and_validate_data_format
        //  due to requirements on lidar_mode
        parse_and_validate_data_format();
        parse_and_validate_calibration_status();
        // parse_and_validate_sensor_info must be run before
        // parse_and_validate_data_format
        //  due to requirements on prod_line
        //  parse_and_validate_config_params must be run before
        //  parse_and_validate_data_format due to requirements on lidar_mode
        //  parse_and_validate_data_format must be run before
        //  parse_and_validate_intrinsics due to requirements on
        //  pixels_per_column
        parse_and_validate_intrinsics();
        parse_and_validate_misc();
    }

   protected:
    // Data
    const jsoncons::json& root;  ///< The json root

    ouster::sensor::sensor_info& sensor_info;  ///< The output sensor info
    ValidatorIssues& issues;                   ///< The validation output

    /**
     * Variable to keep track of the status of the prodline.
     * Prodline is used in later checks to validate certain
     * data.
     */
    bool have_prod_line;

    /**
     * Json path for the prodline. This is located at class level
     * so that it can be used in skippedDueToItem calls.
     */
    const std::string prod_line_string;

    /**
     * Variable to keep track of the status of lidar mode.
     * The lidar mode is used in later checks to validate certain
     * data.
     */
    bool have_lidar_mode;

    /**
     * Json path for the lidar mode. This is located at class level
     * so that it can be used in skippedDueToItem calls.
     */
    const std::string lidar_mode_string;

    /**
     * Variable to keep track of the status of pixels per column.
     * Pixels per column is used in later checks to validate certain
     * data.
     */
    bool have_pixels_per_column;

    /**
     * Json path for pixels per column. This is located at class
     * level so that it can be used in skippedDueToItem calls.
     */
    const std::string pixels_per_column_string;

    // Utilities
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
    void skipped_due_to_item(ValidatorIssues::EntryList& severity,
                             const std::string& item_skipped,
                             const std::string& cause_item,
                             const std::string explanation = "") {
        std::stringstream errorMessage;
        errorMessage << "Item \"" << item_skipped << "\" Skipped"
                     << " Due to failures with \"" << cause_item << "\" "
                     << explanation;

        auto entry =
            ValidatorIssues::ValidatorEntry(item_skipped, errorMessage.str());
        severity.push_back(entry);
    }

    /**
     * Utility function to test if a json path exists in the json data.
     *
     * @param[in] path The path to test.
     *
     * @return If the json path exists in the json data.
     */
    bool path_exists(const std::string& path) {
        return (jsoncons::jsonpath::json_query(root, path).size() > 0);
    }

    /**
     * Utility function to extract a mat4d dataset from a single
     * dimensional vector.
     *
     * @param[out] output The mat4d output to extract to.
     * @param[in] data The single dimensional vector to extract from.
     */
    void decode_transform_array(mat4d& output,
                                const std::vector<double>& data) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                output(i, j) = data[i * 4 + j];
            }
        }
    }

    /**
     * Utility function to emit a validation issue on using a default value.
     *
     * @param[in] path The path to use for emitting the validation issue.
     */
    void default_message(const std::string& path) {
        auto entry = ValidatorIssues::ValidatorEntry(
            path, "Item not found, using defaults");
        issues.information.push_back(entry);
    }

    // Validators
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
        long unsigned int zeros = 0;
        for (auto it : data) {
            if (it == 0) {
                zeros++;
            }
        }

        if (zeros == data.size()) {
            std::stringstream errorMessage;
            errorMessage << "Expected at least some non-zero values in path";

            auto entry =
                ValidatorIssues::ValidatorEntry(path, errorMessage.str());
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
        if (data.length() > 0) {
            return true;
        } else {
            std::stringstream errorMessage;
            errorMessage << "String that was expected to contain data"
                         << " was empty.";

            auto entry =
                ValidatorIssues::ValidatorEntry(path, errorMessage.str());
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
                std::stringstream errorMessage;
                errorMessage << "Item value " << data
                             << " is lower than the lower bound " << lower;

                auto entry =
                    ValidatorIssues::ValidatorEntry(path, errorMessage.str());
                severity.push_back(entry);
                result = false;
            }

            if (data > upper) {
                std::stringstream errorMessage;
                errorMessage << "Item value " << data
                             << " is greater than the upper bound " << upper;

                auto entry =
                    ValidatorIssues::ValidatorEntry(path, errorMessage.str());
                severity.push_back(entry);
                result = false;
            }

            return result;
        };
    }

    // Processors
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
        jsoncons::json value_array = jsoncons::jsonpath::json_query(root, path);
        if (value_array.size() == 1) {
            jsoncons::json value = value_array[0];
            if (value.is<T>() ||
                (relaxed_number_verification && value.is_number() &&
                 std::is_arithmetic<T>::value)) {
                output = value.as<T>();
                bool temp_result =
                    verification_callback(severity, path, value.as<T>());
                return temp_result;
            } else {
                try {
                    output = value.as<T>();
                } catch (...) {
                }
                std::stringstream errorMessage;
                errorMessage
                    << "Type Expected: \"" << fix_typename(typeid(T).name())
                    << "\" Actual Type: " << value.type() << "\" Value: \""
                    << value << "\"";
                auto entry =
                    ValidatorIssues::ValidatorEntry(path, errorMessage.str());
                severity.push_back(entry);
            }
        } else {
            std::stringstream errorMessage;
            errorMessage << "Expected One Item In Query, "
                         << "Number Of Items: " << value_array.size()
                         << " Values: \"" << value_array << "\"";
            auto entry =
                ValidatorIssues::ValidatorEntry(path, errorMessage.str());
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
        if (temp_result) output = data;

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
        jsoncons::jsonpath::json_query(root, path, parse_callback);
        bool result = (index == matches && matches > 0);
        if (verify_count > 0 && matches != verify_count) {
            std::stringstream errorMessage;
            errorMessage << "Invalid array, got " << index << " items, "
                         << matches << " matching items,"
                         << " was expecting " << verify_count
                         << " matching items";
            severity.push_back(
                ValidatorIssues::ValidatorEntry(path, errorMessage.str()));
            result = false;
        }

        if (shadow_output.size() > 0) output = shadow_output;

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
        jsoncons::jsonpath::json_query(root, path, parse_callback);
        bool result = (index == matches && matches > 0);
        if (verify_count > 0 && matches != verify_count) {
            std::stringstream errorMessage;
            errorMessage << "Invalid array, got " << index << " items, "
                         << matches << " matching items,"
                         << " was expecting " << verify_count
                         << " matching items";
            severity.push_back(
                ValidatorIssues::ValidatorEntry(path, errorMessage.str()));
            result = false;
        }

        if (shadow_output.size() > 0) output = shadow_output;

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
     * @param[in] f Function to turn the json data into the cpp enum.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename U, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_enum(ValidatorIssues::EntryList& severity,
                                 const std::string& path,
                                 nonstd::optional<U>& output, F f) {
        T data;
        if (parse_and_validate_item<T>(severity, path, data)) {
            try {
                output = f(data);
            } catch (std::exception& e) {
                std::stringstream errorMessage;
                errorMessage << "Failed To Parse: " << data
                             << " Error Message: \"" << e.what() << "\"";
                severity.push_back(
                    ValidatorIssues::ValidatorEntry(path, errorMessage.str()));
                return false;
            }
            if (output.has_value()) {
                return true;
            } else {
                std::stringstream errorMessage;
                errorMessage << "Invalid Entry: " << data;
                severity.push_back(
                    ValidatorIssues::ValidatorEntry(path, errorMessage.str()));
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
     * @param[in] f Function to turn the json data into the cpp enum.
     *
     * @return If the data was successfully validated.
     */
    template <typename T, typename U, typename F,
              typename = std::enable_if<std::is_function<F>::value, F>>
    bool parse_and_validate_enum(ValidatorIssues::EntryList& severity,
                                 const std::string& path, U& output, F f) {
        nonstd::optional<U> temp_output;
        bool result =
            parse_and_validate_enum<T, U, F>(severity, path, temp_output, f);
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
            std::tm t = {};
            date_data.imbue(std::locale("en_US.utf-8"));
            date_data >> std::get_time(&t, date_format.c_str());
            if (date_data.fail()) {
                std::stringstream errorMessage;
                errorMessage
                    << "Build date not a properly formatted DateTime: \"";
                errorMessage << data << "\"";

                auto entry =
                    ValidatorIssues::ValidatorEntry(path, errorMessage.str());
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

    // Sections
    void parse_and_validate_sensor_info() {
        parse_and_validate_datetime(issues.information,
                                    "$.sensor_info.build_date", "%Y-%m-%dT%TZ",
                                    sensor_info.build_date);

        parse_and_validate_item(issues.information, "$.sensor_info.build_rev",
                                sensor_info.fw_rev, verify_string_not_empty);

        parse_and_validate_item(issues.information, "$.sensor_info.image_rev",
                                sensor_info.image_rev, verify_string_not_empty);

        parse_and_validate_item(issues.information,
                                "$.sensor_info.initialization_id",
                                sensor_info.init_id);

        if (parse_and_validate_item(issues.information, prod_line_string,
                                    sensor_info.prod_line)) {
            try {
                sensor_info.get_product_info();
                have_prod_line = true;
            } catch (std::runtime_error& error) {
                auto entry = ValidatorIssues::ValidatorEntry(prod_line_string,
                                                             error.what());
                issues.warning.push_back(entry);
            }
        }

        parse_and_validate_item(issues.information, "$.sensor_info.prod_pn",
                                sensor_info.prod_pn, verify_string_not_empty);

        parse_and_validate_item(issues.information, "$.sensor_info.prod_sn",
                                sensor_info.sn, verify_string_not_empty);

        parse_and_validate_item(issues.information, "$.sensor_info.status",
                                sensor_info.status, verify_string_not_empty);
    }

    void parse_and_validate_config_params() {
        std::vector<uint64_t> azimuth_window_data;
        if (parse_and_validate_item(
                issues.information, "$.config_params.azimuth_window.*",
                azimuth_window_data, 2,
                make_verify_in_bounds<uint64_t>(0, 360000))) {
            sensor_info.config.azimuth_window = {azimuth_window_data[0],
                                                 azimuth_window_data[1]};
        }

        parse_and_validate_item(issues.information,
                                "$.config_params.columns_per_packet",
                                sensor_info.config.columns_per_packet);

        if (parse_and_validate_enum<std::string>(
                issues.information, lidar_mode_string,
                sensor_info.config.lidar_mode, sensor::lidar_mode_of_string)) {
            have_lidar_mode = true;
        }

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.multipurpose_io_mode",
            sensor_info.config.multipurpose_io_mode,
            ouster::sensor::multipurpose_io_mode_of_string);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.nmea_baud_rate",
            sensor_info.config.nmea_baud_rate,
            ouster::sensor::nmea_baud_rate_of_string);

        uint64_t nmea_ignore_valid_char;
        if (parse_and_validate_item(issues.information,
                                    "$.config_params.nmea_ignore_valid_char",
                                    nmea_ignore_valid_char)) {
            sensor_info.config.nmea_ignore_valid_char =
                (nmea_ignore_valid_char != 0);
        }

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.nmea_in_polarity",
            sensor_info.config.nmea_in_polarity,
            ouster::sensor::polarity_of_string);

        parse_and_validate_item(issues.information,
                                "$.config_params.nmea_leap_seconds",
                                sensor_info.config.nmea_leap_seconds, true);

        const std::string operating_mode_string =
            "$.config_params.operating_mode";
        if (!parse_and_validate_enum<std::string>(
                issues.information, operating_mode_string,
                sensor_info.config.operating_mode,
                ouster::sensor::operating_mode_of_string)) {
            const std::string auto_start_flag_string =
                "$.config_params.auto_start_flag";
            bool auto_start_flag;
            if (parse_and_validate_item<bool>(issues.information,
                                              auto_start_flag_string,
                                              auto_start_flag)) {
                auto entry = ValidatorIssues::ValidatorEntry(
                    auto_start_flag_string,
                    "Please note that auto_start_flag has been deprecated in "
                    "favor "
                    "of operating_mode. Will set operating_mode "
                    "appropriately...");
                issues.information.push_back(entry);
                sensor_info.config.operating_mode =
                    auto_start_flag ? sensor::OPERATING_NORMAL
                                    : sensor::OPERATING_STANDBY;
            } else {
                default_message(operating_mode_string);
            }
        }

        parse_and_validate_item(issues.information,
                                "$.config_params.phase_lock_enable",
                                sensor_info.config.phase_lock_enable);

        parse_and_validate_item(issues.information,
                                "$.config_params.phase_lock_offset",
                                sensor_info.config.phase_lock_offset, true);

        const std::string signal_multiplier_string =
            "$.config_params.signal_multiplier";
        if (parse_and_validate_item(
                issues.information, signal_multiplier_string,
                sensor_info.config.signal_multiplier, true)) {
            try {
                ouster::sensor::check_signal_multiplier(
                    *sensor_info.config.signal_multiplier);
            } catch (std::runtime_error& e) {
                auto entry = ValidatorIssues::ValidatorEntry(
                    signal_multiplier_string, e.what());
                issues.information.push_back(entry);
            }
        }

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.sync_pulse_in_polarity",
            sensor_info.config.sync_pulse_in_polarity,
            ouster::sensor::polarity_of_string);

        parse_and_validate_item(issues.information,
                                "$.config_params.sync_pulse_out_angle",
                                sensor_info.config.sync_pulse_out_angle,
                                make_verify_in_bounds<int>(0, 360), true);

        parse_and_validate_item(
            issues.information, "$.config_params.sync_pulse_out_frequency",
            sensor_info.config.sync_pulse_out_frequency, true);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.sync_pulse_out_polarity",
            sensor_info.config.sync_pulse_out_polarity,
            ouster::sensor::polarity_of_string);

        parse_and_validate_item(
            issues.information, "$.config_params.sync_pulse_out_pulse_width",
            sensor_info.config.sync_pulse_out_pulse_width, true);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.timestamp_mode",
            sensor_info.config.timestamp_mode,
            ouster::sensor::timestamp_mode_of_string);

        if (!parse_and_validate_item(
                issues.information, "$.config_params.udp_dest",
                sensor_info.config.udp_dest, verify_string_not_empty)) {
            parse_and_validate_item(
                issues.information, "$.config_params.udp_ip",
                sensor_info.config.udp_dest, verify_string_not_empty);
        }

        parse_and_validate_item(issues.information,
                                "$.config_params.udp_port_imu",
                                sensor_info.config.udp_port_imu,
                                make_verify_in_bounds<uint16_t>(0, 65535));

        parse_and_validate_item(issues.information,
                                "$.config_params.udp_port_lidar",
                                sensor_info.config.udp_port_lidar,
                                make_verify_in_bounds<uint16_t>(0, 65535));

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.udp_profile_imu",
            sensor_info.config.udp_profile_imu,
            ouster::sensor::udp_profile_imu_of_string);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.udp_profile_lidar",
            sensor_info.config.udp_profile_lidar,
            ouster::sensor::udp_profile_lidar_of_string);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.gyro_fsr",
            sensor_info.config.gyro_fsr,
            ouster::sensor::full_scale_range_of_string);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.accel_fsr",
            sensor_info.config.accel_fsr,
            ouster::sensor::full_scale_range_of_string);

        parse_and_validate_enum<std::string>(
            issues.information, "$.config_params.return_order",
            sensor_info.config.return_order,
            ouster::sensor::return_order_of_string);

        parse_and_validate_item(issues.information,
                                "$.config_params.min_range_threshold_cm",
                                sensor_info.config.min_range_threshold_cm);
    }

    // parse_and_validate_sensor_info must be run before
    // parse_and_validate_data_format due to requirements on prod_line
    // parse_and_validate_config_params must be run before
    // parse_and_validate_data_format due to requirements on lidar_mode
    void parse_and_validate_data_format() {
        if (have_lidar_mode) {
            // lidar mode is present, create default data format
            sensor_info.format = ouster::sensor::default_data_format(
                *sensor_info.config.lidar_mode);
        }
        const std::string columns_per_frame_string =
            "$.lidar_data_format.columns_per_frame";
        bool columns_per_frame_success = false;
        if (parse_and_validate_item<uint32_t>(
                issues.information, columns_per_frame_string,
                sensor_info.format.columns_per_frame)) {
            if (have_lidar_mode) {
                // lidar mode is present, check that number of columns actually
                // matches config value
                auto temp_cols =
                    n_cols_of_lidar_mode(*sensor_info.config.lidar_mode);
                if (sensor_info.format.columns_per_frame != temp_cols) {
                    // found a misconfiguration
                    std::stringstream errorMessage;
                    errorMessage << columns_per_frame_string << "("
                                 << sensor_info.format.columns_per_frame
                                 << ") does not match " << lidar_mode_string
                                 << "(" << temp_cols << ")";

                    auto entry = ValidatorIssues::ValidatorEntry(
                        columns_per_frame_string, errorMessage.str());
                    issues.warning.push_back(entry);
                } else {
                    columns_per_frame_success = true;
                }
            } else {
                // lidar mode not present but columns_per_frame available,
                // nothing to match
                skipped_due_to_item(issues.information,
                                    columns_per_frame_string,
                                    lidar_mode_string);
            }
        } else {
            // need either lidar mode or columns_per_frame
            if (!have_lidar_mode) {
                auto entry = ValidatorIssues::ValidatorEntry(
                    columns_per_frame_string, "Missing field");
                issues.critical.push_back(entry);
            }
            // if we do have lidar mode but not columns_per_frame, we've already
            // applied the default data format
        }

        const std::string column_window_string =
            "$.lidar_data_format.column_window.*";
        std::vector<uint16_t> column_window_data;
        if (!columns_per_frame_success) {
            skipped_due_to_item(issues.information, column_window_string,
                                columns_per_frame_string,
                                "Couldnt verify bounds on column window data");
        }
        auto column_window_callback = [&](ValidatorIssues::EntryList& severity,
                                          const std::string& path,
                                          uint16_t data) {
            if (columns_per_frame_success) {
                return make_verify_in_bounds<uint16_t>(
                    0, sensor_info.format.columns_per_frame - 1)(severity, path,
                                                                 data);
            } else {
                return true;
            }
        };
        if (parse_and_validate_item(issues.information, column_window_string,
                                    column_window_data, 2,
                                    column_window_callback)) {
            sensor_info.format.column_window =
                std::make_pair(column_window_data[0], column_window_data[1]);
        }

        parse_and_validate_item(issues.information,
                                "$.lidar_data_format.columns_per_packet",
                                sensor_info.format.columns_per_packet);

        if (parse_and_validate_item(issues.information,
                                    pixels_per_column_string,
                                    sensor_info.format.pixels_per_column)) {
            if (have_prod_line) {
                // product line is present, check number of pixels per actually
                // matches product line
                auto temp_prod_line = sensor_info.get_product_info();
                if (sensor_info.format.pixels_per_column !=
                    (uint32_t)temp_prod_line.beam_count) {
                    // found a misconfiguration
                    std::stringstream errorMessage;
                    errorMessage << pixels_per_column_string << "("
                                 << sensor_info.format.columns_per_frame
                                 << ") does not match " << prod_line_string
                                 << "(" << temp_prod_line.beam_count
                                 << ") details";

                    auto entry = ValidatorIssues::ValidatorEntry(
                        columns_per_frame_string, errorMessage.str());
                    issues.warning.push_back(entry);
                } else {
                    have_pixels_per_column = true;
                }
            } else {
                // product line not present but pixels_per_column available,
                // nothing to match
                skipped_due_to_item(issues.information,
                                    pixels_per_column_string, prod_line_string);
            }
        } else {
            if (have_prod_line) {
                // if we do have product line but not pixels_per_column,
                // set the pixels_per_column
                auto temp_prod_line = sensor_info.get_product_info();
                sensor_info.format.pixels_per_column =
                    temp_prod_line.beam_count;
                default_message(pixels_per_column_string);
                have_pixels_per_column = true;
            }
        }

        const std::string pixel_shift_string =
            "$.lidar_data_format.pixel_shift_by_row.*";
        int verify_count = 0;
        if (have_prod_line) {
            auto temp_prod_line = sensor_info.get_product_info();
            verify_count = temp_prod_line.beam_count;
        } else {
            skipped_due_to_item(issues.information,
                                pixel_shift_string + ".length()",
                                prod_line_string);
        }
        parse_and_validate_item<int>(issues.information, pixel_shift_string,
                                     sensor_info.format.pixel_shift_by_row,
                                     verify_count);
        // pad pixel shift by row with all zeros if it isnt present or isnt
        // large enough
        if (sensor_info.format.pixel_shift_by_row.size() !=
            sensor_info.format.pixels_per_column) {
            sensor_info.format.pixel_shift_by_row.resize(
                sensor_info.format.pixels_per_column);
        }

        parse_and_validate_enum<std::string>(
            issues.information, "$.lidar_data_format.udp_profile_lidar",
            sensor_info.format.udp_profile_lidar,
            ouster::sensor::udp_profile_lidar_of_string);

        parse_and_validate_enum<std::string>(
            issues.information, "$.lidar_data_format.udp_profile_imu",
            sensor_info.format.udp_profile_imu,
            ouster::sensor::udp_profile_imu_of_string);

        const std::string fps_string = "$.lidar_data_format.fps";
        if (!parse_and_validate_item(issues.information, fps_string,
                                     sensor_info.format.fps)) {
            if (have_lidar_mode) {
                sensor_info.format.fps =
                    frequency_of_lidar_mode(*sensor_info.config.lidar_mode);
                default_message(fps_string);
            } else {
                skipped_due_to_item(issues.information, fps_string,
                                    lidar_mode_string);
            }
        }
    }

    void parse_and_validate_calibration_status() {
        parse_and_validate_datetime(
            issues.information, "$.calibration_status.reflectivity.timestamp",
            "%Y-%m-%dT%T", sensor_info.cal.reflectivity_timestamp);

        if (!parse_and_validate_item(issues.information,
                                     "$.calibration_status.reflectivity.valid",
                                     sensor_info.cal.reflectivity_status)) {
            sensor_info.cal.reflectivity_status.reset();
        }
    }

    /**
     * Emit a type error for angle type issues.
     *
     * @param[in] path The path that has the type issue.
     */
    inline void angle_type_error(const std::string& path) {
        auto entry = ValidatorIssues::ValidatorEntry(
            path,
            "Unexpected type, must be either an array of number or an array of "
            "arrays of numbers.");
        issues.critical.push_back(entry);
    }

    /**
     * Method for parsing and validating beam angles.
     *
     * @param[in] path The path to use for parsing and verification.
     * @param[in] date_format The get_time format to try and decode using.
     * @param[out] output The variable to store the parsed results in.
     */
    void parse_and_validate_angles(const std::string& path,
                                   std::vector<double>& output, size_t width,
                                   size_t height) {
        jsoncons::json value_array = jsoncons::jsonpath::json_query(root, path);
        jsoncons::json angles;
        if (value_array.size() > 0) {
            angles = value_array[0];
        } else {
            auto entry = ValidatorIssues::ValidatorEntry(
                path, "Missing intrinsics field.");
            issues.critical.push_back(entry);
            return;
        }

        bool is_arrays = false;
        bool is_doubles = false;
        if (!angles.is_null()) {
            if (!angles.is_array()) {
                angle_type_error(path);
                return;
            }
            for (const auto& val : angles.array_range()) {
                if (val.is_array()) {
                    if (is_doubles) {
                        angle_type_error(path);
                        return;
                    }
                    is_arrays = true;
                    size_t count = 0;
                    for (const auto& f : val.array_range()) {
                        if (!f.is_number()) {
                            angle_type_error(path);
                            return;
                        }
                        output.push_back(f.as_double());
                        count++;
                    }
                    if (count != height) {
                        auto entry = ValidatorIssues::ValidatorEntry(
                            path, "Each sub-array must have " +
                                      std::to_string(height) + " elements.");
                        issues.critical.push_back(entry);
                        return;
                    }
                } else if (val.is_number()) {
                    if (is_arrays) {
                        angle_type_error(path);
                        return;
                    }
                    is_doubles = true;
                    output.push_back(val.as_double());
                } else {
                    angle_type_error(path);
                    return;
                }
            }

            // now validate outer size
            if (is_doubles || is_arrays) {
                if (angles.size() != width) {
                    auto entry = ValidatorIssues::ValidatorEntry(
                        path, "Must have " + std::to_string(width) +
                                  " elements. Had " +
                                  std::to_string(angles.size()) + " elements.");
                    issues.critical.push_back(entry);
                    return;
                }
            } else {
                // zero size
                auto entry = ValidatorIssues::ValidatorEntry(
                    path, "Cannot be empty array.");
                issues.critical.push_back(entry);
                return;
            }

            // validate not all zero
            verify_all_not_zero(issues.warning, path, output);
        } else {
            // error
            auto entry = ValidatorIssues::ValidatorEntry(path, "Missing.");
            issues.warning.push_back(entry);
        }
    }

    // parse_and_validate_sensor_info must be run before
    // parse_and_validate_data_format due to requirements on prod_line
    // parse_and_validate_config_params must be run before
    // parse_and_validate_data_format due to requirements on lidar_mode
    // parse_and_validate_data_format must be run before
    // parse_and_validate_intrinsics due to requirements on pixels_per_column
    void parse_and_validate_intrinsics() {
        std::vector<double> imu_intrinsics_data;
        const std::string imu_intrinsics_string =
            "$.imu_intrinsics.imu_to_sensor_transform.*";
        if (parse_and_validate_item<double>(issues.information,
                                            imu_intrinsics_string,
                                            imu_intrinsics_data, 16, true)) {
            decode_transform_array(sensor_info.imu_to_sensor_transform,
                                   imu_intrinsics_data);
        } else {
            default_message(imu_intrinsics_string);
            sensor_info.imu_to_sensor_transform =
                ouster::sensor::default_imu_to_sensor;
        }

        std::vector<double> lidar_intrinsics_data;
        const std::string lidar_intrinsics_string =
            "$.lidar_intrinsics.lidar_to_sensor_transform.*";

        if (parse_and_validate_item<double>(issues.information,
                                            lidar_intrinsics_string,
                                            lidar_intrinsics_data, 16, true)) {
            decode_transform_array(sensor_info.lidar_to_sensor_transform,
                                   lidar_intrinsics_data);
        } else {
            default_message(lidar_intrinsics_string);
            sensor_info.lidar_to_sensor_transform =
                ouster::sensor::default_lidar_to_sensor;
        }

        // parse beam angles
        parse_and_validate_angles("$.beam_intrinsics.beam_altitude_angles",
                                  sensor_info.beam_altitude_angles,
                                  sensor_info.format.pixels_per_column,
                                  sensor_info.format.columns_per_frame);
        parse_and_validate_angles("$.beam_intrinsics.beam_azimuth_angles",
                                  sensor_info.beam_azimuth_angles,
                                  sensor_info.format.pixels_per_column,
                                  sensor_info.format.columns_per_frame);

        const std::string lidar_origin_string =
            "$.beam_intrinsics.lidar_origin_to_beam_origin_mm";
        if (!parse_and_validate_item(
                issues.information, lidar_origin_string,
                sensor_info.lidar_origin_to_beam_origin_mm)) {
            sensor_info.lidar_origin_to_beam_origin_mm =
                ouster::sensor::default_lidar_origin_to_beam_origin(
                    sensor_info.prod_line);
            default_message(lidar_origin_string);
        }

        const std::string beam_to_lidar_string =
            "$.beam_intrinsics.beam_to_lidar_transform.*";
        std::vector<double> beam_to_lidar_data;
        if (parse_and_validate_item<double>(issues.information,
                                            beam_to_lidar_string,
                                            beam_to_lidar_data, 16, true)) {
            decode_transform_array(sensor_info.beam_to_lidar_transform,
                                   beam_to_lidar_data);
        } else {
            sensor_info.beam_to_lidar_transform = mat4d::Identity();
            sensor_info.beam_to_lidar_transform(0, 3) =
                sensor_info.lidar_origin_to_beam_origin_mm;
            default_message(beam_to_lidar_string);
        }
    }

    void parse_and_validate_misc() {
        std::vector<double> extrinsic_data;
        const std::string extrinsic_string = "$.'ouster-sdk'.extrinsic.*";
        if (parse_and_validate_item<double>(issues.information,
                                            extrinsic_string, extrinsic_data,
                                            16, true)) {
            decode_transform_array(sensor_info.extrinsic, extrinsic_data);
        } else {
            default_message(extrinsic_string);
            sensor_info.extrinsic = mat4d::Identity();
        }

        parse_and_validate_item(issues.information, "$.user_data",
                                sensor_info.user_data);
    }
};

const std::map<std::string, bool> nonlegacy_metadata_fields = {
    {"sensor_info", true},        {"beam_intrinsics", true},
    {"imu_intrinsics", true},     {"lidar_intrinsics", true},
    {"config_params", true},      {"lidar_data_format", false},
    {"calibration_status", false}};

// Copypasta and changed form sensor_info.cpp
bool is_new_format(const jsoncons::json& root) {
    size_t nonlegacy_fields_present = 0;
    std::string missing_fields = "";
    for (const auto& field_pair : nonlegacy_metadata_fields) {
        if (root.contains(field_pair.first)) {
            nonlegacy_fields_present++;
        }
    }

    if (nonlegacy_fields_present > 0 &&
        nonlegacy_fields_present < nonlegacy_metadata_fields.size()) {
        throw std::runtime_error{"Non-legacy metadata must include fields: " +
                                 missing_fields};
    }

    return nonlegacy_fields_present == nonlegacy_metadata_fields.size();
}

jsoncons::json convert_legacy_to_nonlegacy(jsoncons::json& root) {
    jsoncons::json result;

    // just convert to non-legacy and run the non-legacy parse
    const std::vector<std::string> config_fields{
        "udp_port_imu",
        "udp_port_lidar",
        "lidar_mode",
    };

    const std::vector<std::string> beam_intrinsics_fields{
        "lidar_origin_to_beam_origin_mm", "beam_altitude_angles",
        "beam_azimuth_angles", "beam_to_lidar_transform"};

    const std::vector<std::string> sensor_info_fields{
        "prod_line",         "status",    "prod_pn",    "prod_sn",
        "initialization_id", "build_rev", "build_date", "image_rev",
    };

    if (root.contains("lidar_to_sensor_transform")) {
        result["lidar_intrinsics"]["lidar_to_sensor_transform"] =
            root.at("lidar_to_sensor_transform");
        root.erase("lidar_to_sensor_transform");
    }

    if (root.contains("imu_to_sensor_transform")) {
        result["imu_intrinsics"]["imu_to_sensor_transform"] =
            root.at("imu_to_sensor_transform");
        root.erase("imu_to_sensor_transform");
    }
    if (root.contains("data_format")) {
        result["lidar_data_format"] = root.at("data_format");
        root.erase("data_format");
    }

    if (root.contains("client_version")) {
        result["ouster-sdk"]["client_version"] = root.at("client_version");
        root.erase("client_version");
    }

    for (const auto& field : config_fields) {
        if (root.contains(field)) {
            result["config_params"][field] = root.at(field);
            root.erase(field);
        }
    }

    for (const auto& field : beam_intrinsics_fields) {
        if (root.contains(field)) {
            result["beam_intrinsics"][field] = root.at(field);
            root.erase(field);
        }
    }

    for (const auto& field : sensor_info_fields) {
        if (root.contains(field)) {
            result["sensor_info"][field] = root.at(field);
            root.erase(field);
        }
    }

    for (const auto& it : root.object_range()) {
        result[it.key()] = it.value();
    }

    return result;
}

bool parse_and_validate_metadata(const std::string& json_data,
                                 ouster::sensor::sensor_info& sensor_info,
                                 ValidatorIssues& issues) {
    auto root = jsoncons::json::parse(json_data);

    size_t nonlegacy_fields_present = 0;
    std::vector<ValidatorIssues::ValidatorEntry> missing_fields;
    for (const auto& field_pair : nonlegacy_metadata_fields) {
        if (root.contains(field_pair.first)) {
            nonlegacy_fields_present++;
        } else {
            auto entry = ValidatorIssues::ValidatorEntry(
                "$." + field_pair.first,
                "Non-legacy metadata must include field");
            missing_fields.push_back(entry);
        }
    }

    if (nonlegacy_fields_present != nonlegacy_metadata_fields.size()) {
        root = convert_legacy_to_nonlegacy(root);
    }
    MetadataImpl impl(root, sensor_info, issues);
    if (nonlegacy_fields_present > 0 &&
        nonlegacy_fields_present < nonlegacy_metadata_fields.size()) {
        for (auto it : missing_fields) {
            issues.critical.push_back(it);
        }
    }

    // debug log each issue
    if ((issues.information.size() > 0) || (issues.warning.size() > 0) ||
        (issues.critical.size() > 0)) {
        sensor::logger().debug("Issues encountered during metadata parsing:");
    } else {
        sensor::logger().debug(
            "No issues encountered during metadata parsing.");
    }
    for (auto& i : issues.information) {
        sensor::logger().debug("{}", i.to_string());
    }
    for (auto& i : issues.warning) {
        sensor::logger().debug("{}", i.to_string());
    }
    for (auto& i : issues.critical) {
        sensor::logger().debug("{}", i.to_string());
    }

    return issues.critical.size() == 0;
}

bool parse_and_validate_metadata(const std::string& json_data,
                                 ValidatorIssues& issues) {
    nonstd::optional<ouster::sensor::sensor_info> sensor_info;
    return parse_and_validate_metadata(json_data, sensor_info, issues);
}

bool parse_and_validate_metadata(
    const std::string& json_data,
    nonstd::optional<ouster::sensor::sensor_info>& sensor_info,
    ValidatorIssues& issues) {
    sensor_info = nonstd::nullopt;
    ouster::sensor::sensor_info temp_info;
    bool result = parse_and_validate_metadata(json_data, temp_info, issues);
    if (result) {
        sensor_info =
            nonstd::make_optional<ouster::sensor::sensor_info>(temp_info);
    } else {
        sensor_info.reset();
    }

    return result;
}
};  // namespace ouster
