#pragma once

#include <functional>
#include <string>

#include "ouster/visibility.h"

namespace ouster {
namespace core {

/// An enum that represents a level of severity for non-fatal errors encountered
/// while using a ScanSource.
enum class Severity { OUSTER_WARNING, OUSTER_ERROR };

/**
 * A function type for handling non-fatal errors encountered while reading an
 * OSF. (Fatal errors throw exceptions.)
 */
using error_handler_t =
    std::function<void(Severity, const std::string& message)>;

/**
 * A default error handler, invoked by a ScanSource or osf::Reader when
 * encountering an error.
 *
 * @param[in] severity an enumeration value indicating a warning or error
 * @param[in] message a string description of the error
 */
OUSTER_API_FUNCTION
void default_error_handler(Severity severity, const std::string& message);

}  // namespace core
}  // namespace ouster
