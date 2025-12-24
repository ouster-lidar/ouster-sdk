#include "ouster/error_handler.h"

#include <stdexcept>
#include <string>

#include "ouster/impl/logging.h"

namespace ouster {
namespace sdk {
namespace core {

void default_error_handler(Severity severity, const std::string& message) {
    switch (severity) {
        case Severity::OUSTER_WARNING:
            logger().warn(message);
            break;
        case Severity::OUSTER_ERROR:
            logger().error(message);
            throw std::runtime_error(message);
        default:
            throw std::logic_error("Invalid severity for error message '" +
                                   message + "'");
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
