/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <functional>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/// Installs a signal handler to detect sigints, useful to override existing interrupt handlers
/// Allows optionally installing a callback on signal
/// @throws if signal handler failed to install
OUSTER_API_FUNCTION
void setup_signal_handler(std::function<void()> callback = {}
                          ///< [in] callback callback to install
);

/// Returns true if we were signaled to exit by an installed signal handler
/// @return if we were signaled
OUSTER_API_FUNCTION
bool signaled();

}  // namespace core
}  // namespace sdk
}  // namespace ouster
