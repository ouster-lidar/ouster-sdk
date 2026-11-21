/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/core/signals.h"

#include <csignal>
#include <stdexcept>

namespace {
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
volatile bool was_signaled = false;
std::function<void()> signal_callback;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
}  // namespace

extern "C" {
// These functions are known to be signal safe so can be used.
// NOLINTBEGIN(bugprone-signal-handler)
void signal_handler(int signal) {
    if (signal == SIGINT) {
        was_signaled = true;
        if (signal_callback) {
            signal_callback();
        }
    }
}
// NOLINTEND(bugprone-signal-handler)
}

namespace ouster {
namespace sdk {
namespace core {
bool signaled() {
    return was_signaled;
}

void setup_signal_handler(std::function<void()> callback) {
    static bool handler_installed = false;
    if (!handler_installed) {
        handler_installed = true;
        signal_callback = std::move(callback);
        if (std::signal(SIGINT, signal_handler) == SIG_ERR) {
            throw std::runtime_error("Failed to install signal handler");
        }
    }
}
}  // namespace core
}  // namespace sdk
}  // namespace ouster
