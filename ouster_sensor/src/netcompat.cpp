/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

/** Linting Exceptions:
 *   (misc-include-cleaner): Complex per operating system ifdefs are
 *   located inside of the netcompat.h
 *
 *   (cppcoreguidelines-pro-type-vararg,hicpp-vararg): This complains about an
 *operating system specific function call that we have to use.
 *
 *   (hicpp-deprecated-headers,modernize-deprecated-headers): Need to use the
 *posix version, of std::sterror due to issues around thread safety.
 *
 **/

#include "ouster/impl/netcompat.h"  // NOLINT(misc-include-cleaner)

#include <array>
#include <string>

#if defined _WIN32

#include <winsock2.h>

#else

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#endif

namespace ouster {
namespace sensor {
namespace impl {

#ifdef _WIN32
struct StaticWrapper {
    WSADATA wsa_data;

    StaticWrapper() { WSAStartup(MAKEWORD(1, 1), &wsa_data); }

    ~StaticWrapper() { WSACleanup(); }
};

static StaticWrapper resources = {};
#endif

int socket_close(SOCKET sock) {
#ifdef _WIN32
    return closesocket(sock);
#else
    return close(sock);
#endif
}

std::string socket_get_error() {
    std::array<char, 256> buf = {};
#ifdef _WIN32
    int errnum = WSAGetLastError();
    if (FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, nullptr, errnum, 0,
                      buf.data(), buf.size(), nullptr) != 0) {
        return buf.data();
    } else {
        return std::string{"Unknown WSA error "} + std::to_string(errnum);
    }
#else
#if defined(__EMSCRIPTEN__) || (defined(__APPLE__) && defined(__MACH__)) || \
    ((_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600) && !_GNU_SOURCE)
    // Using XSI-compliant version of strerror_r
    if (strerror_r(errno, buf.data(), buf.size()) == 0) {
        return buf.data();
    } else {
        return "Error: Unknown Socket Error";
    }
#else
    // Using GNU-specific version of strerror_r
    return strerror_r(errno, buf.data(), buf.size());
#endif
#endif
}

bool socket_valid(SOCKET sock) {
#ifdef _WIN32
    return sock != SOCKET_ERROR;
#else
    return sock >= 0;
#endif
}

bool socket_exit() {
#ifdef _WIN32
    auto result = WSAGetLastError();
    return result == WSAECONNRESET || result == WSAECONNABORTED ||
           result == WSAESHUTDOWN;
#else
    return errno == EINTR;
#endif
}

int socket_set_non_blocking(SOCKET sock) {
#ifdef _WIN32
    u_long non_blocking_mode = 1;
    return ioctlsocket(sock, FIONBIO, &non_blocking_mode);
#else
    // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg)
    return fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    // NOLINTEND(cppcoreguidelines-pro-type-vararg)
#endif
}

int socket_set_reuse(SOCKET sock) {
    int option = 1;
#ifndef _WIN32
    int res = setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &option,
                         sizeof(option));  // NOLINT (misc-include-cleaner)
    if (res != 0) {
        return res;
    }
#endif
    return setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
                      (char*)(&option),  // NOLINT (misc-include-cleaner)
                      sizeof(option));
}

int socket_set_rcvtimeout(SOCKET sock, int timeout_sec) {
#ifdef _WIN32
    DWORD timeout_ms = timeout_sec * 1000;
    return setsockopt(
        sock, SOL_SOCKET, SO_RCVTIMEO,
        (const char*)(&timeout_ms),  // NOLINT (misc-include-cleaner)
        sizeof timeout_ms);
#else
    struct timeval tv;  // NOLINT (misc-include-cleaner)
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                      (const char*)(&tv),  // NOLINT (misc-include-cleaner)
                      sizeof tv);
#endif
}

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
