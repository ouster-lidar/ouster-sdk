#include "ouster/impl/netcompat.h"

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
    int status = 0;

#ifdef _WIN32
    status = shutdown(sock, SD_BOTH);
    if (status == 0) {
        status = closesocket(sock);
    }
#else
    status = shutdown(sock, SHUT_RDWR);
    if (status == 0) {
        status = close(sock);
    }
#endif

    return status;
}

std::string socket_get_error() {
#ifdef _WIN32
    int errnum = WSAGetLastError();
    char buf[256] = {0};
    if (FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, errnum, 0, buf,
                      sizeof(buf), NULL) != 0) {
        return std::string(buf);
    } else {
        return std::string{"Unknown WSA error "} + std::to_string(errnum);
    }
#else
    return std::strerror(errno);
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

int socket_set_non_blocking(SOCKET value) {
#ifdef _WIN32
    u_long non_blocking_mode = 0;
    return ioctlsocket(value, FIONBIO, &non_blocking_mode);
#else
    return fcntl(value, F_SETFL, fcntl(value, F_GETFL, 0) | O_NONBLOCK);
#endif
}

int socket_set_reuse(SOCKET value) {
#ifdef _WIN32
    u_long reuse = 1;
    return ioctlsocket(value, SO_REUSEADDR, &reuse);
#else
    int option = 1;
    return setsockopt(value, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
#endif
}

}  // namespace impl
}  // namespace ouster
