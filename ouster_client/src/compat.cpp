#include "ouster/compat.h"

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

int socket_init(void) {
#ifdef _WIN32
    WSADATA wsa_data;
    return WSAStartup(MAKEWORD(1, 1), &wsa_data);
#else
    return 0;
#endif
}

int socket_quit(void) {
#ifdef _WIN32
    return WSACleanup();
#else
    return 0;
#endif
}

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
    wchar_t* char_buffer = NULL;
    std::string result;

    FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   NULL, WSAGetLastError(),
                   MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                   (LPWSTR)&char_buffer, 0, NULL);

    std::wstring wide_string(char_buffer);
    result = std::string(wide_string.begin(), wide_string.end());
    LocalFree(char_buffer);

    return result;
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