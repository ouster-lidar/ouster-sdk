/**
 * @file
 * @brief Compatibility with windows (unsupported)
 */

#pragma once

#include <string>

#if defined _WIN32 // --------- On Windows ---------

#define _USE_MATH_DEFINES
#define WPCAP 1
// Try and limit the stuff windows brings in
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX 1
#define HAVE_U_INT8_T 1
#define HAVE_U_INT16_T 1

// Windows headers
#include <WS2tcpip.h>
#include <WinInet.h>
#include <intrin.h>
#include <process.h>
#include <winbase.h>
#include <windows.h>
#include <winsock2.h>

// ssize_t is available in vs
#ifdef _MSC_VER
#include <BaseTsd.h>
#define ssize_t SSIZE_T
#endif

#else  // --------- Compiling on *nix ---------

// Linux headers
#include <arpa/inet.h>
#include <fcntl.h>
#include <net/ethernet.h>
#include <netdb.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// Define windows types for linux
typedef int SOCKET;
typedef fd_set FDSET;
#define SOCKET_ERROR -1

#endif  // --------- End Platform Differentiation Block ---------

/**
 * Initialize the socket stack (Noop on linux)
 * @return success on windows
 */
int socket_init(void);

/**
 * Shutdown the socket stack (Noop on linux)
 * @return success on windows
 */
int socket_quit(void);

/**
 * Close a specified socket
 * @param sock The socket file descriptor to close
 * @return success
 */
int socket_close(SOCKET sock);

/**
 * Get the error message for socket errors
 * @return The socket error message
 */
std::string socket_get_error();

/**
 * Check if a socket file descriptor is valid
 * @param sock The socket file descriptor to check
 * @return The validity of the socket file descriptor
 */
bool socket_valid(SOCKET value);

/**
 * Check if the last error was a socket exit event
 * @return If the socket has exited
 */
bool socket_exit();

/**
 * Set a specified socket to non-blocking
 * @param sock The socket file descriptor to set non-blocking
 * @return success
 */
int socket_set_non_blocking(SOCKET value);

/**
 * Set a specified socket to reuse
 * @param sock The socket file descriptor to set reuse
 * @return success
 */
int socket_set_reuse(SOCKET value);
