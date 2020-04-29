#pragma once

#include <string>
#include <vector>

#if defined _WIN32  // --------- Compiling on Windows ---------
#define WPCAP 1
// Try and limit the stuff windows brings in
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX 1
#define HAVE_U_INT8_T 1
#define HAVE_U_INT16_T 1

// Fix for windows compiler issue
#define VTK_FALLTHROUGH [[fallthrough]]

// Windows headers
#include <process.h>
#include <WS2tcpip.h>
#include <intrin.h>
#include <winbase.h>
#include <winbase.h>
#include <winsock2.h>
#include <windows.h>
#include <WinInet.h>

#pragma pack(push, 1)
struct ether_header {
    uint8_t padding1[12];
    uint16_t ether_type;
};

struct iphdr {
    uint8_t ihl : 4;
    uint8_t version : 4;
    uint8_t padding1;
    uint16_t tot_len;
    uint16_t id;
    uint16_t frag_off;
    uint8_t ttl;
    uint8_t protocol;
    uint8_t padding2[6];
    uint32_t daddr;
};

struct udphdr {
    uint16_t padding1;
    uint16_t dest;
    uint16_t len;
    uint16_t padding2;
};
#pragma pack(pop)

#define ETHERTYPE_IP 0x0800

// Fix windows issue with M_PI
#define _USE_MATH_DEFINES  // for C++
#include <math.h>
#include <cmath>

#define SET_SIN_ADDR(data, address) data.addr.sin_addr.S_un.S_addr = (address)
#else  // --------- Compiling on Linux ---------
// Linux headers
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>

// Define windows types for linux
typedef int SOCKET;
typedef fd_set FDSET;
#define SOCKET_ERROR -1
#define SET_SIN_ADDR(data, address) data.addr.sin_addr = { address }
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
