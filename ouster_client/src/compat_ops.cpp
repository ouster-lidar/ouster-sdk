/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <ouster/compat_ops.h>

#include <algorithm>
#include <cstring>

#ifdef _WIN32
#include <Windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
// clang-format off
#include <dirent.h>
#include <netdb.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
// clang-format on
#endif

namespace ouster {
namespace core {

#ifdef _WIN32
nonstd::optional<bool> is_directory(const std::string& path) {
    DWORD attrib = GetFileAttributes(path.c_str());

    if (attrib == INVALID_FILE_ATTRIBUTES) {
        return {};
    }

    if ((attrib & FILE_ATTRIBUTE_DIRECTORY) != 0) {
        return true;
    }

    return false;
}
#else
nonstd::optional<bool> is_directory(const std::string& path) {
    struct stat s;
    if (stat(path.c_str(), &s) == 0) {
        if (s.st_mode & S_IFDIR) {
            return true;
        } else if (s.st_mode & S_IFREG) {
            return false;
        }
    }
    return {};
}
#endif

bool is_host(const std::string& name) {
    struct addrinfo hints;
    struct addrinfo* result;

    // Set up the hints structure to specify the desired options (IPv4, TCP)
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;      // IPv4, todo dont care
    hints.ai_socktype = SOCK_STREAM;  // TCP socket

    // Use getaddrinfo to resolve the address.
    if (getaddrinfo(name.c_str(), NULL, &hints, &result) == 0) {
        freeaddrinfo(result);
        return true;
    }
    return false;
}

#ifdef _WIN32
std::vector<std::string> files_in_directory(const std::string& path) {
    std::vector<std::string> files;

    // check directory exists
    char fullpath[MAX_PATH];
    GetFullPathName(path.c_str(), MAX_PATH, fullpath, 0);
    std::string fp(fullpath);
    DWORD attrs = GetFileAttributes(fp.c_str());
    if (attrs == INVALID_FILE_ATTRIBUTES ||
        (attrs & FILE_ATTRIBUTE_DIRECTORY) == 0) {
        return files;
    }

    // get file names
    WIN32_FIND_DATA findfiledata;
    HANDLE hFind = FindFirstFile((LPCSTR)(fp + "\\*").c_str(), &findfiledata);
    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            files.push_back(findfiledata.cFileName);
        } while (FindNextFile(hFind, &findfiledata));
        FindClose(hFind);
    }

    // delete current and parent directories
    files.erase(std::find(files.begin(), files.end(), "."));
    files.erase(std::find(files.begin(), files.end(), ".."));

    // sort in alphabetical order
    std::sort(files.begin(), files.end());

    return files;
}
#else
std::vector<std::string> files_in_directory(const std::string& directory) {
    std::vector<std::string> files;

    // open directory
    DIR* dir;
    dir = opendir(directory.c_str());
    if (dir == NULL) return files;

    // get file names
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) files.push_back(ent->d_name);
    closedir(dir);

    // delete current and parent directories
    files.erase(std::find(files.begin(), files.end(), "."));
    files.erase(std::find(files.begin(), files.end(), ".."));

    // sort in alphabetical order
    std::sort(files.begin(), files.end());

    return files;
}
#endif  // _WIN32

}  // namespace core
}  // namespace ouster
