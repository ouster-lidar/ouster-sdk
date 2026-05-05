/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/impl/compat_ops.h"

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "ouster/impl/logging.h"

#ifdef _WIN32
#include <direct.h>
#include <fcntl.h>
#include <io.h>
#include <share.h>
#include <shlwapi.h>
#include <tchar.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

using ouster::sdk::core::logger;
namespace ouster {
namespace sdk {
namespace osf {

static const std::string FILE_SEPS = "\\/";

namespace {

#ifdef _WIN32
//  ErrorMessage support function.
//  Retrieves the system error message for the GetLastError() code.
//  Note: caller must use LocalFree() on the returned LPCTSTR buffer.
LPCTSTR ErrorMessage(DWORD error) {
    LPVOID lpMsgBuf;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   NULL, error, NULL, (LPTSTR)&lpMsgBuf, 0, NULL);
    return ((LPCTSTR)lpMsgBuf);
}

//  PrintError support function.
//  Simple wrapper function for error output.
void PrintError(LPCTSTR errDesc) {
    LPCTSTR errMsg = ErrorMessage(GetLastError());
    logger().error("ERROR: {} SYSTEM RETURNED: {}", errDesc, errMsg);
    LocalFree((LPVOID)errMsg);
}

// Get the last system error and return it in a string (not wide string)
std::string LastErrorMessageStr() {
    LPCTSTR errMsg = ErrorMessage(GetLastError());
    // NOTE[pb]: errMsg can be a char or wchar and seems the below copy
    // construct is OK for MSVC in both cases. (or no? idk)
    std::string res{errMsg};
    LocalFree((LPVOID)errMsg);
    return res;
}
#endif

void file_size_exception(const std::string& path) {
    throw std::runtime_error("Couldn't read " + path +
                             ": couldn't determine file size.");
}

bool is_file_sep(char character) {
    return (FILE_SEPS.find(character) != std::string::npos);
}

}  // namespace

/// Get the last system error and return it in a string
std::string get_last_error() {
#ifdef _WIN32
    return LastErrorMessageStr();
#else
    std::array<char, 1024> buf = {};
// NOLINTBEGIN(misc-include-cleaner)
#if defined(__EMSCRIPTEN__) || (defined(__APPLE__) && defined(__MACH__)) || \
    ((_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600) && !_GNU_SOURCE)
    // Using XSI-compliant version of strerror_r
    if (strerror_r(errno, buf.data(), buf.size()) == 0) {
        return buf.data();
    } else {
        return "Error: Unknown OSF Error";
    }
// NOLINTEND(misc-include-cleaner)
#else
    // Using GNU-specific version of strerror_r
    return strerror_r(errno, buf.data(),
                      buf.size());  // NOLINT(misc-include-cleaner)
#endif
#endif
}

/// Checking the the path is it directory or not.
bool is_dir(const std::string& path) {
#ifdef _WIN32
    DWORD attrs = GetFileAttributesA(path.c_str());
    if (attrs == INVALID_FILE_ATTRIBUTES) {
        return false;
    }
    if (attrs & FILE_ATTRIBUTE_DIRECTORY) {
        return true;
    }
    return false;
#else
    struct stat statbuf {};
    if (stat(path.c_str(), &statbuf) != 0) {
        if (errno != ENOENT) {
            logger().error("ERROR: stat: {}", get_last_error());
        }
        return false;
    }
    return S_ISDIR(statbuf.st_mode);
#endif
}

/// Check path existence on the system
bool path_exists(const std::string& path) {
#ifdef _WIN32
    // taken from here:
    // https://devblogs.microsoft.com/oldnewthing/20071023-00/?p=24713
    DWORD attrs = GetFileAttributesA(path.c_str());
    return (attrs != INVALID_FILE_ATTRIBUTES);
#else
    struct stat stats {};
    return !(stat(path.c_str(), &stats) != 0);
#endif
}

/// Path concatenation with OS specific path separator
// NOTE: Trying not to be too smart here ... and it's impossible function
//       to make it fully correct, so use it wisely.
std::string path_concat(const std::string& path1, const std::string& path2) {
    if (path1.empty()) {
        return path2;
    }
    if (path2.empty()) {
        return path1;
    }

    if (is_file_sep(path2.front())) {
        return path2;
    }
#ifdef _WIN32
    if (path2.size() > 1 && path2.at(1) == ':') return path2;
#endif

    size_t p1_last_slash = path1.size();
    while (p1_last_slash > 0 && is_file_sep(path1.at(p1_last_slash - 1))) {
        p1_last_slash--;
    }
    return path1.substr(0, p1_last_slash) + FILE_SEP + path2;
}

/// Get the path to unique temp directory and create it.
bool make_tmp_dir(std::string& tmp_path) {
#ifdef _WIN32
    unsigned int uRetVal = 0;
    char lpTempPath[MAX_PATH];
    uRetVal = GetTempPathA(MAX_PATH, lpTempPath);
    if (uRetVal > MAX_PATH || uRetVal == 0) {
        return false;
    }
    char pTmpPath[MAX_PATH];
    if (!tmpnam_s<MAX_PATH>(pTmpPath)) {
        if (CreateDirectoryA(pTmpPath, NULL)) {
            tmp_path = pTmpPath;
            return true;
        }
    }
    logger().error("ERROR: Can't create temp dir.");
    return false;
#else
    // TODO[pb]: Check that it works on Mac OS and especially that
    // temp files are cleaned correctly and don't feel up the CI machine ...
    std::array<char, 25> tmpdir = {"/tmp/ouster-test.XXXXXX"};
    if (::mkdtemp(static_cast<char*>(tmpdir.data())) ==
        nullptr) {  // NOLINT(misc-include-cleaner)
        logger().error("ERROR: Can't create temp dir.");
        return false;
    };
    tmp_path = std::string(static_cast<char*>(tmpdir.data()));
    return true;
#endif
}

/// Make directory
bool make_dir(const std::string& path) {
#ifdef _WIN32
    logger().error("ERROR: Can't create dir: {}", path);
    return (CreateDirectoryA(path.c_str(), NULL) != 0);
#else
    if (mkdir(path.c_str(), 0777) != 0) {
        logger().error("ERROR: Can't create dir: {}", path);
        return false;
    }
    return true;
#endif
}

/// Get environment variable
bool get_env_var(const std::string& name, std::string& value) {
#ifdef _WIN32
    const unsigned int BUFSIZE = 4096;
    char var_value[BUFSIZE];
    unsigned int ret_val =
        GetEnvironmentVariableA(name.c_str(), var_value, BUFSIZE);
    if (ret_val != 0 && ret_val < BUFSIZE) {
        value.assign(var_value);
        return true;
    }
    value.clear();
    return false;
#else
    // No good way to make the get env thread-safe in C++11, so
    // using std::getenv here.
    char* var_value =
        std::getenv(name.c_str());  // NOLINT(concurrency-mt-unsafe)
    if (var_value != nullptr) {
        value = var_value;
        return true;
    }
    value.clear();
    return false;
#endif
}

/// Unlink path (i.e. almost like remove)
bool unlink_path(const std::string& path) {
#ifdef _WIN32
    return (_unlink(path.c_str()) == 0);
#else
    return (unlink(path.c_str()) == 0);
#endif
}

// Remove directory
bool remove_dir(const std::string& path) {
#ifdef _WIN32
    return (_rmdir(path.c_str()) == 0);
#else
    return (rmdir(path.c_str()) == 0);
#endif
}

/// Get file size
uint64_t file_size(const std::string& path) {
#ifdef _WIN32
    LARGE_INTEGER fsize;
    WIN32_FILE_ATTRIBUTE_DATA file_attr_data;
    if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard,
                              &file_attr_data) ||
        file_attr_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        file_size_exception(path);
    }
    fsize.LowPart = file_attr_data.nFileSizeLow;
    fsize.HighPart = file_attr_data.nFileSizeHigh;
    return fsize.QuadPart;
#else
    struct stat stats {};
    if (stat(path.c_str(), &stats) < 0 || !S_ISREG(stats.st_mode)) {
        file_size_exception(path);
    };
    return stats.st_size;
#endif
}

/// File mapping open (read-only operations)
uint8_t* mmap_open(const std::string& path, uintptr_t& memmap_handle) {
#ifdef _WIN32
    HANDLE file = CreateFileA(
        path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        return nullptr;
    }
    HANDLE file_map =
        CreateFileMappingA(file, nullptr, PAGE_READONLY, 0, 0, nullptr);
    memmap_handle = reinterpret_cast<uintptr_t>(file_map);
    if (file_map == nullptr) {
        CloseHandle(file);
        throw std::runtime_error("CreateFileMappingA " + path + ": " +
                                 get_last_error());
    }

    uint8_t* buffer_pointer =
        static_cast<uint8_t*>(MapViewOfFile(file_map, FILE_MAP_READ, 0, 0, 0));

    // TODO: check errors?
    CloseHandle(file);

    return buffer_pointer;
#else
    // Silence unused variable warning
    (void)memmap_handle;

    // Have to use var arg open due to the memory-mapped file API
    // Also technically we are not using the var arg portionion of the API
    int fd = open(path.c_str(),
                  O_RDONLY);  // NOLINT(cppcoreguidelines-pro-type-vararg)
    if (fd < 0) {
        throw std::runtime_error("open " + path + ": " + get_last_error());
    }

    void* map_osf_file =
        mmap(nullptr, file_size(path), PROT_READ, MAP_SHARED, fd, 0);
    if (map_osf_file == MAP_FAILED) {
        ::close(fd);
        throw std::runtime_error("mmap " + path + ": " + get_last_error());
    }
    return static_cast<uint8_t*>(map_osf_file);
#endif
}

/// File mapping close
bool mmap_close(uint8_t* file_buf, const uint64_t file_size,
                uintptr_t memmap_handle) {
    if (file_buf == nullptr || file_size == 0) {
        return false;
    }
#ifdef _WIN32
    bool result = (UnmapViewOfFile(file_buf) != 0);
    if (memmap_handle != 0) {
        HANDLE map_file = reinterpret_cast<HANDLE>(memmap_handle);
        CloseHandle(map_file);
    }
    return result;
#else
    // Silence unused variable warning
    (void)memmap_handle;
    return (munmap(static_cast<void*>(file_buf), file_size) >= 0);
#endif
}

int64_t truncate_file(const std::string& path, uint64_t filesize) {
    uint64_t actual_file_size = file_size(path);
    if (actual_file_size < filesize) {
        return -1;
    }
#ifdef _WIN32
    int file_handle;
    if (file_handle = _sopen(path.c_str(), _O_RDWR, _SH_DENYRW)) {
        _chsize(file_handle, filesize);
        _close(file_handle);
    }
#else
    if (truncate(path.c_str(), static_cast<int64_t>(filesize)) != 0) {
        return -1;
    }
#endif
    return static_cast<int64_t>(file_size(path));
}

int64_t append_binary_file(const std::string& append_to_file_name,
                           const std::string& append_from_file_name) {
    int64_t saved_size = -1;

    std::fstream append_to_file_stream;
    std::fstream append_from_file_stream;

    // clang-format off
    // There something seriously wrong with the clang formatting
    // here.
    append_to_file_stream.open(append_to_file_name, std::fstream::out |
        std::fstream::app | std::fstream::binary);
    append_from_file_stream.open(append_from_file_name,
        std::fstream::in | std::fstream::binary);
    // clang-format on

    if (append_to_file_stream.is_open()) {
        if (append_from_file_stream.is_open()) {
            append_to_file_stream << append_from_file_stream.rdbuf();
            saved_size = append_to_file_stream.tellg();
        } else {
            logger().error("ERROR: Failed to open {} for appending",
                           append_to_file_name);
        }
    } else {
        logger().error("ERROR: Failed to open {} for appending",
                       append_to_file_name);
    }

    if (append_to_file_stream.is_open()) {
        append_to_file_stream.close();
    }
    if (append_from_file_stream.is_open()) {
        append_from_file_stream.close();
    }

    return saved_size;
}

int64_t copy_file_trailing_bytes(const std::string& source_file,
                                 const std::string& target_file,
                                 uint64_t offset) {
    uint64_t actual_file_size = file_size(source_file);
    if (actual_file_size < offset) {
        return -1;
    }

    int64_t saved_size = -1;

    std::fstream source_file_stream;
    std::fstream target_file_stream;

    // clang-format off
    // There something seriously wrong with the clang formatting
    // here.
    target_file_stream.open(target_file, std::fstream::out |
        std::fstream::trunc | std::fstream::binary);
    source_file_stream.open(source_file,
        std::fstream::in | std::fstream::binary);
    // clang-format on

    if (target_file_stream.is_open()) {
        if (source_file_stream.is_open()) {
            source_file_stream.seekg(static_cast<int64_t>(offset));
            target_file_stream << source_file_stream.rdbuf();
            saved_size = target_file_stream.tellg();
        } else {
            logger().error("ERROR: Failed to open {} for copy", target_file);
        }
    } else {
        logger().error("ERROR: Failed to open {} for copy", target_file);
    }

    if (source_file_stream.is_open()) {
        source_file_stream.close();
    }
    if (target_file_stream.is_open()) {
        target_file_stream.close();
    }

    return saved_size;
}

bool file_flush(const std::string& path) {
#ifdef _WIN32
    HANDLE fd = CreateFile(path.c_str(), GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, NULL);
    if (fd == INVALID_HANDLE_VALUE) {
        // somehow failed to open the file
        return false;
    }

    if (!FlushFileBuffers(fd)) {
        // fsync failed
        CloseHandle(fd);
        return false;
    }

    CloseHandle(fd);
#else
    int fd = open(path.c_str(), O_WRONLY, 0644);
    if (fd == -1) {
        // somehow failed to open the file
        return false;
    }

    if (fsync(fd) == -1) {
        // fsync failed
        close(fd);
        return false;
    }

    close(fd);
#endif
    return true;
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
