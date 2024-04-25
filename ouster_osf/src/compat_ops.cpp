/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "compat_ops.h"

#include <fstream>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <direct.h>
#include <fcntl.h>
#include <io.h>
#include <shlwapi.h>
#include <tchar.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#endif

namespace ouster {
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
    _ftprintf(stderr, TEXT("\nERROR: %s SYSTEM RETURNED: %s\n"), errDesc,
              errMsg);
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

}  // namespace

/// Get the last system error and return it in a string
std::string get_last_error() {
#ifdef _WIN32
    return LastErrorMessageStr();
#else
    return std::string(std::strerror(errno));
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
    struct stat statbuf;
    if (stat(path.c_str(), &statbuf) != 0) {
        if (errno != ENOENT) printf("ERROR: stat: %s", std::strerror(errno));
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
    struct stat sb;
    return !(stat(path.c_str(), &sb) != 0);
#endif
}

bool is_file_sep(char c) { return (FILE_SEPS.find(c) != std::string::npos); }

/// Path concatenation with OS specific path separator
// NOTE: Trying not to be too smart here ... and it's impossible function
//       to make it fully correct, so use it wisely.
std::string path_concat(const std::string& path1, const std::string& path2) {
    if (path1.empty()) return path2;
    if (path2.empty()) return path1;

    if (is_file_sep(path2.front())) return path2;
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
    PrintError(TEXT("Can't create temp dir."));
    return false;
#else
    // TODO[pb]: Check that it works on Mac OS and especially that
    // temp files are cleaned correctly and don't feel up the CI machine ...
    char tmpdir[] = "/tmp/ouster-test.XXXXXX";
    if (::mkdtemp(tmpdir) == nullptr) {
        std::cerr << "ERROR: Can't create temp dir." << std::endl;
        return false;
    };
    tmp_path = tmpdir;
    return true;
#endif
}

/// Make directory
bool make_dir(const std::string& path) {
#ifdef _WIN32
    return (CreateDirectoryA(path.c_str(), NULL) != 0);
#else
    if (mkdir(path.c_str(), 0777) != 0) {
        printf("ERROR: Can't create dir: %s\n", path.c_str());
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
    char* var_value;
    if ((var_value = std::getenv(name.c_str())) != nullptr) {
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
int64_t file_size(const std::string& path) {
#ifdef _WIN32
    LARGE_INTEGER fsize;
    WIN32_FILE_ATTRIBUTE_DATA file_attr_data;
    if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard,
                              &file_attr_data)) {
        return -1;
    }
    if (file_attr_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        return -2;
    }
    fsize.LowPart = file_attr_data.nFileSizeLow;
    fsize.HighPart = file_attr_data.nFileSizeHigh;
    return fsize.QuadPart;
#else
    struct stat st;
    if (stat(path.c_str(), &st) < 0) {
        return -1;
    };
    if (!S_ISREG(st.st_mode)) {
        return -2;
    }
    return st.st_size;
#endif
}

/// File mapping open (read-only operations)
uint8_t* mmap_open(const std::string& path) {
#ifdef _WIN32
    HANDLE hFile;
    uint8_t* pBuf;
    hFile = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL,
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
    if (hFile == INVALID_HANDLE_VALUE) {
        return NULL;
    }
    HANDLE hFileMap;
    hFileMap = CreateFileMappingA(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
    if (hFileMap == NULL) {
        CloseHandle(hFile);
        return NULL;
    }

    pBuf =
        static_cast<uint8_t*>(MapViewOfFile(hFileMap, FILE_MAP_READ, 0, 0, 0));

    // TODO: check errors?
    CloseHandle(hFileMap);
    CloseHandle(hFile);

    return pBuf;
#else
    struct stat st;
    if (stat(path.c_str(), &st) < 0) {
        return nullptr;
    };
    if (!S_ISREG(st.st_mode)) {
        return nullptr;
    }
    if (st.st_size == 0) {
        return nullptr;
    }

    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        return nullptr;
    }

    void* map_osf_file = mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (map_osf_file == MAP_FAILED) {
        ::close(fd);
        return nullptr;
    }
    return static_cast<uint8_t*>(map_osf_file);
#endif
}

/// File mapping close
bool mmap_close(uint8_t* file_buf, const uint64_t file_size) {
    if (file_buf == nullptr || file_size == 0) return false;
#ifdef _WIN32
    return (UnmapViewOfFile(file_buf) != 0);
#else
    return (munmap(static_cast<void*>(file_buf), file_size) >= 0);
#endif
}

int64_t truncate_file(const std::string& path, uint64_t filesize) {
    int64_t actual_file_size = file_size(path);
    if (actual_file_size < (int64_t)filesize) {
        return -1;
    }
#ifdef _WIN32
    int file_handle;
    if (file_handle = _sopen(path.c_str(), _O_RDWR, _SH_DENYRW)) {
        _chsize(file_handle, filesize);
        _close(file_handle);
    }
#else
    truncate(path.c_str(), filesize);
#endif
    return file_size(path);
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
            append_from_file_stream.seekg(0, std::ios::end);
            uint64_t from_file_size = append_from_file_stream.tellg();
            append_from_file_stream.seekg(0, std::ios::beg);

            append_to_file_stream.seekg(0, std::ios::end);

            append_to_file_stream << append_from_file_stream.rdbuf();
            saved_size = append_to_file_stream.tellg();
        } else {
            std::cerr << "fail to open " << append_to_file_name << std::endl;
        }
    } else {
        std::cerr << "fail to open " << append_from_file_name << std::endl;
    }

    if (append_to_file_stream.is_open()) append_to_file_stream.close();
    if (append_from_file_stream.is_open()) append_from_file_stream.close();

    return saved_size;
}

int64_t copy_file_trailing_bytes(const std::string& source_file,
                                 const std::string& target_file,
                                 uint64_t offset) {
    int64_t actual_file_size = file_size(source_file);
    if (actual_file_size < (int64_t)offset) {
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
            source_file_stream.seekg(offset);
            target_file_stream << source_file_stream.rdbuf();
            saved_size = target_file_stream.tellg();
        } else {
            std::cerr << "fail to open " << source_file << std::endl;
        }
    } else {
        std::cerr << "fail to open " << target_file << std::endl;
    }

    if (source_file_stream.is_open()) source_file_stream.close();
    if (target_file_stream.is_open()) target_file_stream.close();

    return saved_size;
}

}  // namespace osf
}  // namespace ouster
