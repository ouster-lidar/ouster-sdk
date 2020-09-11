#pragma once

#include <string>

#include "ouster/osf/version.h"

namespace ouster {
namespace OSF {

enum class OpenMode : uint8_t { READ = 0, WRITE = 1 };

enum class FileState : uint8_t { GOOD = 0, BAD = 1 };

// Interface to abstract the way of how we handle file system read/write
// operations.
class OsfFile {
   public:
    explicit OsfFile();

    // Opens the file
    // NOTE: Only OpenMode::READ is supported
    explicit OsfFile(const std::string& filename,
                     OpenMode mode = OpenMode::READ);
    ~OsfFile();

    // Header Info
    size_t size() const { return size_; };
    std::string filename() const { return filename_; }
    OSF_VERSION version() const;

    // Checks the validity of header and session/file_info blocks
    bool valid() const;

    // TODO[pb]: Need to have more states here (eod, valid, error, etc)
    bool good() const { return state_ == FileState::GOOD; }

    // Convenience operators
    bool operator!() const { return !good(); };
    explicit operator bool() const { return good(); };

    // Sequential access to the file
    // This is mimicking the regular file access with the offset
    size_t offset() const { return offset_; }

    // File seek (in mmap mode it's just moving the offset_ pointer
    // without any file system opeations.)
    OsfFile& seek(const size_t pos);

    // Read from file (in current mmap mode it's copying data from
    // mmap address to the 'buf' address)
    // TODO[pb]: Handle errors in future and get the way to read them back
    // with FileState etc.
    OsfFile& read(uint8_t* buf, const size_t count);

    // Mmap access to the file content with the specified offset from the
    // beginning of the file.
    const uint8_t* buf(const size_t offset = 0) const;

    // Clears file handle and allocated resources. In current mmap
    // implementation it's unmapping memory and essentially invalidates the
    // memory addresses that might be captured within ouster::OSF::MessageRef
    // and ouster::OSF::Reader.
    void close();

    // Debug helper method to dump OsfFile state to a string.
    std::string to_string() const;

    // Copy policy
    // Don't allow the copying of the file handler
    OsfFile(const OsfFile&) = delete;
    OsfFile& operator=(const OsfFile&) = delete;

    // Move policy
    // But it's ok to move with the ownership transfer of the underlying file
    // handler (mmap).
    OsfFile(OsfFile&& other);
    OsfFile& operator=(OsfFile&& other);

   private:
    // Convenience method to set error and print it's content.
    // TODO[pb]: Adding more error states will probably extend the set of this
    // function.
    void error(const std::string& msg = std::string());

    // Opened filename as it was passed in contructor.
    std::string filename_;

    // Current offset to the file. (not used in mmaped implementation) but used
    // for copying(reading) blocks of memory from the file to the specified
    // memory.
    size_t offset_;

    // Size of the opened file in bytes
    size_t size_;

    // Mmaped memory address pointed to the beginning of the file (byte 0)
    uint8_t* file_buf_;

    // Internal state
    FileState state_;
};

}  // namespace OSF
}  // namespace ouster