#include "ouster/osf/file.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include "ouster/osf/util.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {

// ======== Construction ============

OsfFile::OsfFile()
    : filename_(),
      offset_(0),
      size_(0),
      file_buf_(nullptr),
      state_(FileState::BAD) {}

OsfFile::OsfFile(const std::string& filename, OpenMode mode)
    : filename_(filename), offset_(0), size_(0), file_buf_(nullptr) {
    // TODO[pb]: Extract to open function
    if (mode == OpenMode::READ) {
        struct stat st;
        if (stat(filename_.c_str(), &st) < 0) {
            error();
            return;
        };
        if (!S_ISREG(st.st_mode)) {
            error("not a file");
            return;
        }
        size_ = st.st_size;

        if (size_ == 0) {
            error("file is empty");
            return;
        }

        auto fd = open(filename_.c_str(), O_RDONLY);
        if (fd < 0) {
            error();
            return;
        }

        auto map_osf_file = mmap(0, size_, PROT_READ, MAP_SHARED, fd, 0);
        if (map_osf_file == MAP_FAILED) {
            ::close(fd);
            error();
            return;
        }
        file_buf_ = reinterpret_cast<uint8_t*>(map_osf_file);

        state_ = FileState::GOOD;
    } else {
        // Write is not yet implemented within this class. And other modes too.
        state_ = FileState::BAD;
    }
}

OSF_VERSION OsfFile::version() const {
    if (!good()) {
        return OSF_VERSION::V_INVALID;
    }
    auto osf_header = get_osf_header_from_buf(file_buf_);
    return static_cast<OSF_VERSION>(osf_header->version());
}

bool OsfFile::valid() const {
    if (!good()) {
        return false;
    }

    // Check flatbuffers osfHeader validity
    if (!verify_osf_header_buf(
            file_buf_,
            readPrefixedSizeFromOffset(file_buf_, 0) + SIZE_OF_PREFIXED_SIZE)) {
        print_error(filename_, "OSF header verification has failed.");
        return false;
    }

    auto osf_header = get_osf_header_from_buf(file_buf_);
    if (osf_header->status() != HEADER_STATUS_VALID) {
        print_error(filename_, "OSF header is not valid.");
        return false;
    }

    if (osf_header->file_length() != size_) {
        print_error(filename_, "OSF header file size field is incorrect.");
        return false;
    }

    size_t session_offset = osf_header->session_offset();
    // Check flatbuffers osfSession validity
    if (!verify_osf_session_buf(file_buf_ + session_offset,
                                size_ - session_offset)) {
        print_error(filename_, "OSF session/info verification has failed.");
        return false;
    }

    return true;
}

// ========= Geneal Data Access =============

OsfFile& OsfFile::seek(const size_t pos) {
    if (!good()) throw std::logic_error("bad osf file");
    if (pos > size_) {
        std::stringstream ss;
        ss << "seek for " << pos << " but the file size is " << size_;
        throw std::out_of_range(ss.str());
    }
    offset_ = pos;
    return *this;
}

OsfFile& OsfFile::read(uint8_t* buf, const size_t count) {
    // TODO[pb]: Check for errors in full implementation
    // and set error flags
    // TODO[pb]: Read from disk if it's not mmap? (buffering, etc to be
    // considered later)

    // Now we just copy from the mapped region from the current offset
    // and advance offset further.
    if (!good()) throw std::logic_error("bad osf file");
    if (offset_ + count > size_) {
        std::stringstream ss;
        ss << "read till " << (offset_ + count) << " but the file size is "
           << size_;
        throw std::out_of_range(ss.str());
    }
    std::memcpy(buf, file_buf_ + offset_, count);
    offset_ += count;
    return *this;
}

// ===== Mmapped access to the file content memory =====

const uint8_t* OsfFile::buf(const size_t offset) const {
    if (!good()) throw std::logic_error("bad osf file");
    if (offset >= size_)
        throw std::out_of_range("out of range osf file access");
    return file_buf_ + offset;
}

// ======= Helpers =============

void OsfFile::error(const std::string& msg) {
    state_ = FileState::BAD;
    if (!msg.empty()) {
        print_error(filename_, msg);
    } else {
        print_error(filename_, std::strerror(errno));
    }
}

std::string OsfFile::to_string() const {
    std::stringstream ss;
    ss << "OsfFile [filename = '" << filename_ << "', "
       << "state = " << static_cast<int>(state_) << ", "
       << "version = " << version() << ", "
       << "size = " << size_ << ", offset = " << offset_;
    if (this->good()) {
        auto osf_header = get_osf_header_from_buf(file_buf_);
        ss << ", osf.fileLength = " << osf_header->file_length() << ", "
           << "osf.sessionOffset = " << osf_header->session_offset() << ", "
           << "osf.status = " << osf_header->status();
    }
    ss << "]";
    return ss.str();
}

// ======= Move semantics ===================

OsfFile::OsfFile(OsfFile&& other)
    : filename_(other.filename_),
      offset_(other.offset_),
      size_(other.size_),
      file_buf_(other.file_buf_),
      state_(other.state_) {
    other.file_buf_ = nullptr;
    other.state_ = FileState::BAD;
}

OsfFile& OsfFile::operator=(OsfFile&& other) {
    if (this != &other) {
        close();
        filename_ = other.filename_;
        offset_ = other.offset_;
        size_ = other.size_;
        file_buf_ = other.file_buf_;
        state_ = other.state_;
        other.file_buf_ = nullptr;
        other.state_ = FileState::BAD;
    }
    return *this;
};

// ========= Release resources =================

void OsfFile::close() {
    if (file_buf_) {
        if (munmap(file_buf_, size_) < 0) {
            error();
        };
        file_buf_ = nullptr;
        state_ = FileState::BAD;
    }
}

OsfFile::~OsfFile() {
    // Release file memory mapping
    close();
}

}  // namespace OSF
}  // namespace ouster