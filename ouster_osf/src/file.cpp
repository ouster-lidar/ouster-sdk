/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/file.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include "compat_ops.h"
#include "fb_utils.h"
#include "ouster/osf/crc32.h"

namespace ouster {
namespace osf {

namespace {

// Print errors only in DEBUG mode
#ifndef NDEBUG
inline void print_error(const std::string& filename, const std::string& msg) {
    fprintf(stderr, "Error Osf[%s]: %s\n", filename.c_str(), msg.c_str());
}
#else
#define print_error(a, b) ((void)0)
#endif

}  // namespace

// ======== Construction ============

OsfFile::OsfFile()
    : filename_(),
      offset_(0),
      size_(0),
      file_buf_(nullptr),
      file_stream_{},
      header_chunk_{nullptr},
      metadata_chunk_{nullptr},
      chunk_cache_{nullptr},
      chunk_cache_offset_{std::numeric_limits<uint64_t>::max()},
      state_(FileState::BAD) {}

OsfFile::OsfFile(const std::string& filename, OpenMode mode) : OsfFile() {
    filename_ = filename;

    // TODO[pb]: Extract to open function
    if (mode == OpenMode::READ) {
        if (is_dir(filename_)) {
            error("got a dir, but expected a file");
            return;
        }

        int64_t sz = file_size(filename_);
        if (sz <= 0) {
            error();
            return;
        }
        // TODO[pb]: This leads to incorrect file size for 4Gb+ files on the
        //           32 bit systems like Emscripten/WASM. We need to check
        //           size_t everywhere and replace it so it can hold file
        //           sizes and file offsets bigger than 4Gb in 32 bit systems.
        size_ = static_cast<uint64_t>(sz);

#ifdef OUSTER_OSF_NO_MMAP
        // TODO[pb]: Maybe consider adding a runtime parameter to open file
        // with mmap or open/read? Also better handling/removing of class
        // members for OsfFile can be done so we are not copying empty
        // values when NO_MMAP is absent... But I can't make my mind
        // about RUNTIME/COMPILATION time parametrization and for now
        // will leave it in a half backed state: COMPILE time directive
        // but some fields will be left empty and copied/check during runtime
        // there is no hit in performance/memory due to this leftovers
        // that I could spot.
        file_stream_ =
            std::ifstream(filename_, std::ios::in | std::ios::binary);
        if (!file_stream_.good()) {
            error();
            return;
        }
#else
        file_buf_ = mmap_open(filename_);
        if (!file_buf_) {
            error();
            return;
        }
#endif

        state_ = FileState::GOOD;
    } else {
        // Write is not yet implemented within this class. And other modes
        // too.
        error("write mode not implemented");
        return;
    }
}

OSF_VERSION OsfFile::version() {
    if (!good()) {
        return OSF_VERSION::V_INVALID;
    }
    auto osf_header = get_osf_header_from_buf(get_header_chunk_ptr());
    return static_cast<OSF_VERSION>(osf_header->version());
}

uint64_t OsfFile::metadata_offset() {
    if (!good()) throw std::logic_error("bad osf file");
    auto osf_header = get_osf_header_from_buf(get_header_chunk_ptr());
    return osf_header->metadata_offset();
}

uint64_t OsfFile::chunks_offset() {
    if (!good()) throw std::logic_error("bad osf file");
    const uint32_t header_size = get_prefixed_size(get_header_chunk_ptr());
    if (version() < OSF_VERSION::V_2_0) {
        throw std::logic_error("bad osf file: only version >= 20 supported");
    }
    return FLATBUFFERS_PREFIX_LENGTH + header_size + osf::CRC_BYTES_SIZE;
}

bool OsfFile::valid() {
    if (!good()) {
        return false;
    }

    uint32_t header_size =
        get_prefixed_size(get_header_chunk_ptr()) + FLATBUFFERS_PREFIX_LENGTH;

    // Check flatbuffers osfHeader validity
    if (!verify_osf_header_buf(get_header_chunk_ptr(), header_size)) {
        print_error(filename_, "OSF header verification has failed.");
        return false;
    }

    if (!check_prefixed_size_block_crc(get_header_chunk_ptr(),
                                       header_size + CRC_BYTES_SIZE)) {
        print_error(filename_, "OSF header has an invalid CRC.");
        return false;
    }

    auto osf_header = get_osf_header_from_buf(get_header_chunk_ptr());
    if (osf_header->status() != v2::HEADER_STATUS::VALID) {
        print_error(filename_, "OSF header is not valid.");
        return false;
    }

    if (osf_header->file_length() != size_) {
        print_error(filename_, "OSF header file size field is incorrect.");
        return false;
    }

    uint64_t metadata_offset = osf_header->metadata_offset();

    if (osf_header->version() < OSF_VERSION::V_2_0) {
        // Check flatbuffers osfSession validity [V1]
        print_error(filename_, "OSF prior version 2.0 is not supported!");
        return false;
    }

    // Check flatbuffers metadata validity
    if (!ouster::osf::check_osf_metadata_buf(get_metadata_chunk_ptr(),
                                             size_ - metadata_offset)) {
        print_error(filename_, "OSF metadata verification has failed.");
        return false;
    }

    return true;
}

// ========= Geneal Data Access =============

OsfFile& OsfFile::seek(const uint64_t pos) {
    if (!good()) throw std::logic_error("bad osf file");
    if (pos > size_) {
        std::stringstream ss;
        ss << "seek for " << pos << " but the file size is " << size_;
        throw std::out_of_range(ss.str());
    }
    if (file_stream_.is_open()) {
        file_stream_.seekg(pos);
    }
    offset_ = pos;
    return *this;
}

OsfFile& OsfFile::read(uint8_t* buf, const uint64_t count) {
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
    if (file_stream_.is_open()) {
        file_stream_.read(reinterpret_cast<char*>(buf), count);
        offset_ = file_stream_.tellg();
    } else if (file_buf_ != nullptr) {
        std::memcpy(buf, file_buf_ + offset_, count);
        offset_ += count;
    }
    return *this;
}

// ===== Mmapped access to the file content memory =====

const uint8_t* OsfFile::buf(const uint64_t offset) const {
    if (!good()) throw std::logic_error("bad osf file");
    if (!is_memory_mapped()) throw std::logic_error("not a mmap file");
    if (offset >= size_)
        throw std::out_of_range("out of range osf file access");
    return file_buf_ + offset;
}

bool OsfFile::is_memory_mapped() const { return file_buf_ != nullptr; }

// ======= Helpers =============

void OsfFile::error(const std::string& msg) {
    state_ = FileState::BAD;
    if (!msg.empty()) {
        print_error(filename_, msg);
    } else {
        print_error(filename_, get_last_error());
    }
}

std::string OsfFile::to_string() {
    std::stringstream ss;
    ss << "OsfFile [filename = '" << filename_ << "', "
       << "state = " << static_cast<int>(state_) << ", "
       << "version = " << version() << ", "
       << "size = " << size_ << ", offset = " << offset_;
    if (this->good()) {
        auto osf_header = get_osf_header_from_buf(get_header_chunk_ptr());
        ss << ", osf.file_length = " << osf_header->file_length() << ", "
           << "osf.metadata_offset = " << osf_header->metadata_offset() << ", "
           << "osf.status = " << static_cast<int>(osf_header->status());
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
      file_stream_(std::move(other.file_stream_)),
      header_chunk_(std::move(other.header_chunk_)),
      metadata_chunk_(std::move(other.metadata_chunk_)),
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
        file_stream_ = std::move(other.file_stream_);
        header_chunk_ = std::move(other.header_chunk_);
        metadata_chunk_ = std::move(other.metadata_chunk_);
        state_ = other.state_;
        other.file_buf_ = nullptr;
        other.state_ = FileState::BAD;
    }
    return *this;
};

// ========= Release resources =================

void OsfFile::close() {
    if (file_buf_) {
        if (!mmap_close(file_buf_, size_)) {
            error();
            return;
        }
        file_buf_ = nullptr;
        state_ = FileState::BAD;
    }
    if (file_stream_.is_open()) {
        file_stream_.close();
        if (file_stream_.fail()) {
            error();
            return;
        }
        state_ = FileState::BAD;
    }
}

OsfFile::~OsfFile() {
    // Release file memory mapping
    close();
}

std::shared_ptr<ChunkBuffer> OsfFile::read_chunk(const uint64_t offset) {
    if (!good()) {
        return nullptr;
    }
    // check whether it was read last and we have it in cache already
    if (chunk_cache_offset_ == offset && chunk_cache_) {
        return chunk_cache_;
    }
    auto chunk_buf = std::make_shared<ChunkBuffer>(FLATBUFFERS_PREFIX_LENGTH);
    seek(offset);
    read(chunk_buf->data(), FLATBUFFERS_PREFIX_LENGTH);
    uint32_t full_chunk_size = get_prefixed_size(chunk_buf->data()) +
                               FLATBUFFERS_PREFIX_LENGTH + CRC_BYTES_SIZE;
    if (offset + full_chunk_size > size_) {
        std::stringstream ss;
        ss << "read till " << (offset + full_chunk_size)
           << " but the file size is " << size_;
        throw std::out_of_range(ss.str());
    }
    chunk_buf->resize(full_chunk_size);
    read(chunk_buf->data() + FLATBUFFERS_PREFIX_LENGTH,
         full_chunk_size - FLATBUFFERS_PREFIX_LENGTH);

    // update cached chunk
    if (chunk_cache_) {
        chunk_cache_.swap(chunk_buf);
    } else {
        chunk_cache_ = std::move(chunk_buf);
    }
    chunk_cache_offset_ = offset;

    return chunk_cache_;
}

uint8_t* OsfFile::get_header_chunk_ptr() {
    if (!file_stream_.good()) {
        if (header_chunk_) header_chunk_.reset();
        return nullptr;
    }
    if (header_chunk_) return header_chunk_->data();

    auto tmp_offset = offset_;
    header_chunk_ = read_chunk(0);
    seek(tmp_offset);
    return header_chunk_->data();
}

uint8_t* OsfFile::get_metadata_chunk_ptr() {
    uint64_t meta_offset = metadata_offset();
    if (!file_stream_.good()) {
        if (metadata_chunk_) metadata_chunk_.reset();
        return nullptr;
    }
    if (metadata_chunk_) return metadata_chunk_->data();

    auto tmp_offset = offset_;
    metadata_chunk_ = read_chunk(meta_offset);
    seek(tmp_offset);
    return metadata_chunk_->data();
}

}  // namespace osf
}  // namespace ouster