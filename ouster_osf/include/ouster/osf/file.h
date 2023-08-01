/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file file.h
 * @brief common OSF file resource for Reader and Writer operations
 *
 */
#pragma once

#include <fstream>
#include <string>

#include "ouster/osf/basics.h"

namespace ouster {
namespace osf {

enum class OpenMode : uint8_t { READ = 0, WRITE = 1 };

/** State of %OSF file */
enum class FileState : uint8_t { GOOD = 0, BAD = 1 };

/** Chunk buffer type to store raw byte buffers of data. */
using ChunkBuffer = std::vector<uint8_t>;

/**
 * Interface to abstract the way of how we handle file system read/write
 * operations.
 */
class OsfFile {
   public:
    explicit OsfFile();

    /**
     * Opens the file.
     * @note Only OpenMode::READ is supported
     */
    explicit OsfFile(const std::string& filename,
                     OpenMode mode = OpenMode::READ);
    ~OsfFile();

    // Header Info
    uint64_t size() const { return size_; };
    std::string filename() const { return filename_; }
    OSF_VERSION version();
    uint64_t metadata_offset();
    uint64_t chunks_offset();

    /** Checks the validity of header and session/file_info blocks. */
    bool valid();

    /**
     * Get the goodness of the file.
     * @todo Need to have more states here (eod, valid, error, etc)
     */
    bool good() const { return state_ == FileState::GOOD; }

    // Convenience operators
    bool operator!() const { return !good(); };
    explicit operator bool() const { return good(); };

    /**
     * Sequential access to the file.
     * This is mimicking the regular file access with the offset
     */
    uint64_t offset() const { return offset_; }

    /**
     * File seek (in mmap mode it's just moving the offset_ pointer
     * without any file system opeations.)
     * @param pos position in the file
     */
    OsfFile& seek(const uint64_t pos);

    /**
     * Read from file (in current mmap mode it's copying data from
     * mmap address to the 'buf' address).
     *
     * @todo Handle errors in future and get the way to read them back
     * with FileState etc.
     */
    OsfFile& read(uint8_t* buf, const uint64_t count);

    bool is_memory_mapped() const;

    /**
     * Mmap access to the file content with the specified offset from the
     * beginning of the file.
     */
    const uint8_t* buf(const uint64_t offset = 0) const;

    /**
     * Clears file handle and allocated resources. In current mmap
     * implementation it's unmapping memory and essentially invalidates the
     * memory addresses that might be captured within MessageRefs
     * and Reader.
     * @sa ouster::osf::MessageRef, ouster::osf::Reader
     */
    void close();

    /** Debug helper method to dump OsfFile state to a string. */
    std::string to_string();

    // Copy policy
    // Don't allow the copying of the file handler
    OsfFile(const OsfFile&) = delete;
    OsfFile& operator=(const OsfFile&) = delete;

    // Move policy
    // But it's ok to move with the ownership transfer of the underlying file
    // handler (mmap).
    OsfFile(OsfFile&& other);
    OsfFile& operator=(OsfFile&& other);

    std::shared_ptr<ChunkBuffer> read_chunk(uint64_t offset);

    uint8_t* get_header_chunk_ptr();
    uint8_t* get_metadata_chunk_ptr();

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
    uint64_t offset_;

    // Size of the opened file in bytes
    uint64_t size_;

    // Mmaped memory address pointed to the beginning of the file (byte 0)
    uint8_t* file_buf_;

    // File reading access
    std::ifstream file_stream_;
    std::shared_ptr<ChunkBuffer> header_chunk_;
    std::shared_ptr<ChunkBuffer> metadata_chunk_;

    // Last read chunk cached, to save the double read on the sequence of verify
    // and then read iterator access (used only when compiled with
    // OUSTER_OSF_NO_MMAP, and in mmap version we rely on the OS/kernel caching)
    std::shared_ptr<ChunkBuffer> chunk_cache_;
    uint64_t chunk_cache_offset_;

    // Internal state
    FileState state_;
};

}  // namespace osf
}  // namespace ouster
