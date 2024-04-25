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

/**
 * Enum representing the available file opening modes.
 */
enum class OpenMode : uint8_t {
    READ = 0,  ///< Open the file in read-only mode.
    WRITE = 1  ///< Open the file in write-only mode. (CURRENTLY NOT SUPPORTED)
};

/**
 * Enum representing the state of the %OSF file.
 */
enum class FileState : uint8_t {
    GOOD = 0,  ///< The file is good.
    BAD = 1    ///< There is something wrong with the file.
};

/**
 * Chunk buffer type to store raw byte buffers of data.
 */
using ChunkBuffer = std::vector<uint8_t>;

/**
 * Interface to abstract the way of how we handle file system read/write
 * operations.
 */
class OsfFile {
   public:
    /**
     * Default constructor, sets most data to nullptr and 0.
     */
    explicit OsfFile();

    /**
     * Opens the OSF file.
     * @note Only OpenMode::READ is supported
     *
     * @param[in] filename The OSF file to open
     * @param[in] mode The mode to open the file in, this argument is optional.
     */
    explicit OsfFile(const std::string& filename,
                     OpenMode mode = OpenMode::READ);

    /**
     * Cleans up any filebuffers/memory mapping.
     */
    ~OsfFile();

    // Header Info

    /**
     * Returns the size of the OSF file.
     *
     * @return The size of the OSF file in bytes.
     */
    uint64_t size() const;

    /**
     * Returns the filename of the open OSF file.
     *
     * @return The filename of the open OSF file.
     */
    std::string filename() const;

    /**
     * Returns the version of the OSF file.
     *
     * @return The version of the OSF file.
     */
    OSF_VERSION version();

    /**
     * Returns the offset in the OSF file where the
     * metadata section is located.
     *
     * @throws std::logic_error Exception on bad osf file.
     *
     * @return Offset to the metadata in bytes
     */
    uint64_t metadata_offset();

    /**
     * Returns the offset in the OSF file where the
     * chunk section is located.
     *
     * @throws std::logic_error Exception on bad osf file.
     *
     * @return Offset to the chunks in bytes
     */
    uint64_t chunks_offset();

    /**
     * Checks the validity of header and session/file_info blocks.
     *
     * @return If the header, session, and file_info blocks are valid.
     */
    bool valid();

    /**
     * Return the status of the OSF file.
     * @todo Need to have more states here (eod, valid, error, etc)
     *
     * @return If the OSF file is good or not.
     */
    bool good() const;

    // Convenience operators
    /**
     * Return the negated status of the OSF file.
     *
     * @relates good
     *
     * @return If the OSF file is good or not, negated.
     */
    bool operator!() const;

    /**
     * Return the status of the OSF file.
     *
     * @relates good
     *
     * @return If the OSF file is good or not.
     */
    explicit operator bool() const;

    /**
     * Get the current offset in the OSF file.
     *
     * @return The current offset in the OSF file.
     */
    uint64_t offset() const;

    /**
     * File seek (in mmap mode it's just moving the offset_ pointer
     * without any file system opeations.)
     *
     * @throws std::logic_error Exception on bad osf file.
     * @throws std::out_of_range Exception on out of range seek.
     *
     * @param[in] pos position in the file
     * @return A reference to `this` object.
     */
    OsfFile& seek(uint64_t pos);

    /**
     * Read from file (in current mmap mode it's copying data from
     * mmap address to the 'buf' address).
     *
     * @todo Handle errors in future and get the way to read them back
     * with FileState etc.
     *
     * @throws std::logic_error Exception on bad osf file.
     * @throws std::out_of_range Exception on out of range read.
     *
     * @param[out] buf The buffer to write to.
     * @param[in] count The number of bytes to write to buf.
     * @return A reference to `this` object.
     */
    OsfFile& read(uint8_t* buf, const uint64_t count);

    /**
     * Returns whether the OSF file is memory mapped or not.
     *
     * @return Is the OSF file memory mapped or not.
     */
    bool is_memory_mapped() const;

    /**
     * Mmap access to the file content with the specified offset from the
     * beginning of the file.
     *
     * @throws std::logic_error Exception on bad osf file.
     * @throws std::logic_error Exception not being memory mapped.
     * @throws std::out_of_range Exception on out of range read.
     *
     * @param[in] offset The specified offset to access into the OSF file, this
     *                   argument is optional.
     * @return The pointer to the OSF file.
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

    /**
     * Debug helper method to dump OsfFile state to a string.
     *
     * @return The string representation of OsfFile
     */
    std::string to_string();

    /**
     * Copy policy:
     * Don't allow the copying of the file handler
     */
    OsfFile(const OsfFile&) = delete;

    /**
     * @copydoc OsfFile::OsfFile(const OsfFile&)
     */
    OsfFile& operator=(const OsfFile&) = delete;

    /**
     * Move policy:
     * Allow transferring ownership of the underlying file
     * handler (mmap).
     */
    OsfFile(OsfFile&& other);

    /**
     * @copydoc OsfFile::OsfFile(OsfFile&& other)
     */
    OsfFile& operator=(OsfFile&& other);

    /**
     * Read chunk specified at offset.
     *
     * @throws std::out_of_range Exception on out of range read.
     *
     * @param[in] offset The offset to read the chunk from.
     * @return Shared pointer to the chunk. nullptr if osf file is bad
     */
    std::shared_ptr<ChunkBuffer> read_chunk(uint64_t offset);

    /**
     * Get a pointer to the start of the header chunk.
     *
     * @return Pointer to the header chunk. nullptr if filestream is bad.
     */
    uint8_t* get_header_chunk_ptr();

    /**
     * Get a pointer to the start of the header chunk.
     *
     * @return Pointer to the metadata chunk. nullptr if filestream is bad.
     */
    uint8_t* get_metadata_chunk_ptr();

   private:
    /**
     * Convenience method to set error and print it's content.
     *
     * @todo [pb] Adding more error states will probably extend the set of this
     * function.
     *
     * @param[in] msg Message to print
     */
    void error(const std::string& msg = std::string());

    /**
     * Opened filename as it was passed in contructor.
     */
    std::string filename_;

    /**
     * Current offset to the file. (not used in mmaped implementation) but used
     * for copying(reading) blocks of memory from the file to the specified
     * memory.
     */
    uint64_t offset_;

    /**
     * Size of the opened file in bytes.
     */
    uint64_t size_;

    /**
     * Mmaped memory address pointed to the beginning of the file (byte 0)
     */
    uint8_t* file_buf_;

    /**
     * File stream for reading.
     */
    std::ifstream file_stream_;

    /**
     * Pointer for the osf file header chunk.
     */
    std::shared_ptr<ChunkBuffer> header_chunk_;

    /**
     * Pointer for the metadata chunk
     */
    std::shared_ptr<ChunkBuffer> metadata_chunk_;

    /**
     * Last read chunk cached, to save the double read on the sequence of verify
     * and then read iterator access (used only when compiled with
     * OUSTER_OSF_NO_MMAP, and in mmap version we rely on the OS/kernel caching)
     */
    std::shared_ptr<ChunkBuffer> chunk_cache_;

    uint64_t chunk_cache_offset_;

    /**
     * Internal state of the OSF file.
     */
    FileState state_;
};

}  // namespace osf
}  // namespace ouster
