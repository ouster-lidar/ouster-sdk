/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file file.h
 * @brief common OSF file resource for Reader and Writer operations
 *
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <ostream>
#include <string>
#include <vector>

#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/offset.h"
#include "ouster/version.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

// TODO[tws] remove FileState; a future change will ensure all OsfFile instances
// are "GOOD" after construction ;)
/**
 * Enum representing the state of the %OSF file.
 */
enum class FileState : uint8_t {
    GOOD = 0,  ///< The osf is good.
    BAD = 1    ///< There is something wrong with the OSF.
};

// TODO[tws] likely replace with OsfBuffer.
/**
 * Chunk buffer type to store raw byte buffers of data.
 */
using ChunkBuffer = std::vector<uint8_t>;

/**
 * @brief Base class for OSF file handling.
 *
 * This class provides an interface for reading OSF files, including methods
 * for accessing metadata, reading data, and managing file state.
 */
class OUSTER_API_CLASS OsfFile {
   public:
    /**
     * Opens the OSF file.
     * @param[in] path The path to the OSF.
     */
    OUSTER_API_FUNCTION
    explicit OsfFile(const std::string& path);

    /**
     * Cleans up any filebuffers/memory mapping.
     */
    OUSTER_API_FUNCTION
    virtual ~OsfFile();

    // Header Info

    /**
     * Returns the size of the OSF file.
     *
     * @return The size of the OSF file in bytes.
     */
    OUSTER_API_FUNCTION
    virtual uint64_t size() const = 0;

    /**
     * Returns the path of the OSF file.
     *
     * @return The path of the OSF file.
     */
    OUSTER_API_FUNCTION
    std::string path() const;

    /**
     * Returns the version of the OSF file.
     *
     * @return The version of the OSF file.
     */
    OUSTER_API_FUNCTION
    ouster::sdk::core::Version version();

    /// The most recent public version of the OSF schema.
    static const ouster::sdk::core::Version CURRENT_VERSION;
    OUSTER_DIAGNOSTIC_PUSH
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED
    /**
     * @deprecated Use CURRENT_VERSION
     */
    OUSTER_DEPRECATED_MSG(CURRENT_VERSION,                        // NOLINT
                          OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)  // NOLINT
    static const ouster::sdk::core::Version current_version;
    OUSTER_DIAGNOSTIC_POP
    /**
     * Convert the parsed version to its integer representation.
     *
     * @param[in] parsed_version the version to encode.
     * @return The integer value that should be written to the header given this
     * parsed version.
     */
    OUSTER_API_FUNCTION
    static uint64_t serialized_version(
        ouster::sdk::core::Version parsed_version);

    /**
     * Returns the osf_offset in the OSF file where the
     * metadata section is located.
     *
     * @return Offset to the metadata
     */
    OUSTER_API_FUNCTION
    OsfOffset metadata_offset();

    /**
     * Returns the osf_offset in the OSF file where the
     * chunk section is located.
     *
     * @throws std::runtime_error Exception on bad osf file.
     *
     * @return Offset to the chunks
     */
    OUSTER_API_FUNCTION
    OsfOffset chunks_offset();

    /**
     * Return the status of the OSF file.
     * @todo Need to have more states here (eod, valid, error, etc)
     *
     * @return If the OSF file is good or not.
     */
    OUSTER_API_FUNCTION
    bool good() const;

    // Convenience operators
    /**
     * Return the negated status of the OSF file.
     *
     * @relates good
     *
     * @return If the OSF file is good or not, negated.
     */
    OUSTER_API_FUNCTION
    bool operator!() const;  // TODO[tws] probably remove

    /**
     * Return the status of the OSF file.
     *
     * @relates good
     *
     * @return If the OSF file is good or not.
     */
    OUSTER_API_FUNCTION
    explicit operator bool() const;  // TODO[tws] probably remove

    /**
     * Read the following span of bytes (represented by OsfOffset)
     * into the provided OsfBuffer.
     * @throws std::out_of_range if the offset exceeds file size or is invalid.
     * @param[in] offset The offset and number of bytes to read.
     * @return an OsfBuffer initialized with the data specified by the offset.
     */
    OUSTER_API_FUNCTION
    virtual OsfBuffer read(OsfOffset offset) = 0;

    // TODO[tws] define base_offset
    /**
     * Read the following span of bytes (represented by OsfOffset)
     * using the provided base offset
     * into the provided OsfBuffer.
     * @throws std::out_of_range if the offset exceeds file size or is invalid.
     * @param[in] offset The offset and number of bytes to read.
     * @param[in] base_offset
     * @return an OsfBuffer initialized with the data specified by the offset.
     */
    OUSTER_API_FUNCTION
    virtual OsfBuffer read(OsfOffset base_offset, OsfOffset offset) = 0;

    /**
     * Clears file handle and allocated resources. In current mmap
     * implementation it's unmapping memory and essentially invalidates the
     * memory addresses that might be captured within MessageRefs
     * and Reader.
     * @sa ouster::sdk::osf::MessageRef, ouster::sdk::osf::Reader
     */
    OUSTER_API_FUNCTION
    virtual void close();

    // TODO[tws] - add an "is_open" or "is_closed" method.

    /**
     * Debug helper method to dump OsfFile state to a string.
     *
     * @return The string representation of OsfFile
     */
    OUSTER_API_FUNCTION
    std::string to_string();

    /**
     * Copy policy:
     * Don't allow the copying of the file handler
     */
    OUSTER_API_FUNCTION
    OsfFile(const OsfFile&) = delete;

    /**
     * Copy policy:
     * Don't allow the copying of the file handler
     */
    OUSTER_API_FUNCTION
    OsfFile& operator=(const OsfFile&) = delete;

    /**
     * Move policy:
     * Allow transferring ownership of the underlying file
     * handler (mmap).
     *
     * @param[in] other The OSF file to move
     */
    OUSTER_API_FUNCTION
    OsfFile(OsfFile&& other);

    /**
     * Move policy:
     * Allow transferring ownership of the underlying file
     * handler (mmap).
     *
     * @param[in] other The OSF file to move
     */
    OUSTER_API_FUNCTION
    OsfFile& operator=(OsfFile&& other);

    /**
     * Get a pointer to the start of the header chunk.
     *
     * @return Pointer to the header chunk. nullptr if filestream is bad.
     */
    OUSTER_API_FUNCTION
    OsfOffset get_header_chunk_offset();

    /**
     * Get a pointer to the start of the header chunk.
     *
     * @return Pointer to the metadata chunk. nullptr if filestream is bad.
     */
    OUSTER_API_FUNCTION
    OsfOffset get_metadata_chunk_offset();

    /**
     * Returns a reference to OSF buffer containing the OSF header chunk.
     *
     * @return Reference to the OsfBuffer containing header data.
     */
    OUSTER_API_FUNCTION
    const OsfBuffer& get_header_chunk();

    /**
     * Returns a reference to buffer containing the OSF metadata chunk.
     *
     * @return Reference to the OsfBuffer containing metadata.
     */
    OUSTER_API_FUNCTION
    const OsfBuffer& get_metadata_chunk();

   protected:
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
     * Internal state of the OSF.
     */
    FileState state_;

    /**
     * Path to the OSF as it was passed in contructor.
     */
    std::string path_;

    /// initializes the header and metadata chunks
    void initialize_header_and_metadata();

    /// The OSF spec version of this file
    OsfVersion version_;

    /// TODO[tws] consider encapsulating the header and metadata to permit
    /// reading them w/o having to re-parse
    OsfBuffer header_chunk_;

    OsfBuffer metadata_chunk_;

    OsfOffset header_offset_;

    OsfOffset metadata_offset_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster

/// @cond DOXYGEN_SHOULD_SKIP_THIS
template <>
struct std::hash<ouster::sdk::osf::OsfOffset> {
    std::size_t operator()(const ouster::sdk::osf::OsfOffset& k) const;
    /// @endcond
};
