/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "chunk_generated.h"
#include "flatbuffers/flatbuffers.h"
#include "header_generated.h"
#include "metadata_generated.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/visibility.h"

// OSF v2 basic types for LidarSensor and LidarScan/Imu Streams
#include "os_sensor/lidar_scan_stream_generated.h"
#include "os_sensor/lidar_sensor_generated.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/file.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace impl {

/**
 * Implementation details that emits buffer() content as proper
 * Flatbuffer MetadataEntry object.
 *
 * @param[in] fbb The flatbuffer builder to use to make the entry.
 * @return An offset into a flatbuffer for the new entry.
 */
flatbuffers::Offset<gen::MetadataEntry> make_entry(
    const MetadataEntry& entry, flatbuffers::FlatBufferBuilder& fbb);

/**
 * Serialize the MetadataStore to the specified flatbuffer builder
 * and return the resulting byte vector.
 *
 * @param[in] fbb The flatbuffer builder to use.
 * @return The resulting serialized byte vector.
 */
std::vector<flatbuffers::Offset<gen::MetadataEntry>> make_entries(
    const MetadataStore& store, flatbuffers::FlatBufferBuilder& fbb);

class WriterImpl {
   public:
    WriterImpl() = default;
    /**
     * The internal chunk layout of the OSF file.
     */
    ChunksLayout chunks_layout{ChunksLayout::STREAMING};

    /**
     * The internal vector of chunks.
     */
    std::vector<gen::ChunkOffset> chunks{};
};

/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
inline const gen::Metadata* get_osf_metadata_from_buf(const uint8_t* buf) {
    return gen::GetSizePrefixedMetadata(buf);
}

/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
inline const gen::Header* get_osf_header_from_buf(const uint8_t* buf) {
    return gen::GetSizePrefixedHeader(buf);
}

/**
 * Verifies the validity of Header buffer and whether it's safe to read it.
 * It's just checking the well formed Flatbuffer table (not CRC32 check here)
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] buf Header buffer, size prefixed
 * @param[in] buf_size buffer size (with prefix size bytes but not including
 *                     CRC32)
 * @return true if buffer is valid and can be read
 */
OUSTER_API_FUNCTION
inline bool verify_osf_header_buf(const uint8_t* buf, uint32_t buf_size) {
    auto verifier = flatbuffers::Verifier(buf, buf_size);
    return gen::VerifySizePrefixedHeaderBuffer(verifier);
}

/**
 * Checks the validity of a Metadata buffer and whether it's safe to read it.
 * It's checking the well formed Flatbuffer table and CRC32.
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] buf metadata buffer, size prefixed
 * @param[in] buf_size buffer size (with CRC32 and prefix size bytes)
 * @return true if buffer is valid and can be read
 */
OUSTER_API_FUNCTION
bool check_osf_metadata_buf(const ouster::sdk::osf::OsfBuffer& buf);

/**
 * Checks the validity of a Chunk buffer and whether it's safe to read it.
 * It's checking the well formed Flatbuffer table and CRC32.
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] buf      metadata buffer, size prefixed
 * @param[in] buf_size buffer size (with CRC32 and prefix size bytes)
 * @return true if buffer is valid and can be read
 */
OUSTER_API_FUNCTION
bool check_osf_chunk_buf(const ouster::sdk::osf::OsfBuffer& buf);

/**
 * Transforms Flatbuffers vector to a std::vector.
 * @TODO Change up tests to not use this stuff
 *
 * @tparam T The type of the vector to transform.
 *
 * @param[in] fb_vec The vector to transform.
 * @return The transformed vector.
 **/
template <typename T>
OUSTER_API_FUNCTION std::vector<T> vector_from_fb_vector(
    const flatbuffers::Vector<T>* fb_vec);

// ============ File operations ==========================

/**
 * Saves the buffer content to the file with additional 4 bytes of calculated
 * CRC32 field in the end. Successfull operation writes size + 4 bytes to the
 * file.
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] buf pointer to the data to save, full content of the buffer used
 *            to calculate CRC
 * @param[in] size number of bytes to read from buffer and store to the file
 * @param[in] filename full path to the file
 * @param[in] append if true appends the content to the end of the file,
 *                   otherwise - overwrite the file with the current buffer.
 * @return Number of bytes actually written to the file. Successfull write is
 *         size + 4 bytes (4 bytes for CRC field)
 */
OUSTER_API_FUNCTION
uint64_t buffer_to_file(const uint8_t* buf, uint64_t size,
                        const std::string& filename, bool append = false);

/**
 * Saves the content of Flatbuffer builder to the file with CRC32 field
 * appended to the actual bytes. Usually it's a size prefixed finished builder
 * but not necessarily
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] builder Flatbuffers builder
 * @param[in] filename filename to save bytes
 * @param[in] append if true appends the content to the end of the file,
 *                   otherwise - overwrite the file with the current buffer.
 * @return Number of bytes actually written to the file. Successfull write is
 *         size + 4 bytes (4 bytes for CRC field)
 */
OUSTER_API_FUNCTION
uint64_t builder_to_file(flatbuffers::FlatBufferBuilder& builder,
                         const std::string& filename, bool append = false);

/**
 * Starts the OSF v2 file with a header (in INVALID state).
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] filename of the file to be created. Overwrite if file exists.
 * @return Number of bytes actually written to the file.
 */
OUSTER_API_FUNCTION
uint64_t start_osf_file(const std::string& filename);

/**
 * Finish OSF v2 file with updated offset to metadata and filesize. As a
 * result file left in VALID state.
 * @TODO Change up tests to not use this stuff
 *
 * @param[in] filename of the file to be created. Overwrite if file exists.
 * @param[in] metadata_offset The offset to the metadata blob.
 * @param[in] metadata_size The size of the metadata blob.
 * @param[in] version The version of the OSF file.
 * @return Number of bytes actually written to the file.
 */
OUSTER_API_FUNCTION
uint64_t finish_osf_file(const std::string& filename, uint64_t metadata_offset,
                         uint32_t metadata_size,
                         ouster::sdk::core::Version version =
                             ouster::sdk::osf::OsfFile::CURRENT_VERSION);

}  // namespace impl
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
