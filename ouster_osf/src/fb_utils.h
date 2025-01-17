/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once
#include "chunk_generated.h"
#include "header_generated.h"
#include "metadata_generated.h"
#include "ouster/osf/basics.h"
#include "ouster/visibility.h"

// OSF v2 basic types for LidarSensor and LidarScan/Imu Streams
#include "os_sensor/lidar_scan_stream_generated.h"
#include "os_sensor/lidar_sensor_generated.h"

namespace ouster {
namespace osf {

/// @TODO Change up tests to not use this stuff
OUSTER_API_FUNCTION
inline const gen::Metadata* get_osf_metadata_from_buf(const uint8_t* buf) {
    return ouster::osf::gen::GetSizePrefixedMetadata(buf);
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
bool check_osf_metadata_buf(const uint8_t* buf, uint32_t buf_size);

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
bool check_osf_chunk_buf(const uint8_t* buf, uint32_t buf_size);

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
 * @return Number of bytes actually written to the file.
 */
OUSTER_API_FUNCTION
uint64_t finish_osf_file(const std::string& filename, uint64_t metadata_offset,
                         uint32_t metadata_size);
}  // namespace osf
}  // namespace ouster
