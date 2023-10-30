/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once
#include "chunk_generated.h"
#include "header_generated.h"
#include "metadata_generated.h"
#include "ouster/osf/basics.h"

// OSF v2 basic types for LidarSensor and LidarScan/Imu Streams
#include "os_sensor/lidar_scan_stream_generated.h"
#include "os_sensor/lidar_sensor_generated.h"

namespace ouster {
namespace osf {

inline const gen::Metadata* get_osf_metadata_from_buf(const uint8_t* buf) {
    return ouster::osf::gen::GetSizePrefixedMetadata(buf);
}

inline const gen::Header* get_osf_header_from_buf(const uint8_t* buf) {
    return gen::GetSizePrefixedHeader(buf);
}

/**
 * Verifies the validity of Header buffer and whether it's safe to read it.
 * It's just checking the well formed Flatbuffer table (not CRC32 check here)
 *
 * @param buf      Header buffer, size prefixed
 * @param buf_size buffer size (with prefix size bytes but not including CRC32)
 * @return         true if buffer is valid and can be read
 */
inline bool verify_osf_header_buf(const uint8_t* buf, const uint32_t buf_size) {
    auto verifier = flatbuffers::Verifier(buf, buf_size);
    return gen::VerifySizePrefixedHeaderBuffer(verifier);
}

/**
 * Checks the validity of a Metadata buffer and whether it's safe to read it.
 * It's checking the well formed Flatbuffer table and CRC32.
 *
 * @param buf      metadata buffer, size prefixed
 * @param buf_size buffer size (with CRC32 and prefix size bytes)
 * @return         true if buffer is valid and can be read
 */
bool check_osf_metadata_buf(const uint8_t* buf, const uint32_t buf_size);

/**
 * Checks the validity of a Chunk buffer and whether it's safe to read it.
 * It's checking the well formed Flatbuffer table and CRC32.
 *
 * @param buf      metadata buffer, size prefixed
 * @param buf_size buffer size (with CRC32 and prefix size bytes)
 * @return         true if buffer is valid and can be read
 */
bool check_osf_chunk_buf(const uint8_t* buf, const uint32_t buf_size);

/** transforms Flatbuffers vector to a std::vector. */
template <typename T>
std::vector<T> vector_from_fb_vector(const flatbuffers::Vector<T>* fb_vec);

// ============ File operations ==========================

/**
 * Saves the buffer content to the file with additional 4 bytes of calculated
 * CRC32 field in the end. Successfull operation writes size + 4 bytes to the
 * file.
 *
 * @param buf pointer to the data to save, full content of the buffer used
 *            to calculate CRC
 * @param size number of bytes to read from buffer and store to the file
 * @param filename full path to the file
 * @param append if true appends the content to the end of the file,
 *               otherwise - overwrite the file with the current buffer.
 * @return number of bytes actuallt written to the file. Successfull write is
 *         size + 4 bytes (4 bytes for CRC field)
 *
 */
uint64_t buffer_to_file(const uint8_t* buf, const uint64_t size,
                        const std::string& filename, bool append = false);

/**
 * Saves the content of Flatbuffer builder to the file with CRC32 field
 * appended to the actual bytes. Usually it's a size prefixed finished builder
 * but not necessarily
 *
 * @param builder Flatbuffers builder
 * @param filename filename to save bytes
 * @param append if true appends the content to the end of the file,
 *               otherwise - overwrite the file with the current buffer.
 * @return number of bytes actuallt written to the file. Successfull write is
 *         size + 4 bytes (4 bytes for CRC field)
 */
uint64_t builder_to_file(flatbuffers::FlatBufferBuilder& builder,
                         const std::string& filename, bool append = false);

/**
 * Starts the OSF v2 file with a header (in INVALID state).
 *
 * @param filename of the file to be created. Overwrite if file exists.
 *
 */
uint64_t start_osf_file(const std::string& filename);

/**
 * Finish OSF v2 file with updated offset to metadata and filesize. As a
 * result file left in VALID state.
 *
 * @param filename of the file to be created. Overwrite if file exists.
 * @return number of bytes actuallt written to the file.
 */
uint64_t finish_osf_file(const std::string& filename,
                         const uint64_t metadata_offset,
                         const uint32_t metadata_size);

/** Debug method to print Flatbuffers Metadata buffer */
void print_metadata_buf(const uint8_t* buf, const uint32_t buf_size);

}  // namespace osf
}  // namespace ouster
