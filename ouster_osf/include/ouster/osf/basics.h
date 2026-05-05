/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file basics.h
 * @brief basic functions for OSF
 *
 */
#pragma once

#include "ouster/deprecation.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/offset.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

// OSF basic types for LidarSensor and LidarScan/Imu Streams
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace ouster {
namespace sdk {

/**
 * %OSF v2 space
 */
namespace osf {

/**
 * Enumerator for the OSF Version. This will change whenever the underlying
 * flatbuffer structures change.
 */
enum class OsfVersion : uint64_t {
    V_INVALID = 0,  ///< Invalid OSF Version
    V_1_0,          ///< Original version of the OSF (2019/9/16)
    V_1_1,          ///< Add gps/imu/car trajectory to the OSF (2019/11/14)
    V_1_2,          ///< Change gps_waypoint type to Table in order to
                    ///< support Python language generator
    V_1_3,          ///< Add extension for Message in osfChunk
                    ///< and for Session in osfSession (2020/03/18)
    V_1_4,          ///< Gen2/128 support (2020/08/11)

    V_2_0 = 20,  ///< Second Generation OSF v2
    V_2_1 = 21   ///< Add full index and addtional info to LidarScans
    // IMPORTANT: do not add new entries here; doing so is unnecessary.
    // Instead, use ouster::sdk::core::Version class to specify a semver-style
    // version. The OSF implementation is capable of handling versions beyond
    // those listed here.
};

/**
 * @deprecated Use `OsfVersion` instead.
 */
OUSTER_DEPRECATED_TYPE(OSF_VERSION, OsfVersion,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);

/**
 * Chunking strategies. Refer to RFC0018 for more details.
 */
enum class ChunksLayout {
    STANDARD = 0,   ///< not used currently
    STREAMING = 1,  ///< default layout (the only one for a user)

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::osf::ChunksLayout::STANDARD instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(LAYOUT_STANDARD, STANDARD,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::osf::ChunksLayout::STREAMING instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(LAYOUT_STREAMING, STREAMING,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/**
 * To String Functionality For ChunksLayout
 *
 * @param[in] chunks_layout The data to get the string representation format
 * @return The string representation
 */
OUSTER_API_FUNCTION
std::string to_string(ChunksLayout chunks_layout);

/**
 * From String Conversion Functionality To ChunksLayout
 *
 * @param[in] s The String Representation of ChunksLayout
 * @return The corrosponding ChunksLayout object
 */
OUSTER_API_FUNCTION
ChunksLayout chunks_layout_of_string(const std::string& s);

/**
 * Common timestamp for all time in ouster::osf.
 * Nanoseconds were chosen due to the data coming off of the sensor.
 */
using ts_t = std::chrono::nanoseconds;

/**
 * Standard Flatbuffers prefix size
 * @todo [pb]: Rename this beast?
 */
static constexpr uint32_t FLATBUFFERS_PREFIX_LENGTH = 4;

/**
 * Debug method to get hex buf values in string
 *
 * @param[in] buf The buffer to dump to string.
 * @param[in] count The size of the buffer.
 * @param[in] max_show_count The number of bytes to dump. This arg is optional.
 * @return The string representation
 */
OUSTER_API_FUNCTION
std::string to_string(const uint8_t* buf, const size_t count,
                      const size_t max_show_count = 0);

/**
 * Internal method for reading a file and returning the text
 * data.
 *
 * @param[in] filename The file to read.
 * @return The text of the file specified.
 */
OUSTER_API_FUNCTION
std::string read_text_file(const std::string& filename);

/**
 * Reads the prefix size of the Flatbuffers buffer. First 4 bytes.
 *
 * @param[in] buf Pointer to Flatbuffers buffer stared with prefixed size
 * @return the size recovered from the stored prefix size
 */
OUSTER_API_FUNCTION
uint32_t get_prefixed_size(const OsfBuffer& buf);

/**
 * Reads the prefix size of the Flatbuffers buffer. First 4 bytes.
 *
 * @param[in] buf Pointer to Flatbuffers buffer stared with prefixed size
 * @param[in] offset Offset in the buffer where the prefixed size starts
 * @return the size recovered from the stored prefix size
 */
OUSTER_API_FUNCTION
uint32_t get_prefixed_size(const OsfBuffer& buf, OsfOffset offset);

/**
 * Calculates the full size of the block (prefixed_size + size + CRC32).
 *
 * @param[in] buf Pointer to Flatbuffers buffer stared with prefixed size
 * @return the calculated size of the block
 */
OUSTER_API_FUNCTION
uint32_t get_block_size(const OsfBuffer& buf);

/**
 * Check the prefixed size buffer CRC32 fields.
 *
 * @param[in] buf Structured as size prefixed Flatbuffer buffer, i.e. first
 *                4 bytes is the size of the buffer (excluding 4 bytes of the
 *                size), and the 4 bytes that follows right after the
 *                4 + [prefixed_size] is the CRC32 bytes.
 * @param[in] max_size Total number of bytes that can be accessed in the buffer,
 *                     as a safety precaution if buffer is not well formed, or
 *                     if first prefixed size bytes are broken.
 * @return true if CRC field is correct, false otherwise
 */
OUSTER_API_FUNCTION
bool check_prefixed_size_block_crc(
    const OsfBuffer& buf,
    const uint32_t max_size = std::numeric_limits<uint32_t>::max());

/** @defgroup OsfBatchingFunctions Osf Batching Functions. */

}  // namespace osf
}  // namespace sdk
}  // namespace ouster

#include "ouster/osf/impl/deprecated/enums.h"
