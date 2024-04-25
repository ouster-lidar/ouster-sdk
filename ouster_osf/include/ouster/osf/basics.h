/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file basics.h
 * @brief basic functions for OSF
 *
 */
#pragma once

#include "chunk_generated.h"
#include "header_generated.h"
#include "metadata_generated.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

// OSF basic types for LidarSensor and LidarScan/Imu Streams
#include "os_sensor/lidar_scan_stream_generated.h"
#include "os_sensor/lidar_sensor_generated.h"

namespace ouster {

/**
 * %OSF v2 space
 */
namespace osf {

// current fb generated code in ouster::osf::gen
namespace gen {
using namespace v2;
}

/**
 * Enumerator for the OSF Version. This will change whenever the underlying
 * flatbuffer structures change.
 */
enum OSF_VERSION {
    V_INVALID = 0,  ///< Invalid OSF Version
    V_1_0,          ///< Original version of the OSF (2019/9/16)
    V_1_1,          ///< Add gps/imu/car trajectory to the OSF (2019/11/14)
    V_1_2,          ///< Change gps_waypoint type to Table in order to
                    ///< support Python language generator
    V_1_3,          ///< Add extension for Message in osfChunk
                    ///< and for Session in osfSession (2020/03/18)
    V_1_4,          ///< Gen2/128 support (2020/08/11)

    V_2_0 = 20  ///< Second Generation OSF v2
};

/**
 * Chunking strategies. Refer to RFC0018 for more details.
 */
enum ChunksLayout {
    LAYOUT_STANDARD = 0,  ///< not used currently
    LAYOUT_STREAMING = 1  ///< default layout (the only one for a user)
};

/**
 * To String Functionality For ChunksLayout
 *
 * @param[in] chunks_layout The data to get the string representation format
 * @return The string representation
 */
std::string to_string(ChunksLayout chunks_layout);

/**
 * From String Conversion Functionality To ChunksLayout
 *
 * @param[in] s The String Representation of ChunksLayout
 * @return The corrosponding ChunksLayout object
 */
ChunksLayout chunks_layout_of_string(const std::string& s);

// stable common types mapped to ouster::osf
using v2::HEADER_STATUS;

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
 * To String Functionality For HEADER_STATUS
 *
 * @param[in] status The data to get the string representation format
 * @return The string representation
 */
std::string to_string(const HEADER_STATUS status);

/**
 * Debug method to get hex buf values in string
 *
 * @param[in] buf The buffer to dump to string.
 * @param[in] count The size of the buffer.
 * @param[in] max_show_count The number of bytes to dump. This arg is optional.
 * @return The string representation
 */
std::string to_string(const uint8_t* buf, const size_t count,
                      const size_t max_show_count = 0);

/**
 * Internal method for reading a file and returning the text
 * data.
 *
 * @param[in] filename The file to read.
 * @return The text of the file specified.
 */
std::string read_text_file(const std::string& filename);

/**
 * Reads the prefix size of the Flatbuffers buffer. First 4 bytes.
 *
 * @param[in] buf Pointer to Flatbuffers buffer stared with prefixed size
 * @return the size recovered from the stored prefix size
 */
uint32_t get_prefixed_size(const uint8_t* buf);

/**
 * Calculates the full size of the block (prefixed_size + size + CRC32).
 *
 * @param[in] buf Pointer to Flatbuffers buffer stared with prefixed size
 * @return the calculated size of the block
 */
uint32_t get_block_size(const uint8_t* buf);

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
bool check_prefixed_size_block_crc(
    const uint8_t* buf,
    const uint32_t max_size = std::numeric_limits<uint32_t>::max());

/** @defgroup OsfBatchingFunctions Osf Batching Functions. */

/**
 * Makes the closure to batch lidar_packets and emit LidarScan object.
 * Result returned through callback handler(ts, LidarScan).
 * LidarScan uses user modified field types
 *
 * @ingroup OsfBatchingFunctions
 *
 * @param[in] info The sensor metadata to use.
 * @param[in] ls_field_types The field types to use.
 * @param[in] handler  The callback to use on the results.
 * @return Closure to batch and emit LidarScan objects.
 */
std::function<void(const osf::ts_t, const uint8_t*)> make_build_ls(
    const ouster::sensor::sensor_info& info,
    const LidarScanFieldTypes& ls_field_types,
    std::function<void(const ts_t, const ouster::LidarScan&)> handler);

/**
 * The above make_build_ls() function overload. In this function, LidarScan
 * uses default field types by the profile
 *
 * @ingroup OsfBatchingFunctions
 *
 * @param[in] info The sensor metadata to use.
 * @param[in] handler  The callback to use on the results.
 * @return Closure to batch and emit LidarScan objects.
 */
std::function<void(const osf::ts_t, const uint8_t*)> make_build_ls(
    const ouster::sensor::sensor_info& info,
    std::function<void(const ts_t, const ouster::LidarScan&)> handler);

}  // namespace osf
}  // namespace ouster
