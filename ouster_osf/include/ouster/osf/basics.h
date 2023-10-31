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

enum OSF_VERSION {
    V_INVALID = 0,
    V_1_0,  // Original version of the OSF (2019/9/16)
    V_1_1,  // Add gps/imu/car trajectory to the OSF (2019/11/14)
    V_1_2,  // Change gps_waypoint type to Table in order to
            // support Python language generator
    V_1_3,  // Add extension for Message in osfChunk
            // and for Session in osfSession (2020/03/18)
    V_1_4,  // Gen2/128 support (2020/08/11)

    V_2_0 = 20  // Second Generation OSF v2
};

/// Chunking strategies. Refer to RFC0018 for more details.
enum ChunksLayout {
    LAYOUT_STANDARD = 0,  ///< not used currently
    LAYOUT_STREAMING = 1  ///< default layout (the only one for a user)
};

std::string to_string(ChunksLayout chunks_layout);
ChunksLayout chunks_layout_of_string(const std::string& s);

// stable common types mapped to ouster::osf
using v2::HEADER_STATUS;

/** Common timestamp for all time in ouster::osf */
using ts_t = std::chrono::nanoseconds;

/**
 * Standard Flatbuffers prefix size
 * @todo [pb]: Rename this beast?
 */
static constexpr uint32_t FLATBUFFERS_PREFIX_LENGTH = 4;

/** Return string representation of header */
std::string to_string(const HEADER_STATUS status);

/** Debug method to get hex buf values in string */
std::string to_string(const uint8_t* buf, const size_t count,
                      const size_t max_show_count = 0);

/// Open read test file to a string.
std::string read_text_file(const std::string& filename);

/**
 * Reads the prefix size of the Flatbuffers buffer. First 4 bytes.
 * @param buf pointer to Flatbuffers buffer stared with prefixed size
 * @return the size recovered from the stored prefix size
 */
uint32_t get_prefixed_size(const uint8_t* buf);

/**
 * Calculates the full size of the block (prefixed_size + size + CRC32).
 * @param buf pointer to Flatbuffers buffer stared with prefixed size
 * @return the calculated size of the block
 */
uint32_t get_block_size(const uint8_t* buf);

/**
 * Check the prefixed size buffer CRC32 fields.
 *
 * @param buf is structured as size prefixed Flatbuffer buffer, i.e. first
 *            4 bytes is the size of the buffer (excluding 4 bytes of the size),
 *            and the 4 bytes that follows right after the 4 + <prefixed_size>
 *            is the CRC32 bytes.
 * @param max_size total number of bytes that can be accessed in the buffer,
 *                 as a safety precaution if buffer is not well formed, or if
 *                 first prefixed size bytes are broken.
 * @return true if CRC field is correct, false otherwise
 *
 */
bool check_prefixed_size_block_crc(
    const uint8_t* buf,
    const uint32_t max_size = std::numeric_limits<uint32_t>::max());

/**
 * Makes the closure to batch lidar_packets and emit LidarScan object.
 * Result returned through callback handler(ts, LidarScan).
 * LidarScan uses user modified field types
 */
template <typename ResultHandler>
std::function<void(const osf::ts_t, const uint8_t*)> make_build_ls(
    const ouster::sensor::sensor_info& info,
    const LidarScanFieldTypes& ls_field_types, ResultHandler&& handler) {
    const auto w = info.format.columns_per_frame;
    const auto h = info.format.pixels_per_column;

    std::shared_ptr<LidarScan> ls(nullptr);
    if (ls_field_types.empty()) {
        auto default_ls_field_types = get_field_types(info);
        ls = std::make_shared<LidarScan>(w, h, default_ls_field_types.begin(),
                                         default_ls_field_types.end());

    } else {
        ls = std::make_shared<LidarScan>(w, h, ls_field_types.begin(),
                                         ls_field_types.end());
    }

    auto pf = ouster::sensor::get_format(info);
    auto build_ls_imp = ScanBatcher(w, pf);
    osf::ts_t first_msg_ts{-1};
    return [handler, build_ls_imp, ls, first_msg_ts](
               const osf::ts_t msg_ts, const uint8_t* buf) mutable {
        if (first_msg_ts == osf::ts_t{-1}) {
            first_msg_ts = msg_ts;
        }
        if (build_ls_imp(buf, *ls)) {
            handler(first_msg_ts, *ls);
            // At this point we've just started accumulating new LidarScan, so
            // we are saving the msg_ts (i.e. timestamp of a UDP packet)
            // which contained the first lidar_packet
            first_msg_ts = msg_ts;
        }
    };
}

/**
 * The above make_build_ls() function overload. In this function, LidarScan
 * uses default field types by the profile
 */
template <typename ResultHandler>
std::function<void(const osf::ts_t, const uint8_t*)> make_build_ls(
    const ouster::sensor::sensor_info& info, ResultHandler&& handler) {
    return make_build_ls(info, {}, handler);
}

}  // namespace osf
}  // namespace ouster
