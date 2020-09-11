#pragma once

#include "flatbuffers/flatbuffers.h"
#include "osfChunk_generated.h"
#include "osfHeader_generated.h"
#include "osfSession_generated.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/file_info.h"
#include "ouster/osf/lidar_scan_stat.h"
#include "ouster/osf/util.h"
#include "ouster/osf/version.h"

namespace ouster {
namespace OSF {

// Print errors only in DEBUG mode
#ifndef NDEBUG
inline void print_error(const std::string& filename, const std::string& msg) {
    fprintf(stderr, "Error Osf[%s]: %s\n", filename.c_str(), msg.c_str());
}
#else
#define print_error(a, b) ((void)0)
#endif

void saveFlatBuffer(flatbuffers::FlatBufferBuilder& builder,
                    std::string filename, bool append = false);

// Debug method to get hex buf values in string
std::string to_string(const uint8_t* buf, const size_t count);

inline const osfHeader* get_osf_header_from_buf(const uint8_t* buf) {
    return GetSizePrefixedosfHeader(buf);
}

inline const osfSession* get_osf_session_from_buf(const uint8_t* buf) {
    return GetSizePrefixedosfSession(buf);
}

inline const osfChunk* get_osf_chunk_from_buf(const uint8_t* buf) {
    return GetSizePrefixedosfChunk(buf);
}

/**
 * Checks the validity of osfSession buffer and whether it's safe to read it.
 *
 * @param buf      osfSession buffer, size prefixed
 * @param buf_size buffer size
 * @return         true if buffer is valid and can be read
 */
inline bool verify_osf_session_buf(const uint8_t* buf, const size_t buf_size) {
    auto verifier = flatbuffers::Verifier(buf, buf_size);
    return VerifySizePrefixedosfSessionBuffer(verifier);
}

/**
 * Checks the validity of osfHeader buffer and whether it's safe to read it.
 *
 * @param buf      osfHeader buffer, size prefixed
 * @param buf_size buffer size
 * @return         true if buffer is valid and can be read
 */
inline bool verify_osf_header_buf(const uint8_t* buf, const size_t buf_size) {
    auto verifier = flatbuffers::Verifier(buf, buf_size);
    return VerifySizePrefixedosfHeaderBuffer(verifier);
}

/**
 * Checks the validity of osfChunk buffer and whether it's safe to read it.
 *
 * @param buf      osfChunk buffer, size prefixed
 * @param buf_size buffer size
 * @return         true if buffer is valid and can be read
 */
inline bool verify_osf_chunk_buf(const uint8_t* buf, const size_t buf_size) {
    auto verifier = flatbuffers::Verifier(buf, buf_size);
    return VerifySizePrefixedosfChunkBuffer(verifier);
}

// buf - points to the beginning of the OSF file
inline size_t get_chunks_offset_from_buf(const uint8_t* buf) {
    const size_t header_size = readPrefixedSizeFromOffset(buf);
    // chunks starts right after the osfHeader
    return SIZE_OF_PREFIXED_SIZE + header_size;
}

// Get an ordered sequence of offsets by frame index or by timestamp
std::map<uint64_t, size_t> get_map_from_file_info(const FileInfo& file_info);

std::unique_ptr<sensor> sensor_from_osf_sensor(const osf_sensor* s);

// Enum conversions from/to
// ouster::sensor::lidar_mode <-> ouster::OSF::LIDAR_MODE
ouster::sensor::lidar_mode lidar_mode_from_osf_mode(
    ouster::OSF::LIDAR_MODE osf_mode);
OSF::LIDAR_MODE osf_mode_from_lidar_mode(ouster::sensor::lidar_mode lidar_mode);

// Converts string as defined in
// external/shared_sw/ouster_example/ouster_client/src/types.cpp:lidar_mode_strings
// to the ouster::OSF::LIDAR_MODE
OSF::LIDAR_MODE osf_lidar_mode_of_string(const std::string& mode);

// Functions to construct our objects from general OSF StampedMessage
std::unique_ptr<LidarScan> lidar_scan_from_osf_message(
    const StampedMessage* msg, const FileInfo& file_info);
std::unique_ptr<Gps> gps_from_osf_message(const StampedMessage* msg);
std::unique_ptr<Imu> imu_from_osf_message(const StampedMessage* msg);
std::unique_ptr<Trajectory> traj_from_osf_message(const StampedMessage* msg);

struct FileStatsFramed {
    double dt_mean;
    double dt_var;
    uint64_t dt_past_cnt;
    uint64_t dt_jumps_cnt;
    uint64_t ls_total_cnt;
    uint64_t ls_with_traj_cnt;  // matched only with traj in a frame
    uint64_t start_end_mismatch_cnt;
    uint64_t overlapped_frames_cnt;
    FileInfo::ts_t session_start_ts;
    FileInfo::ts_t session_end_ts;
    uint64_t full_frames_cnt;
    double full_frames_ratio;
    uint64_t frames_cnt;
};

FileStatsFramed calc_framed_file_stats(const std::string& file);
std::string to_string(const FileStatsFramed& fstats);

std::ostream& operator<<(std::ostream& os, const LidarScanStat& lss);
std::ostream& operator<<(std::ostream& os, const LidarScan& ls);

int get_max_lidar_frequency(const FileInfo::sensors_map& sensors);

OSF::ts_t get_max_frame_duration(const OSF::sensors_map& sensors);

flatbuffers::Offset<OSF::osf_sensor> create_osf_sensor(
    flatbuffers::FlatBufferBuilder& fbb, const OSF::sensor& sensor);

void build_session(flatbuffers::FlatBufferBuilder& fbb, const std::string& id,
                   const OSF::OSF_SESSION_MODE mode,
                   const OSF::OSF_FRAME_MODE lidar_frame_mode,
                   const uint8_t range_multiplier,
                   const OSF::sensors_map& sensors, const OSF::ts_t start_ts,
                   const OSF::ts_t end_ts,
                   const std::map<uint64_t, size_t>& id_to_offset);

}  // namespace OSF
}  // namespace ouster
