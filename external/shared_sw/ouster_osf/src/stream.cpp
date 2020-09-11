#include "ouster/osf/stream.h"

#include <cstdio>
#include <fstream>

#include "chunk.h"
#include "flatbuffers/util.h"
#include "osfSession_generated.h"
#include "util_impl.h"

using OSFS = ouster::OSF::OSFStream;

OSFS::OSFStream(const ouster::OSF::sensors_map& sensors,
                const std::string& dump_path,
                const OSF::OSF_FRAME_MODE lidar_frame_mode,
                const int range_mult)
    : sensor_map(sensors),
      ts_to_offset(std::map<uint64_t, uint64_t>()),
      chunk_builder(
          new OSFChunk(5 * 1024 * 1024, lidar_frame_mode, range_mult)),
      current_offset(0),
      ts_start(0),
      ts_end(0),
      newChunk(true),
      dump_path(dump_path),
      temp_path(flatbuffers::ConCatPathFileName(dump_path, "stream.osf.tmp")),
      header_size(initEmptyOsfFile(temp_path)),
      range_multiplier(range_mult),
      session_id("newSession") {}

OSFS::~OSFStream() = default;

void OSFS::logLidarScan(uint8_t id, LidarScan& lidar_scan, uint64_t ts) {
    // check new chunk
    checkNewChunk(ts);
    chunk_builder->logLidarScan(
        id, sensor_map[id]->meta.format.pixel_shift_by_row, lidar_scan, ts);
    updateStartEnd(ts);
    // check chunk size
    checkChunk(ts);
}

void OSFS::logTrajectory(uint8_t id, const osf_poses& poses, uint64_t ts) {
    // check new chunk
    checkNewChunk(ts);
    chunk_builder->logTrajectory(id, poses, ts);
    updateStartEnd(ts);
    // check chunk size
    checkChunk(ts);
}

void OSFS::logGps(uint8_t id, const Gps& gps, uint64_t ts) {
    // check new chunk
    checkNewChunk(ts);
    chunk_builder->logGps(id, gps, ts);
    updateStartEnd(ts);
    // check chunk size
    checkChunk(ts);
}

void OSFS::logImu(uint8_t id, const std::array<double, 3>& angular_vel,
                  const std::array<double, 3>& linear_accel,
                  const std::array<uint64_t, 3>& ts_v, uint64_t ts) {
    // check new chunk
    checkNewChunk(ts);
    chunk_builder->logImu(id, angular_vel, linear_accel, ts_v, ts);
    updateStartEnd(ts);
    // check chunk size
    checkChunk(ts);
}

void OSFS::setSessionId(std::string sId) { session_id = sId; }

std::string OSFS::getSessionId() { return session_id; }

void OSFS::updateStartEnd(uint64_t ts) {
    if (ts_start == 0 || ts < ts_start) ts_start = ts;
    if (ts > ts_end) ts_end = ts;
}

std::string OSFS::createSession(const std::string& file_name) {
    auto session_builder = flatbuffers::FlatBufferBuilder(1024);

    build_session(
        session_builder, session_id, ouster::OSF::OSF_SESSION_MODE_OSF_STREAM,
        chunk_builder->frame_mode, chunk_builder->range_multiplier, sensor_map,
        ouster::OSF::ts_t(ts_start), ouster::OSF::ts_t(ts_end), ts_to_offset);

    auto session_size = session_builder.GetSize();

    saveFlatBuffer(session_builder, temp_path, true);

    closeOsfFile(temp_path, current_offset + header_size, session_size);

    const std::string& file = file_name.empty()
                                  ? "session-" + std::to_string(ts_start) +
                                        "-" + std::to_string(ts_end) + ".osf"
                                  : file_name;

    // Left here for occasional use in generation ml-data OSFs
    // const std::string& file = file_name.empty()
    //                             ? "0000.osf"
    //                             : file_name;

    auto new_path = flatbuffers::ConCatPathFileName(dump_path, file);
    std::rename(temp_path.c_str(), new_path.c_str());

    return file;
}

void OSFS::checkNewChunk(uint64_t ts) {
    if (newChunk) {
        ts_to_offset[ts] = current_offset;
        newChunk = false;
    }
}

void OSFS::checkChunk(uint64_t ts) {
    if (chunk_builder->checkChunkSize(ts)) {
        saveChunk();
    }
}

void OSFS::checkTempFileIsInit() {
    std::ifstream tmpFile;
    tmpFile.open(temp_path, std::fstream::in);
    tmpFile.seekg(0, std::ios::end);
    if ((!tmpFile.is_open()) || tmpFile.tellg() == 0) {
        // Empty File
        std::cerr << "Empty tmpChunk.osf , initialiazing it." << std::endl;
        header_size = initEmptyOsfFile(temp_path);
    }
}

void OSFS::saveChunk() {
    checkTempFileIsInit();
    current_offset += chunk_builder->saveChunk(
        temp_path);  // +4 for prefixedSize ????????????
    newChunk = true;
}

size_t OSFS::getSize() { return current_offset + chunk_builder->getSize(); }
