#include "chunk.h"

#include <chrono>

#include "ouster/lidar_scan.h"
#include "ouster/osf/lidar_scan_stat.h"
#include "png_tools.h"
#include "util_impl.h"

using OSFC = ouster::OSF::OSFChunk;

const int NUM_BYTES_TO_SHOW = 64;

OSFC::OSFChunk(const int chunk_size,
               const ouster::OSF::OSF_FRAME_MODE frameMode,
               const int range_mult)
    : max_chunk_size(chunk_size),
      builder(flatbuffers::FlatBufferBuilder(chunk_size)),
      message_stream(std::vector<flatbuffers::Offset<StampedMessage>>()),
      range_multiplier(range_mult),
      start_ts(0),
      stop_ts(0),
      frame_mode(frameMode) {}

void OSFC::logLidarScan(const uint8_t id, const std::vector<int>& px_offset,
                        LidarScan& lidar_scan, uint64_t ts) {
    // Check if it is the first
    checkFirstMessage(ts);

    // Encode LidarScan to PNG buffers
    ScanData scan_data =
        scanEncode(lidar_scan, px_offset, frame_mode, range_multiplier);

    // Add PNG encoded channels to lidar_scan.scan vector
    std::vector<flatbuffers::Offset<ouster::OSF::scan_channel>> channels;
    for (const auto& channel_data : scan_data) {
        auto m_png = builder.CreateVector(channel_data);
        channels.emplace_back(Createscan_channel(builder, m_png));
    }
    auto scan_msg = builder.CreateVector(channels);

    // Add timestamps as lidar_scan.ts vector
    std::vector<uint64_t> ts_v(lidar_scan.ts.size());
    for (size_t i = 0; i < lidar_scan.ts.size(); ++i) {
        ts_v[i] = lidar_scan.ts[i].count();
    }
    auto ts_msg = builder.CreateVector(ts_v);
    auto scan = Createlidar_scan(builder, scan_msg, ts_msg);

    auto tm_scan =
        CreateStampedMessage(builder, ts, id, Message_lidar_scan, scan.Union());
    message_stream.push_back(tm_scan);

    updateEnd(ts);
}

void OSFC::logTrajectory(const uint8_t id, const osf_poses& poses,
                         uint64_t ts) {
    // Check if it is the first
    checkFirstMessage(ts);

    // Convert poses -> osf_poses
    std::vector<ouster::OSF::pose_f> pp;
    for (auto p : poses) {
        pp.emplace_back(p.orientation.w(), p.orientation.x(), p.orientation.y(),
                        p.orientation.z(), p.position.x(), p.position.y(),
                        p.position.z());
    }
    auto m_poses = builder.CreateVectorOfStructs(pp);
    auto trajectory = Createtrajectory(builder, m_poses);
    auto m_trajectory = CreateStampedMessage(
        builder, ts, id, Message_trajectory, trajectory.Union());
    message_stream.push_back(m_trajectory);

    updateEnd(ts);
}

void OSFC::logGps(const uint8_t id, const Gps& gps, uint64_t ts) {
    checkFirstMessage(ts);
    auto m_gps =
        Creategps_waypoint(builder, gps.latitude, gps.epy, gps.longitude,
                           gps.epx, gps.altitude, gps.epv, gps.ept, gps.speed,
                           gps.eps, gps.track, gps.epd, gps.climb, gps.epc);

    auto gps_message = CreateStampedMessage(
        builder, ts, id, Message_gps_waypoint, m_gps.Union());
    message_stream.push_back(gps_message);

    updateEnd(ts);
}

void OSFC::logImu(const uint8_t id, const std::array<double, 3>& angular_vel,
                  const std::array<double, 3>& linear_accel,
                  const std::array<uint64_t, 3>& ts_v, const uint64_t ts) {
    checkFirstMessage(ts);
    const ouster::OSF::vec3 osf_angular_vel(angular_vel[0], angular_vel[1],
                                            angular_vel[2]);
    const ouster::OSF::vec3 osf_linear_accel(linear_accel[0], linear_accel[1],
                                             linear_accel[2]);
    auto m_ts = builder.CreateVector(ts_v.data(), ts_v.size());
    auto m_imu = Createimu(builder, &osf_angular_vel, &osf_linear_accel, m_ts);
    auto imu_message =
        CreateStampedMessage(builder, ts, id, Message_imu, m_imu.Union());
    message_stream.push_back(imu_message);

    updateEnd(ts);
}

void OSFC::logMessage(const ouster::OSF::StampedMessage* msg) {
    switch (msg->message_type()) {
        case ouster::OSF::Message_lidar_scan:
            logScanMessage(msg);
            break;
        case ouster::OSF::Message_trajectory:
            logTrajectoryMessage(msg);
            break;
        case ouster::OSF::Message_gps_waypoint:
            logGpsMessage(msg);
            break;
        case ouster::OSF::Message_imu:
            logImuMessage(msg);
            break;
        default:
            std::cerr << "UNKNOWN MESSAGE TYPE" << std::endl;
            break;
    }
}

void OSFC::logScanMessage(const ouster::OSF::StampedMessage* msg) {
    // Filter empty messages (we can have them from broken OSF files)
    if (!msg->message_as_lidar_scan()) return;
    if (!msg->message_as_lidar_scan()->scan()) return;

    checkFirstMessage(msg->ts());

    auto msg_scan_vec = msg->message_as_lidar_scan()->scan();
    std::vector<flatbuffers::Offset<ouster::OSF::scan_channel>> channels;
    if (msg_scan_vec) {
        // Copy all scan channels without looking inside
        for (size_t i = 0; i < msg_scan_vec->size(); ++i) {
            auto channel_buffer = msg_scan_vec->Get(i)->buffer();
            auto scan_channel = builder.CreateVector(channel_buffer->data(),
                                                     channel_buffer->size());
            channels.emplace_back(Createscan_channel(builder, scan_channel));
        }
    }
    auto scan_msg = builder.CreateVector(channels);

    // Copy lidar_scan.ts
    auto msg_ls_ts_vec = msg->message_as_lidar_scan()->ts();
    flatbuffers::Offset<flatbuffers::Vector<uint64_t>> ts_msg(0);
    if (msg_ls_ts_vec) {
        std::vector<uint64_t> ts_v;
        ts_v.reserve(msg_ls_ts_vec->size());
        for (size_t i = 0; i < msg_ls_ts_vec->size(); ++i) {
            ts_v.push_back(msg_ls_ts_vec->Get(i));
        }
        ts_msg = builder.CreateVector(ts_v);
    }

    auto scan = Createlidar_scan(builder, scan_msg, ts_msg);
    auto tm_scan = CreateStampedMessage(builder, msg->ts(), msg->id(),
                                        Message_lidar_scan, scan.Union());
    message_stream.push_back(tm_scan);

    updateEnd(msg->ts());
}

void OSFC::logTrajectoryMessage(const ouster::OSF::StampedMessage* msg) {
    // Filter empty messages (we can have them from broken OSF files)
    if (!msg->message_as_trajectory()) return;
    if (!msg->message_as_trajectory()->poses()) return;

    checkFirstMessage(msg->ts());

    auto m_poses = builder.CreateVectorOfStructs(
        reinterpret_cast<const ouster::OSF::pose_f*>(
            msg->message_as_trajectory()->poses()->Data()),
        msg->message_as_trajectory()->poses()->size());
    auto trajectory = Createtrajectory(builder, m_poses);
    auto m_trajectory = CreateStampedMessage(
        builder, msg->ts(), msg->id(), Message_trajectory, trajectory.Union());
    message_stream.push_back(m_trajectory);

    updateEnd(msg->ts());
}

void OSFC::logGpsMessage(const ouster::OSF::StampedMessage* msg) {
    // Filter empty messages (we can have them from broken OSF files)
    if (!msg->message_as_gps_waypoint()) return;

    checkFirstMessage(msg->ts());

    auto gps_msg = msg->message_as_gps_waypoint();
    auto gps = Creategps_waypoint(
        builder, gps_msg->latitude(), gps_msg->epy(), gps_msg->longitude(),
        gps_msg->epx(), gps_msg->altitude(), gps_msg->epv(), gps_msg->ept(),
        gps_msg->speed(), gps_msg->eps(), gps_msg->track(), gps_msg->epd(),
        gps_msg->climb(), gps_msg->epc());
    auto m_gps = CreateStampedMessage(builder, msg->ts(), msg->id(),
                                      Message_gps_waypoint, gps.Union());
    message_stream.push_back(m_gps);

    updateEnd(msg->ts());
}

void OSFC::logImuMessage(const ouster::OSF::StampedMessage* msg) {
    // Filter empty messages (we can have them from broken OSF files)
    if (!msg->message_as_imu()) return;

    checkFirstMessage(msg->ts());

    auto imu_ts = builder.CreateVector(msg->message_as_imu()->ts()->data(), 3);
    auto imu = Createimu(builder, msg->message_as_imu()->angular_vel(),
                         msg->message_as_imu()->linear_accel(), imu_ts);
    auto m_imu = CreateStampedMessage(builder, msg->ts(), msg->id(),
                                      Message_imu, imu.Union());
    message_stream.push_back(m_imu);

    updateEnd(msg->ts());
}

void OSFC::setFrameMode(ouster::OSF::OSF_FRAME_MODE fMode) {
    frame_mode = fMode;
}

ouster::OSF::OSF_FRAME_MODE OSFC::getFrameMode() { return frame_mode; }

void OSFC::restartChunk() {
    start_ts = 0;
    stop_ts = 0;
    builder = flatbuffers::FlatBufferBuilder(max_chunk_size);
    message_stream.clear();
}

void OSFC::updateEnd(uint64_t ts) {
    if (ts > stop_ts) stop_ts = ts;
}

bool OSFC::checkChunkSize(uint64_t ts) {
    if (builder.GetSize() > max_chunk_size) {
        stop_ts = ts;
        return true;
    }
    return false;
}

void OSFC::checkFirstMessage(uint64_t ts) {
    if (message_stream.empty()) {
        start_ts = ts;
    }
}

size_t OSFC::getSize() { return builder.GetSize(); }

size_t OSFC::saveChunk(std::string filename) {
    auto stream = builder.CreateVectorOfSortedTables(&message_stream);

    auto my_chunk = CreateosfChunk(builder, start_ts, stop_ts, stream);
    builder.FinishSizePrefixed(my_chunk, osfChunkIdentifier());

    // Check arbitrary memory corruption for OSF chunk with Flatbuffer vierifer.
    // Quick and fast (~10-20 microsecs) but not thorough (like CRC)
    if (!verify_osf_chunk_buf(builder.GetBufferPointer(), builder.GetSize())) {
        std::cerr << "ERROR: Broken OSF chunk formed in memory."
                  << ", chunk size = " << builder.GetSize() << ", bytes: "
                  << to_string(builder.GetBufferPointer(),
                               builder.GetSize() < NUM_BYTES_TO_SHOW
                                   ? builder.GetSize()
                                   : NUM_BYTES_TO_SHOW)
                  << std::endl;
    }

    saveFlatBuffer(builder, filename, true);
    size_t size = builder.GetSize();
    restartChunk();
    return size;
}
