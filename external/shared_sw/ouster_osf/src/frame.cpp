#include "frame.h"

#include "flatbuffers/util.h"
#include "ouster/osf/version.h"

namespace ouster {
namespace OSF {

FramesSaver::FramesSaver(const std::string& dest_file,
                         const OSF::FileInfo& source_file_info)
    : FramesSaver(dest_file, source_file_info.sensors(), source_file_info.id(),
                  source_file_info.lidar_frame_mode(),
                  source_file_info.range_multiplier()) {}

FramesSaver::FramesSaver(const std::string& dest_file,
                         const OSF::sensors_map& sensors,
                         const std::string& session_id,
                         const OSF::OSF_FRAME_MODE lidar_frame_mode,
                         const uint8_t range_multiplier)
    : sensors_(sensors),
      session_id_(session_id),
      lidar_frame_mode_(lidar_frame_mode),
      range_multiplier_(range_multiplier),
      session_start_ts_(0),
      session_end_ts_(0),
      id_to_offset_(),
      current_offset_(0),
      chunk_builder_(
          OSFChunk(5 * 1024 * 1024, lidar_frame_mode_, range_multiplier_)),
      frame_id_(0),
      next_start_ts_(0),
      ls_traj_counter_(sensors_),
      dest_file_(dest_file),
      dest_file_tmp_(dest_file_ + ".tmp"),
      header_size_(OSF::initEmptyOsfFile(dest_file_tmp_)),
      max_frame_dur_(get_max_frame_duration(sensors_)),
      skipped_cnt_(0) {}

FramesSaver::Status FramesSaver::logMessage(const OSF::MessageRef& msg) {
    FramesSaver::Status pre_log_status =
        pre_log_check(msg.id(), msg.type(), msg.ts());
    if (pre_log_status != FramesSaver::Status::OK) return pre_log_status;
    return saveMessage(msg);
}

FramesSaver::Status FramesSaver::logTrajectory(const uint32_t id,
                                               const OSF::Trajectory& traj,
                                               OSF::ts_t ts) {
    FramesSaver::Status pre_log_status =
        pre_log_check(id, OSF::MessageType::TRAJECTORY, ts);
    if (pre_log_status != FramesSaver::Status::OK) return pre_log_status;
    return saveTrajectory(id, traj, ts);
}

FramesSaver::Status FramesSaver::pre_log_check(const uint32_t id,
                                               const OSF::MessageType type,
                                               const OSF::ts_t ts) {
    // Always skip messages that happens before the frame start_ts
    if (ts == OSF::ts_t(0) || ts < OSF::ts_t(chunk_builder_.start_ts)) {
        skipped_cnt_++;
        return Status::SKIPPED;
    }

    // Check upper bound and signal FINISH frame back
    if (!isTimestampInFrame(ts)) {
        return Status::FINISH;
    }

    if (type == OSF::MessageType::LIDAR_SCAN) {
        // Need to check for valid sensor id here because only
        // lidar_scans from sensors map should cause Status::FINISH
        if (ls_traj_counter_.valid_id(id) &&
            !ls_traj_counter_.add_lidar_scan(id, ts)) {
            next_start_ts_ = ts;
            return Status::FINISH;
        }
    } else if (type == OSF::MessageType::TRAJECTORY) {
        // Need to check for valid sensor id here because we want to
        // return (skip) only sensor trajectories if they present
        if (ls_traj_counter_.valid_id(id) &&
            !ls_traj_counter_.add_trajectory(id, ts)) {
            return Status::SKIPPED;
        }
    }

    return Status::OK;
}

FramesSaver::Status FramesSaver::saveMessage(const OSF::MessageRef& msg) {
    FramesSaver::Status pre_save_status = pre_save_check(msg.ts());
    if (pre_save_status != FramesSaver::Status::OK) return pre_save_status;

    if (msg.type() == MessageType::LIDAR_SCAN &&
        msg.file_info().lidar_frame_mode() != lidar_frame_mode_) {
        // Recode lidar_scan because destination frame mode is differet
        // going up to LidarScan and back then
        auto ls = msg.as_lidar_scan();
        if (!ls) {
            std::cout << "ERROR: Can't read lidar_scan for msg.ts = "
                      << msg.ts().count()
                      << ", id = " << static_cast<int>(msg.id()) << std::endl;
            return Status::SKIPPED;
        }
        auto sensor = sensors_[msg.id()];
        chunk_builder_.logLidarScan(msg.id(),
                                    sensor->meta.format.pixel_shift_by_row, *ls,
                                    msg.ts().count());
    } else {
        chunk_builder_.logMessage(
            reinterpret_cast<const StampedMessage*>(msg.buf()));
    }

    return Status::OK;
}

FramesSaver::Status FramesSaver::saveTrajectory(const uint32_t id,
                                                const OSF::Trajectory& traj,
                                                const OSF::ts_t ts) {
    FramesSaver::Status pre_save_status = pre_save_check(ts);
    if (pre_save_status != FramesSaver::Status::OK) return pre_save_status;
    chunk_builder_.logTrajectory(static_cast<uint8_t>(id), traj, ts.count());
    return Status::OK;
}

FramesSaver::Status FramesSaver::pre_save_check(const OSF::ts_t ts) {
    // messages with ts == 0 considered anomalies for frameChunker and skipped
    if (ts == OSF::ts_t(0)) return Status::SKIPPED;

    if (session_start_ts_.count() == 0 || session_start_ts_ > ts) {
        session_start_ts_ = ts;
    }
    if (session_end_ts_ < ts) {
        session_end_ts_ = ts;
    }
    return Status::OK;
}

void FramesSaver::saveFrame() {
    // Skip saving if frame isn't started
    if (chunk_builder_.start_ts == 0) return;

    id_to_offset_[frame_id_] = current_offset_;

    // Append current chunk to tmp file
    current_offset_ += chunk_builder_.saveChunk(dest_file_tmp_);

    // Clean everything and prepare to start new frame
    chunk_builder_.restartChunk();
    ls_traj_counter_.reset();
    next_start_ts_ = OSF::ts_t(0);
    frame_id_++;
}

void FramesSaver::finish() {
    saveFrame();

    // create session object
    auto session_builder = flatbuffers::FlatBufferBuilder(1024);

    build_session(session_builder, session_id_,
                  ouster::OSF::OSF_SESSION_MODE_OSF_FRAMED, lidar_frame_mode_,
                  range_multiplier_, sensors_, session_start_ts_,
                  session_end_ts_, id_to_offset_);

    auto session_size = session_builder.GetSize();
    ouster::OSF::saveFlatBuffer(session_builder, dest_file_tmp_, true);
    ouster::OSF::closeOsfFile(dest_file_tmp_, current_offset_ + header_size_,
                              session_size);

    std::rename(dest_file_tmp_.c_str(), dest_file_.c_str());
}

bool FramesSaver::isTimestampInFrame(const OSF::ts_t ts) const {
    // Everything in the frame if it's not started (start_ts == 0)
    if (chunk_builder_.start_ts == 0) return true;

    // Upper bound cut-off on the next_start_ts_ if it's present
    const auto max_ts =
        next_start_ts_.count() > 0
            ? next_start_ts_
            : OSF::ts_t(chunk_builder_.start_ts) + max_frame_dur_;

    return OSF::ts_t(chunk_builder_.start_ts) <= ts && ts < max_ts;
}

}  // namespace OSF
}  // namespace ouster
