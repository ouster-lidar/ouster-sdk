#include <unordered_map>

#include "chunk.h"
#include "ouster/osf/file_info.h"
#include "ouster/osf/message.h"
#include "ouster/osf/util.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {

// Helper struct to collect paired lidar_scans with trajectories
// organized as a map by
// message_id :-> (lidar_scan_timestamp, trajectory_timestamp)
struct FrameLidarScanTrajectoryCounter {
    FrameLidarScanTrajectoryCounter(const FileInfo::sensors_map& sm) {
        for (const auto s : sm) {
            ls_traj_[s.first] =
                std::make_pair(FileInfo::ts_t(0), FileInfo::ts_t(0));
        }
    }

    // Clear all sensor slots and prepare to accept messages for the next frame
    void reset() {
        for (auto& lt : ls_traj_) {
            lt.second = std::make_pair(FileInfo::ts_t(0), FileInfo::ts_t(0));
        }
    }

    // Get the number of full frames with trajectory and lidar scan filled
    // strict timestamp equality assumed for the saved ls/tr pair.
    uint64_t count_pairs() const {
        uint64_t res = 0;
        for (const auto& lt : ls_traj_) {
            if (lt.second.first.count() > 0 &&
                lt.second.first == lt.second.second) {
                // both present and non-empty
                res++;
            }
        }
        return res;
    }

    // Add lidar scan message timestamp to the message id (mid) slot.
    // Returns:
    // false - if lidar_scan slot is already filled for the mid or it's
    //         not a valid sensor id.
    // true  - if lidar_scan timestamp added to the slot and it was empty.
    bool add_lidar_scan(const uint32_t mid, const FileInfo::ts_t mts) {
        if (!valid_id(mid)) return false;
        if (ls_traj_[mid].first.count() > 0) return false;
        ls_traj_[mid].first = mts;
        return true;
    }

    // Add trajectory message timestamp to the message id (mid) slot.
    // Returns:
    // false - if trajectory slot is already filled for the mid or it's
    //         not a valid sensor id.
    // true  - if lidar_scan timestamp added to the slot and it was empty.
    bool add_trajectory(const uint32_t mid, const FileInfo::ts_t mts) {
        if (!valid_id(mid)) return false;
        if (ls_traj_[mid].second.count() > 0) return false;
        ls_traj_[mid].second = mts;
        return true;
    }

    // Checks whether message id (mid) represents the valid sensor id.
    //   - it's mainly to filter out vehicle trajectory message which has
    //     the same type but message id in the 127+ space.
    bool valid_id(const uint32_t mid) const { return ls_traj_.count(mid) > 0; }

   private:
    // Maps lidar_scan with trajectories:
    //   - ls_traj_[id].first - > lidar scan timestamp
    //   - ls_traj_[id].second -> trajectory timestamp
    std::unordered_map<uint32_t, std::pair<FileInfo::ts_t, FileInfo::ts_t>>
        ls_traj_;
};

// Track the framed OSF session and facilitates the framed message saving
// to the OSF file.
class FramesSaver {
   public:
    // Add message Status, see logMessage() comments for details.
    enum Status { OK = 0, SKIPPED, FINISH };

    FramesSaver(const std::string& dest_file, const OSF::sensors_map& sensors,
                const std::string& session_id = "newSession",
                const OSF::OSF_FRAME_MODE lidar_frame_mode =
                    OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_32,
                const uint8_t range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

    FramesSaver(const std::string& dest_file,
                const OSF::FileInfo& source_file_info);

    // Checks whether the give ts belongs to the current frame.
    bool isTimestampInFrame(const OSF::ts_t ts) const;

    // Adds msg or object to the frame with boundary chekcs and state tracking.
    // Expects messages logged in a ts order and lidar_scan/trajectories tracked
    // in pairs for every sensor (sensors specified in constructor)
    //
    // Returns: Status of the operation
    //    OK      - msg added to the chunk/frame successfully
    //    SKIPPED - msg wasn't saved because it's timing before the frame
    //              start_ts
    //    FINISH  - msg wasn't added to the chunk/frame because it's out of
    //              frame upper bound and the driver need to finish current
    //              frame with saveFrame() in order to proceed.
    //
    FramesSaver::Status logMessage(const OSF::MessageRef& msg);
    FramesSaver::Status logTrajectory(const uint32_t id,
                                      const OSF::Trajectory& traj,
                                      const OSF::ts_t ts);

    // Unconstrained add of message and objects directly to chunk/frame without
    // any checks.
    // But still can be Status::SKIPPED due to bad ts for example.
    FramesSaver::Status saveMessage(const OSF::MessageRef& msg);
    FramesSaver::Status saveTrajectory(const uint32_t id,
                                       const OSF::Trajectory& traj,
                                       const OSF::ts_t ts);

    // Current frame_id of a tracked frame
    int frame_id() const { return frame_id_; }

    // Current start timestamp of a tracked frame.
    // Returns: OSF::ts_t(0) if frame wasn't started yet.
    OSF::ts_t start_ts() const { return OSF::ts_t(chunk_builder_.start_ts); }

    // Max frame duration is set to be a minimal of (1 / sensor_freq_i)
    // where sensor_freq_i is the sensor[i] frequency in Hz.
    OSF::ts_t max_frame_duration() const { return max_frame_dur_; }

    // Number of logMessages() attempts that cause SKIPPED status.
    uint64_t skipped() const { return skipped_cnt_; }

    // End the current frame and prepare to accept messages for the next frame
    void saveFrame();

    // Finish the session and save the OSF file to the dest_file path.
    // Current frame saver can't be used after call of this method and further
    // behavior is undefined.
    void finish();

   private:
    FramesSaver::Status pre_log_check(const uint32_t id,
                                      const OSF::MessageType type,
                                      const OSF::ts_t ts);
    FramesSaver::Status pre_save_check(const OSF::ts_t ts);

    // Session params and sensors set
    OSF::sensors_map sensors_;
    std::string session_id_;
    OSF::OSF_FRAME_MODE lidar_frame_mode_;
    uint8_t range_multiplier_;

    // Tracking session start/end
    OSF::ts_t session_start_ts_;
    OSF::ts_t session_end_ts_;

    // Session map
    std::map<uint64_t, size_t> id_to_offset_;

    // Offset to the current frame from the beginning of the session object
    size_t current_offset_;

    ouster::OSF::OSFChunk chunk_builder_;

    // Current frame id
    int frame_id_;

    // Next frame start_ts, sets when encounetered ls/traj out of current frame
    OSF::ts_t next_start_ts_;

    // Track the ls/traj pairs filling
    FrameLidarScanTrajectoryCounter ls_traj_counter_;

    // Resulting OSF file
    std::string dest_file_;
    std::string dest_file_tmp_;

    size_t header_size_;

    // Calculated maximum frame duration from sensors frequencies
    OSF::ts_t max_frame_dur_;

    // Number of messages that get's skipped status back
    uint64_t skipped_cnt_;
};

}  // namespace OSF
}  // namespace ouster
