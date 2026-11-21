#pragma once

#include <memory>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class ActiveTimeCorrection
 * @brief Class to handle active time correction for lidar and camera frames.
 *
 * This class provides methods to correct and reset timestamps in the frames of
 * a FrameSet based on sensor information. It checks for synchronization between
 * sensors, ensures monotonicity of timestamps, and applies corrections as
 * needed. Lidar and camera frames are treated alike: a LidarFrame has one
 * timestamp per column, a CameraFrame a single timestamp.
 *
 * Typical usage pattern:
 * ActiveTimeCorrection time_correction(infos);
 *
 * for (auto frames : frame_set_source) {
 *      time_correction.update(frames);  // to apply time correction
 *      slam_engine.update(frames);     // SLAM processing
 *      time_correction.reset(frames);  // to reset the timestamps
 * }
 */
class OUSTER_API_CLASS ActiveTimeCorrection {
   public:
    /**
     * @brief Construct an ActiveTimeCorrection object from sensor info
     * structures.
     *
     * @param[in] sensor_infos A unified vector of SensorInfo, one per sensor,
     * sharing the FrameSet index space (sensor i describes frame i) in any
     * order. Per-sensor state is sized to this list and indexed by frameset
     * position, so lidar/camera ordering does not matter. Each sensor's fps()
     * sets its expected frame period; a sensor reporting fps() <= 0 cannot
     * have its timestamps extrapolated when they go backwards.
     */
    OUSTER_API_FUNCTION
    explicit ActiveTimeCorrection(
        const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos);

    /**
     * @brief pre-registration frames time check and synchronization.
     * It modifies the frames by adding corrected timestamp field. Use the
     * reset() method to remove the added corrected frame timestamps field.
     *
     * @param[in,out] frame_set The lidar and camera frames to be processed.
     *
     * Steps taken:
     *  • Compute overall frame timestamp range.
     *  • Check inter-sensor synchronization:
     *      - If sensors are out of sync, enable packet-offset mode.
     *  • Monotonicity check (ignoring zero timestamps):
     *      - For any frame whose timestamps go backwards, correct its
     * timestamps and enable packet-offset mode. • Packet-offset handling:
     *      - If offset mode is active, compute fallback timestamp offsets.
     *  • Finally: it detects lidar frames that come out of order by frame id.
     *
     * In packet-offset mode each frame's first valid timestamp is mapped onto
     * its host receive time: the first lidar packet for a LidarFrame, the
     * first non-zero receive_timestamp() entry for a CameraFrame. A frame with
     * no receive time keeps its original clock.
     *
     * @remarks when an out of order frame is detected, its range field is
     * zeroed out to invalidate it. This behavior will be revised in the future
     * to avoid modifying the frame data.
     */
    OUSTER_API_FUNCTION
    void update(core::FrameSet& frame_set);

    /**
     * @brief reset frames sensor time post-registration
     *
     * @param[in,out] frame_set The lidar and camera frames to be processed.
     */
    OUSTER_API_FUNCTION
    void reset(core::FrameSet& frame_set);

    // TODO[UN]: made public since it is required by tests
    /**
     * @brief get frame ts ranges
     * @return vector of (start_ts, end_ts) pairs, one per sensor, holding the
     * range of the most recent frame seen from that sensor, or (-1, -1) if
     * none has been seen yet.
     */
    OUSTER_API_FUNCTION
    std::vector<std::pair<int64_t, int64_t>>& last_frame_ts_range() {
        return last_frame_ts_range_;
    }

    /**
     * @brief Determines whether a set of frames are pre-synchronized
     * based on their start timestamps. This function compares the earliest and
     * latest start timestamps from a range of frames and checks if their
     * difference is within the shortest lidar frame duration (or, with no
     * lidar, the shortest frame duration of the other sensors). If the time
     * difference is less than or equal to that gap, the frames are considered
     * pre-synchronized. Missing frames, marked by a negative start, are ignored.
     *
     * @param[in] frame_ts_range A vector of (start_ts, end_ts) pairs
     * representing the time value range for each frame.
     *
     * @return true if the frames are pre-synchronized, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool check_sensors_synchronization(
        const std::vector<std::pair<int64_t, int64_t>>& frame_ts_range) const;

    /**
     * @brief Checks if the timestamps in a frame are strictly
     * increasing (ignoring any zeros) and its first non-zero timestamp is
     * greater than the previous frame’s end timestamp.
     *
     * @param[in] timestamps a vector representing frame timestamps.
     * @param[in] last_frame_stop_ts The end timestamp of the previous frame.
     *
     * @return true if the input timestamps are monotonically increasing, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    static bool is_monotonically_increasing(
        Eigen::Ref<const core::Frame::Header<uint64_t>> timestamps, int64_t last_frame_stop_ts);

    /**
     * @brief Corrects the timestamps of the input frame based on the
     * previous frame's timestamp range and the sensor's frame duration.
     * Leaves the frame untouched if the sensor's frame rate is unknown.
     *
     * @param[in,out] frame The lidar or camera frame whose timestamps need to
     * be corrected.
     * @param[in] sensor_idx The index of the sensor associated with the frame.
     */
    OUSTER_API_FUNCTION
    void correct_frame_ts(core::Frame& frame, size_t sensor_idx);

   private:
    // NOTE: these methods were made public since they are required by tests
    /**
     * @brief Calculates fallback timestamp offsets for the frames of a
     * FrameSet.
     *
     * @param[in] frame_set The lidar and camera frames to process.
     *
     * This function computes, for each frame:
     *  • the packet timestamp offset relative to the earliest packet timestamp
     * across all non-None frames, then subtracts the frame's first valid column
     * timestamp, • or returns None if the frame itself is None.
     *
     * @return A vector of timestamp offsets for each frame.
     */
    OUSTER_API_FUNCTION
    static std::vector<int64_t> calculate_fallback_ts_offset(const core::FrameSet& frame_set);

    /**
     * @brief This method detects if there are any lidar frame jumps (i.e a gap
     * of more than one frame between subsequent frames) or if there are any
     * out-of-order frames in the provided FrameSet. A jump happens
     * when there is a dropped frame or frames resulting in a gap in frame ids
     * for a specific sensor. It returns the minimum frame ID difference for
     * valid frames.
     *
     * @param[in,out] frame_set A vector of LidarFrame objects to
     * be processed. if a frame is out of order, it will be nullified by setting
     * its range values to zero.
     */
    OUSTER_API_FUNCTION
    void detect_out_of_order_frames(core::FrameSet& frame_set);

    std::vector<double> frame_durations_;
    std::vector<int64_t> max_frame_id_;
    double sync_tolerance_ns_{0.0};
    // state variables
    bool use_packet_offset_;
    std::vector<int64_t> last_frame_id_;
    std::vector<std::pair<int64_t, int64_t>> last_frame_ts_range_;

    std::vector<nonstd::optional<core::Frame::Header<uint64_t>>> original_frame_timestamps_;
    std::vector<nonstd::optional<core::Frame::Header<uint64_t>>> original_imu_timestamps_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
