#pragma once

#include <memory>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class ActiveTimeCorrection
 * @brief Class to handle active time correction for LidarFrames.
 *
 * This class provides methods to correct and reset timestamps in LidarFrames
 * based on sensor information. It checks for synchronization between sensors,
 * ensures monotonicity of timestamps, and applies corrections as needed.
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
     * @param[in] infos A vector of shared pointers to SensorInfo objects, one
     * per sensor.
     */
    OUSTER_API_FUNCTION
    ActiveTimeCorrection(const std::vector<std::shared_ptr<core::SensorInfo>>& infos);

    /**
     * @brief pre-registration frames time check and synchronization.
     * It modifies the frames by adding corrected timestamp field. Use the
     * reset() method to remove the added corrected frame timestamps field.
     *
     * @param[in,out] frame_set A vector of LidarFrame objects to
     * be processed.
     *
     * Steps taken:
     *  • Compute overall frame timestamp range.
     *  • Check inter-sensor synchronization:
     *      - If sensors are out of sync, enable packet-offset mode.
     *  • Monotonicity check (ignoring zero timestamps):
     *      - For any frame whose timestamps go backwards, correct its
     * timestamps and enable packet-offset mode. • Packet-offset handling:
     *      - If offset mode is active, compute fallback timestamp offsets.
     *  • Finally: it detects frames that come out of order.
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
     * @param[in,out] frame_set A vector of LidarFrame objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void reset(core::FrameSet& frame_set);

    // TODO[UN]: made public since it is required by tests
    /**
     * @brief get frame ts ranges
     * @return vector of (start_ts, end_ts) pairs for each frame within the
     * frame set
     */
    OUSTER_API_FUNCTION
    std::vector<std::pair<int64_t, int64_t>>& last_frame_ts_range() {
        return last_frame_ts_range_;
    }

    /**
     * @brief Determines whether a set of lidar frames are pre-synchronized
     * based on their start timestamps. This function compares the earliest and
     * latest start timestamps from a range of frames and checks if their
     * difference is within the smallest frame duration available. If the time
     * difference is less than or equal to the minimum frame gap, the frames are
     * considered pre-synchronized.
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
     * @brief Checks if the timestamps in each lidar frame are strictly
     * increasing (ignoring any zeros) and its first non-zero timestamp is
     * greater than the previous frame’s end timestamp.
     *
     * @param[in] timestamps a vector representing lidar timestamps.
     * @param[in] last_frame_stop_ts The end timestamp of the previous frame.
     *
     * @return true if the input timestamps are monotonically increasing, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    static bool is_monotonically_increasing(
        Eigen::Ref<const core::LidarFrame::Header<uint64_t>> timestamps,
        int64_t last_frame_stop_ts);

    /**
     * @brief Corrects the timestamps of the input Lidar frame based on the
     * previous frame's timestamp range and the sensor's frame duration.
     *
     * @param[in,out] frame The LidarFrame object whose timestamps need to be
     * corrected.
     * @param[in] sensor_idx The index of the sensor associated with the frame.
     */
    OUSTER_API_FUNCTION
    void correct_frame_ts(core::LidarFrame& frame, size_t sensor_idx);

   private:
    // NOTE: these methods were made public since they are required by tests
    /**
     * @brief Calculates fallback timestamp offsets for a list of LidarFrame
     * instances.
     *
     * @param[in] frame_set List of LidarFrame instances (frame set) to process.
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
    // state variables
    bool use_packet_offset_;
    std::vector<int64_t> last_frame_id_;
    std::vector<std::pair<int64_t, int64_t>> last_frame_ts_range_;

    std::vector<nonstd::optional<core::LidarFrame::Header<uint64_t>>> original_frame_timestamps_;
    std::vector<nonstd::optional<core::LidarFrame::Header<uint64_t>>> original_imu_timestamps_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
