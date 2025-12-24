#pragma once

#include <memory>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/lidar_scan_set.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class ActiveTimeCorrection
 * @brief Class to handle active time correction for LidarScans.
 *
 * This class provides methods to correct and reset timestamps in LidarScans
 * based on sensor information. It checks for synchronization between sensors,
 * ensures monotonicity of timestamps, and applies corrections as needed.
 *
 * Typical usage pattern:
 * ActiveTimeCorrection time_correction(infos);
 *
 * for (auto scans : scan_source) {
 *      time_correction.update(scans);  // to apply time correction
 *      slam_engine.update(scans);      // SLAM processing
 *      time_correction.reset(scans);   // to reset the timestamps
 * }
 */
class ActiveTimeCorrection {
   public:
    /**
     * @brief Construct an ActiveTimeCorrection object from sensor info
     * structures.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects, one
     * per sensor.
     */
    OUSTER_API_FUNCTION
    ActiveTimeCorrection(
        const std::vector<std::shared_ptr<core::SensorInfo>>& infos);

    /**
     * @brief pre-registration scans time check and synchronization.
     * It modifies the scans by adding corrected timestamp filed. Use the
     * reset() method to remove the added corrected scan timestamps field.
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed.
     *
     * Steps taken:
     *  • Compute overall frame timestamp range.
     *  • Check inter-sensor synchronization:
     *      - If sensors are out of sync, enable packet-offset mode.
     *  • Monotonicity check (ignoring zero timestamps):
     *      - For any scan whose timestamps go backwards, correct its timestamps
     *      and enable packet-offset mode.
     *  • Packet-offset handling:
     *      - If offset mode is active, compute fallback timestamp offsets.
     *  • Finally: it detects scans that come out of order.
     *
     * @remarks when an out of order scan is detected, its range field is
     * zeroed out to invalidate it. This behavior will be revised in the future
     * to avoid modifying the scan data.
     */
    OUSTER_API_FUNCTION
    void update(core::LidarScanSet& scans);

    /**
     * @brief reset scans sensor time post-registration
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed.
     */
    OUSTER_API_FUNCTION
    void reset(core::LidarScanSet& scans);

    // TODO[UN]: made public since it is required by tests
    /**
     * @brief get frame ts ranges
     * @return vector of (start_ts, end_ts) pairs for each scan within the scan
     * set
     */
    OUSTER_API_FUNCTION
    std::vector<std::pair<int64_t, int64_t>>& last_frame_ts_range() {
        return last_frame_ts_range_;
    }

    /**
     * @brief Determines whether a set of lidar scans are pre-synchronized based
     * on their start timestamps. This function compares the earliest and latest
     * start timestamps from a range of scans and checks if their difference is
     * within the smallest frame duration available. If the time difference is
     * less than or equal to the minimum frame gap, the scans are considered
     * pre-synchronized.
     *
     * @param[in] frame_ts_range A vector of (start_ts, end_ts) pairs
     * representing the time value range for each scan.
     *
     * @return true if the scans are pre-synchronized, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool check_sensors_synchronization(
        const std::vector<std::pair<int64_t, int64_t>>& frame_ts_range) const;

    /**
     * @brief Checks if the timestamps in each lidar scan are strictly
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
        Eigen::Ref<const core::LidarScan::Header<uint64_t>> timestamps,
        int64_t last_frame_stop_ts);

    /**
     * @brief Corrects the timestamps of the input Lidar scan based on the
     * previous frame's timestamp range and the sensor's frame duration.
     *
     * @param[in,out] scan The LidarScan object whose timestamps need to be
     * corrected.
     * @param[in] sensor_idx The index of the sensor associated with the scan.
     */
    OUSTER_API_FUNCTION
    void correct_scan_ts(core::LidarScan& scan, size_t sensor_idx);

   private:
    // NOTE: these methods were made public since they are required by tests
    /**
     * @brief Calculates fallback timestamp offsets for a list of LidarScan
     * instances.
     *
     * @param[in] scans List of LidarScan instances to process.
     *
     * This function computes, for each scan:
     *  • the packet timestamp offset relative to the earliest packet timestamp
     * across all non‐None scans, • then subtracts the scan’s first valid column
     * timestamp, • or returns None if the scan itself is None.
     *
     * @return A vector of timestamp offsets for each scan.
     */
    OUSTER_API_FUNCTION
    static std::vector<int64_t> calculate_fallback_ts_offset(
        const core::LidarScanSet& scans);

    /**
     * @brief This method detects if there are any lidarscan jumps (i.e a gap of
     * more than one frame between subsequent scans) or if there are any
     * out-of-order scans in the provided LidarScanSet. A jump happens
     * when there is a dropped frame or frames resulting in a gap in frame ids
     * for a specific sensor. It returns the minimum frame ID difference for
     * valid scans.
     *
     * @param[in,out] scans A vector of shared pointers to LidarScan objects to
     * be processed. if a scan is out of order, it will be nullified by setting
     * its range values to zero.
     */
    OUSTER_API_FUNCTION
    void detect_out_of_order_scans(core::LidarScanSet& scans);

    std::vector<double> frame_durations_;
    std::vector<int64_t> max_frame_id_;
    // state variables
    bool use_packet_offset_;
    std::vector<int64_t> last_frame_id_;
    std::vector<std::pair<int64_t, int64_t>> last_frame_ts_range_;

    std::vector<nonstd::optional<core::LidarScan::Header<uint64_t>>>
        original_scan_timestamps_;
    std::vector<nonstd::optional<core::LidarScan::Header<uint64_t>>>
        original_imu_timestamps_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
