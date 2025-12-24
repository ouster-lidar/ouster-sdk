#pragma once

#include <ouster/lidar_scan.h>
#include <ouster/lidar_scan_set.h>
#include <ouster/types.h>
#include <ouster/visibility.h>

#include <memory>
#include <nonstd/optional.hpp>
#include <vector>

namespace ouster {
namespace sdk {
namespace mapping {

class ActiveTimeCorrection {
   public:
    ActiveTimeCorrection(
        const std::vector<std::shared_ptr<core::SensorInfo>>& infos);

    /**
     * @brief pre-registration scans time check, may modify scans sensor time.
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
     *
     * @return A vector of timestamp offsets for each sensor.
     */
    OUSTER_API_FUNCTION
    std::vector<int64_t> update(core::LidarScanSet& scans);

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
     */
    OUSTER_API_FUNCTION
    bool check_sensors_synchronization(
        const std::vector<std::pair<int64_t, int64_t>>& frame_ts_range) const;

    // TODO[UN]: make private (made public since it is required by tests)
    /**
     * @brief Checks if the timestamps in each lidar scan are strictly
     * increasing (ignoring any zeros) and its first non-zero timestamp is
     * greater than the previous frame’s end timestamp.
     *
     * @param[in] scan The lidar scan to check.
     * @param[in] sensor_idx The index of the sensor.
     *
     * @return true if the scan is monotonically increasing, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool check_monotonic_increase_ts(const core::LidarScan& scan,
                                     size_t sensor_idx) const;

    /**
     * @brief Corrects the timestamps of the input Lidar scan based on the
     * previous frame's timestamp range and the sensor's frame duration.
     */
    OUSTER_API_FUNCTION
    void correct_scan_ts(core::LidarScan& scan, size_t sensor_idx);

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
    std::vector<int64_t> calculate_fallback_ts_offset(
        const core::LidarScanSet& scans) const;

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
     *
     * @return The minimum frame ID difference for valid scans.
     */
    OUSTER_API_FUNCTION
    void detect_out_of_order_scans(core::LidarScanSet& scans);

   private:
    std::vector<double> frame_durations_;
    // state variables
    bool use_packet_offset_;
    std::vector<int64_t> last_frame_id_;
    std::vector<std::pair<int64_t, int64_t>> last_frame_ts_range_;
    std::vector<nonstd::optional<core::LidarScan::Header<uint64_t>>>
        orig_ts_cache_;
    std::vector<core::PacketFormat> formats_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
