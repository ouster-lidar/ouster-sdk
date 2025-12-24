#include "ouster/active_time_correction.h"

#include <ouster/impl/logging.h>

#include "slam_util.h"

using ouster::sdk::core::logger;
using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace mapping {

namespace {

/*
 * Returns a list of (first_valid_column_timestamp,
 * last_valid_column_timestamp) for each scan. If a scan is None, returns
 * (-1, -1) for that position.
 */
std::vector<std::pair<int64_t, int64_t>> get_frame_ts_range(
    const LidarScanSet& scans) {
    std::vector<std::pair<int64_t, int64_t>> ranges(scans.size(), {-1, -1});
    for (size_t idx : scans.valid_indices()) {
        const auto& scan = *scans[idx];
        ranges[idx] = {
            static_cast<int64_t>(scan.get_first_valid_column_timestamp()),
            static_cast<int64_t>(scan.get_last_valid_column_timestamp())};
    }

    return ranges;
}

}  // namespace

ActiveTimeCorrection::ActiveTimeCorrection(
    const std::vector<std::shared_ptr<SensorInfo>>& infos)
    : frame_durations_(infos.size()),
      max_frame_id_(infos.size()),
      use_packet_offset_(false),
      last_frame_id_(infos.size(), -1),
      last_frame_ts_range_(infos.size(), {-1, -1}) {
    for (size_t i = 0; i < infos.size(); ++i) {
        frame_durations_[i] = 1e9 / infos[i]->format.fps;
        max_frame_id_[i] = PacketFormat(*infos[i]).max_frame_id;
    }
}

void ActiveTimeCorrection::update(LidarScanSet& scans) {
    auto frame_ts_range = get_frame_ts_range(scans);

    // Sensor sync check. fall back to packet timestamp offsets if sensors
    // are not synchronized.
    bool is_synced = check_sensors_synchronization(frame_ts_range);
    if (!is_synced && !use_packet_offset_) {
        use_packet_offset_ = true;
        logger().warn(
            "Sensors appear unsynchronized. Using estimated clock offsets, "
            "results may be affected");
    }

    // Monotonicity check correct any scans timestamp go backwards. If so,
    // correct its timestamps
    bool all_monotonic = true;
    original_scan_timestamps_.resize(scans.size(), nonstd::nullopt);
    original_imu_timestamps_.resize(scans.size(), nonstd::nullopt);
    for (size_t idx : scans.valid_indices()) {
        auto& scan = *scans[idx];
        Eigen::Ref<const LidarScan::Header<uint64_t>> timestamps =
            scan.timestamp();
        if (!is_monotonically_increasing(timestamps,
                                         last_frame_ts_range_[idx].second)) {
            // Save original timestamps for non-monotonic sensors and restore
            // them after registration.
            original_scan_timestamps_[idx] = timestamps;
            correct_scan_ts(scan, idx);
            all_monotonic = false;
        }

        // TODO[UN]: consider checking imu timestamps monotonicity as well
    }

    if (!all_monotonic && !use_packet_offset_) {
        use_packet_offset_ = true;
        logger().warn(
            "Lidarscan timestamp are not monotonically increasing. "
            "Using estimated clock offsets, results may be "
            "affected.");
    }

    // If in packet-offset mode, compute offsets
    std::vector<int64_t> scans_ts_offsets(scans.size(), 0);
    if (use_packet_offset_) {
        scans_ts_offsets = calculate_fallback_ts_offset(scans);

        for (size_t idx : scans.valid_indices()) {
            auto& scan = *scans[idx];

            Eigen::Ref<LidarScan::Header<uint64_t>> scan_ts = scan.timestamp();
            if (original_scan_timestamps_[idx] == nonstd::nullopt) {
                // cache original timestamps for later restore
                original_scan_timestamps_[idx] = scan_ts;
            }

            Eigen::Ref<const LidarScan::Header<uint32_t>> scan_status =
                scan.status();
            for (Eigen::Index col = 0; col < scan_ts.size(); ++col) {
                if ((scan_status(col) & 0x01) != 0) {
                    scan_ts[col] = static_cast<uint64_t>(
                        static_cast<int64_t>(scan_ts[col]) +
                        scans_ts_offsets[idx]);
                }
            }

            // if has imu timestamps, also apply offsets
            if (scan.has_field(ChanField::IMU_TIMESTAMP)) {
                Eigen::Ref<LidarScan::Header<uint64_t>> imu_ts =
                    scan.field(ChanField::IMU_TIMESTAMP);
                if (original_imu_timestamps_[idx] == nonstd::nullopt) {
                    // cache original imu timestamps for later restore
                    original_imu_timestamps_[idx] = imu_ts;
                }
                Eigen::Ref<const LidarScan::Header<uint16_t>> imu_status =
                    scan.field(ChanField::IMU_STATUS);
                for (Eigen::Index col = 0; col < imu_ts.size(); ++col) {
                    if ((imu_status(col) & 0x01) != 0) {
                        imu_ts[col] = static_cast<uint64_t>(
                            static_cast<int64_t>(imu_ts[col]) +
                            scans_ts_offsets[idx]);
                    }
                }
            }
        }
    }

    last_frame_ts_range_ = frame_ts_range;

    detect_out_of_order_scans(scans);
}

void ActiveTimeCorrection::reset(LidarScanSet& scans) {
    // restore only those timestamps we cached
    for (size_t idx : scans.valid_indices()) {
        auto& scan = *scans[idx];
        auto& scan_ts = original_scan_timestamps_[idx];
        if (scan_ts.has_value()) {
            scan.timestamp() = scan_ts.value();
        }

        auto& imu_ts = original_imu_timestamps_[idx];
        if (imu_ts.has_value()) {
            Eigen::Ref<LidarScan::Header<uint64_t>> imu_ts_dst =
                scan.field(ChanField::IMU_TIMESTAMP);
            imu_ts_dst = imu_ts.value();
        }
    }
    original_scan_timestamps_.clear();
    original_imu_timestamps_.clear();
}

// NOTE[UN]: Hao made a valid point that the applies to the entire flow of
// ActiveTimeCorrection in which it doesn't gracefully handle the case when
// one of the scans in a scan set is missing. basically get_frame_ts_range will
// return -1,-1 for that scan and then the synchronization check will likely
// fail because of that. Even if the rest of scans are actually in sync.
bool ActiveTimeCorrection::check_sensors_synchronization(
    const std::vector<std::pair<int64_t, int64_t>>& frame_ts_range) const {
    int64_t min_start_ts = std::numeric_limits<int64_t>::max();
    int64_t max_start_ts = std::numeric_limits<int64_t>::min();
    for (const auto& ts_pair : frame_ts_range) {
        if (ts_pair.first < min_start_ts) {
            min_start_ts = ts_pair.first;
        }
        if (ts_pair.first > max_start_ts) {
            max_start_ts = ts_pair.first;
        }
    }
    double min_frame_gap =
        *std::min_element(frame_durations_.begin(), frame_durations_.end());
    return static_cast<double>(max_start_ts - min_start_ts) <= min_frame_gap;
}

bool ActiveTimeCorrection::is_monotonically_increasing(
    Eigen::Ref<const LidarScan::Header<uint64_t>> timestamps,
    int64_t last_frame_stop_ts) {
    // get the valid non-zero timestamps
    std::vector<int64_t> filtered_ts;
    std::copy_if(timestamps.data(), timestamps.data() + timestamps.size(),
                 std::back_inserter(filtered_ts),
                 [](int64_t timestamp) { return timestamp != 0; });

    // 0 or 1 non-zero timestamp, return true
    if (filtered_ts.size() <= 1) {
        return true;
    }

    // only check inter-frame ordering if we actually have a previous
    // end
    if (last_frame_stop_ts >= 0 && filtered_ts[0] <= last_frame_stop_ts) {
        return false;
    }

    // check if timestamps are strictly increasing
    return std::adjacent_find(filtered_ts.begin(), filtered_ts.end(),
                              [](int64_t a, int64_t b) { return b <= a; }) ==
           filtered_ts.end();
}

// NOTE[UN]: This method is wrong on multiple fronts:
// 1. It uses the frame_durations when computing the new start but it applies
// to the first valid column which could be non-zero due to missing columns. in
// this this will cause problems for all subsequent frames (if they didn't have
// missing columns) as the next frame's start will be affected.
// 2. Another issue is that we don't know which was the actual column
// corresponding to the last frame's end timestamp. Say this column corresponded
// to the middle of of the last frame, then using the full frame duration to
// compute the new start of the current frame will cause the
// monotonically_increase intra frame to likely fail over and over whenever we
// have some missing columns here and there.
// TODO[UN]: fix the above issues.
void ActiveTimeCorrection::correct_scan_ts(LidarScan& scan, size_t sensor_idx) {
    int start_col = scan.get_first_valid_column();
    int stop_col = scan.get_last_valid_column();
    int64_t last_start_ts = last_frame_ts_range_[sensor_idx].first;
    int64_t new_start_ts =
        last_start_ts == -1
            ? scan.timestamp()[start_col]
            : last_start_ts +
                  static_cast<int64_t>(frame_durations_[sensor_idx]);

    double column_duration =
        frame_durations_[sensor_idx] / static_cast<double>(scan.w);

    Eigen::Ref<LidarScan::Header<uint64_t>> timestamps = scan.timestamp();
    Eigen::Ref<const LidarScan::Header<uint32_t>> status = scan.status();
    for (int col = start_col; col <= stop_col; ++col) {
        if ((status[col] & 0x01) == 0) {
            continue;
        }
        timestamps(col) = static_cast<uint64_t>(
            new_start_ts +
            static_cast<int64_t>((col - start_col) * column_duration));
    }
}

std::vector<int64_t> ActiveTimeCorrection::calculate_fallback_ts_offset(
    const LidarScanSet& scans) {
    std::vector<int64_t> offsets(scans.size(), 0);
    for (size_t idx : scans.valid_indices()) {
        const auto& scan = *scans[idx];
        int64_t first_pkt_ts =
            static_cast<int64_t>(scan.get_first_valid_packet_timestamp());
        int64_t frame_start_ts =
            static_cast<int64_t>(scan.get_first_valid_column_timestamp());
        offsets[idx] = first_pkt_ts - frame_start_ts;
    }

    return offsets;
}

// TODO: refactor the code such that we don't need to nullify the scan when
// it comes out-of-order scan.
// TODO: revise the code of detection jumps to handle frames with partially
// missing columns.
void ActiveTimeCorrection::detect_out_of_order_scans(LidarScanSet& scans) {
    for (size_t idx : scans.valid_indices()) {
        auto& scan = *scans[idx];
        if (last_frame_id_[idx] < 0) {
            last_frame_id_[idx] = scan.frame_id;
            continue;
        }

        // detect missing scans, ignoring anything similar to the frame_id
        // wrap around distance
        auto overflow_protection_threshold = -(max_frame_id_[idx] - 35);
        auto id_diff = scan.frame_id - last_frame_id_[idx];
        if (id_diff <= 0 && id_diff > overflow_protection_threshold) {
            logger().warn("Out-of-order scan detected; invalidating scan");
            scans[idx]->field<uint32_t>(ChanField::RANGE) =
                img_t<uint32_t>::Zero(scans[idx]->h, scans[idx]->w);
            continue;  // don't adopt the out-of-order scan's frame_id
        }

        last_frame_id_[idx] = scan.frame_id;
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
