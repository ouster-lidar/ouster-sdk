#include "ouster/mapping/active_time_correction.h"

#include <ouster/core/frame.h>
#include <ouster/core/frame_set.h>
#include <ouster/core/impl/logging.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/lidar_info.h>
#include <ouster/core/packet.h>
#include <ouster/core/sensor_info.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "nonstd/optional.hpp"
#include "slam_util.h"

using ouster::sdk::core::logger;
using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace mapping {

namespace {

/*
 * Returns a list of (first_valid_column_timestamp,
 * last_valid_column_timestamp) for each frame. If a frame is None, has no
 * valid columns, or carries only zero (unset) timestamps, returns (-1, -1)
 * for that position.
 */
std::vector<std::pair<int64_t, int64_t>> get_frame_ts_range(const core::FrameSet& frames) {
    std::vector<std::pair<int64_t, int64_t>> ranges(frames.size(), {-1, -1});
    for (size_t idx : frames.valid_indices()) {
        const auto& frame = frames[idx];
        if (!impl::has_valid_columns(*frame)) {
            continue;  // leave ranges[idx] at {-1, -1} for invalid frames
        }
        const auto start_ts =
            static_cast<int64_t>(frame->timestamp()[frame->get_first_valid_column()]);
        const auto stop_ts =
            static_cast<int64_t>(frame->timestamp()[frame->get_last_valid_column()]);
        // A CameraFrame's status defaults to valid even when its timestamp was
        // never set; zero is how downstream SLAM code marks "no timestamp".
        if (start_ts == 0 && stop_ts == 0) {
            continue;
        }
        ranges[idx] = {start_ts, stop_ts};
    }

    return ranges;
}

/*
 * Host receive time matching the frame's first valid timestamp, or nullopt if
 * the frame carries none.
 */
nonstd::optional<uint64_t> first_valid_receive_ts(const core::Frame& frame) {
    if (const auto* lidar = dynamic_cast<const core::LidarFrame*>(&frame)) {
        // The no-argument overload also folds in IMU and zone packets, whose
        // receive times do not correspond to the first valid lidar column.
        return lidar->get_first_valid_receive_timestamp(PacketType::Lidar);
    }
    // A CameraFrame has a single status entry but its receive_timestamp length
    // may vary, so the status-to-packet mapping in the Frame base accessors does
    // not apply; take the first populated entry instead.
    for (uint64_t receive_ts : frame.receive_timestamp()) {
        if (receive_ts != 0) {
            return receive_ts;
        }
    }
    return nonstd::nullopt;
}

}  // namespace

ActiveTimeCorrection::ActiveTimeCorrection(
    const std::vector<std::shared_ptr<core::SensorInfo>>& sensor_infos)
    : frame_durations_(sensor_infos.size(), 0.0),
      max_frame_id_(sensor_infos.size(), 0),
      use_packet_offset_(false),
      last_frame_id_(sensor_infos.size(), -1),
      last_frame_ts_range_(sensor_infos.size(), {-1, -1}) {
    // Per-sensor state is indexed by frameset position, so the list may mix
    // lidar and camera calibration in any order. A duration of 0 means the
    // sensor's frame rate is unknown.
    double min_lidar_duration = std::numeric_limits<double>::max();
    double min_other_duration = std::numeric_limits<double>::max();
    for (size_t i = 0; i < sensor_infos.size(); ++i) {
        const auto& info = sensor_infos[i];
        if (!info || info->fps() <= 0) {
            continue;
        }
        frame_durations_[i] = 1e9 / info->fps();
        if (auto lidar = std::dynamic_pointer_cast<LidarInfo>(info)) {
            max_frame_id_[i] = lidar->format.max_frame_id();
            min_lidar_duration = std::min(min_lidar_duration, frame_durations_[i]);
        } else {
            min_other_duration = std::min(min_other_duration, frame_durations_[i]);
        }
    }
    // A camera exposure may fall anywhere within a collated lidar sweep, so
    // when lidars are present only their period bounds the expected spread of
    // frame start times. A faster camera's period would be too strict.
    if (min_lidar_duration != std::numeric_limits<double>::max()) {
        sync_tolerance_ns_ = min_lidar_duration;
    } else if (min_other_duration != std::numeric_limits<double>::max()) {
        sync_tolerance_ns_ = min_other_duration;
    }
}

void ActiveTimeCorrection::update(FrameSet& frame_set) {
    auto frame_ts_range = get_frame_ts_range(frame_set);

    // Sensor sync check. fall back to packet timestamp offsets if sensors
    // are not synchronized.
    bool is_synced = check_sensors_synchronization(frame_ts_range);
    if (!is_synced && !use_packet_offset_) {
        use_packet_offset_ = true;
        logger().warn(
            "Sensors appear unsynchronized. Using estimated clock offsets, "
            "results may be affected");
    }

    // Monotonicity check: correct any frame timestamps that go backwards. If
    // so, correct its timestamps
    bool all_monotonic = true;
    original_frame_timestamps_.resize(frame_set.size(), nonstd::nullopt);
    original_imu_timestamps_.resize(frame_set.size(), nonstd::nullopt);
    for (size_t idx : frame_set.valid_indices()) {
        auto& frame = *frame_set[idx];
        Eigen::Ref<const Frame::Header<uint64_t>> timestamps = frame.timestamp();
        if (!is_monotonically_increasing(timestamps, last_frame_ts_range_[idx].second)) {
            // Save original timestamps for non-monotonic sensors and restore
            // them after registration.
            original_frame_timestamps_[idx] = timestamps;
            correct_frame_ts(frame, idx);
            all_monotonic = false;
        }

        // TODO[UN]: consider checking imu timestamps monotonicity as well
    }

    if (!all_monotonic && !use_packet_offset_) {
        use_packet_offset_ = true;
        logger().warn(
            "Frame timestamps are not monotonically increasing. "
            "Using estimated clock offsets, results may be "
            "affected.");
    }

    // If in packet-offset mode, compute offsets
    std::vector<int64_t> frame_ts_offsets(frame_set.size(), 0);
    if (use_packet_offset_) {
        frame_ts_offsets = calculate_fallback_ts_offset(frame_set);

        for (size_t idx : frame_set.valid_indices()) {
            auto& frame = *frame_set[idx];

            Eigen::Ref<Frame::Header<uint64_t>> frame_ts = frame.timestamp();
            if (original_frame_timestamps_[idx] == nonstd::nullopt) {
                // cache original timestamps for later restore
                original_frame_timestamps_[idx] = frame_ts;
            }

            Eigen::Ref<const Frame::Header<uint32_t>> frame_status = frame.status();
            for (Eigen::Index col = 0; col < frame_ts.size(); ++col) {
                if ((frame_status(col) & 0x01) != 0) {
                    frame_ts[col] = static_cast<uint64_t>(static_cast<int64_t>(frame_ts[col]) +
                                                          frame_ts_offsets[idx]);
                }
            }

            // if has imu timestamps, also apply offsets
            if (frame.has_field(ChanField::IMU_TIMESTAMP)) {
                Eigen::Ref<Frame::Header<uint64_t>> imu_ts = frame.field(ChanField::IMU_TIMESTAMP);
                if (original_imu_timestamps_[idx] == nonstd::nullopt) {
                    // cache original imu timestamps for later restore
                    original_imu_timestamps_[idx] = imu_ts;
                }
                Eigen::Ref<const Frame::Header<uint16_t>> imu_status =
                    frame.field(ChanField::IMU_STATUS);
                for (Eigen::Index col = 0; col < imu_ts.size(); ++col) {
                    if ((imu_status(col) & 0x01) != 0) {
                        imu_ts[col] = static_cast<uint64_t>(static_cast<int64_t>(imu_ts[col]) +
                                                            frame_ts_offsets[idx]);
                    }
                }
            }
        }
    }

    // Carry the last seen range forward across missing frames so that sensors
    // which skip frame sets (e.g. a camera slower than the lidar) still have
    // their next frame checked against the previous one.
    if (last_frame_ts_range_.size() < frame_ts_range.size()) {
        last_frame_ts_range_.resize(frame_ts_range.size(), {-1, -1});
    }
    for (size_t idx = 0; idx < frame_ts_range.size(); ++idx) {
        if (frame_ts_range[idx].first >= 0) {
            last_frame_ts_range_[idx] = frame_ts_range[idx];
        }
    }

    detect_out_of_order_frames(frame_set);
}

void ActiveTimeCorrection::reset(FrameSet& frame_set) {
    // restore only those timestamps we cached
    for (size_t idx : frame_set.valid_indices()) {
        auto& frame = *frame_set[idx];
        auto& cached_ts = original_frame_timestamps_[idx];
        if (cached_ts.has_value()) {
            frame.timestamp() = cached_ts.value();
        }

        auto& imu_ts = original_imu_timestamps_[idx];
        if (imu_ts.has_value()) {
            Eigen::Ref<Frame::Header<uint64_t>> imu_ts_dst = frame.field(ChanField::IMU_TIMESTAMP);
            imu_ts_dst = imu_ts.value();
        }
    }
    original_frame_timestamps_.clear();
    original_imu_timestamps_.clear();
}

bool ActiveTimeCorrection::check_sensors_synchronization(
    const std::vector<std::pair<int64_t, int64_t>>& frame_ts_range) const {
    if (sync_tolerance_ns_ <= 0.0) {
        return true;
    }
    int64_t min_start_ts = std::numeric_limits<int64_t>::max();
    int64_t max_start_ts = std::numeric_limits<int64_t>::min();
    size_t present = 0;
    for (const auto& ts_pair : frame_ts_range) {
        if (ts_pair.first < 0) {
            continue;  // missing frame; says nothing about clock agreement
        }
        min_start_ts = std::min(min_start_ts, ts_pair.first);
        max_start_ts = std::max(max_start_ts, ts_pair.first);
        ++present;
    }
    if (present <= 1) {
        return true;
    }
    return static_cast<double>(max_start_ts - min_start_ts) <= sync_tolerance_ns_;
}

bool ActiveTimeCorrection::is_monotonically_increasing(
    Eigen::Ref<const Frame::Header<uint64_t>> timestamps, int64_t last_frame_stop_ts) {
    // get the valid non-zero timestamps
    std::vector<int64_t> filtered_ts;
    std::copy_if(timestamps.data(), timestamps.data() + timestamps.size(),
                 std::back_inserter(filtered_ts), [](int64_t timestamp) { return timestamp != 0; });

    if (filtered_ts.empty()) {
        return true;
    }

    // only check inter-frame ordering if we actually have a previous end. This
    // must run even for a single timestamp: a CameraFrame only ever has one.
    if (last_frame_stop_ts >= 0 && filtered_ts[0] <= last_frame_stop_ts) {
        return false;
    }

    // check if timestamps are strictly increasing
    return std::adjacent_find(filtered_ts.begin(), filtered_ts.end(),
                              [](int64_t a, int64_t b) { return b <= a; }) == filtered_ts.end();
}

// NOTE[UN]: This method is wrong on multiple fronts:
// 1. It uses the frame_durations when computing the new start but it applies
// to the first valid column which could be non-zero due to missing columns. in
// this will cause problems for all subsequent frames (if they didn't have
// missing columns) as the next frame's start will be affected.
// 2. Another issue is that we don't know which was the actual column
// corresponding to the last frame's end timestamp. Say this column corresponded
// to the middle of the last frame, then using the full frame duration to
// compute the new start of the current frame will cause the
// monotonically_increase intra frame to likely fail over and over whenever we
// have some missing columns here and there.
// TODO[UN]: fix the above issues.
void ActiveTimeCorrection::correct_frame_ts(core::Frame& frame, size_t sensor_idx) {
    if (!impl::has_valid_columns(frame)) {
        return;  // nothing to correct
    }
    int64_t last_start_ts = last_frame_ts_range_[sensor_idx].first;
    double frame_duration = frame_durations_[sensor_idx];
    if (last_start_ts != -1 && frame_duration <= 0.0) {
        return;  // unknown frame rate; cannot extrapolate from the last frame
    }
    int start_col = frame.get_first_valid_column();
    int stop_col = frame.get_last_valid_column();
    int64_t new_start_ts = last_start_ts == -1
                               ? static_cast<int64_t>(frame.timestamp()[start_col])
                               : last_start_ts + static_cast<int64_t>(frame_duration);

    Eigen::Ref<Frame::Header<uint64_t>> timestamps = frame.timestamp();
    // timestamp() holds one entry per column for lidar but a single entry for
    // a camera, so Frame::w is not the timestamp count in general.
    double column_duration = frame_duration / static_cast<double>(timestamps.size());

    Eigen::Ref<const Frame::Header<uint32_t>> status = frame.status();
    for (int col = start_col; col <= stop_col; ++col) {
        if ((status[col] & 0x01) == 0) {
            continue;
        }
        timestamps(col) = static_cast<uint64_t>(
            new_start_ts + static_cast<int64_t>((col - start_col) * column_duration));
    }
}

std::vector<int64_t> ActiveTimeCorrection::calculate_fallback_ts_offset(
    const core::FrameSet& frames) {
    std::vector<int64_t> offsets(frames.size(), 0);
    for (size_t idx : frames.valid_indices()) {
        const auto& frame = *frames[idx];
        if (!impl::has_valid_columns(frame)) {
            continue;  // leave offset at 0 for frames with no valid columns
        }
        const auto first_receive_ts = first_valid_receive_ts(frame);
        if (!first_receive_ts) {
            continue;  // no host time to map onto; leave offset at 0
        }
        int64_t frame_start_ts =
            static_cast<int64_t>(frame.timestamp()[frame.get_first_valid_column()]);
        offsets[idx] = static_cast<int64_t>(*first_receive_ts) - frame_start_ts;
    }

    return offsets;
}

// TODO: refactor the code such that we don't need to nullify the frame when
// it comes out-of-order.
// TODO: revise the code of detection jumps to handle frames with partially
// missing columns.
void ActiveTimeCorrection::detect_out_of_order_frames(FrameSet& frame_set) {
    for (size_t idx : frame_set.valid_indices()) {
        // last_frame_id_/max_frame_id_ share the FrameSet index space (one entry
        // per sensor, in FrameSet order). Skip camera or other non-lidar frames:
        // they have no frame id (their ordering is covered by the timestamp
        // monotonicity check) and may not carry a RANGE field to invalidate.
        if (idx >= last_frame_id_.size()) {
            continue;
        }
        auto lidar = std::dynamic_pointer_cast<core::LidarFrame>(frame_set[idx]);
        if (!lidar) {
            continue;
        }
        auto& frame = *lidar;
        if (last_frame_id_[idx] < 0) {
            last_frame_id_[idx] = frame.frame_id;
            continue;
        }

        // detect missing frames, ignoring anything similar to the frame_id
        // wrap around distance
        auto overflow_protection_threshold = -(max_frame_id_[idx] - 35);
        auto id_diff = frame.frame_id - last_frame_id_[idx];
        if (id_diff <= 0 && id_diff > overflow_protection_threshold) {
            logger().warn("Out-of-order frame detected; invalidating frame");
            frame_set[idx]->field<uint32_t>(ChanField::RANGE) =
                img_t<uint32_t>::Zero(static_cast<Eigen::Index>(frame_set[idx]->h),
                                      static_cast<Eigen::Index>(frame_set[idx]->w));
            continue;  // don't adopt the out-of-order frame's frame_id
        }

        last_frame_id_[idx] = frame.frame_id;
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
