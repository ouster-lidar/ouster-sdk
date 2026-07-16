/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/object_util.h"

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "ouster/core/pose_util.h"

namespace ouster {
namespace sdk {
namespace core {

namespace {

bool column_valid(const LidarFrame& frame, int col) {
    return (frame.status()[col] & 1u) != 0;
}

}  // namespace

Pose pose_at_timestamp(const LidarFrame& frame, uint64_t lidar_ts) {
    const auto& timestamps = frame.timestamp();
    const auto poses = Eigen::Map<const MatrixX16dR>(
        // NOLINTNEXTLINE(cppcoreguidelines-narrowing-conversions)
        frame.body_to_world().get<double>(), frame.w, 16);

    int col_before = -1;
    for (int col = static_cast<int>(frame.w) - 1; col >= 0; --col) {
        if (!column_valid(frame, col)) {
            continue;
        }
        if (timestamps[col] <= lidar_ts) {
            col_before = col;
            break;
        }
    }

    int col_after = -1;
    for (int col = 0; col < static_cast<int>(frame.w); ++col) {
        if (!column_valid(frame, col)) {
            continue;
        }
        if (timestamps[col] >= lidar_ts) {
            col_after = col;
            break;
        }
    }

    if (col_before < 0 || col_after < 0) {
        throw std::invalid_argument(
            "pose_at_timestamp: lidar timestamp is out of range for valid "
            "columns");
    }

    const int64_t t_before = static_cast<int64_t>(timestamps[col_before]);
    const int64_t t_after = static_cast<int64_t>(timestamps[col_after]);
    Eigen::Map<const Matrix4dR> pose_before(poses.row(col_before).data());

    if (col_before == col_after || t_before == t_after) {
        return Pose(pose_before);
    }

    if (t_before > t_after) {
        throw std::invalid_argument(
            "pose_at_timestamp: bracketing valid columns have non-monotonic "
            "timestamps");
    }

    Eigen::Map<const Matrix4dR> pose_after(poses.row(col_after).data());

    const std::vector<int64_t> x_interp{static_cast<int64_t>(lidar_ts)};
    const auto results = interp_pose(x_interp, t_before, pose_before, t_after, pose_after);
    return Pose(results[0]);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
