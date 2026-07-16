#include "slam_util.h"

#include <ouster/core/impl/logging.h>
#include <ouster/core/object_util.h>
#include <ouster/core/pose_util.h>
#include <ouster/core/sensor_info.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <nonstd/optional.hpp>
#include <stdexcept>

namespace ChanField = ouster::sdk::core::ChanField;
using ouster::sdk::core::FrameSet;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::MatrixX16dR;
// Users can enable OpenMP within ouster_client by first enabling omp through
// the compiler and then adding '-DOUSTER_OMP' option to the DCMAKE_CXX_FLAGS
#ifdef OUSTER_OMP
#define __OUSTER_UTILIZE_OPENMP__
#endif

// get the number of threads through OpenMP
#ifdef __OUSTER_UTILIZE_OPENMP__
#include <omp.h>
#define MAX_NUM_THREADS (omp_get_max_threads())
#else
#define MAX_NUM_THREADS (1)
#endif

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

nonstd::optional<double> determine_voxel_size(const FrameSet& frame_set, double start_pct,
                                              double end_pct) {
    if (start_pct < 0.0 || start_pct > 1.0 || end_pct < 0.0 || end_pct > 1.0 ||
        start_pct >= end_pct) {
        throw std::invalid_argument(
            "start_pct and end_pct must be in the range [0, 1] and "
            "start_pct must be less than end_pct.");
    }

    std::vector<uint32_t> selected_ranges;

    for (const auto& frame : frame_set.valid_frames()) {
        if (!has_valid_columns(*frame)) {
            continue;  // no valid columns; skip rather than throw
        }
        auto range = frame->field<uint32_t>(ChanField::RANGE);
        const int start_col = frame->get_first_valid_column();
        const int stop_col = frame->get_last_valid_column();
        Eigen::Ref<const LidarFrame::Header<uint32_t>> status = frame->status();

        // Flatten valid range values into a vector
        std::vector<uint32_t> frame_range;
        frame_range.reserve((stop_col - start_col + 1) * range.rows());
        for (int x = start_col; x <= stop_col; ++x) {
            if (status[x] == 0) {
                continue;
            }
            for (int y = 0; y < range.rows(); ++y) {
                if (range(y, x) != 0) {
                    frame_range.push_back(range(y, x));
                }
            }
        }

        if (frame_range.empty()) {
            continue;
        }

        // Sort and select the percentile range
        std::sort(frame_range.begin(), frame_range.end());
        size_t num_ranges = frame_range.size();
        size_t start_idx = static_cast<size_t>(
            std::ceil(start_pct * 100.0 / 100.0 * static_cast<double>(num_ranges)));
        size_t end_idx = static_cast<size_t>(
            std::floor(end_pct * 100.0 / 100.0 * static_cast<double>(num_ranges)));
        if (end_idx >= num_ranges) {
            end_idx = num_ranges - 1;
        }

        for (size_t i = start_idx; i <= end_idx; ++i) {
            selected_ranges.push_back(frame_range[i]);
        }
    }

    if (selected_ranges.empty()) {
        return nonstd::nullopt;
    }

    // Compute mean and convert to meters, then divide by 46.0
    double sum = 0.0;
    for (auto val : selected_ranges) {
        sum += static_cast<double>(val);
    }
    double average = 0.001 * (sum / static_cast<double>(selected_ranges.size())) / 46.0;
    return 2.0 * average;
}

core::Matrix3dR make_ortho(const core::Matrix3dR& matrix) {
    // Perform SVD
    Eigen::JacobiSVD<core::Matrix3dR> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Calculate the rotation matrix R = U * V^T
    core::Matrix3dR matrix_u = svd.matrixU();
    core::Matrix3dR matrix_v = svd.matrixV();
    core::Matrix3dR rotation = matrix_u * matrix_v.transpose();
    // Check if the determinant is negative (indicating a reflection)
    if (rotation.determinant() < 0.0) {
        // Flip the sign of the column of U corresponding to the smallest
        // singular value
        matrix_u.col(2) *= -1.0;
        rotation = matrix_u * matrix_v.transpose();
    }

    return rotation;
}

core::Matrix4dR make_ortho(const core::Matrix4dR& matrix) {
    core::Matrix4dR result = matrix;
    core::Matrix3dR rotation = matrix.block<3, 3>(0, 0);
    result.block<3, 3>(0, 0) = make_ortho(rotation);
    return result;
}

void init_valid_column_poses(LidarFrame& frame, Eigen::Ref<const core::Matrix4dR> pose) {
    std::vector<int> valid_cols = get_valid_columns<uint32_t>(frame.status());
    for (int col : valid_cols) {
        frame.set_column_pose(col, pose);
    }
}

void init_valid_column_poses(FrameSet& frame_set, Eigen::Ref<const core::Matrix4dR> pose) {
    for (auto& frame : frame_set.valid_frames()) {
        init_valid_column_poses(*frame, pose);
    }
}

void transform_inplace(LidarFrame& frame, const core::Matrix4dR& transform) {
    Eigen::Ref<const LidarFrame::Header<uint32_t>> status = frame.status();
    auto poses = Eigen::Map<MatrixX16dR>(frame.body_to_world().get<double>(),
                                         static_cast<Eigen::Index>(frame.w), 16);
    for (size_t j = 0; j < frame.w; ++j) {
        if (status[static_cast<Eigen::Index>(j)] == 0) {
            continue;
        }
        auto pose = Eigen::Map<core::Matrix4dR>(poses.row(static_cast<Eigen::Index>(j)).data());
        pose = transform * pose;
    }
}

void transform_inplace(FrameSet& frame_set, const core::Matrix4dR& transform) {
    for (auto& frame : frame_set.valid_frames()) {
        transform_inplace(*frame, transform);
    }
}

void update_object_poses(LidarFrame& frame) {
    for (auto& object_map : frame.objects()) {
        for (auto& object : object_map.second) {
            try {
                object.body_to_world = core::pose_at_timestamp(frame, object.timestamp);
            } catch (const std::invalid_argument&) {
                // This frame doesn't bracket the timestamp, try the next one.
            }
        }
    }
}

void update_object_poses(FrameSet& frame_set) {
    // Per-frame objects interpolate directly from their owning frame's columns.
    for (auto& frame : frame_set.valid_frames()) {
        update_object_poses(*frame);
    }

    // FrameSet-level objects are not tied to a single frame, so resolve an
    // owning frame to interpolate against.
    for (auto& object_map : frame_set.objects()) {
        for (auto& object : object_map.second) {
            //    Use the first valid frame whose valid columns bracket
            //    the object's timestamp. If no frame brackets it, the object is
            //    left unchanged.
            for (auto& frame : frame_set.valid_frames()) {
                try {
                    object.body_to_world = core::pose_at_timestamp(*frame, object.timestamp);
                    break;
                } catch (const std::invalid_argument&) {
                    // This frame doesn't bracket the timestamp, try the next one.
                }
            }
        }
    }
}

std::vector<double> get_valid_timestamps(Eigen::Ref<const Eigen::ArrayX<uint64_t>> ts_field,
                                         const std::vector<int>& valid) {
    std::vector<double> timestamps;
    timestamps.reserve(valid.size());
    std::transform(valid.begin(), valid.end(), std::back_inserter(timestamps),
                   [&ts_field](int idx) { return static_cast<double>(ts_field(idx)) * 1e-9; });
    return timestamps;
};

// Returns the adjusted timestamp range for a FrameSet
std::pair<uint64_t, uint64_t> compute_frame_ts_range(const FrameSet& frame_set) {
    uint64_t frame_start_ts = std::numeric_limits<uint64_t>::max();
    uint64_t frame_stop_ts = std::numeric_limits<uint64_t>::min();

    for (size_t idx : frame_set.valid_indices()) {
        const auto& frame = frame_set[idx];
        if (!has_valid_columns(*frame)) {
            continue;  // no valid columns; skip rather than throw
        }
        Eigen::Ref<const LidarFrame::Header<uint64_t>> c_timestamps = frame->timestamp();
        uint64_t start_ts = c_timestamps[frame->get_first_valid_column()];
        uint64_t stop_ts = c_timestamps[frame->get_last_valid_column()];
        frame_start_ts = std::min(frame_start_ts, start_ts);
        frame_stop_ts = std::max(frame_stop_ts, stop_ts);
    }

    return {frame_start_ts, frame_stop_ts};
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
