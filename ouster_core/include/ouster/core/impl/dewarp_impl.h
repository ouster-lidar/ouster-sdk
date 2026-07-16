#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

// A method that provides a quick upper bound on the number of points that can
// be produced from a FrameSet
size_t max_number_of_valid_points(const FrameSet& frame_set);

// Implementation for single-frame dewarp. Optionally fills per-point provenance
// vectors (col_idxs, timestamps_ns) when non-null; pass nullptr to skip.
template <typename T>
std::vector<Eigen::Vector3<T>> dewarp_impl(const LidarFrame& lidar_frame, const XYZLutT<T>& xyzlut,
                                           double min_range, double max_range,
                                           std::vector<uint32_t>* col_idxs,
                                           std::vector<uint64_t>* timestamps_ns) {
    // Note[UN]: for future we can optimize this method even further by
    //           embedding the cartesian directly into this and allowing
    //           it to skip over invalid columns.
    auto range = lidar_frame.field<uint32_t>(ChanField::RANGE);
    PointCloudXYZ<T> pts = xyzlut(range);

    uint32_t min_r = static_cast<uint32_t>(std::ceil(min_range * 1e3));
    uint32_t max_r = static_cast<uint32_t>(std::floor(max_range * 1e3));

    const int height = range.rows();
    const int width = range.cols();
    int start_col = 0;
    int stop_col = 0;
    try {
        start_col = lidar_frame.get_first_valid_column();
        stop_col = lidar_frame.get_last_valid_column();
    } catch (const std::exception&) {
        return {};
    }
    if (stop_col < start_col) {
        return {};
    }
    std::vector<Eigen::Vector3<T>> dewarped_pts;
    dewarped_pts.reserve(height * (stop_col - start_col + 1));

    auto poses =
        Eigen::Map<const MatrixX16dR>(lidar_frame.body_to_world().get<double>(), lidar_frame.w, 16);
    Eigen::Ref<const LidarFrame::Header<uint32_t>> status = lidar_frame.status();
    Eigen::Ref<const LidarFrame::Header<uint64_t>> timestamps = lidar_frame.timestamp();

    for (int x = start_col; x <= stop_col; ++x) {
        if (status[x] == 0) {
            continue;
        }
        Eigen::Map<const Matrix4dR> pose(poses.row(x).data());
        const Matrix3R<T> rotation = pose.topLeftCorner<3, 3>().cast<T>();
        const Eigen::Vector3<T> translation = pose.topRightCorner<3, 1>().cast<T>();
        const uint64_t col_ts = timestamps[x];
        for (int y = 0; y < height; ++y) {
            uint32_t r = range(y, x);
            if (r >= min_r && r <= max_r) {
                Eigen::Vector3<T> pt = pts.row(y * width + x);
                pt = rotation * pt + translation;
                dewarped_pts.emplace_back(std::move(pt));
                if (col_idxs) col_idxs->push_back(static_cast<uint32_t>(x));
                if (timestamps_ns) timestamps_ns->push_back(col_ts);
            }
        }
    }

    return dewarped_pts;
}

// Implementation for multi-frame dewarp. Optionally fills per-point provenance
// vectors (frame_idxs, col_idxs, timestamps_ns) when non-null; pass nullptr to
// skip.
template <typename T>
std::vector<Eigen::Vector3<T>> dewarp_impl(const FrameSet& frame_set,
                                           const std::vector<XYZLutT<T>>& xyzluts, double min_range,
                                           double max_range, std::vector<uint32_t>* frame_idxs,
                                           std::vector<uint32_t>* col_idxs,
                                           std::vector<uint64_t>* timestamps_ns) {
    assert(frame_set.size() == xyzluts.size() &&
           "Number of frames and number of XYZLuts must be the same");
    const size_t upper_bound = max_number_of_valid_points(frame_set);
    std::vector<Eigen::Vector3<T>> total_pts;
    total_pts.reserve(upper_bound);
    if (frame_idxs) frame_idxs->reserve(frame_idxs->size() + upper_bound);
    if (col_idxs) col_idxs->reserve(col_idxs->size() + upper_bound);
    if (timestamps_ns) timestamps_ns->reserve(timestamps_ns->size() + upper_bound);

    for (size_t idx : frame_set.valid_indices()) {
        const auto& frame = *frame_set[idx];
        const size_t before = total_pts.size();
        auto dewarped_pts =
            dewarp_impl<T>(frame, xyzluts[idx], min_range, max_range, col_idxs, timestamps_ns);
        total_pts.insert(total_pts.end(), std::make_move_iterator(dewarped_pts.begin()),
                         std::make_move_iterator(dewarped_pts.end()));
        if (frame_idxs) {
            frame_idxs->insert(frame_idxs->end(), total_pts.size() - before,
                               static_cast<uint32_t>(idx));
        }
    }

    return total_pts;
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
