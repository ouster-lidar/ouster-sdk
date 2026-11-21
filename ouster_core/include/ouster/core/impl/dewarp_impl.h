#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "ouster/core/camera_info.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

// A method that provides a quick upper bound on the number of points that can
// be produced from a FrameSet
size_t max_number_of_valid_points(const FrameSet& frame_set);

// Implementation for single-frame dewarp. Works for any Frame that carries a
// uint32 RANGE field: LidarFrame frames carry one pose/timestamp/status entry
// per column (rolling-shutter dewarp), while single-pose frames such as
// CameraFrame carry exactly one pose/timestamp/status entry that is applied
// rigidly to every column. The only difference between the two is therefore
// the number of poses; the projection, range filtering and provenance are
// identical.
//
// Optionally fills per-point provenance vectors (col_idxs, timestamps_ns) when
// non-null; pass nullptr to skip. col_idxs stores the *pose* index (the column
// index for lidar, 0 for single-pose frames), so it can be fed straight back
// into Frame::get_column_pose(). timestamps_ns falls back to
// fallback_timestamp_ns when a pose's own timestamp is zero (used for
// global-shutter cameras whose per-frame timestamp may be unset).
template <typename T>
std::vector<Eigen::Vector3<T>> dewarp_impl(const Frame& frame, const XYZLutT<T>& xyzlut,
                                           double min_range, double max_range,
                                           std::vector<uint32_t>* col_idxs,
                                           std::vector<uint64_t>* timestamps_ns) {
    // Note[UN]: for future we can optimize this method even further by
    //           embedding the cartesian directly into this and allowing
    //           it to skip over invalid columns.
    if (!frame.has_field(ChanField::RANGE)) {
        return {};
    }
    auto range = frame.field<uint32_t>(ChanField::RANGE);

    const int height = static_cast<int>(range.rows());
    const int width = static_cast<int>(range.cols());
    if (height == 0 || width == 0) {
        return {};
    }
    // The LUT must describe the same pixel grid as the frame; otherwise the
    // projection below would read out of bounds. Skip mismatched/empty LUTs.
    if (xyzlut.w != static_cast<size_t>(width) || xyzlut.h != static_cast<size_t>(height)) {
        return {};
    }
    PointCloudXYZ<T> pts = xyzlut(range);

    uint32_t min_r = static_cast<uint32_t>(std::ceil(min_range * 1e3));
    uint32_t max_r = static_cast<uint32_t>(std::floor(max_range * 1e3));

    Eigen::Ref<const Frame::Header<uint32_t>> status = frame.status();
    Eigen::Ref<const Frame::Header<uint64_t>> timestamps = frame.timestamp();
    // Number of poses: w for a LidarFrame (one per column), 1 for a
    // single-pose frame such as a CameraFrame. status/timestamps/body_to_world
    // all share this length.
    const int num_poses = static_cast<int>(timestamps.size());
    if (num_poses == 0) {
        return {};
    }
    auto poses = Eigen::Map<const MatrixX16dR>(frame.body_to_world().get<double>(), num_poses, 16);

    std::vector<Eigen::Vector3<T>> dewarped_pts;
    dewarped_pts.reserve(static_cast<size_t>(height) * static_cast<size_t>(width));

    for (int x = 0; x < width; ++x) {
        // Single-pose frames apply pose/status/timestamp index 0 to every
        // column; per-column frames index by the column itself.
        const int pidx = (num_poses == 1) ? 0 : x;
        if (status[pidx] == 0) {
            continue;
        }
        Eigen::Map<const Matrix4dR> pose(poses.row(pidx).data());
        const Matrix3R<T> rotation = pose.topLeftCorner<3, 3>().cast<T>();
        const Eigen::Vector3<T> translation = pose.topRightCorner<3, 1>().cast<T>();
        uint64_t col_ts = timestamps[pidx];
        for (int y = 0; y < height; ++y) {
            uint32_t r = range(y, x);
            // A range of zero means "no return"; skip it for both lidar and
            // camera rather than emitting a spurious point at the origin.
            if (r != 0 && r >= min_r && r <= max_r) {
                Eigen::Vector3<T> pt = pts.row(y * width + x);
                pt = rotation * pt + translation;
                dewarped_pts.emplace_back(std::move(pt));
                if (col_idxs) {
                    col_idxs->push_back(static_cast<uint32_t>(pidx));
                }
                if (timestamps_ns) {
                    timestamps_ns->push_back(col_ts);
                }
            }
        }
    }

    return dewarped_pts;
}

// Implementation for multi-frame dewarp. Each frame is projected with the LUT
// cached on its own sensor_info(); frames without a RANGE field or without
// sensor info are skipped before the LUT is looked up, so their metadata need
// not be projectable. Camera frames whose LUT cannot be built (e.g. a model
// mutated to a non-positive focal length) are skipped too, so one bad camera
// does not abort SLAM or localization; LUT errors for other sensors propagate.
// Optionally fills per-point provenance vectors (frame_idxs, col_idxs,
// timestamps_ns) when non-null; pass nullptr to skip.
template <typename T>
std::vector<Eigen::Vector3<T>> dewarp_impl(const FrameSet& frame_set, double min_range,
                                           double max_range, std::vector<uint32_t>* frame_idxs,
                                           std::vector<uint32_t>* col_idxs,
                                           std::vector<uint64_t>* timestamps_ns) {
    // Both lidar and (RANGE-carrying) camera frames are dewarped through the
    // shared single-frame dewarp_impl above; lidar frames use their per-column
    // poses and camera frames their single rigid pose. Frames whose LUT does
    // not match their dimensions produce no points.
    const size_t upper_bound = max_number_of_valid_points(frame_set);
    std::vector<Eigen::Vector3<T>> total_pts;
    total_pts.reserve(upper_bound);
    if (frame_idxs) {
        frame_idxs->reserve(frame_idxs->size() + upper_bound);
    }
    if (col_idxs) {
        col_idxs->reserve(col_idxs->size() + upper_bound);
    }
    if (timestamps_ns) {
        timestamps_ns->reserve(timestamps_ns->size() + upper_bound);
    }

    for (size_t idx : frame_set.valid_indices()) {
        const auto& frame_ptr = frame_set[idx];
        if (!frame_ptr || !frame_ptr->has_field(ChanField::RANGE) || !frame_ptr->sensor_info()) {
            continue;
        }
        std::shared_ptr<const XYZLutT<T>> xyzlut;
        try {
            xyzlut = frame_ptr->sensor_info()->xyzlut<T>();
        } catch (const std::invalid_argument&) {
            if (!dynamic_cast<const CameraInfo*>(frame_ptr->sensor_info().get())) {
                throw;
            }
            continue;
        }
        const size_t before = total_pts.size();
        auto dewarped_pts =
            dewarp_impl<T>(*frame_ptr, *xyzlut, min_range, max_range, col_idxs, timestamps_ns);
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
