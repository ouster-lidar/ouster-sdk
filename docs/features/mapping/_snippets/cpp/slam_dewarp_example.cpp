#pragma once

#include <ouster/core/chanfield.h>

#include <Eigen/Core>
#include <algorithm>
#include <string>
#include <vector>
//! [doc-stag-slam-dewarp-imports]
#include <ouster/core/frame.h>
#include <ouster/core/frame_set.h>
#include <ouster/core/open_source.h>
#include <ouster/core/pose_util.h>
#include <ouster/core/xyzlut.h>
#include <ouster/mapping/slam_engine.h>

#include "ouster/osf/osf_frame_set_source.h"
using namespace ouster::sdk;
//! [doc-etag-slam-dewarp-imports]

namespace ouster {
namespace sdk {
namespace docs {

using namespace ouster::sdk;

namespace {
core::MatrixX16R<double> flatten_pose(const core::Frame& frame) {
    const auto pose_field = frame.body_to_world();
    // Number of poses: one per column for a LidarFrame, a single pose for a
    // CameraFrame. Use the pose field's own leading dimension so both work.
    const auto num_poses = pose_field.shape()[0];
    const auto flattened = pose_field.reshape(num_poses, 16);
    Eigen::Map<const core::MatrixX16R<double>> poses_view(
        flattened.template get<double>(), flattened.shape()[0], flattened.shape()[1]);
    return core::MatrixX16R<double>(poses_view);
}
}  // namespace

// clang-format off
//! [doc-stag-slam-dewarp-cpp]
inline Eigen::MatrixXf slam_dewarp_once(const std::string& source_file) {
    auto source = open_source(source_file);
    auto slam_config = mapping::LIOSlamConfig{};
    slam_config.deskew_method = "auto";

    // sensor_info() carries both lidar and camera calibration; SlamEngine
    // partitions it internally so mixed lidar+camera sources just work.
    auto slam_engine = mapping::SlamEngine::create(source.sensor_info(), slam_config);
    // Process frames
    for (auto frame_set : source) {
        slam_engine->update(frame_set);
        // Find the first valid RANGE-carrying frame (lidar or camera).
        size_t frame_idx = 0;
        bool found = false;
        for (size_t i = 0; i < frame_set.size(); ++i) {
            if (frame_set[i] && frame_set[i]->has_field(core::ChanField::RANGE)) {
                frame_idx = i;
                found = true;
                break; } }

        if (!found) {
            continue;}

        // Compute Cartesian coordinates (XYZ) with the XYZLut cached on the
        // sensor info. The same XYZLut projects lidar and camera RANGE alike.
        auto& frame = *frame_set[frame_idx];
        const auto xyzlut = frame.sensor_info()->xyzlut<double>();
        const auto xyz = (*xyzlut)(frame);
        // Dewarp the point cloud using the trajectory from SLAM. A lidar frame
        // has one pose per column (rolling shutter); a camera frame has a
        // single pose applied rigidly. core::dewarp handles both.
        const auto poses = flatten_pose(frame);
        const auto dewarped = core::dewarp<double>(xyz, poses);
        return dewarped.template cast<float>();
    }
    //! [doc-etag-slam-dewarp-cpp]
    // clang-format on
    return {};
}

}  // namespace docs
}  // namespace sdk
}  // namespace ouster
