#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <string>
#include <vector>
//! [doc-stag-slam-dewarp-imports]
#include <ouster/core/lidar_frame.h>
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
core::MatrixX16R<double> flatten_pose(const core::LidarFrame& frame) {
    const auto pose_field = frame.body_to_world();
    const auto flattened = pose_field.reshape(frame.w, 16);
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

    auto slam_engine = mapping::SlamEngine::create(source.sensor_info(), slam_config);
    // Process frames
    for (auto frame_set : source) {
        slam_engine->update(frame_set);
        // Find the first valid frame index
        size_t frame_idx = 0;
        bool found = false;
        for (size_t i = 0; i < frame_set.size(); ++i) {
            if (frame_set[i]) {
                frame_idx = i;
                found = true;
                break; } }

        if (!found) {
            continue;}
        
        const auto& infos = source.sensor_info();
        if (frame_idx >= infos.size() || !infos[frame_idx]) {
            continue; }
        // Compute Cartesian coordinates (XYZ)
        auto xyzlut = core::XYZLut(*infos[frame_idx],
                                   /*use_extrinsics=*/true);
        auto& frame = *frame_set[frame_idx];
        const auto xyz = xyzlut(frame);
        // Dewarp the point cloud using the trajectory from SLAM
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
