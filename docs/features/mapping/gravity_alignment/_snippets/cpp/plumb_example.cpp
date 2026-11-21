#include <iostream>
#include <string>

#include "ouster/algorithm/plumb.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"

using namespace ouster::sdk;
using ouster::sdk::algorithm::plumb;
using ouster::sdk::algorithm::plumb_imu;
using ouster::sdk::algorithm::plumb_walls;

core::Matrix4dR plumb_example(const std::string& source_file) {
    //! [doc-stag-plumb]
    auto source = open_source(source_file);
    core::FrameSet frames = *source.begin();

    // IMU first, then the wall refinement seeded with it. Each stage logs
    // what it found; the result is empty when neither produced a correction.
    auto correction = plumb(frames);
    if (!correction) {
        std::cerr << "could not plumb the frame set\n";
        return core::Matrix4dR::Identity();
    }
    for (auto& info : source.sensor_info()) {
        info->sensor_to_body = *correction * info->sensor_to_body;
        // drop any XYZLut cached with the old extrinsics
        info->clear_cache();
    }
    //! [doc-etag-plumb]

    return *correction;
}

nonstd::optional<core::Matrix4dR> plumb_imu_example(const std::string& source_file) {
    //! [doc-stag-plumb-imu]
    auto source = open_source(source_file);
    core::FrameSet frames = *source.begin();

    // The accelerometer alone. No wall search, so it is cheap and works
    // where there is no vertical structure, but it needs usable IMU data.
    auto imu_correction = plumb_imu(frames);
    if (!imu_correction) {
        std::cerr << "no usable IMU data to plumb from\n";
        return imu_correction;
    }
    for (auto& info : source.sensor_info()) {
        info->sensor_to_body = *imu_correction * info->sensor_to_body;
        // drop any XYZLut cached with the old extrinsics
        info->clear_cache();
    }
    //! [doc-etag-plumb-imu]

    return imu_correction;
}

nonstd::optional<core::Matrix4dR> plumb_walls_example(const std::string& source_file) {
    //! [doc-stag-plumb-walls]
    auto source = open_source(source_file);
    core::FrameSet frames = *source.begin();

    // Identity means "refine the orientation the extrinsics already
    // carry". Pass an IMU-derived rotation to refine that instead.
    auto walls_correction = plumb_walls(frames, core::Matrix4dR::Identity());
    if (!walls_correction) {
        std::cerr << "wall refinement did not converge\n";
        return walls_correction;
    }
    for (auto& info : source.sensor_info()) {
        info->sensor_to_body = *walls_correction * info->sensor_to_body;
        // drop any XYZLut cached with the old extrinsics
        info->clear_cache();
    }
    //! [doc-etag-plumb-walls]

    return walls_correction;
}
