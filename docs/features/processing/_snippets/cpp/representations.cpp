/*
 * Copyright (c) Ouster, Inc.
 * All rights reserved.
 *
 * Documentation snippets for the processing guide.
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

//! [doc-stag-xyzlut-imports]
#include "ouster/core/lidar_frame.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/xyzlut.h"
using namespace ouster::sdk;
//! [doc-etag-xyzlut-imports]

#include "ouster/sensor/sensor_frame_set_source.h"

namespace ouster {
namespace docs {

void project_with_metadata_extrinsics(const core::SensorInfo& info, const core::LidarFrame& frame) {
    // core::XYZLut(info, true) multiplies the lookup table by
    // info.sensor_to_body * info.lidar_to_sensor_transform before projection.
    //! [doc-stag-cpp-xyzlut-extrinsics]
    auto xyzlut = core::XYZLut(info, /*use_extrinsics=*/true);
    auto cloud_in_extrinsics_frame = xyzlut(frame);
    //! [doc-etag-cpp-xyzlut-extrinsics]
    (void)cloud_in_extrinsics_frame;  // suppress unused warning in snippet
}

// Helper: Extract a channel (0=x, 1=y, 2=z), make it contiguous, and destagger
core::img_t<double> destagger_channel(const core::PointCloudXYZd& cloud,
                                      const core::SensorInfo& info, int channel_idx) {
    const auto h = info.format.pixels_per_column;
    const auto w = static_cast<size_t>(info.format.columns_per_frame);
    const auto& shifts = info.format.pixel_shift_by_row;

    core::img_t<double> staggered(h, w);
    // Manually copy interleaved data to contiguous 2D image
    for (size_t u = 0; u < h; ++u) {
        for (size_t v = 0; v < w; ++v) {
            staggered(u, v) = cloud(u * w + v, channel_idx);
        }
    }
    return core::destagger<double>(staggered, shifts);
};

std::vector<Eigen::Vector3d> filter_points_per_constraints(
    const core::img_t<uint32_t>& range_destaggered, const core::img_t<double>& x,
    const core::img_t<double>& y, const core::img_t<double>& z, double min_range_mm,
    size_t col_limit) {
    const size_t h = range_destaggered.rows();
    std::vector<Eigen::Vector3d> filtered;
    // 1. Iterate through rows and columns (up to the azimuth limit)
    for (size_t row = 0; row < h; ++row) {
        for (size_t col = 0; col < col_limit; ++col) {
            // 2. Keep only points whose destaggered RANGE exceeds the
            // threshold.
            if (range_destaggered(row, col) > min_range_mm) {
                // 3. Add valid points to the list
                filtered.emplace_back(x(row, col), y(row, col), z(row, col));
            }
        }
    }
    return filtered;
}

// clang-format off
//! [doc-stag-filter-3d-full]
core::PointCloudXYZd filter_points_by_range_and_azimuth(
    const core::SensorInfo& info, const core::LidarFrame& frame,
    double range_min_m, double azimuth_fraction = 0.75) {
    //! [doc-stag-filter-3d-setup]
    // azimuth_fraction = 0.75
    // destagger RANGE so each column maps to a fixed azimuth.
    auto range_field = frame.field<uint32_t>(core::ChanField::RANGE);
    auto& shifts = info.format.pixel_shift_by_row;
    auto range_destaggered = core::destagger<uint32_t>(range_field, shifts);
    
    // obtain destaggered xyz representation
    auto xyzlut = core::XYZLut(info, true);
    auto cloud = xyzlut(frame);
    //! [doc-etag-filter-3d-setup]
    //! [doc-stag-filter-3d-destagger]
    // cloud shape is (H*W, 3)
    // We need to destagger each channel separately
    auto x_destaggered = destagger_channel(cloud, info, 0);
    auto y_destaggered = destagger_channel(cloud, info, 1);
    auto z_destaggered = destagger_channel(cloud, info, 2);
    //! [doc-etag-filter-3d-destagger]

    const auto w = static_cast<size_t>(info.format.columns_per_frame);

    //! [doc-stag-filter-3d-mask]
    auto min_range_mm = range_min_m * 1000.0;
    // 1. Create mask: 1.0 if range > min, else 0.0
    auto mask =
        (range_destaggered.cast<double>().array() > min_range_mm).cast<double>();

    // 2. Apply mask: Invalid points become (0, 0, 0)
    // Coefficient-wise multiplication zeros out invalid points
    auto x_masked = x_destaggered * mask;
    auto y_masked = y_destaggered * mask;
    auto z_masked = z_destaggered * mask;
    //! [doc-etag-filter-3d-mask]
    //! [doc-stag-filter-3d]
    // 3. Slicing: Limit to the first azimuth_fraction(front 3/4 by default).
    auto col_limit = static_cast<size_t>(w * azimuth_fraction);
    // .leftCols() is the Eigen equivalent to NumPy's [:, :col_limit]
    auto x_filtered = x_masked.leftCols(col_limit);
    auto y_filtered = y_masked.leftCols(col_limit);
    auto z_filtered = z_masked.leftCols(col_limit);
    //! [doc-etag-filter-3d]
    // Flatten the 2D arrays to 1D and assign to columns
    // We evaluate the block expression to ensure contiguous memory for mapping
    core::PointCloudXYZd filtered_points(x_filtered.size(), 3);
    filtered_points.col(0) = Eigen::Map<const Eigen::VectorXd>(x_filtered.eval().data(), x_filtered.size());
    filtered_points.col(1) = Eigen::Map<const Eigen::VectorXd>(y_filtered.eval().data(), y_filtered.size());
    filtered_points.col(2) = Eigen::Map<const Eigen::VectorXd>(z_filtered.eval().data(), z_filtered.size());

    return filtered_points;
}
//! [doc-etag-filter-3d-full]
// clang-format on

core::PointCloudXYZd filter_points_by_range_and_azimuth(const std::string& hostname,
                                                        int lidar_port = 7502,
                                                        double range_min_m = 2.0) {
    sensor::SensorFrameSetSource source(hostname,
                                        [&](auto& opts) { opts.lidar_port = lidar_port; });
    auto frame_pair = source.get_frame();
    const auto& info = *source.sensor_info()[frame_pair.first];
    const auto& frame = *frame_pair.second;

    return filter_points_by_range_and_azimuth(info, frame, range_min_m);
}

core::PointCloudXYZd apply_metadata_transform_xyzlut(const core::SensorInfo& info,
                                                     const core::LidarFrame& frame) {
    //! [doc-stag-xyzlut-metadata-transform]
    // Custom pose: flip Z/Y and translate up 20 m and 1.5 m along X.
    core::mat4d transform = core::mat4d::Identity();
    transform(2, 2) = -1.0;
    transform(1, 1) = -1.0;
    transform(2, 3) = 20000.0;  // millimetres
    transform(0, 3) = 1500.0;   // millimetres
    // Compose with the existing lidar_to_sensor transform.
    core::SensorInfo adjusted = info;
    core::mat4d combined = transform * info.lidar_to_sensor_transform;

    // Override the transform before building the LUT.
    adjusted.lidar_to_sensor_transform = combined;

    core::XYZLut lut_adjusted(adjusted, /*use_extrinsics=*/false);
    auto cloud_adjusted = lut_adjusted(frame);
    return cloud_adjusted;
    //! [doc-etag-xyzlut-metadata-transform]
}

}  // namespace docs
}  // namespace ouster
