/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Dense>
#include <exception>

#include "ouster/core/array_view.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object.h"
#include "ouster/core/visibility.h"
#include "ouster/core/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Interpolate the body-to-world pose at a lidar timestamp.
 *
 * Uses per-column poses from the frame, considering only columns with a valid
 * status word (bit 0 set).
 *
 * @param[in] frame Lidar frame providing column timestamps, status, and poses
 * @param[in] lidar_ts Lidar timestamp in nanoseconds
 *
 * @return Interpolated pose at the requested lidar timestamp
 *
 * @throws std::invalid_argument if the timestamp cannot be bracketed by valid
 * columns
 */
OUSTER_API_FUNCTION
Pose pose_at_timestamp(const LidarFrame& frame, uint64_t lidar_ts);

/**
 * This function restores instance id array based on object's bounding box.
 *
 * @param[in] object An object, such as produced by detection engine
 * @param[in] points A Eigen matrix of shape (N, 3) representing the 3D points.
 *                   Each row corresponds to a point in 3D space.
 * @param[out] instance_ids An Eigen matrix of shape (H, W) representing
 *                          instance id pixel field
 */
template <typename T>
void restore_instance_ids(const Object& object, const PointCloudXYZ<T>& points,
                          Eigen::Ref<img_t<uint32_t>> instance_ids) {
    if (points.rows() != instance_ids.size()) {
        throw std::invalid_argument(
            "Mismatch between point cloud size and "
            "instance id map size");
    }

    ArrayView1<uint32_t> flattened_ids{instance_ids.data(),
                                       {instance_ids.rows() * instance_ids.cols()}};

    Eigen::Array3<T> half_extents = (object.dimensions * 0.5).array().template cast<T>();

    Eigen::Matrix4<T> transform = object.object_to_body.to_matrix().inverse().template cast<T>();
    Eigen::Transform<T, 3, Eigen::Affine> affine{transform};

    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3<T> pt = points.row(i);
        Eigen::Vector3<T> pt_in_frame = affine * pt;

        if ((pt_in_frame.array().abs() < half_extents).all()) {
            flattened_ids(i) = object.id;
        }
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
