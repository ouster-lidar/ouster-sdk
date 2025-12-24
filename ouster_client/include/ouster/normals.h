/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Normal values computation for LidarScan
 */

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <utility>

#include "ouster/typedefs.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/// Default target neighbour distance in meters (25 mm).
constexpr double DEFAULT_TARGET_DISTANCE_METER = 0.025;
/// Default minimum incidence angle (1 deg, ~0.01745 rad) used for AOI gating.
constexpr double DEFAULT_MIN_ANGLE_INCIDENCE_RAD = 1 * M_PI / 180.0;

/**
 * Compute normal values for a single return of a destaggered point cloud.
 *
 * xyz and sensor_origins_xyz must all be expressed in the same coordinate
 * frame (commonly world) so normals stay consistent frame to frame.
 *
 * @param[in] xyz Destaggered XYZ coordinates for the return (H, W, 3).
 * @param[in] range Destaggered range image for the return (H, W).
 * @param[in] sensor_origins_xyz Per-column sensor origins in the same frame as
 *            the points (W, 3). Used to derive per-beam direction vectors.
 *            - World-frame xyz: sensor_origins_xyz can be computed with
 *              an Eigen matrix of shape (W, 3), e.g.:
 *              ``Eigen::MatrixXd sensor_origins(W, 3);`` then
 *              ``for (int c = 0; c < scan.w; ++c)`` set
 *              ``sensor_origins.row(c) = (scan.get_column_pose(c) *
 *              scan.sensor_info->extrinsic).block<3, 1>(0, 3).transpose();``.
 *            - Sensor-frame xyz: pass zeros with shape (W, 3)
 *              (e.g. ``Eigen::MatrixXd::Zero(W, 3)``).
 * @param[in] pixel_search_range Axial pixel radius used to find neighbouring
 *            points (default: 1 px).
 * @param[in] min_angle_of_incidence_rad Minimum allowable incidence angle
 *            between a beam and surface (default: 1 deg, ~0.01745 rad).
 * @param[in] target_distance_m Target neighbour distance used when selecting
 *            candidate points (default: 0.025 m).
 *
 * @throws std::runtime_error if xyz shape is not (H * W, 3).
 * @throws std::runtime_error if sensor_origins_xyz width does not match W.
 * @throws std::runtime_error if min_angle_of_incidence_rad or
 *         target_distance_m is non-positive.
 * @return Flattened row-major matrix (H * W, 3) storing normal vectors.
 */
OUSTER_API_FUNCTION
MatrixX3dR normals(
    const Eigen::Ref<const PointCloudXYZd> xyz,
    const Eigen::Ref<const img_t<uint32_t>> range,
    const Eigen::Ref<const MatrixX3dR> sensor_origins_xyz,
    size_t pixel_search_range = 1,
    double min_angle_of_incidence_rad = DEFAULT_MIN_ANGLE_INCIDENCE_RAD,
    double target_distance_m = DEFAULT_TARGET_DISTANCE_METER);

/**
 * Compute normal values for both returns of a destaggered point cloud.
 *
 * xyz, xyz2 and sensor_origins_xyz must all be expressed in the same
 * coordinate frame (commonly world) so normals stay consistent frame to frame.
 *
 * @param[in] xyz Destaggered XYZ coordinates for the first return (H, W, 3).
 * @param[in] range Destaggered range image for the first return (H, W).
 * @param[in] xyz2 Destaggered XYZ coordinates for the second return (H, W, 3).
 * @param[in] range2 Destaggered range image for the second return (H, W).
 * @param[in] sensor_origins_xyz Per-column sensor origins in the same frame as
 *            the points (W, 3). Used to derive per-beam direction vectors.
 *            - World-frame xyz: sensor_origins_xyz can be computed with
 *              an Eigen matrix of shape (W, 3), e.g.:
 *              ``Eigen::MatrixXd sensor_origins(W, 3);`` then
 *              ``for (int c = 0; c < scan.w; ++c)`` set
 *              ``sensor_origins.row(c) = (scan.get_column_pose(c) *
 *              scan.sensor_info->extrinsic).block<3, 1>(0, 3).transpose();``.
 *            - Sensor-frame xyz: pass zeros with shape (W, 3)
 *              (e.g. ``Eigen::MatrixXd::Zero(W, 3)``).
 * @param[in] pixel_search_range Axial pixel radius used to find neighbouring
 *            points (default: 1 px).
 * @param[in] min_angle_of_incidence_rad Minimum allowable incidence angle
 *            between a beam and surface (default: 1 deg, ~0.01745 rad).
 * @param[in] target_distance_m Target neighbour distance used when selecting
 *            candidate points (default: 0.025 m).
 *
 * @throws std::runtime_error if xyz or xyz2 shape is not (H * W, 3).
 * @throws std::runtime_error if range2 dimensions differ from range.
 * @throws std::runtime_error if sensor_origins_xyz width does not match W.
 * @return Pair of flattened row-major matrices (first_return_normals,
 * second_return_normals).
 */
OUSTER_API_FUNCTION
std::pair<MatrixX3dR, MatrixX3dR> normals(
    const Eigen::Ref<const PointCloudXYZd> xyz,
    const Eigen::Ref<const img_t<uint32_t>> range,
    const Eigen::Ref<const PointCloudXYZd> xyz2,
    const Eigen::Ref<const img_t<uint32_t>> range2,
    const Eigen::Ref<const MatrixX3dR> sensor_origins_xyz,
    size_t pixel_search_range = 1,
    double min_angle_of_incidence_rad = DEFAULT_MIN_ANGLE_INCIDENCE_RAD,
    double target_distance_m = DEFAULT_TARGET_DISTANCE_METER);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
