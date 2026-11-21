/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Gravity alignment (plumbing) from IMU data and wall geometry
 *
 * What counts as a wall is WallSegEngine's business (impl/wall.h): it marks
 * the pixels that stand on a real one in a WALL field. This header is what
 * to do with them: plumb_walls() has the frame set segmented when it is not
 * already, and hands the marked points to the search below.
 */

#pragma once

#include "nonstd/optional.hpp"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
class FrameSet;
}  // namespace core

namespace algorithm {

/// Cell size in meters of the grid used to score wall footprint candidates.
constexpr double PLUMB_WALLS_GRID_RESOLUTION_M = 0.025;
/// Minimum pooled wall points required for a trustworthy refinement. With
/// fewer, the occupied-cell landscape the search ranks candidates on is too
/// sparse to trust, and a scene with no real wall can still clear it.
constexpr size_t PLUMB_WALLS_MIN_WALL_POINTS = 500;
/// Solutions at or beyond this angle are pinned to the search boundary and
/// untrustworthy (the coordinate descent reaches at most 25 + 2 degrees).
constexpr double PLUMB_SEARCH_EDGE_DEG = 26.5;
/// Below this contrast the objective is nearly flat and the scene carries
/// little plumb information.
constexpr double PLUMB_WALLS_LOW_CONTRAST = 1.05;
/// The contrast probes sample the footprint this many degrees off the found
/// minimum along the x and y axis directions.
constexpr double PLUMB_CONTRAST_PROBE_DEG = 3.0;
/// Surface normals within this many degrees of horizontal count as wall.
constexpr double PLUMB_WALLS_ANGLE_TOL_DEG = 20.0;

/**
 * Level the body frame from IMU data.
 *
 * Averages the valid accelerometer samples of the frame from the sensor
 * whose IMU reports the highest fraction of valid samples, breaking ties
 * by sample count and then lowest index; a single valid sample is enough
 * when nothing better exists (FW 3.2+ IMU profiles; legacy IMU packets are
 * not supported). Each sample is first derotated into the frame of the
 * first sample by integrating the gyro, so rotation during the frame does
 * not smear the average direction. The anchor sensor and the gravity
 * direction it measured are logged.
 *
 * Both ``LidarFrame`` and ``CameraFrame`` frames are considered: any frame
 * carrying the IMU fields (``IMU_ACC``/``IMU_GYRO``/``IMU_TIMESTAMP``/
 * ``IMU_STATUS``) participates. Each frame uses its info's ``sensor_to_body``
 * and ``imu_to_sensor_transform`` (both ``SensorInfo`` fields) to rotate the
 * estimate into the body frame. Both kinds take these from ``sensor_info``; a
 * camera's ``imu_to_sensor_transform`` defaults to identity (IMU coincident
 * with the camera frame) unless calibrated.
 *
 * @param[in] frames Frame set to sample; typically the first of a stream.
 *            Each frame's extrinsics rotate the estimate into the body frame.
 * @return Rotation-only correction to left-multiply onto the current
 *         orientation, taking the measured gravity direction to +Z; empty
 *         when no sensor has usable IMU data.
 */
OUSTER_API_FUNCTION
nonstd::optional<core::Matrix4dR> plumb_imu(const core::FrameSet& frames);

/**
 * Refine the gravity alignment of a frame set from its wall geometry.
 *
 * Wall points are the pixels WallSegEngine marks WALL: near-horizontal
 * normals on a continuous vertical surface large enough to be a wall, so
 * clutter never votes. A frame that already carries a WALL field is read as
 * it lies and never recomputed; the frames without one are segmented here
 * under @p gravity_rotation, in memory only. The set is never written to. The IMU is never
 * consulted; the existing extrinsic orientation plus @p gravity_rotation is the anchor the
 * refinement corrects. The pooled points are scored by projecting them
 * along z and counting occupied grid cells: a plumb wall collapses to a
 * thin line while a tilted one smears into a band, so the roll/pitch that
 * minimizes the count renders the walls plumb. The wall point count, the
 * solution and how sharp its minimum was are logged.
 *
 * Any frame carrying a RANGE image votes, lidar or camera; a color-only
 * camera has no geometry to stand a wall on and simply contributes nothing.
 *
 * @param[in] frames Frame set to sample; typically the first of a stream.
 *            Each frame's sensor_info extrinsics are assumed rigid and
 *            correct so the pooled cloud is coherent.
 * @param[in] gravity_rotation Rotation-only alignment applied before the
 *            wall mask, e.g. from plumb_imu(); identity to refine from the
 *            extrinsic orientation alone.
 * @return Rotation-only correction to left-multiply onto the current
 *         orientation; empty when there are no usable frames, fewer than
 *         PLUMB_WALLS_MIN_WALL_POINTS pixels stand on a wall, or the solution
 *         pinned at the search boundary.
 */
OUSTER_API_FUNCTION
nonstd::optional<core::Matrix4dR> plumb_walls(
    const core::FrameSet& frames,
    const core::Matrix4dR& gravity_rotation = core::Matrix4dR::Identity());

/**
 * Plumb a frame set: level from the IMU, then refine from wall geometry.
 *
 * plumb_imu() followed by plumb_walls() seeded with the IMU levelling.
 * Without usable IMU data the wall refinement runs from the current
 * orientation alone, which assumes it is already within the wall search
 * range of level; without a trustworthy wall solution the IMU levelling
 * stands. Each outcome is logged.
 *
 * @param[in] frames Frame set to sample; see plumb_imu() and plumb_walls().
 * @param[in] use_imu Run the IMU stage.
 * @param[in] use_walls Run the wall stage.
 * @return Rotation-only correction to left-multiply onto the current
 *         orientation; empty when no requested stage produced a correction.
 */
OUSTER_API_FUNCTION
nonstd::optional<core::Matrix4dR> plumb(const core::FrameSet& frames, bool use_imu = true,
                                        bool use_walls = true);

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
