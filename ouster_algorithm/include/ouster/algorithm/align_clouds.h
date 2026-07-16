#pragma once

#include <vector>

#include "ouster/core/typedefs.h"

namespace ouster {
namespace sdk {
namespace core {
class LidarFrame;
class FrameSet;
}  // namespace core
}  // namespace sdk
}  // namespace ouster

namespace ouster {
namespace sdk {
namespace algorithm {

/**
 * @brief Align two point clouds using point-to-point ICP.
 *
 * Returns ``source_to_target_transform``: @p source_points are transformed by
 * the returned pose and @p target_points stay fixed as the reference cloud.
 *
 * Iteratively matches each transformed source point to its nearest target
 * point within @p max_corr_dist and estimates a rigid transform using robust
 * Huber weighting. Non-finite points are ignored.
 *
 * @param[in] source_points Source points transformed by the returned pose as
 *            ``N x 3`` XYZ rows.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``.
 * @param[in] max_corr_dist Maximum correspondence distance in meters. Must be
 *            finite and greater than zero.
 * @return Estimated ``source_to_target_transform``. Returns @p initial_guess if
 *         there are fewer than 20 usable points or correspondences.
 * @throws std::invalid_argument If @p max_corr_dist is not finite and
 *         positive.
 */
OUSTER_API_FUNCTION
core::Matrix4dR point_to_point_align(
    Eigen::Ref<const core::ArrayX3dR> source_points,
    Eigen::Ref<const core::ArrayX3dR> target_points,
    const core::Matrix4dR& initial_guess = core::Matrix4dR::Identity(),
    double max_corr_dist = 0.25);

/**
 * @brief Align two point clouds using point-to-plane ICP.
 *
 * Returns ``source_to_target_transform``: @p source_points are transformed by
 * the returned pose and @p target_points stay fixed as the reference cloud.
 *
 * Iteratively matches each transformed source point to its nearest target
 * point, rejects correspondences whose normals differ by more than
 * @p max_normal_angle_deg, and minimizes point-to-plane residuals with robust
 * Huber weighting. Non-finite points and normals are ignored, and usable
 * normals are normalized internally.
 *
 * @param[in] source_points Source points transformed by the returned pose as
 *            ``N x 3`` XYZ rows.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] source_normals Normals aligned with the source
 *            @p source_points.
 * @param[in] target_normals Normals aligned with the reference
 *            @p target_points.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``.
 * @param[in] max_corr_dist Maximum correspondence distance in meters. Must be
 *            finite and greater than zero.
 * @param[in] max_normal_angle_deg Maximum angle between corresponding normals
 *            in degrees. Must be finite and in ``[0, 180]``.
 * @return Estimated ``source_to_target_transform``. Returns @p initial_guess if
 *         there are fewer than 20 usable points or correspondences.
 * @throws std::invalid_argument If point and normal row counts differ, or an
 *         ICP threshold is outside its valid range.
 */
OUSTER_API_FUNCTION
core::Matrix4dR point_to_plane_align(
    Eigen::Ref<const core::ArrayX3dR> source_points,
    Eigen::Ref<const core::ArrayX3dR> target_points,
    Eigen::Ref<const core::ArrayX3dR> source_normals,
    Eigen::Ref<const core::ArrayX3dR> target_normals,
    const core::Matrix4dR& initial_guess = core::Matrix4dR::Identity(), double max_corr_dist = 0.25,
    double max_normal_angle_deg = 20.0);

/**
 * @brief Estimate frame-to-frame alignment from two high-level lidar frames.
 *
 * Returns ``source_to_target_transform``: points from @p source_frame are
 * transformed by the returned pose into the fixed @p target_frame frame.
 *
 * Dewarps and extracts features using per-column poses and
 * ``sensor_info.sensor_to_body``. The rotation component is expected to be
 * gravity-aligned; the full existing extrinsic, including translation, is used
 * as the initial prior. The caller is responsible for setting a gravity-aligned
 * extrinsic (e.g. computed from IMU data) before calling; the function does not
 * read IMU fields or perform any internal gravity estimation.
 *
 * Ground segmentation is applied to filter ground points from the XY
 * features used for initial yaw/translation matching. If the source frame
 * already carries a ``"GROUND"`` field (written by ``GroundSegEngine``), it is
 * reused directly; otherwise ground segmentation is computed during the call.
 *
 * @param[in] source_frame Source frame.
 * @param[in] target_frame Target frame.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``
 *            in the extrinsic-applied frame (the frame produced by
 *            ``xyzlut(extrinsics=True)``).
 * @return Estimated ``source_to_target_transform`` in the extrinsic-applied
 * frame.
 * @throws std::invalid_argument If a frame lacks a RANGE field or
 *         ``sensor_info``.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(const ouster::sdk::core::LidarFrame& source_frame,
                             const ouster::sdk::core::LidarFrame& target_frame,
                             const core::Matrix4dR& initial_guess = core::Matrix4dR::Identity());

/**
 * @brief Estimate frame-to-frame alignment and report confidence.
 *
 * @param[in] source_frame Source frame.
 * @param[in] target_frame Target frame.
 * @param[out] confidence Confidence score in ``[0, 1]`` for the estimated
 * pose.
 * @return Estimated ``source_to_target_transform`` in the extrinsic-applied
 * frame.
 * @throws std::invalid_argument If a frame lacks a RANGE field or
 *         ``sensor_info``.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(const ouster::sdk::core::LidarFrame& source_frame,
                             const ouster::sdk::core::LidarFrame& target_frame, double& confidence);

/**
 * @brief Estimate frame-to-frame alignment from an initial guess and report
 * confidence.
 *
 * @param[in] source_frame Source frame.
 * @param[in] target_frame Target frame.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``
 *            in the extrinsic-applied frame (the frame produced by
 *            ``xyzlut(extrinsics=True)``).
 * @param[out] confidence Confidence score in ``[0, 1]`` for the estimated
 * pose.
 * @return Estimated ``source_to_target_transform`` in the extrinsic-applied
 * frame.
 * @throws std::invalid_argument If a frame lacks a RANGE field or
 *         ``sensor_info``.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(const ouster::sdk::core::LidarFrame& source_frame,
                             const ouster::sdk::core::LidarFrame& target_frame,
                             const core::Matrix4dR& initial_guess, double& confidence);

/**
 * @brief Align every valid frame in a FrameSet to the first frame.
 *
 * For three or more valid frames, runs all-pairs initial XY/yaw matching, keeps
 * a consistency-checked spanning tree, then refines the tree with XY/yaw,
 * Z-only, and full 6-DoF passes. Two valid frames use the pairwise alignment
 * path directly. The input frames are expected to already have gravity-aligned
 * ``sensor_info.sensor_to_body`` rotations; the full existing extrinsics,
 * including translations, are used as initial priors. The first returned
 * transform is the first frame's existing extrinsic; each subsequent transform
 * is the frame's input extrinsic
 * left-multiplied by the estimated correction.
 *
 * @param[in] frames Frame set to align. ``frames[0]`` must be valid and is used
 * as the anchor.
 * @return One 4x4 aligned extrinsic matrix per frame in ``frames``. For each
 *         frame after the anchor, the alignment component follows the
 *         ``source_to_target_transform`` convention, with that frame as source
 *         and frame 0 as target.
 * @throws std::invalid_argument If ``frames[0]`` is null, or a frame lacks a
 *         RANGE field or ``sensor_info``.
 * @throws std::runtime_error If the spanning-tree matching cannot connect all
 *         sensors.
 */
OUSTER_API_FUNCTION
std::vector<core::Matrix4dR> align_clouds(const ouster::sdk::core::FrameSet& frames);

/**
 * @brief Estimate alignment from two point clouds.
 *
 * Inputs must already be in a gravity-aligned common frame. The caller
 * is responsible for applying any IMU-based gravity rotation and dewarping
 * before calling. Returns ``source_to_target_transform``.
 *
 * @param[in] source_points Source points transformed by the result as
 *            ``N x 3`` XYZ rows.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``.
 * @return Estimated ``source_to_target_transform``.
 * @throws std::invalid_argument If arrays are malformed or have a row
 *         count mismatch.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                             Eigen::Ref<const core::ArrayX3dR> target_points,
                             const core::Matrix4dR& initial_guess = core::Matrix4dR::Identity());

/**
 * @brief Estimate alignment from two point clouds.
 *
 * Inputs must already be in a gravity-aligned common frame. The caller
 * is responsible for applying any IMU-based gravity rotation and dewarping
 * before calling. Returns ``source_to_target_transform``.
 *
 * @param[in] source_points Source points transformed by the result as
 *            ``N x 3`` XYZ rows.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``.
 * @param[out] confidence Confidence score in ``[0, 1]`` for the estimated
 * pose.
 * @return Estimated ``source_to_target_transform``.
 * @throws std::invalid_argument If arrays are malformed or have a row
 *         count mismatch.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                             Eigen::Ref<const core::ArrayX3dR> target_points,
                             const core::Matrix4dR& initial_guess, double& confidence);

/**
 * @brief Estimate alignment from two point clouds.
 *
 * Inputs must already be in a gravity-aligned common frame. The caller
 * is responsible for applying any IMU-based gravity rotation and dewarping
 * before calling. Returns ``source_to_target_transform``.
 *
 * @param[in] source_points Source points transformed by the result as
 *            ``N x 3`` XYZ rows.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[out] confidence Confidence score in ``[0, 1]`` for the estimated
 * pose.
 * @return Estimated ``source_to_target_transform``.
 * @throws std::invalid_argument If arrays are malformed or have a row
 *         count mismatch.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                             Eigen::Ref<const core::ArrayX3dR> target_points, double& confidence);

/**
 * @brief Estimate alignment from point clouds with normals.
 *
 * Inputs must already be in a gravity-aligned common frame. The caller
 * is responsible for applying any IMU-based gravity rotation and dewarping
 * (including transforming normals) before calling.
 * Returns ``source_to_target_transform``.
 *
 * @param[in] source_points Source points transformed by the result as
 *            ``N x 3`` XYZ rows.
 * @param[in] source_normals Source normals aligned with `source_points`.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] target_normals Target normals aligned with `target_points`.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``.
 * @return Estimated ``source_to_target_transform``.
 * @throws std::invalid_argument If arrays are malformed or have a row
 *         count mismatch.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                             Eigen::Ref<const core::ArrayX3dR> source_normals,
                             Eigen::Ref<const core::ArrayX3dR> target_points,
                             Eigen::Ref<const core::ArrayX3dR> target_normals,
                             const core::Matrix4dR& initial_guess = core::Matrix4dR::Identity());

/**
 * @brief Estimate alignment from point clouds with normals.
 *
 * Inputs must already be in a gravity-aligned common frame. The caller
 * is responsible for applying any IMU-based gravity rotation and dewarping
 * (including transforming normals) before calling.
 * Returns ``source_to_target_transform``.
 *
 * @param[in] source_points Source points transformed by the result as
 *            ``N x 3`` XYZ rows.
 * @param[in] source_normals Source normals aligned with `source_points`.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] target_normals Target normals aligned with `target_points`.
 * @param[out] confidence Confidence score in ``[0, 1]`` for the estimated
 * pose.
 * @return Estimated ``source_to_target_transform``.
 * @throws std::invalid_argument If arrays are malformed or have a row
 *         count mismatch.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                             Eigen::Ref<const core::ArrayX3dR> source_normals,
                             Eigen::Ref<const core::ArrayX3dR> target_points,
                             Eigen::Ref<const core::ArrayX3dR> target_normals, double& confidence);

/**
 * @brief Estimate alignment from point clouds with normals.
 *
 * Inputs must already be in a gravity-aligned common frame. The caller
 * is responsible for applying any IMU-based gravity rotation and dewarping
 * (including transforming normals) before calling.
 * Returns ``source_to_target_transform``.
 *
 * @param[in] source_points Source points transformed by the result as
 *            ``N x 3`` XYZ rows.
 * @param[in] source_normals Source normals aligned with `source_points`.
 * @param[in] target_points Reference target points as ``M x 3`` XYZ rows.
 * @param[in] target_normals Target normals aligned with `target_points`.
 * @param[in] initial_guess Initial estimate of ``source_to_target_transform``.
 * @param[out] confidence Confidence score in ``[0, 1]`` for the estimated
 * pose.
 * @return Estimated ``source_to_target_transform``.
 * @throws std::invalid_argument If arrays are malformed or have a row
 *         count mismatch.
 */
OUSTER_API_FUNCTION
core::Matrix4dR align_clouds(Eigen::Ref<const core::ArrayX3dR> source_points,
                             Eigen::Ref<const core::ArrayX3dR> source_normals,
                             Eigen::Ref<const core::ArrayX3dR> target_points,
                             Eigen::Ref<const core::ArrayX3dR> target_normals,
                             const core::Matrix4dR& initial_guess, double& confidence);

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
