#pragma once

#include <ouster/core/lidar_frame.h>
#include <ouster/core/types.h>
#include <ouster/core/visibility.h>

#include <Eigen/Core>
#include <nonstd/optional.hpp>

#include "ouster/core/frame_set.h"

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

/**
 * Average highest 92% to 96% range readings and use this averaged range value
 * to calculate the voxel map size.
 * Returns: optional voxel size in meters, or std::nullopt if no valid ranges.
 */
nonstd::optional<double> determine_voxel_size(const ouster::sdk::core::FrameSet& frame_set,
                                              double start_pct = 0.92, double end_pct = 0.96);

/**
 * Takes a 3x3 rotation matrix and returns an orthonormal matrix of it using
 * SVD.
 *
 * @param matrix The input 3x3 matrix to be orthonormalized
 * @return An orthonormal 3x3 rotation matrix
 */
core::Matrix3dR make_ortho(const core::Matrix3dR& matrix);

/**
 * Takes a 4x4 transformation matrix and returns an orthonormal version of it.
 * The 3x3 rotation part is orthonormalized while preserving the translation.
 *
 * @param matrix The input 4x4 matrix to be orthonormalized
 * @return An orthonormal 4x4 transformation matrix
 */
core::Matrix4dR make_ortho(const core::Matrix4dR& matrix);

/**
 * Sets the pose of every valid column in the given LidarFrame to the provided
 * pose.
 *
 * @param[in,out] frame The LidarFrame whose valid column poses are initialized.
 * @param[in] pose The pose to set for each valid column.
 */
void init_valid_column_poses(ouster::sdk::core::LidarFrame& frame,
                             Eigen::Ref<const core::Matrix4dR> pose);

/**
 * Sets the pose of every valid column in all valid LidarFrames of the given
 * FrameSet to the provided pose.
 *
 * @param[in,out] frame_set The FrameSet whose valid column poses are
 *        initialized.
 * @param[in] pose The pose to set for each valid column in all frames.
 */
void init_valid_column_poses(ouster::sdk::core::FrameSet& frame_set,
                             Eigen::Ref<const core::Matrix4dR> pose);

/**
 * Transforms the poses in the given LidarFrame in place using the provided
 * transformation matrix.
 */
void transform_inplace(ouster::sdk::core::LidarFrame& frame, const core::Matrix4dR& transform);

/**
 * Transforms the poses in all valid LidarFrames in the given FrameSet in
 * place using the provided transformation matrix.
 */
void transform_inplace(ouster::sdk::core::FrameSet& frame_set, const core::Matrix4dR& transform);

/**
 * @brief Re-anchors each object's body_to_world to the interpolated frame pose
 * at the object's timestamp.
 *
 * object_to_body is preserved. Objects whose timestamp cannot be bracketed by
 * valid columns are left unchanged.
 *
 * @param[in,out] frame The LidarFrame whose object poses are re-anchored.
 */
void update_object_poses(ouster::sdk::core::LidarFrame& frame);

/**
 * @brief Re-anchors object poses for both the per-frame objects of every valid
 * LidarFrame and the FrameSet-level objects.
 *
 * Per-frame objects use their own frame's columns. FrameSet-level objects use
 * the first frame that brackets the object's timestamp; unbracketed objects
 * are left unchanged.
 *
 * @param[in,out] frame_set The FrameSet whose object poses are re-anchored.
 */
void update_object_poses(ouster::sdk::core::FrameSet& frame_set);

template <typename T>
size_t count_valid_columns(Eigen::Ref<const ouster::sdk::core::LidarFrame::Header<T>> status) {
    size_t result = 0;
    // Column status bit 0 indicates lidar column validity.
    for (Eigen::Index i = 0; i < status.size(); ++i) {
        if (status[i] & 0x01) {
            ++result;
        }
    }

    return result;
}

template <typename T>
std::vector<int> get_valid_columns(
    Eigen::Ref<const ouster::sdk::core::LidarFrame::Header<T>> status) {
    std::vector<int> result;
    result.reserve(status.size());
    for (Eigen::Index i = 0; i < status.size(); ++i) {
        if (status[i] & 0x01) {
            result.push_back(static_cast<int>(i));
        }
    }

    return result;
}

/*
 * Extracts valid timestamps from a LidarFrame timestamp field based on the
 * supplied valid column indices.
 */
std::vector<double> get_valid_timestamps(Eigen::Ref<const Eigen::ArrayX<uint64_t>> ts_field,
                                         const std::vector<int>& valid);

/**
 * @brief Computes the FrameSet total time range using frame timestamps
 * @param[in] frame_set FrameSet to compute timestamp range for
 * @return pair of (min timestamp, max timestamp) across all frames in the set
 */
std::pair<uint64_t, uint64_t> compute_frame_ts_range(const ouster::sdk::core::FrameSet& frame_set);

/**
 * @return true if the frame has at least one valid column (status bit 0 set).
 * Short-circuits on the first valid column. Used to skip frames that would
 * otherwise make get_first_valid_column() throw.
 */
inline bool has_valid_columns(const ouster::sdk::core::LidarFrame& frame) {
    // Same status-check idiom used throughout lidar_frame.cpp; any()
    // short-circuits on the first valid column.
    return frame.status().unaryExpr([](uint32_t status_val) { return status_val & 0x01; }).any();
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
