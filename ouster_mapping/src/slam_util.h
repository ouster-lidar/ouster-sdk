#pragma once

#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <ouster/visibility.h>

#include <Eigen/Core>
#include <nonstd/optional.hpp>
#include <sophus/se3.hpp>

#include "ouster/lidar_scan_set.h"

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

/**
 * Average highest 92% to 96% range readings and use this averaged range value
 * to calculate the voxel map size.
 * Returns: optional voxel size in meters, or std::nullopt if no valid ranges.
 */
nonstd::optional<double> determine_voxel_size(
    const ouster::sdk::core::LidarScanSet& scans, double start_pct = 0.92,
    double end_pct = 0.96);

/**
 * Takes a 3x3 rotation matrix and returns an orthonormal matrix of it using
 * SVD.
 *
 * @param matrix The input 3x3 matrix to be orthonormalized
 * @return An orthonormal 3x3 rotation matrix
 */
Eigen::Matrix3d make_ortho(const Eigen::Matrix3d& matrix);

/**
 * Takes a 4x4 transformation matrix and returns an orthonormal version of it.
 * The 3x3 rotation part is orthonormalized while preserving the translation.
 *
 * @param matrix The input 4x4 matrix to be orthonormalized
 * @return An orthonormal 4x4 transformation matrix
 */
Eigen::Matrix4d make_ortho(const Eigen::Matrix4d& matrix);

/**
 * Transforms the poses in the given LidarScan in place using the provided
 * transformation matrix.
 */
void transform_inplace(ouster::sdk::core::LidarScan& scan,
                       const ouster::sdk::core::Matrix4dR& transform);

/**
 * Transforms the poses in all valid LidarScans in the given LidarScanSet in
 * place using the provided transformation matrix.
 */
void transform_inplace(ouster::sdk::core::LidarScanSet& scan_set,
                       const ouster::sdk::core::Matrix4dR& transform);

template <typename T>
std::vector<int> get_valid_columns(
    Eigen::Ref<const ouster::sdk::core::LidarScan::Header<T>> status) {
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
 * Extracts valid timestamps from a LidarScan timestamp field based on the
 * supplied valid column indices.
 */
std::vector<double> get_valid_timestamps(
    Eigen::Ref<const Eigen::ArrayX<uint64_t>> ts_field,
    const std::vector<int>& valid);

/**
 * @brief Computes the LidarScanSet total time range using scans timestamps
 * @param[in] scans LidarScanSet to compute timestamp range for
 * @return pair of (min timestamp, max timestamp) across all scans in the set
 */
std::pair<uint64_t, uint64_t> compute_frame_ts_range(
    const ouster::sdk::core::LidarScanSet& scans);

/**
 * @brief Finds the midest point in a LidarScanSet given the precomputed scan
 * set range
 * @param[in] scans LidarScanSet to find mid point in
 * @param[in] ts_range The pre-computed time range of the LidarScanSet
 * @return pair of (scan index, column index) corresponding to the mid point
 */
std::pair<size_t, size_t> find_scan_set_mid(
    const ouster::sdk::core::LidarScanSet& scans,
    const std::pair<int64_t, int64_t>& ts_range);

/**
 * @brief Interpolates poses for each valid column in the given LidarScan based
 * on the provided timestamp and pose lists.
 *
 * @param[in, out] scan The LidarScan to interpolate its poses.
 * @param[in] t0 The x-coordinate value corresponding to the first known pose.
 * @param[in] x0 The first known 4x4 transformation matrix.
 * @param[in] t1 The x-coordinate value corresponding to the second known pose.
 * @param[in] x1 The second known 4x4 transformation matrix.
 */
void interp_pose(ouster::sdk::core::LidarScan& scan, double t0,
                 Eigen::Ref<const ouster::sdk::core::Matrix4dR> x0, double t1,
                 Eigen::Ref<const ouster::sdk::core::Matrix4dR> x1);

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
