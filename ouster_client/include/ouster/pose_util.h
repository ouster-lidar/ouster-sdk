#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <limits>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "ouster/cartesian.h"
#include "ouster/impl/transform_homogeneous.h"
#include "ouster/impl/transform_vector.h"
#include "ouster/lidar_scan.h"
#include "ouster/lidar_scan_set.h"
#include "ouster/visibility.h"
#include "ouster/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {

using Poses = MatrixX16dR;
using Pose = Vector16d;

template <typename T>
using PointsT [[deprecated("Use PointCloudXYZ instead")]] = PointCloudXYZ<T>;
template <typename T>
using PosesT = MatrixX16R<T>;

/**
 * This function takes in a set of 3D points and a set of 4x4 pose matrices
 *
 * @param[out] dewarped An eigen matrix of shape (N, 3) to hold the dewarped
 * 3D points, where the same number of points are transformed by each
 * corresponding pose matrix.
 * @param[in] points A Eigen matrix of shape (N, 3) representing the 3D points.
 * Each row corresponds to a point in 3D space.
 * @param[in] poses A Eigen matrix of shape (W, 16) representing W 4x4
 * transformation matrices. Each row is a flattened 4x4 pose matrix
 */
template <typename T>
void dewarp(Eigen::Ref<PointCloudXYZ<T>> dewarped,
            const Eigen::Ref<const PointCloudXYZ<T>> points,
            const Eigen::Ref<const PosesT<T>> poses) {
    const int W = poses.rows();                  // Number of pose matrices
    const int H = points.rows() / poses.rows();  // Points per pose matrix

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (int w = 0; w < W; ++w) {
        // Map the w-th row of `poses` (flattened 4×4) back into a 4×4 matrix
        Eigen::Map<const Matrix4R<T>> pose_matrix(poses.row(w).data());
        const Matrix3R<T> rotation = pose_matrix.template topLeftCorner<3, 3>();
        const Eigen::Vector3<T> translation =
            pose_matrix.template topRightCorner<3, 1>();

        // For each point corresponding to this pose
        for (int i = 0; i < H; ++i) {
            const Eigen::Index ix = i * W + w;
            const Eigen::Vector3<T> s = points.row(ix);
            dewarped.row(ix) = rotation * s + translation;
        }
    }
}

/**
 * This function takes in a set of 3D points and a set of 4x4 pose matrices
 *
 * @param[in] points A Eigen matrix of shape (N, 3) representing the 3D points.
 * Each row corresponds to a point in 3D space.
 * @param[in] poses A Eigen matrix of shape (W, 16) representing W 4x4
 * transformation matrices. Each row is a flattened 4x4 pose matrix
 *
 * @return A matrix of shape (N, 3) containing the dewarped 3D points,
 * where the same number of points are transformed by each corresponding pose
 * matrix.
 */
template <typename T>
PointCloudXYZ<T> dewarp(const Eigen::Ref<const PointCloudXYZ<T>>& points,
                        const Eigen::Ref<const PosesT<T>> poses) {
    // Allocate output with the same shape as input
    PointCloudXYZ<T> dewarped(points.rows(), points.cols());

    // Call the in‐place overload
    dewarp<T>(dewarped, points, poses);

    return dewarped;
}

/**
 * This function takes in a set of 3D points and a set of 4x4 pose matrices
 *
 * @param[in] points A Eigen matrix of shape (N, 3) representing the 3D points.
 * Each row corresponds to a point in 3D space.
 * @param[in] pose A Eigen matrix of shape (W, 16) representing W 4x4
 * transformation matrices. Each row is a flattened 4x4 pose matrix
 *
 * @return A matrix of shape (N, 3) containing the dewarped 3D points,
 * where the same number of points are transformed by each corresponding pose
 * matrix.
 */
OUSTER_API_FUNCTION
inline PointCloudXYZd dewarp(const PointCloudXYZd& points, const Pose& pose) {
    // forward to your templated two-arg version
    PosesT<double> poses(1, pose.size());
    poses.row(0) = pose;
    return dewarp<double>(points, poses);
}

/**
 *  Applies a single 4x4 pose transformation to a set of 3D points.
 *
 * This function takes in a set of 3D points and applies a single 4x4
 * transformation matrix (Pose) to all points.
 *
 * @param[out] transformed A matrix of shape (N, 3) containing the transformed
 * 3D points, where each point is rotated and translated by the given pose.
 * @param[in] points A matrix of shape (N, 3) representing the 3D points.
 *               Each row corresponds to a point in 3D space.
 * @param[in] pose A vector of 16 elements representing a flattened 4x4
 * transformation matrix.
 */
template <typename T>
void transform(Eigen::Ref<PointCloudXYZ<T>> transformed,
               const Eigen::Ref<const PointCloudXYZ<T>> points,
               const Eigen::Ref<const Vector16<T>> pose) {
    Matrix4R<T> pose_matrix = Eigen::Map<const Matrix4R<T>>(pose.data());
    auto rotation = pose_matrix.template topLeftCorner<3, 3>();
    auto translation = pose_matrix.template topRightCorner<3, 1>();

    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Vector3<T> p = points.row(i);
        const Eigen::Vector3<T> new_p = (rotation * p) + translation;
        transformed.row(i) = new_p.transpose();
    }
}

/**
 *  Applies a single 4x4 pose transformation to a set of 3D points.
 *
 * This function takes in a set of 3D points and applies a single 4x4
 * transformation matrix (Pose) to all points.
 *
 * @param[in] points A matrix of shape (N, 3) representing the 3D points.
 *               Each row corresponds to a point in 3D space.
 * @param[in] pose A vector of 16 elements representing a flattened 4x4
 * transformation matrix.
 *
 * @return A matrix of shape (N, 3) containing the transformed 3D points,
 *         where each point is rotated and translated by the given pose.
 */
template <typename T>
PointCloudXYZ<T> transform(const Eigen::Ref<const PointCloudXYZ<T>> points,
                           const Eigen::Ref<const Vector16<T>> pose) {
    PointCloudXYZ<T> transformed(points.rows(), points.cols());
    transform<T>(transformed, points, pose);
    return transformed;
}

/**
 *  Applies a single 4x4 pose transformation to a set of 3D points.
 *
 * This function takes in a set of 3D points and applies a single 4x4
 * transformation matrix (Pose) to all points.
 *
 * @param[in] points A matrix of shape (N, 3) representing the 3D points.
 *               Each row corresponds to a point in 3D space.
 * @param[in] pose A vector of 16 elements representing a flattened 4x4
 * transformation matrix.
 *
 * @return A matrix of shape (N, 3) containing the transformed 3D points,
 *         where each point is rotated and translated by the given pose.
 */
OUSTER_API_FUNCTION
inline PointCloudXYZd transform(const PointCloudXYZd& points,
                                const Pose& pose) {
    // forward to your templated two-arg version
    return transform<double>(points, pose);
}

// NOTE[UN]: there is no point in templatizing the matrices here since the lie
// group operations are only defined for double precision in
// impl/transform_homogeneous.h
/**
 * @brief One-dimensional linear interpolation for monotonically increasing
 * sample transformation matrices. This method interpolates between two poses.
 *
 * @param[in] x_interp A vector of x-coordinate values at which to compute the
 * interpolated transformation matrices.
 * @param[in] t0 The x-coordinate value corresponding to the first known pose.
 * @param[in] x0 The first known 4x4 transformation matrix.
 * @param[in] t1 The x-coordinate value corresponding to the second known pose.
 * @param[in] x1 The second known 4x4 transformation matrix.
 *
 * @return A vector of 4x4 transformation matrices representing the interpolated
 * poses at x-coordinate values given by x_interp.
 *
 * @note If x_interp contains values outside the range of t0 and t1 it will
 * extrapolate the values.
 *
 * @throws std::invalid_argument if t0 and t1 are the same value.
 */
template <typename T>
std::vector<Matrix4dR> interp_pose(const std::vector<T>& x_interp, T t0,
                                   Eigen::Ref<const Matrix4dR> x0, T t1,
                                   Eigen::Ref<const Matrix4dR> x1) {
    static_assert(std::is_signed<T>::value, "T must be a signed type");
    T duration = t1 - t0;
    if (std::abs(duration) < std::numeric_limits<T>::epsilon()) {
        throw std::invalid_argument(
            "Cannot interpolate with zero duration between poses");
    }
    std::vector<Matrix4dR> result;
    result.resize(x_interp.size());
    ouster::sdk::core::impl::PoseH a(x0);
    ouster::sdk::core::impl::PoseH b(x1);
    // The relative transformation from a to b is (a^-1 * b)
    ouster::sdk::core::impl::PoseH a_inv = a.inverse();
    ouster::sdk::core::impl::PoseV twist = (a_inv * b).log();
    ouster::sdk::core::impl::PoseV delta;
    ouster::sdk::core::impl::PoseH interp;
    for (size_t idx = 0; idx < x_interp.size(); ++idx) {
        auto t = (x_interp[idx] - t0) / duration;
        delta = t * twist;
        interp = a * delta.exp();
        result[idx] = interp.matrix();
    }
    return result;
}
/**
 * @brief One-dimensional linear interpolation for monotonically increasing
 * sample transformation matrices.
 *
 * This function performs linear interpolation on a set of monotonically
 * increasing and non-repeated x-coordinate values and their corresponding 4x4
 * pose matrices. It evaluates the interpolated poses at the specified
 * x-coordinate values provided in x_interp.
 *
 * @param[in] x_interp A vector of x-coordinate values at which to compute the
 * interpolated transformation matrices.
 * @param[in] x_known A vector of x-coordinate values corresponding to the
 * known transformation matrices. Must be monotonically increasing and not
 * repeated.
 * @param[in] poses_known A vector of 4x4 transformation matrices associated
 * with each x-coordinate value in x_known.
 *
 * @return A vector of 4x4 transformation matrices representing the interpolated
 * poses at x-coordinate values given by x_interp.
 *
 * @note If x_interp contains values outside the range of x_known, the function
 * uses the first two and last two poses to extrapolate accordingly.
 *
 * @throws std::invalid_argument if the sizes of x_known and poses_known do not
 * match, if their sizes are less than 2, or if x_known is not
 * monotonically increasing.
 */

template <typename T, typename Scalar>
OUSTER_API_FUNCTION PosesT<Scalar> interp_pose(
    const Eigen::Ref<const Eigen::VectorX<T>> x_interp,
    const Eigen::Ref<const Eigen::VectorX<T>> x_known,
    const Eigen::Ref<const PosesT<Scalar>> poses_known) {
    if (x_known.size() != poses_known.rows()) {
        throw std::invalid_argument(
            "x_known and poses_known sizes are not matching");
    }
    if (x_known.size() < 2) {
        throw std::invalid_argument(
            "Not enough evaluation poses for interpolation");
    }

    const size_t x_known_n = x_known.size();
    const size_t x_interp_n = x_interp.size();
    size_t current_interval = 0;

    PosesT<Scalar> results(x_interp_n, 16);
    std::vector<ouster::sdk::core::impl::PoseV> pose_diff_intervals(x_known_n -
                                                                    1);

    for (size_t i = 0; i < x_known_n - 1; i++) {
        T x = x_known(i);
        if (i > 0 && x <= x_known(i - 1)) {
            throw std::invalid_argument(
                "input x_known values are not monotonically increasing or "
                "values repeated");
        }
        // Map the i-th row of poses_known (flattened 4×4) back into a 4×4
        // matrix
        Eigen::Map<const Matrix4R<Scalar>> prev_pose_matrix(
            poses_known.row(i).data());
        Eigen::Map<const Matrix4R<Scalar>> curr_pose_matrix(
            poses_known.row(i + 1).data());

        ouster::sdk::core::impl::PoseH prev_poseh(
            prev_pose_matrix.template cast<double>());
        ouster::sdk::core::impl::PoseH curr_poseh(
            curr_pose_matrix.template cast<double>());
        ouster::sdk::core::impl::PoseV pose_diff =
            (curr_poseh * ouster::sdk::core::impl::PoseH(prev_poseh.inverse()))
                .log();
        pose_diff_intervals[i] = pose_diff;
    }

    nonstd::optional<T> last_x = nonstd::nullopt;
    for (size_t i = 0; i < x_interp_n; i++) {
        T x = x_interp(i);
        if (x <= x_known(0)) {
            T x0 = x_known(0);
            T x1 = x_known(1);
            double ratio = -1 * double(x0 - x) / double(x1 - x0);

            Eigen::Map<const Matrix4R<Scalar>> first_pose_matrix(
                poses_known.row(0).data());
            ouster::sdk::core::impl::PoseH prev_poseh(
                first_pose_matrix.template cast<double>());
            ouster::sdk::core::impl::PoseV pose_diff = pose_diff_intervals[0];
            ouster::sdk::core::impl::PoseV delta = ratio * pose_diff;
            ouster::sdk::core::impl::PoseH interp_poseh =
                (delta.exp()) * prev_poseh;
            Matrix4R<Scalar> result_matrix =
                interp_poseh.matrix().template cast<Scalar>();
            Eigen::Map<const Vector16<Scalar>> result_row(result_matrix.data());
            results.row(i) = result_row;
            continue;
        }

        if (x >= x_known(x_known_n - 1)) {
            T x0 = x_known(x_known_n - 2);
            T x1 = x_known(x_known_n - 1);
            double ratio = double(x - x1) / double(x1 - x0);

            Eigen::Map<const Matrix4R<Scalar>> last_pose_matrix(
                poses_known.row(x_known_n - 1).data());
            ouster::sdk::core::impl::PoseH prev_poseh(
                last_pose_matrix.template cast<double>());
            ouster::sdk::core::impl::PoseV pose_diff =
                pose_diff_intervals.back();
            ouster::sdk::core::impl::PoseV delta = ratio * pose_diff;
            ouster::sdk::core::impl::PoseH interp_poseh =
                (delta.exp()) * prev_poseh;
            Matrix4R<Scalar> result_matrix =
                interp_poseh.matrix().template cast<Scalar>();
            Eigen::Map<const Vector16<Scalar>> result_row(result_matrix.data());
            results.row(i) = result_row;
            continue;
        }

        if (last_x != nonstd::nullopt && x < last_x) {
            while (current_interval > 0 && x < x_known(current_interval)) {
                current_interval--;
            }
        }
        last_x = x;

        while (current_interval < x_known_n - 1 &&
               x_known(current_interval + 1) < x) {
            current_interval++;
        }

        T x0 = x_known(current_interval);
        T x1 = x_known(current_interval + 1);
        double ratio = double(x - x0) / double(x1 - x0);

        Eigen::Map<const Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>>
            interval_pose_matrix(poses_known.row(current_interval).data());
        ouster::sdk::core::impl::PoseH prev_poseh(
            interval_pose_matrix.template cast<double>());
        ouster::sdk::core::impl::PoseV pose_diff =
            pose_diff_intervals[current_interval];
        ouster::sdk::core::impl::PoseV delta = ratio * pose_diff;
        ouster::sdk::core::impl::PoseH interp_poseh = delta.exp() * prev_poseh;
        Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> result_matrix =
            interp_poseh.matrix().template cast<Scalar>();
        Eigen::Map<const Eigen::Matrix<Scalar, 1, 16, Eigen::RowMajor>>
            result_row(result_matrix.data());
        results.row(i) = result_row;
    }

    return results;
}

/**
 * @brief Computes piecewise linear interpolated 4x4 transformation matrices
 * based on input x-coordinate values.
 *
 * This function is used by PoseOptimizer and trajectory processing.
 * For general use, prefer the templated version above.
 *
 * @param[in] x_interp A vector of x-coordinate values at which to compute the
 * interpolated transformation matrices.
 * @param[in] x_known A vector of reference x-coordinate values corresponding to
 * the known transformation matrices. Must be monotonically increasing and not
 * repeated.
 * @param[in] poses_known An Eigen::Matrix of dynamic size representing the
 * flattened 4x4 transformation matrices (in row-major order, 16 elements each)
 * associated with each x-coordinate value in x_known.
 *
 * @return An Eigen::Matrix of dynamic size containing the interpolated
 * transformation matrices (flattened in row-major order, 16 elements each)
 * at the x-coordinate values given by x_interp.
 */

OUSTER_API_FUNCTION
Poses interp_pose(const std::vector<double>& x_interp,
                  const std::vector<double>& x_known, const Poses& poses_known);

/**
 * @brief Computes piecewise linear interpolated 4x4 transformation matrices
 * based on input x-coordinate values.
 *
 * This function is used by PoseOptimizer and trajectory processing.
 * For general use, prefer the templated version above.
 *
 * @param[in] x_interp A vector of x-coordinate values at which to compute the
 * interpolated transformation matrices.
 * @param[in] x_known A vector of reference x-coordinate values corresponding to
 * the known transformation matrices. Must be monotonically increasing and not
 * repeated.
 * @param[in] poses_known A vector of Matrix4dR representing the 4x4
 * transformation matrices (in row-major order, 16 elements each)
 * associated with each x-coordinate value in x_known.
 *
 * @return An Eigen::Matrix of dynamic size containing the interpolated
 * transformation matrices (flattened in row-major order, 16 elements each)
 * at the x-coordinate values given by x_interp.
 */

OUSTER_API_FUNCTION
Poses interp_pose(const std::vector<double>& x_interp,
                  const std::vector<double>& x_known,
                  const std::vector<Matrix4dR>& poses_known);

/**
 * This function takes in a LidarScan and an XYZLut and applies dewarping to it
 * based on the poses associated with each column in the LidarScan. It filters
 * out points with zero range and points outside the specified range limits.
 *
 * @param[in] lidar_scan The LidarScan containing the range data and associated
 * poses.
 * @param[in] xyzlut The XYZLut used to convert range data to 3D points.
 * @param[in] min_range Minimum range limit in meters. Points with range below
 * this value will be filtered out.
 * @param[in] max_range Maximum range limit in meters. Points with range above
 * this value will be filtered out.
 *
 * @tparam T The numeric type for the output 3D points (e.g., float or double).
 *
 * @note This method only operates on the first return at the moment.
 *
 * @return A vector of Eigen::Vector3<T> containing the dewarped 3D points.
 */
template <typename T>
OUSTER_API_FUNCTION std::vector<Eigen::Vector3<T>> dewarp(
    const LidarScan& lidar_scan, const XYZLutT<T>& xyzlut, double min_range,
    double max_range) {
    // Filter out points with zero range and collect points/timestamps
    auto range = lidar_scan.field<uint32_t>(ChanField::RANGE);
    // Note[UN]: for future we can optimize this method even further by
    //           embedding the cartesian directly into this and allowing
    //           it to skip over invalid columns.
    PointCloudXYZ<T> pts = cartesianT(range, xyzlut.direction, xyzlut.offset);

    // convert min/max range limits to millimeters
    uint32_t min_r = static_cast<uint32_t>(std::ceil(min_range * 1e3));
    uint32_t max_r = static_cast<uint32_t>(std::floor(max_range * 1e3));

    // Only dewarp valid points that are within range
    const int height = range.rows();
    const int width = range.cols();
    // Pre-allocate memory that can fit all points that are valid
    const int start_col = lidar_scan.get_first_valid_column();
    const int stop_col = lidar_scan.get_last_valid_column();
    if (start_col < 0 || stop_col < 0 || stop_col < start_col) {
        return {};
    }
    std::vector<Eigen::Vector3<T>> dewarped_pts;
    dewarped_pts.reserve(height * (stop_col - start_col + 1));

    auto poses = Eigen::Map<const MatrixX16dR>(lidar_scan.pose().get<double>(),
                                               lidar_scan.w, 16);
    Eigen::Ref<const LidarScan::Header<uint32_t>> status = lidar_scan.status();

    for (int x = start_col; x <= stop_col; ++x) {
        if (status[x] == 0) {
            continue;
        }
        Eigen::Map<const Matrix4dR> pose(poses.row(x).data());
        const Matrix3R<T> rotation = pose.topLeftCorner<3, 3>().cast<T>();
        const Eigen::Vector3<T> translation =
            pose.topRightCorner<3, 1>().cast<T>();
        for (int y = 0; y < height; ++y) {
            uint32_t r = range(y, x);
            if (r >= min_r && r <= max_r) {
                Eigen::Vector3<T> pt = pts.row(y * width + x);
                pt = rotation * pt + translation;
                dewarped_pts.emplace_back(std::move(pt));
            }
        }
    }

    return dewarped_pts;
}

namespace impl {

// A method that provides a quick upper bound on the number of points that can
// be produced from a LidarScanSet
size_t max_number_of_valid_points(const LidarScanSet& lidar_scan_set);

}  // namespace impl

/**
 * This function takes in a LidarScanSet and a list of XYZLut(s) then applies
 * dewarping to each LidarScan within the set based on the poses associated with
 * each column in the LidarScan. It filters out points with zero range and
 * points outside the specified range limits.
 */
template <typename T>
OUSTER_API_FUNCTION std::vector<Eigen::Vector3<T>> dewarp(
    const LidarScanSet& lidar_scan_set, const std::vector<XYZLutT<T>>& xyzluts,
    double min_range, double max_range) {
    assert(lidar_scan_set.size() == xyzluts.size() &&
           "Number of scans and number of XYZLuts must be the same");
    std::vector<Eigen::Vector3<T>> total_pts;
    total_pts.reserve(impl::max_number_of_valid_points(lidar_scan_set));

    for (size_t idx : lidar_scan_set.valid_indices()) {
        const auto& scan = *lidar_scan_set[idx];
        auto dewarped_pts = dewarp(scan, xyzluts[idx], min_range, max_range);
        total_pts.insert(total_pts.end(),
                         std::make_move_iterator(dewarped_pts.begin()),
                         std::make_move_iterator(dewarped_pts.end()));
    }

    return total_pts;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
