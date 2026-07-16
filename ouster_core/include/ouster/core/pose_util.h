#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <limits>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/impl/dewarp_impl.h"
#include "ouster/core/impl/transform_homogeneous.h"
#include "ouster/core/impl/transform_vector.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/visibility.h"
#include "ouster/core/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {

using Poses = MatrixX16dR;

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
void dewarp(Eigen::Ref<PointCloudXYZ<T>> dewarped, const Eigen::Ref<const PointCloudXYZ<T>> points,
            const Eigen::Ref<const MatrixX16R<T>> poses) {
    const int W = poses.rows();                  // Number of pose matrices
    const int H = points.rows() / poses.rows();  // Points per pose matrix

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (int w = 0; w < W; ++w) {
        // Map the w-th row of `poses` (flattened 4×4) back into a 4×4 matrix
        Eigen::Map<const Matrix4R<T>> pose_matrix(poses.row(w).data());
        const Matrix3R<T> rotation = pose_matrix.template topLeftCorner<3, 3>();
        const Eigen::Vector3<T> translation = pose_matrix.template topRightCorner<3, 1>();

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
                        const Eigen::Ref<const MatrixX16R<T>> poses) {
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
inline PointCloudXYZd dewarp(const PointCloudXYZd& points, const Vector16d& pose) {
    // forward to your templated two-arg version
    MatrixX16R<double> poses(1, pose.size());
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
inline PointCloudXYZd transform(const PointCloudXYZd& points, const Vector16d& pose) {
    // forward to your templated two-arg version
    return transform<double>(points, pose);
}

/**
 * Computes the rotation matrix needed to align an acceleration vector with the
 * gravity axis [0, 0, 1].
 *
 * @param[in] accel_x x-component of acceleration.
 * @param[in] accel_y y-component of acceleration.
 * @param[in] accel_z z-component of acceleration.
 * @param[in] fix_yaw If true, apply a counter-yaw so heading is neutralized.
 * If false, return the shortest gravity-alignment rotation without forcing
 * yaw.
 *
 * @return 3x3 rotation matrix.
 */
OUSTER_API_FUNCTION
Eigen::Matrix3d get_rot_matrix_to_align_to_gravity(double accel_x, double accel_y, double accel_z,
                                                   bool fix_yaw = true);

namespace impl {

template <typename T>
inline void check_increasing_throw(T prev, T curr) {
    if (curr < prev) {
        throw std::invalid_argument("x_interp values must be monotonically increasing: " +
                                    std::to_string(curr) + " < " + std::to_string(prev));
    }
}

/**
 * Internal implementation of pose interpolation over a range of x_interp
 * values between two known poses.
 */
template <typename T, typename TIME_CONTAINER, typename OUTPUT_POSE_WRITER>
void interp_pose_range(const typename TIME_CONTAINER::const_iterator& x_interp_begin,
                       const typename TIME_CONTAINER::const_iterator& x_interp_end, T t0,
                       Eigen::Ref<const Matrix4dR> x0, T t1, Eigen::Ref<const Matrix4dR> x1,
                       OUTPUT_POSE_WRITER pose_writer) {
    static_assert(std::is_signed<T>::value, "T must be a signed type");
    T duration = t1 - t0;
    if (std::abs(duration) < std::numeric_limits<T>::epsilon()) {
        throw std::invalid_argument("Cannot interpolate with zero duration between poses");
    }
    // TODO[UN]: Avoid having to construct a and b objects
    ouster::sdk::core::impl::PoseH a(x0);
    ouster::sdk::core::impl::PoseH b(x1);
    // The relative transformation from a to b is (a^-1 * b)
    ouster::sdk::core::impl::PoseH a_inv = a.inverse();
    ouster::sdk::core::impl::PoseV twist = (a_inv * b).log();
    ouster::sdk::core::impl::PoseV scaled_twist = (1.0 / duration) * twist;
    ouster::sdk::core::impl::PoseV delta;
    ouster::sdk::core::impl::PoseH interp;
    auto last_x_interp = x_interp_begin;
    for (auto x_interp = x_interp_begin; x_interp != x_interp_end; ++x_interp) {
        // monotonicity check
        check_increasing_throw(*last_x_interp, *x_interp);
        last_x_interp = x_interp;
        auto dt = *x_interp - t0;
        delta = dt * scaled_twist;
        interp = a * delta.exp();
        pose_writer(interp.matrix());
    }
}

/**
 * Internal implementation of pose interpolation over a set of known poses.
 *
 * The function is written as a template to allow for different container types
 * including Eigen matrices.
 */
template <typename T, typename TIME_CONTAINER, typename KNOWN_POSE_ACCESSOR,
          typename OUTPUT_POSE_WRITER>
void interp_pose(const typename TIME_CONTAINER::const_iterator& x_interp_begin,
                 const typename TIME_CONTAINER::const_iterator& x_interp_end,
                 const typename TIME_CONTAINER::const_iterator& x_known_begin,
                 const typename TIME_CONTAINER::const_iterator& x_known_end,
                 KNOWN_POSE_ACCESSOR poses_known_accessor, OUTPUT_POSE_WRITER pose_writer) {
    auto x_known_n = std::distance(x_known_begin, x_known_end);

    if (x_known_n < 2) {
        throw std::invalid_argument("Not enough evaluation poses for interpolation");
    }

    auto x_interp_curr = x_interp_begin;

    // the general case: interpolate between known poses
    for (int i = 0; i < x_known_n - 1; ++i) {
        if (*(x_known_begin + i) >= *(x_known_begin + i + 1)) {
            throw std::invalid_argument(
                "input x_known values are not monotonically increasing or "
                "values repeated");
        }

        auto it = std::lower_bound(x_interp_curr, x_interp_end, *(x_known_begin + i + 1));

        if (it == x_interp_curr) {
            continue;
        }

        interp_pose_range<T, TIME_CONTAINER>(x_interp_curr, it, *(x_known_begin + i),
                                             poses_known_accessor(i), *(x_known_begin + i + 1),
                                             poses_known_accessor(i + 1), pose_writer);

        x_interp_curr = it;
    }

    // identify values in x_interp that fall after the last known pose
    if (x_interp_curr < x_interp_end) {
        interp_pose_range<T, TIME_CONTAINER>(
            x_interp_curr, x_interp_end, *(x_known_begin + x_known_n - 2),
            poses_known_accessor(x_known_n - 2), *(x_known_begin + x_known_n - 1),
            poses_known_accessor(x_known_n - 1), pose_writer);
    }
}

}  // namespace impl

// NOTE[UN]: Currently there is no point in templatizing the matrices here since
// the lie group operations are only defined for double precision in
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
 * @note x_interp must be sorted in ascending order otherwise, it will throw
 * std::invalid_argument.
 *
 * @throws std::invalid_argument if the sizes of x_known and poses_known do not
 * match, if their sizes are less than 2, or if x_known is not
 * monotonically increasing or if x_interp is not monotonically increasing.
 */
template <typename T>
std::vector<Matrix4dR> interp_pose(const std::vector<T>& x_interp, T t0,
                                   Eigen::Ref<const Matrix4dR> x0, T t1,
                                   Eigen::Ref<const Matrix4dR> x1) {
    std::vector<Matrix4dR> result;
    result.reserve(x_interp.size());
    impl::interp_pose_range<T, std::vector<T>>(
        x_interp.begin(), x_interp.end(), t0, x0, t1, x1,
        [&result](const Matrix4dR& pose) { result.emplace_back(pose); });
    return result;
}

// NOTE[UN]: Currently there is no point in templatizing the matrices here since
// the lie group operations are only defined for double precision in
// impl/transform_homogeneous.h
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
 * @note x_interp must be sorted in ascending order otherwise, it will throw
 * std::invalid_argument.
 *
 * @throws std::invalid_argument if the sizes of x_known and poses_known do not
 * match, if their sizes are less than 2, or if x_known is not
 * monotonically increasing or if x_interp is not monotonically increasing.
 */
template <typename T>
OUSTER_API_FUNCTION std::vector<Matrix4dR> interp_pose(const std::vector<T>& x_interp,
                                                       const std::vector<T>& x_known,
                                                       const std::vector<Matrix4dR>& poses_known) {
    if (x_known.size() != poses_known.size()) {
        throw std::invalid_argument("x_known and poses_known sizes are not matching");
    }

    std::vector<Matrix4dR> result;
    result.reserve(x_interp.size());

    impl::interp_pose<T, std::vector<T>>(
        x_interp.begin(), x_interp.end(), x_known.begin(), x_known.end(),
        [&poses_known](size_t idx) { return poses_known[idx]; },
        [&result](const Matrix4dR& pose) { result.emplace_back(pose); });

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
 * @param[in] x_interp An Eigen vector of x-coordinate values at which to
 * compute the interpolated transformation matrices.
 * @param[in] x_known An Eigen vector of x-coordinate values corresponding to
 * the known transformation matrices. Must be monotonically increasing and not
 * repeated.
 * @param[in] poses_known An Eigen matrix of 4x4 transformation matrices (stored
 * as X rows with 16 elements) where each x-coordinate is associated a value in
 * x_known.
 *
 * @return An Eigen matrix of 4x4 transformation matrices representing the
 * interpolated poses at x-coordinate values given by x_interp  (stored as X
 * rows with 16 elements).
 *
 * @note If x_interp contains values outside the range of x_known, the function
 * uses the first two and last two poses to extrapolate accordingly.
 *
 * @throws std::invalid_argument if the sizes of x_known and poses_known do not
 * match, if their sizes are less than 2, or if x_known is not
 * monotonically increasing.
 */
template <typename T, typename Scalar>
OUSTER_API_FUNCTION MatrixX16R<Scalar> interp_pose(
    const Eigen::Ref<const Eigen::VectorX<T>> x_interp,
    const Eigen::Ref<const Eigen::VectorX<T>> x_known,
    const Eigen::Ref<const MatrixX16R<Scalar>> poses_known) {
    if (x_known.size() != poses_known.rows()) {
        throw std::invalid_argument("x_known and poses_known sizes are not matching");
    }

    MatrixX16R<Scalar> result(x_interp.size(), 16);
    Eigen::Index write_idx = 0;

    auto pose_writer = [&](const Matrix4dR& pose) {
        Eigen::Map<const Eigen::Matrix<double, 1, 16, Eigen::RowMajor>> pose_flat(pose.data());
        result.row(write_idx++) = pose_flat.template cast<Scalar>();
    };

    auto poses_known_accessor = [&](size_t idx) {
        Eigen::Map<const Matrix4R<Scalar>> pose_map(poses_known.row(idx).data());
        return pose_map.template cast<double>();
    };
    impl::interp_pose<T, Eigen::Ref<const Eigen::VectorX<T>>>(x_interp.cbegin(), x_interp.cend(),
                                                              x_known.cbegin(), x_known.cend(),
                                                              poses_known_accessor, pose_writer);

    return result;
}

/**
 * This function takes in a LidarFrame and an XYZLut and applies dewarping to it
 * based on the poses associated with each column in the LidarFrame. It filters
 * out points with zero range and points outside the specified range limits.
 *
 * @param[in] lidar_frame The LidarFrame containing the range data and
 * associated poses.
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
OUSTER_API_FUNCTION std::vector<Eigen::Vector3<T>> dewarp(const LidarFrame& lidar_frame,
                                                          const XYZLutT<T>& xyzlut,
                                                          double min_range, double max_range);

/**
 * This function takes in a FrameSet and a list of XYZLut(s) then applies
 * dewarping to each LidarFrame within the set based on the poses associated
 * with each column in the LidarFrame. It filters out points with zero range and
 * points outside the specified range limits.
 *
 * @param[in] frame_set The set of LidarFrames to dewarp.
 * @param[in] xyzluts One XYZLut per frame in frame_set, used to project
 * range measurements into Cartesian coordinates.
 * @param[in] min_range Minimum range (meters); points closer than this are
 * excluded.
 * @param[in] max_range Maximum range (meters); points farther than this are
 * excluded.
 */
template <typename T>
OUSTER_API_FUNCTION std::vector<Eigen::Vector3<T>> dewarp(const FrameSet& frame_set,
                                                          const std::vector<XYZLutT<T>>& xyzluts,
                                                          double min_range, double max_range);

// Public definitions for the 4-arg overloads declared above.
template <typename T>
OUSTER_API_FUNCTION std::vector<Eigen::Vector3<T>> dewarp(const LidarFrame& lidar_frame,
                                                          const XYZLutT<T>& xyzlut,
                                                          double min_range, double max_range) {
    return impl::dewarp_impl<T>(lidar_frame, xyzlut, min_range, max_range, nullptr, nullptr);
}

template <typename T>
OUSTER_API_FUNCTION std::vector<Eigen::Vector3<T>> dewarp(const FrameSet& frame_set,
                                                          const std::vector<XYZLutT<T>>& xyzluts,
                                                          double min_range, double max_range) {
    return impl::dewarp_impl<T>(frame_set, xyzluts, min_range, max_range, nullptr, nullptr,
                                nullptr);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
