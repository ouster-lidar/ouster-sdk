#include <Eigen/Dense>
#include <vector>

#include "ouster/impl/transform_homogeneous.h"
#include "ouster/impl/transform_vector.h"
#include "ouster/lidar_scan.h"
#include "ouster/visibility.h"

namespace ouster {
namespace core {

typedef Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> Points;
typedef Eigen::Matrix<double, Eigen::Dynamic, 16, Eigen::RowMajor> Poses;
typedef Eigen::Matrix<double, 1, 16, Eigen::RowMajor> Pose;

template <typename T>
using PointsT = Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
template <typename T>
using PosesT = Eigen::Matrix<T, Eigen::Dynamic, 16, Eigen::RowMajor>;

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
void dewarp(Eigen::Ref<PointsT<T>> dewarped,
            const Eigen::Ref<const PointsT<T>> points,
            const Eigen::Ref<const PosesT<T>> poses) {
    const int W = poses.rows();                  // Number of pose matrices
    const int H = points.rows() / poses.rows();  // Points per pose matrix

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (int w = 0; w < W; ++w) {
        // Map the w-th row of `poses` (flattened 4×4) back into a 4×4 matrix
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor>> pose_matrix(
            poses.row(w).data());

        auto rotation = pose_matrix.template topLeftCorner<3, 3>();
        auto translation = pose_matrix.template topRightCorner<3, 1>();

        // For each point corresponding to this pose
        for (int i = 0; i < H; ++i) {
            const Eigen::Index ix = i * W + w;

            // Map source and destination 3-vectors
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> s(points.row(ix).data());
            Eigen::Map<Eigen::Matrix<T, 3, 1>> p(dewarped.row(ix).data());

            // Apply the rigid‐body transform
            p = rotation * s + translation;
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
PointsT<T> dewarp(const Eigen::Ref<const PointsT<T>>& points,
                  const Eigen::Ref<const PosesT<T>>& poses) {
    // Allocate output with the same shape as input
    PointsT<T> dewarped(points.rows(), points.cols());

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
inline Points dewarp(const Points& points, const Pose& pose) {
    // forward to your templated two-arg version
    return dewarp<double>(points, pose);
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
void transform(
    Eigen::Ref<Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>>
        transformed,
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>>
        points,
    const Eigen::Ref<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose) {
    Eigen::Matrix<T, 4, 4, Eigen::RowMajor> pose_matrix =
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor>>(pose.data());
    auto rotation = pose_matrix.template topLeftCorner<3, 3>();
    auto translation = pose_matrix.template topRightCorner<3, 1>();

    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const Eigen::Matrix<T, 3, 1>& p = points.row(i);
        Eigen::Matrix<T, 3, 1> new_p = rotation * p + translation;
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
Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor> transform(
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>>
        points,
    const Eigen::Ref<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose) {
    Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor> transformed(
        points.rows(), points.cols());
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
inline Points transform(const Points& points, const Pose& pose) {
    // forward to your templated two-arg version
    return transform<double>(points, pose);
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
template <typename T>
OUSTER_API_FUNCTION std::vector<Eigen::Matrix<double, 4, 4>> interp_pose(
    const std::vector<T>& x_interp, const std::vector<T>& x_known,
    const std::vector<Eigen::Matrix<double, 4, 4>>& poses_known) {
    if (x_known.size() != poses_known.size()) {
        throw std::invalid_argument(
            "x_known and poses_known sizes are not matching");
    }
    if (x_known.size() < 2) {
        throw std::invalid_argument(
            "Not enough evaluation poses for interpolation");
    }

    const size_t x_known_n = x_known.size();
    const size_t x_interp_n = x_interp.size();
    // sample poses internal
    size_t current_interval = 0;

    std::vector<Eigen::Matrix<double, 4, 4>> results(x_interp_n);
    std::vector<ouster::impl::PoseV> pose_diff_intervals(x_known_n - 1);

    nonstd::optional<T> last_x = nonstd::nullopt;
    for (size_t i = 0; i < x_known_n - 1; i++) {
        T x = x_known[i];
        if (last_x != nonstd::nullopt && x <= last_x) {
            throw std::invalid_argument(
                "input x_known values are not monotonically increasing or "
                "values repeated");
        }
        last_x = x;
        ouster::impl::PoseH prev_poseh(poses_known[i]);
        ouster::impl::PoseH curr_poseh(poses_known[i + 1]);
        ouster::impl::PoseV pose_diff =
            (curr_poseh * ouster::impl::PoseH(prev_poseh.inverse())).log();
        pose_diff_intervals[i] = pose_diff;
    }

    last_x = nonstd::nullopt;
    // Loop over each interpolation timestamp.
    for (size_t i = 0; i < x_interp_n; i++) {
        T x = x_interp[i];
        // If before the first evaluation, extrapolate using first interval.
        if (x <= x_known.front()) {
            T x0 = x_known[0];
            T x1 = x_known[1];
            double ratio = -1 * double(x0 - x) / double(x1 - x0);

            ouster::impl::PoseH prev_poseh(poses_known[0]);
            ouster::impl::PoseV pose_diff = pose_diff_intervals[0];
            ouster::impl::PoseV delta = ratio * pose_diff;
            ouster::impl::PoseH interp_poseh = (delta.exp()) * prev_poseh;
            results[i] = interp_poseh;
            continue;
        }

        // If after the last evaluation, extrapolate using the last interval.
        if (x >= x_known.back()) {
            T x0 = x_known[x_known_n - 2];
            T x1 = x_known[x_known_n - 1];
            double ratio = double(x - x1) / double(x1 - x0);

            ouster::impl::PoseH prev_poseh(poses_known.back());
            ouster::impl::PoseV pose_diff = pose_diff_intervals.back();
            ouster::impl::PoseV delta = ratio * pose_diff;
            ouster::impl::PoseH interp_poseh = (delta.exp()) * prev_poseh;
            results[i] = interp_poseh;
            continue;
        }

        if (last_x != nonstd::nullopt && x < last_x) {
            while (current_interval > 0 && x < x_known[current_interval]) {
                current_interval--;
            }
        }
        last_x = x;

        while (current_interval < x_known_n - 1 &&
               x_known[current_interval + 1] < x) {
            current_interval++;
        }

        T x0 = x_known[current_interval];
        T x1 = x_known[current_interval + 1];
        double ratio = double(x - x0) / double(x1 - x0);

        ouster::impl::PoseH prev_poseh(poses_known[current_interval]);
        ouster::impl::PoseV pose_diff = pose_diff_intervals[current_interval];
        ouster::impl::PoseV delta = ratio * pose_diff;
        ouster::impl::PoseH interp_poseh = (delta.exp()) * prev_poseh;
        results[i] = interp_poseh;
    }

    return results;
}

/**
 * @brief Computes piecewise linear interpolated 4x4 transformation matrices
 * based on input x-coordinate values.
 *
 * This function is the Python binding version of the above function.
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

}  // namespace core
}  // namespace ouster
