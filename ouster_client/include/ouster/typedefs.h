#pragma once
#include <Eigen/Core>

#if !EIGEN_VERSION_AT_LEAST(3,4,0)
namespace Eigen {
    template<typename Scalar, int Rows>
    using Vector = Matrix<Scalar, Rows, 1>;

    template<typename Scalar>
    using Vector3 = Vector<Scalar, 3>;

    template <typename Scalar>
    using VectorX = Vector<Scalar, Eigen::Dynamic>;

    template <typename Scalar>
    using ArrayX = Array<Scalar, Eigen::Dynamic, 1>;

}
#endif

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief Vector of 16 elements.
 * @tparam T The data type for the vector.
 */
template <typename T>
using Vector16 = Eigen::Vector<T, 16>;

/**
 * @brief 3x3 matrix with row-major storage.
 * @tparam T The data type for the matrix.
 */
template <typename T>
using Matrix3R = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;

/**
 * @brief 4x4 matrix with row-major storage.
 * @tparam T The data type for the matrix.
 */
template <typename T>
using Matrix4R = Eigen::Matrix<T, 4, 4, Eigen::RowMajor>;

/**
 * @brief Nx16 matrix with row-major storage.
 * @tparam T The data type for the matrix.
 */
template <typename T>
using MatrixX16R = Eigen::Matrix<T, Eigen::Dynamic, 16, Eigen::RowMajor>;

/**
 * @brief Nx3 matrix with row-major storage.
 * @tparam T The data type for the matrix.
 */
template <typename T>
using MatrixX3R = Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;

/**
 * @brief Nx3 array with row-major storage.
 * @tparam T The data type for the array.
 */
template <typename T>
using ArrayX3R = Eigen::Array<T, Eigen::Dynamic, 3, Eigen::RowMajor>;

/**
 * @brief 16-element vector of doubles
 */
using Vector16d = Vector16<double>;
/**
 * @brief 16-element vector of floats
 */
using Vector16f = Vector16<float>;

/**
 * @brief 3x3 matrix with row-major storage (float)
 */
using Matrix3fR = Matrix3R<float>;
/**
 * @brief 3x3 matrix with row-major storage (double)
 */
using Matrix3dR = Matrix3R<double>;

/**
 * @brief 4x4 matrix with row-major storage (float)
 */
using Matrix4fR = Matrix4R<float>;
/**
 * @brief 4x4 matrix with row-major storage (double)
 */
using Matrix4dR = Matrix4R<double>;

/**
 * @brief Nx16 matrix with row-major storage (float)
 */
using MatrixX16fR = MatrixX16R<float>;
/**
 * @brief Nx16 matrix with row-major storage (double)
 */
using MatrixX16dR = MatrixX16R<double>;

/**
 * @brief Nx3 matrix with row-major storage (float)
 */
using MatrixX3fR = MatrixX3R<float>;
/**
 * @brief Nx3 matrix with row-major storage (double)
 */
using MatrixX3dR = MatrixX3R<double>;

/**
 * @brief Nx3 array with row-major storage (float)
 */
using ArrayX3fR = ArrayX3R<float>;
/**
 * @brief Nx3 array with row-major storage (double)
 */
using ArrayX3dR = ArrayX3R<double>;

/**
 * For image operations.
 *
 * @tparam T The data type for the array.
 */
template <typename T>
using img_t = Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// TODO: deprecate mat4d in favor of Matrix4dR
/** Used for transformations. */
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;

// TODO[tws] use non-deprecated type names
/**
 * Construct a mat4d from a std::array<double, 16> (assumed row-major)
 * @param[in] arr array of 16 doubles
 * @return mat4d matrix
 */
OUSTER_API_FUNCTION
mat4d mat4d_from_array(const std::array<double, mat4d::SizeAtCompileTime>& arr);

/**
 * Convert a mat4d to a std::array<double, 16> (row-major)
 * @param[in] mat 4x4 matrix
 * @return array of 16 doubles
 */
OUSTER_API_FUNCTION
std::array<double, mat4d::SizeAtCompileTime> mat4d_to_array(const mat4d& mat);

/**
 * Point cloud with 3D coordinates, row-major layout (Nx3).
 * @tparam T Data type (e.g., float or double)
 */
template <typename T>
using PointCloudXYZ = MatrixX3R<T>;

/**
 * @brief 3D point cloud with single precision
 */
using PointCloudXYZf = PointCloudXYZ<float>;

/**
 * @brief 3D point cloud with double precision
 */
using PointCloudXYZd = PointCloudXYZ<double>;

// Backward compatibility (will trigger a deprecation warning)
template <typename T>
using PointsT [[deprecated("Use PointCloudXYZ instead of PointsT")]] =
    PointCloudXYZ<T>;
using PointsD [[deprecated("Use PointCloudXYZd instead of PointsD")]] =
    PointCloudXYZd;
using PointsF [[deprecated("Use PointCloudXYZf instead of PointsF")]] =
    PointCloudXYZf;

}  // namespace core
}  // namespace sdk
}  // namespace ouster
