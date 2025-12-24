#include <Eigen/Dense>

#include "ouster/typedefs.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief Converts a pose given in Euler angles and translation to a homogeneous
 * transformation matrix. The rotation is applied in the order of yaw, pitch,
 * roll.
 *
 * The pose vector should contain the following elements in this order:
 * - roll (rotation along X-axis in radians)
 * - pitch (rotation along Y-axis in radians)
 * - yaw (rotation along Z-axis in radians)
 * - x (translation along X-axis)
 * - y (translation along Y-axis)
 * - z (translation along Z-axis)
 *
 * @param[in] pose A 6x1 vector containing the pose information.
 * @return Eigen::Matrix4d The 4x4 homogeneous transformation matrix.
 */
OUSTER_API_FUNCTION
Eigen::Matrix4d euler_pose_to_matrix(const Eigen::Matrix<double, 6, 1>& pose);

/**
 * @brief Converts a pose given in quaternion and translation to a homogeneous
 * transformation matrix.
 *
 * The pose vector should contain the following elements in this order:
 * - qw (quaternion w component)
 * - qx (quaternion x component)
 * - qy (quaternion y component)
 * - qz (quaternion z component)
 * - x (translation along X-axis)
 * - y (translation along Y-axis)
 * - z (translation along Z-axis)
 *
 * @param[in] pose A 7x1 vector containing the pose information.
 * @return Eigen::Matrix4d The 4x4 homogeneous transformation matrix.
 */
OUSTER_API_FUNCTION
Eigen::Matrix4d quaternion_pose_to_matrix(
    const Eigen::Matrix<double, 7, 1>& pose);

/**
 * Convert Euler angles (roll, pitch, yaw) to a 3D rotation matrix.
 *
 * Parameters:
 *     roll   : Rotation about the x-axis (rad)
 *     pitch  : Rotation about the y-axis (rad)
 *     yaw    : Rotation about the z-axis (rad)
 *
 * Returns:
 *     R : 3x3 rotation matrix
 */
Matrix3dR euler_to_rotation_matrix(double roll, double pitch, double yaw);

/**
 * Convert a 3x3 rotation matrix to Euler angles (roll, pitch, yaw).
 *
 * Parameters:
 *     matrix : 3x3 rotation matrix
 *
 * Returns:
 *     Tuple of (roll, pitch, yaw) angles in radians
 *     roll   : Rotation about the x-axis (rad)
 *     pitch  : Rotation about the y-axis (rad)
 *     yaw    : Rotation about the z-axis (rad)
 */
Eigen::Vector3d matrix_to_euler(const Matrix3dR& matrix);

/**
 * Convert position and Euler angles to a 4x4 transformation matrix.
 *
 * Parameters:
 *     px, py, pz : Position coordinates
 *     r, p, y    : Euler angles (roll, pitch, yaw) in radians
 *
 * Returns:
 *     4x4 transformation matrix
 */
Matrix4dR xyzrpy_to_matrix(double px, double py, double pz, double r, double p,
                           double y);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
