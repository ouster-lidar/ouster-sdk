#include <Eigen/Dense>

#include "ouster/visibility.h"

namespace ouster {
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

}  // namespace core
}  // namespace ouster
