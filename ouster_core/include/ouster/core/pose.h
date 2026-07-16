/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief A rigid-body pose represented by translation and rotation.
 */
class OUSTER_API_CLASS Pose {
   public:
    /**
     * @brief Constructs an identity pose with zero translation.
     */
    OUSTER_API_FUNCTION
    Pose();

    /**
     * @brief Constructs a pose from a 4x4 homogeneous transformation matrix.
     *
     * @param[in] matrix Matrix4dR with rotation in the upper-left 3x3 block
     * and translation in the upper-right 3x1 block.
     */
    OUSTER_API_FUNCTION
    explicit Pose(const Matrix4dR& matrix);

    /**
     * @brief Constructs a pose from translation and rotation.
     *
     * @param[in] position Translation vector (x, y, z).
     * @param[in] rotation Orientation as a unit quaternion.
     */
    OUSTER_API_FUNCTION
    Pose(Eigen::Vector3d position, Eigen::Quaterniond rotation);

    /**
     * @brief Returns the translation component.
     *
     * @return position vector
     */
    OUSTER_API_FUNCTION
    const Eigen::Vector3d& position() const;

    /**
     * @brief Sets the translation component.
     *
     * @param[in] position Translation vector (x, y, z).
     */
    OUSTER_API_FUNCTION
    void set_position(const Eigen::Vector3d& position);

    /**
     * @brief Returns the orientation as a unit quaternion.
     *
     * @return rotation quaternion
     */
    OUSTER_API_FUNCTION
    const Eigen::Quaterniond& rotation() const;

    /**
     * @brief Sets the orientation from a unit quaternion.
     *
     * @param[in] rotation Orientation quaternion (normalized on assignment).
     */
    OUSTER_API_FUNCTION
    void set_rotation(const Eigen::Quaterniond& rotation);

    /**
     * @brief Converts this pose to a 4x4 homogeneous transformation matrix.
     *
     * @return Matrix4dR with rotation in the upper-left 3x3 block and
     * translation in the upper-right 3x1 block.
     */
    OUSTER_API_FUNCTION
    Matrix4dR to_matrix() const;

    /**
     * @brief Returns fixed-axis Euler angles (roll, pitch, yaw) in radians.
     *
     * @return Eigen::Vector3d with roll (x), pitch (y), and yaw (z).
     */
    OUSTER_API_FUNCTION
    Eigen::Vector3d euler_angles() const;

    /**
     * @brief Sets rotation from a 3x3 rotation matrix.
     *
     * @param[in] rotation_matrix 3x3 rotation matrix.
     */
    OUSTER_API_FUNCTION
    void set_rotation(const Matrix3dR& rotation_matrix);

    /**
     * @brief Sets rotation from fixed-axis Euler angles (roll, pitch, yaw).
     *
     * @param[in] euler_angles Vector with roll (x), pitch (y), and yaw (z) in
     * radians.
     */
    OUSTER_API_FUNCTION
    void set_rotation(const Eigen::Vector3d& euler_angles);

    /**
     * @brief Returns the inverse of this pose.
     *
     * @return Pose whose homogeneous matrix is the inverse of this pose's
     * matrix.
     */
    OUSTER_API_FUNCTION
    Pose inverse() const;

    /**
     * @brief Composes two poses.
     *
     * Returns the pose whose homogeneous matrix is the product of this pose's
     * matrix and @p other 's matrix (apply @p other first, then this pose).
     *
     * @param[in] other The pose to compose with.
     * @return The composed pose.
     */
    OUSTER_API_FUNCTION
    Pose operator*(const Pose& other) const;

    /**
     * @brief Compares two poses for equality.
     *
     * Poses are equal when their homogeneous transformation matrices match
     * within machine precision (quaternion sign is ignored).
     */
    OUSTER_API_FUNCTION
    bool operator==(const Pose& other) const;

    /**
     * @brief Transforms a 3D point by this pose.
     *
     * @param[in] point Point in the pose's source frame.
     * @return Transformed point in the target frame.
     */
    OUSTER_API_FUNCTION
    Eigen::Vector3d operator*(const Eigen::Vector3d& point) const;

   private:
    Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation_{Eigen::Quaterniond::Identity()};
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
