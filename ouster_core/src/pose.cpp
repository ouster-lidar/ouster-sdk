/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/pose.h"

#include "ouster/core/pose_conversion.h"
#include "ouster/core/typedefs.h"

namespace ouster {
namespace sdk {
namespace core {

Pose::Pose() = default;

Pose::Pose(const Matrix4dR& matrix)
    : position_(matrix.topRightCorner<3, 1>()),
      rotation_(Eigen::Quaterniond(matrix.topLeftCorner<3, 3>())) {
    rotation_.normalize();
}

Pose::Pose(Eigen::Vector3d position, Eigen::Quaterniond rotation)
    : position_(std::move(position)), rotation_(std::move(rotation)) {
    rotation_.normalize();
}

const Eigen::Vector3d& Pose::position() const {
    return position_;
}

void Pose::set_position(const Eigen::Vector3d& position) {
    position_ = position;
}

const Eigen::Quaterniond& Pose::rotation() const {
    return rotation_;
}

void Pose::set_rotation(const Eigen::Quaterniond& rotation) {
    rotation_ = rotation;
    rotation_.normalize();
}

Matrix4dR Pose::to_matrix() const {
    Matrix4dR transform = Matrix4dR::Identity();
    transform.block<3, 3>(0, 0) = rotation_.toRotationMatrix();
    transform.block<3, 1>(0, 3) = position_;
    return transform;
}

Eigen::Vector3d Pose::euler_angles() const {
    Matrix3dR rot = rotation_.toRotationMatrix();
    return matrix_to_euler(rot);
}

void Pose::set_rotation(const Matrix3dR& rotation_matrix) {
    rotation_ = Eigen::Quaterniond(rotation_matrix);
    rotation_.normalize();
}

void Pose::set_rotation(const Eigen::Vector3d& euler_angles) {
    rotation_ = Eigen::Quaterniond(
        euler_to_rotation_matrix(euler_angles(0), euler_angles(1), euler_angles(2)));
    rotation_.normalize();
}

bool Pose::operator==(const Pose& other) const {
    return to_matrix().isApprox(other.to_matrix());
}

Pose Pose::inverse() const {
    const Eigen::Quaterniond inv_rotation = rotation_.conjugate();
    return Pose{inv_rotation * -position_, inv_rotation};
}

Pose Pose::operator*(const Pose& other) const {
    return {rotation_ * other.position_ + position_, rotation_ * other.rotation_};
}

Eigen::Vector3d Pose::operator*(const Eigen::Vector3d& point) const {
    return rotation_ * point + position_;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
