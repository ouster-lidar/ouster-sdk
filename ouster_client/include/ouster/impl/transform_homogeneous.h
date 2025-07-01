#pragma once

#include "ouster/impl/transform_typedefs.h"

namespace ouster {
namespace impl {

class RotV;
class RotH;
class PoseV;

/**
 *
 * RotH represents an element of SO(3), the special orthogonal group.
 * In other words, it's a 3-by-3 rotation matrix.
 * An element of SO(3) has the following properties:
 *   1. Its columns are orthonormal.
 *   2. It has a determinant of +1.
 */
class RotH : public Eigen::Matrix3d {
   public:
    RotH() {}

    template <typename OtherDerived>
    RotH(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Matrix3d(m) {}

    template <typename OtherDerived>
    RotH& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix3d::operator=(other);
        return *this;
    }
    template <typename OtherDerived>
    auto operator*(const Eigen::MatrixBase<OtherDerived>& p) const {
        return Eigen::Matrix3d::operator*(p);
    }
    RotH operator*(const RotH& other) const {
        return Eigen::Matrix3d::operator*(other);
    }
    RotH transpose() const { return RotH(Eigen::Matrix3d::transpose()); }
    /**
     * Performs the logarithm of SO(3) to return an element of so(3).
     */
    RotV log() const;

    /**
     * Performs the logarithm of SO(3) to return an element of so(3), and
     * saves the angle and cos to avoid recomputing them later.
     */
    RotV log(double& angle_out, double& cos_angle_out) const;
};

/**
 * PoseH represents an element in SE(3), the special Euclidean group.
 * The 4-by-4 matrix has the following properties:
 *   1. The top-left 3-by-3 submatrix is an element of SO(3), i.e.
 *      the rotational component of the rigid transformation.
 *   2. The top-right 3-by-1 submatrix represents the translation.
 *   3. The bottom row is 0 0 0 1.
 */
class PoseH : public Eigen::Matrix4d {
   public:
    /**
     * A single homogeneous transformation.
     */
    PoseH() : Eigen::Matrix4d(Eigen::Matrix4d::Identity()) {}

    template <typename OtherDerived>
    PoseH(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Matrix4d(m) {}

    template <typename OtherDerived>
    PoseH& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix4d::operator=(other);
        return *this;
    }
    PoseH operator*(const PoseH& other) const {
        return PoseH(Eigen::Matrix4d::operator*(other));
    }

    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const {
        return RotH(r()) * p + t();
    }

    RotH r() const { return RotH(topLeftCorner<3, 3>()); }
    TransH t() const { return TransH(topRightCorner<3, 1>()); }

    void set_rot(const RotH& rot_h) { topLeftCorner<3, 3>() = rot_h; }

    void set_trans(const TransH& trans_h) { topRightCorner<3, 1>() = trans_h; }

    void reorthogonalize();
    PoseV log() const;

    PoseV log(double& angle_out, double& sin_angle_out,
              double& cos_angle_out) const;
};

}  // namespace impl
}  // namespace ouster
