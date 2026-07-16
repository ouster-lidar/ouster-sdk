#pragma once

#include "ouster/core/impl/transform_typedefs.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
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
class OUSTER_API_CLASS RotH : public Eigen::Matrix3d {
   public:
    OUSTER_API_FUNCTION RotH() = default;

    template <typename OtherDerived>
    OUSTER_API_FUNCTION RotH(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Matrix3d(m) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION RotH& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix3d::operator=(other);
        return *this;
    }

    template <typename OtherDerived>
    OUSTER_API_FUNCTION auto operator*(const Eigen::MatrixBase<OtherDerived>& p) const {
        return Eigen::Matrix3d::operator*(p);
    }

    OUSTER_API_FUNCTION
    RotH operator*(const RotH& other) const {
        return Eigen::Matrix3d::operator*(other);
    }

    OUSTER_API_FUNCTION
    RotH transpose() const {
        return RotH(Eigen::Matrix3d::transpose());
    }

    /**
     * Performs the logarithm of SO(3) to return an element of so(3).
     */
    OUSTER_API_FUNCTION
    RotV log() const;

    /**
     * Performs the logarithm of SO(3) to return an element of so(3), and
     * saves the angle and cos to avoid recomputing them later.
     */
    OUSTER_API_FUNCTION
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
class OUSTER_API_CLASS PoseH : public Matrix4dR {
   public:
    /**
     * A single homogeneous transformation.
     */
    OUSTER_API_FUNCTION PoseH() : Matrix4dR(Matrix4dR::Identity()) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION PoseH(const Eigen::MatrixBase<OtherDerived>& m) : Matrix4dR(m) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION PoseH& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Matrix4dR::operator=(other);
        return *this;
    }
    OUSTER_API_FUNCTION PoseH operator*(const PoseH& other) const {
        return PoseH(Matrix4dR::operator*(other));
    }

    OUSTER_API_FUNCTION Eigen::Vector3d operator*(const Eigen::Vector3d& p) const {
        return RotH(r()) * p + t();
    }

    OUSTER_API_FUNCTION RotH r() const {
        return RotH(topLeftCorner<3, 3>());
    }
    OUSTER_API_FUNCTION TransH t() const {
        return TransH(topRightCorner<3, 1>());
    }

    OUSTER_API_FUNCTION
    void set_rot(const RotH& rot_h) {
        topLeftCorner<3, 3>() = rot_h;
    }

    OUSTER_API_FUNCTION
    void set_trans(const TransH& trans_h) {
        topRightCorner<3, 1>() = trans_h;
    }

    OUSTER_API_FUNCTION
    void reorthogonalize();

    OUSTER_API_FUNCTION
    PoseV log() const;

    OUSTER_API_FUNCTION
    PoseV log(double& angle_out, double& sin_angle_out, double& cos_angle_out) const;
};

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
