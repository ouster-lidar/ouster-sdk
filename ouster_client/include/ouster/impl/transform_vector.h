#pragma once

#include "ouster/impl/transform_typedefs.h"

namespace ouster {
namespace impl {

class RotH;
class PoseQ;
class PoseH;

/**
 * RotV represents an element of so(3), the Lie algebra associated with SO(3).
 * An element of so(3) can be thought of as the axis-angle representation of a
 * 3D rotation.
 * The direction of the vector is the axis and the norm is the angle.
 */
class RotV : public Eigen::Vector3d {
   public:
    RotV() {}
    RotV(const double x, const double y, const double z)
        : Eigen::Vector3d(x, y, z) {}

    template <typename OtherDerived>
    RotV(const Eigen::MatrixBase<OtherDerived>& v) : Eigen::Vector3d(v) {}

    template <typename OtherDerived>
    RotV& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector3d::operator=(other);
        return *this;
    }

    /*
     * Directly converts from axis-angle representation to so(3)
     */
    template <typename OtherDerived>
    RotV(const Eigen::AngleAxis<OtherDerived>& other)
        : Eigen::Vector3d(other.axis() * other.angle()) {}

    /*
     * Directly converts a quaternion to so(3)
     */
    template <typename OtherDerived>
    RotV(const Eigen::Quaternion<OtherDerived>& other)
        : RotV(Eigen::AngleAxis<OtherDerived>{other}) {}

    /**
     * returns the quaternion corresponding to this rotation
     */
    RotQ q() const;

    RotQ q(double& angle, double& sin_angle, double& cos_angle) const;

    /**
     * Performs the exponentiation of so(3) to return an element of SO(3).
     */
    RotH exp() const;
    /**
     * Performs the exponentiation of so(3) to return an element of SO(3), and
     * saves the angle, sin and cos to avoid recomputing them later.
     */
    RotH exp(double& angle_out, double& sin_angle_out,
             double& cos_angle_out) const;
    /**
     * The V matrix is used in the translational component of SE(3) <--> se(3).
     */
    Eigen::Matrix3d vee(const double angle, const double sin_angle,
                        const double cos_angle) const;
    Eigen::Matrix3d vee() const {
        const double angle = norm();
        const double sin_angle = std::sin(angle);
        const double cos_angle = std::cos(angle);
        return vee(angle, sin_angle, cos_angle);
    }
};

class PoseV : public Eigen::Vector6d {
   public:
    PoseV();
    template <typename OtherDerived>
    PoseV(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Vector6d(m) {}

    template <typename OtherDerived>
    PoseV& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector6d::operator=(other);
        return *this;
    }

    RotV r() const { return RotV(head<3>()); }
    TransV t() const { return TransV(tail<3>()); }

    PoseQ q() const;

    void set_rot(const RotV& rot_v) { head<3>() = rot_v; }
    void set_trans(const TransV& trans_v) { tail<3>() = trans_v; }

    PoseH exp() const;
};

}  // namespace impl
}  // namespace ouster
