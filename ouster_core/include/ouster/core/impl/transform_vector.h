#pragma once

#include <cmath>

#include "ouster/core/impl/transform_typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
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
class OUSTER_API_CLASS RotV : public Eigen::Vector3d {
   public:
    OUSTER_API_FUNCTION
    RotV() = default;

    OUSTER_API_FUNCTION
    RotV(const double x, const double y, const double z) : Eigen::Vector3d(x, y, z) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION RotV(const Eigen::MatrixBase<OtherDerived>& v) : Eigen::Vector3d(v) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION RotV& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector3d::operator=(other);
        return *this;
    }

    /*
     * Directly converts from axis-angle representation to so(3)
     */
    template <typename OtherDerived>
    OUSTER_API_FUNCTION RotV(const Eigen::AngleAxis<OtherDerived>& other)
        : Eigen::Vector3d(other.axis() * other.angle()) {}

    /*
     * Directly converts a quaternion to so(3)
     */
    template <typename OtherDerived>
    OUSTER_API_FUNCTION RotV(const Eigen::Quaternion<OtherDerived>& other)
        : RotV(Eigen::AngleAxis<OtherDerived>{other}) {}

    /**
     * returns the quaternion corresponding to this rotation
     */
    OUSTER_API_FUNCTION
    RotQ q() const;

    OUSTER_API_FUNCTION
    RotQ q(double& angle, double& sin_angle, double& cos_angle) const;

    /**
     * Performs the exponentiation of so(3) to return an element of SO(3).
     */
    OUSTER_API_FUNCTION
    RotH exp() const;
    /**
     * Performs the exponentiation of so(3) to return an element of SO(3), and
     * saves the angle, sin and cos to avoid recomputing them later.
     */
    OUSTER_API_FUNCTION
    RotH exp(double& angle_out, double& sin_angle_out, double& cos_angle_out) const;
    /**
     * The V matrix is used in the translational component of SE(3) <--> se(3).
     */
    OUSTER_API_FUNCTION
    Eigen::Matrix3d vee(const double angle, const double sin_angle, const double cos_angle) const;

    OUSTER_API_FUNCTION
    Eigen::Matrix3d vee() const {
        const double angle = norm();
        const double sin_angle = std::sin(angle);
        const double cos_angle = std::cos(angle);
        return vee(angle, sin_angle, cos_angle);
    }
};

class OUSTER_API_CLASS PoseV : public Eigen::Vector6d {
   public:
    OUSTER_API_FUNCTION
    PoseV();
    template <typename OtherDerived>
    OUSTER_API_FUNCTION PoseV(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Vector6d(m) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION PoseV& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector6d::operator=(other);
        return *this;
    }

    OUSTER_API_FUNCTION
    RotV r() const {
        return RotV(head<3>());
    }

    OUSTER_API_FUNCTION
    TransV t() const {
        return TransV(tail<3>());
    }

    OUSTER_API_FUNCTION
    PoseQ q() const;

    OUSTER_API_FUNCTION
    void set_rot(const RotV& rot_v) {
        head<3>() = rot_v;
    }

    OUSTER_API_FUNCTION
    void set_trans(const TransV& trans_v) {
        tail<3>() = trans_v;
    }

    OUSTER_API_FUNCTION
    PoseH exp() const;
};

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
