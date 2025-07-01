#include "ouster/impl/transform_vector.h"

#include "ouster/impl/transform_homogeneous.h"
#include "ouster/impl/transform_quaternion.h"

namespace ouster {
namespace impl {

inline Eigen::Matrix3d skewSymmetric(Eigen::Vector3d v) {
    // the skew-symmetric matrix representation of cross product with v
    Eigen::Matrix3d ss;
    // clang-format off
    ss <<    0, -v(2),  v(1),
           v(2),     0, -v(0),
          -v(1),  v(0),     0;
    // clang-format on
    return ss;
}

RotQ RotV::q() const {
    double unused1 = 0.0, unused2 = 0.0, unused3 = 0.0;
    return q(unused1, unused2, unused3);
}
RotH RotV::exp() const {
    double unused1 = 0.0, unused2 = 0.0, unused3 = 0.0;
    return exp(unused1, unused2, unused3);
}

RotH RotV::exp(double& angle_out, double& sin_angle_out,
               double& cos_angle_out) const {
    angle_out = norm();
    sin_angle_out = std::sin(angle_out);
    cos_angle_out = std::cos(angle_out);
    if (angle_out < NUMERIC_EPS) {
        return RotH(Eigen::Matrix3d::Identity() + skewSymmetric(*this));
    }
    Eigen::Matrix3d axis_hat = skewSymmetric(operator/(angle_out));
    return RotH(Eigen::Matrix3d::Identity() + sin_angle_out * axis_hat +
                (1 - cos_angle_out) * axis_hat * axis_hat);
}

Eigen::Matrix3d RotV::vee(const double angle, const double sin_angle,
                          const double cos_angle) const {
    if (angle < EPS) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix3d axis_hat = skewSymmetric(operator/(angle));
    return Eigen::Matrix3d::Identity() + (1.0 - cos_angle) * axis_hat / angle +
           (angle - sin_angle) * axis_hat * axis_hat / angle;
}

RotQ RotV::q(double& angle, double& sin_angle, double& cos_angle) const {
    angle = norm();
    double sin_angle_2 = std::sin(angle / 2);
    double cos_angle_2 = std::cos(angle / 2);
    sin_angle = 2 * sin_angle_2 * cos_angle_2;
    cos_angle = cos_angle_2 * cos_angle_2 - sin_angle_2 * sin_angle_2;
    RotQ q(1, 0, 0, 0);
    if (std::abs(angle) > EPS) {
        q = RotQ(cos_angle_2, sin_angle_2 * operator()(0) / angle,
                 sin_angle_2 * operator()(1) / angle,
                 sin_angle_2 * operator()(2) / angle);
    }
    return q;
}

PoseQ PoseV::q() const {
    PoseQ ret;
    double angle = 0.0, sin_angle = 0.0, cos_angle = 0.0;
    RotQ q = r().q(angle, sin_angle, cos_angle);
    Eigen::Vector3d t_h = r().vee(angle, sin_angle, cos_angle) * t();
    ret.set_rot(q);
    ret.set_trans(t_h);
    return ret;
}

/**
 * Default constructor for PoseV.
 */
PoseV::PoseV() : Eigen::Vector6d(Eigen::Vector6d::Zero()) {}

/**
 * Converts PoseV from vector format to homogeneous format.
 */
PoseH PoseV::exp() const {
    double angle;
    double sin_angle;
    double cos_angle;
    PoseH h;
    h.set_rot(r().exp(angle, sin_angle, cos_angle));
    h.set_trans(r().vee(angle, sin_angle, cos_angle) * t());
    return h;
}

}  // namespace impl
}  // namespace ouster
