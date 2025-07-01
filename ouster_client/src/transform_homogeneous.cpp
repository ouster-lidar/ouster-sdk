#include "ouster/impl/transform_homogeneous.h"

#include "ouster/impl/transform_vector.h"

namespace ouster {
namespace impl {

RotV RotH::log() const {
    double unused1, unused2;
    return log(unused1, unused2);
}

PoseV PoseH::log() const {
    double unused1, unused2, unused3;
    return log(unused1, unused2, unused3);
}

/**
 * Converting homogeneous rotation into axis angle representation.
 * We may sometimes need to save some intermediate things,
 * such as sin(angle), cos(angle), etc.
 */
RotV RotH::log(double& angle_out, double& cos_angle_out) const {
    cos_angle_out =
        0.5 * (operator()(0, 0) + operator()(1, 1) + operator()(2, 2) - 1.0);
    cos_angle_out = std::max(cos_angle_out, -1.0 + EPS);
    cos_angle_out = std::min(cos_angle_out, 1.0 - EPS);
    angle_out = std::acos(cos_angle_out);
    RotV rot_v(operator()(2, 1) - operator()(1, 2),
               operator()(0, 2) - operator()(2, 0),
               operator()(1, 0) - operator()(0, 1));

    if (rot_v.squaredNorm() > EPS) {
        rot_v.normalize();
        rot_v *= angle_out;
    } else {
        rot_v /= 2;
    }
    return rot_v;
}

/**
 * Ensures rotational component is an orthogonal matrix.
 */
void PoseH::reorthogonalize() {
    RotH reorthogonalized = r().log().exp();
    set_rot(reorthogonalized);
}

PoseV PoseH::log(double& angle_out, double& sin_angle_out,
                 double& cos_angle_out) const {
    PoseV v;
    v.set_rot(r().log(angle_out, cos_angle_out));
    sin_angle_out = std::sin(angle_out);
    v.set_trans(v.r().vee(angle_out, sin_angle_out, cos_angle_out).inverse() *
                t());
    return v;
}

}  // namespace impl
}  // namespace ouster
