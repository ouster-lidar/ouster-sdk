#include "ouster/impl/transform_quaternion.h"

#include "ouster/impl/transform_vector.h"

namespace ouster {
namespace impl {

PoseV PoseQ::v() const {
    PoseV ret;
    ret.set_rot(RotV(r()));
    ret.set_trans(ret.r().vee().inverse() * t());
    return ret;
}

}  // namespace impl
}  // namespace ouster
