#pragma once

#include "ouster/core/impl/transform_typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

class PoseV;
using TransQ = Eigen::Vector3d;

class OUSTER_API_CLASS PoseQ : public Eigen::Vector7d {
   public:
    /**
     * A single vector transformation.
     */

    OUSTER_API_FUNCTION
    PoseQ() : Eigen::Vector7d(Eigen::Vector7d::Zero()) {
        operator()(3) = 1;
    }
    template <typename OtherDerived>
    OUSTER_API_FUNCTION PoseQ(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Vector7d(m) {}

    template <typename OtherDerived>
    OUSTER_API_FUNCTION PoseQ& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector7d::operator=(other);
        return *this;
    }

    OUSTER_API_FUNCTION
    RotQ r() const {
        return RotQ(head<4>());
    }

    OUSTER_API_FUNCTION
    TransQ t() const {
        return TransQ(tail<3>());
    }

    OUSTER_API_FUNCTION
    void set_rot(const Eigen::Vector4d& rot_v) {
        head<4>() = rot_v.array().transpose();
    }

    OUSTER_API_FUNCTION
    void set_rot(const RotQ& rot_v) {
        set_rot(rot_v.coeffs());
    }

    OUSTER_API_FUNCTION
    void set_trans(const TransQ& trans_v) {
        tail<3>() = trans_v.array().transpose();
    }

    OUSTER_API_FUNCTION
    PoseV v() const;
};

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
