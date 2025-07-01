#pragma once

#include "ouster/impl/transform_typedefs.h"

namespace ouster {
namespace impl {

class PoseV;
typedef Eigen::Vector3d TransQ;

class PoseQ : public Eigen::Vector7d {
   public:
    /**
     * A single vector transformation.
     */
    PoseQ() : Eigen::Vector7d(Eigen::Vector7d::Zero()) { operator()(3) = 1; }
    template <typename OtherDerived>
    PoseQ(const Eigen::MatrixBase<OtherDerived>& m) : Eigen::Vector7d(m) {}

    template <typename OtherDerived>
    PoseQ& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Vector7d::operator=(other);
        return *this;
    }
    RotQ r() const { return RotQ(head<4>()); }
    TransQ t() const { return TransQ(tail<3>()); }
    void set_rot(const Eigen::Vector4d& rot_v) {
        head<4>() = rot_v.array().transpose();
    }
    void set_rot(const RotQ& rot_v) { set_rot(rot_v.coeffs()); }
    void set_trans(const TransQ& trans_v) {
        tail<3>() = trans_v.array().transpose();
    }
    PoseV v() const;
};

}  // namespace impl
}  // namespace ouster
