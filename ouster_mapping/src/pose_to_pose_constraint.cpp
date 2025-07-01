#include "ouster/impl/pose_to_pose_constraint.h"

#include <ceres/ceres.h>

#include "ouster/impl/utils.h"

namespace ouster {
namespace mapping {
namespace impl {

template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<T>& quaternion) {
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    if (normalized_quaternion.w() < 0.) {
        normalized_quaternion.w() = -1. * normalized_quaternion.w();
        normalized_quaternion.x() = -1. * normalized_quaternion.x();
        normalized_quaternion.y() = -1. * normalized_quaternion.y();
        normalized_quaternion.z() = -1. * normalized_quaternion.z();
    }
    const T angle = 2. * atan2(normalized_quaternion.vec().norm(),
                               normalized_quaternion.w());
    constexpr double kCutoffAngle = 1e-7;
    const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / 2.);
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(),
                                  scale * normalized_quaternion.z());
}

template <typename T>
std::array<T, 6> ScaleError(const std::array<T, 6>& error,
                            double rotation_weight, double translation_weight) {
    return {{error[0] * rotation_weight, error[1] * rotation_weight,
             error[2] * rotation_weight, error[3] * translation_weight,
             error[4] * translation_weight, error[5] * translation_weight}};
}

template <typename T>
std::array<T, 6> ComputeUnscaledError(const Eigen::Quaternion<double>& qua,
                                      const Eigen::Matrix<double, 3, 1>& pos,
                                      const T* const start_rotation,
                                      const T* const start_translation,
                                      const T* const end_rotation,
                                      const T* const end_translation) {
    const Eigen::Quaternion<T> R_i_inverse(
        start_rotation[3], -start_rotation[0], -start_rotation[1],
        -start_rotation[2]);

    const Eigen::Matrix<T, 3, 1> delta(
        end_translation[0] - start_translation[0],
        end_translation[1] - start_translation[1],
        end_translation[2] - start_translation[2]);
    const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

    const Eigen::Quaternion<T> h_rotation_inverse =
        Eigen::Quaternion<T>(end_rotation[3], -end_rotation[0],
                             -end_rotation[1], -end_rotation[2]) *
        Eigen::Quaternion<T>(start_rotation[3], start_rotation[0],
                             start_rotation[1], start_rotation[2]);

    const Eigen::Matrix<T, 3, 1> angle_axis_difference =
        RotationQuaternionToAngleAxisVector(h_rotation_inverse * qua.cast<T>());

    return {{angle_axis_difference[0], angle_axis_difference[1],
             angle_axis_difference[2], T(pos.x()) - h_translation[0],
             T(pos.y()) - h_translation[1], T(pos.z()) - h_translation[2]}};
}

template <typename T>
bool PoseToPoseConstraint::operator()(const T* const c_i_rotation,
                                      const T* const c_i_translation,
                                      const T* const c_j_rotation,
                                      const T* const c_j_translation,
                                      T* residual) const {
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(diff_r_, diff_t_, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation),
        rotation_weight_, translation_weight_);
    std::copy(std::begin(error), std::end(error), residual);
    return true;
}

PoseToPoseConstraint::PoseToPoseConstraint(const std::shared_ptr<Node> node1,
                                           const std::shared_ptr<Node>& node2,
                                           const Eigen::Quaterniond& diff_r,
                                           const Eigen::Vector3d& diff_t,
                                           double rotation_weight,
                                           double translation_weight)
    : node1_(node1),
      node2_(node2),
      diff_r_(diff_r),
      diff_t_(diff_t),
      rotation_weight_(rotation_weight),
      translation_weight_(translation_weight) {}

PoseToPoseConstraint::PoseToPoseConstraint(const std::shared_ptr<Node> node1,
                                           const std::shared_ptr<Node>& node2,
                                           const ouster::impl::PoseH& diff,
                                           double rotation_weight,
                                           double translation_weight)
    : node1_(node1),
      node2_(node2),
      rotation_weight_(rotation_weight),
      translation_weight_(translation_weight) {
    ouster::impl::PoseQ diff_q = diff.log().q();
    diff_r_ = diff_q.r();
    diff_t_ = diff_q.t();
}

ceres::CostFunction* PoseToPoseConstraint::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<PoseToPoseConstraint, 6, 4, 3, 4, 3>(
        new PoseToPoseConstraint(node1_, node2_, diff_r_, diff_t_,
                                 rotation_weight_, translation_weight_));
}

void PoseToPoseConstraint::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    problem.AddResidualBlock(
        this->create_cost_function(), loss_func,
        node1_->rotation.coeffs().data(), node1_->position.data(),
        node2_->rotation.coeffs().data(), node2_->position.data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace ouster
