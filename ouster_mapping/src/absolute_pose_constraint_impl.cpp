#include "ouster/mapping/impl/absolute_pose_constraint_impl.h"

#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>

#include <memory>
#include <stdexcept>

#include "ouster/mapping/pose_optimizer_node.h"

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

AbsolutePoseConstraintImpl::AbsolutePoseConstraintImpl(
    const std::shared_ptr<Node>& source_node, const std::shared_ptr<const Node>& target_node,
    double rotation_weight, Eigen::Array3d translation_weights)
    : source_node_(source_node),
      target_node_(target_node),
      diff_r_(1.0, 0.0, 0.0, 0.0),
      diff_t_(0.0, 0.0, 0.0),
      rotation_weight_(rotation_weight),
      translation_weights_(std::move(translation_weights)) {}

AbsolutePoseConstraintImpl::AbsolutePoseConstraintImpl(
    const std::shared_ptr<Node>& source_node, const std::shared_ptr<const Node>& target_node,
    Eigen::Quaterniond diff_r, Eigen::Vector3d diff_t, double rotation_weight,
    Eigen::Array3d translation_weights)
    : source_node_(source_node),
      target_node_(target_node),
      diff_r_(std::move(diff_r)),
      diff_t_(std::move(diff_t)),
      rotation_weight_(rotation_weight),
      translation_weights_(std::move(translation_weights)) {}

ceres::CostFunction* AbsolutePoseConstraintImpl::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<AbsolutePoseConstraintImpl, 6, 4, 3>(
        new AbsolutePoseConstraintImpl(source_node_, target_node_, diff_r_, diff_t_,
                                       rotation_weight_, translation_weights_));
}

template <typename T>
bool AbsolutePoseConstraintImpl::operator()(const T* const rotation, const T* const translation,
                                            T* residual) const {
    Eigen::Quaternion<T> q_current(rotation[3], rotation[0], rotation[1], rotation[2]);
    Eigen::Vector3<T> t_current(translation[0], translation[1], translation[2]);

    Eigen::Quaternion<T> q_target = target_node_->rotation().template cast<T>();
    q_target.normalize();
    const auto& target_position = target_node_->position();
    Eigen::Vector3<T> t_target(static_cast<T>(target_position.x()),
                               static_cast<T>(target_position.y()),
                               static_cast<T>(target_position.z()));

    Eigen::Quaternion<T> q_diff(
        static_cast<T>(this->diff_r_.w()), static_cast<T>(this->diff_r_.x()),
        static_cast<T>(this->diff_r_.y()), static_cast<T>(this->diff_r_.z()));
    q_diff.normalize();
    Eigen::Vector3<T> t_diff(static_cast<T>(this->diff_t_.x()), static_cast<T>(this->diff_t_.y()),
                             static_cast<T>(this->diff_t_.z()));

    const Eigen::Quaternion<T> q_target_final = q_target * q_diff;
    Eigen::Vector3<T> t_target_final = t_target + (q_target * t_diff);

    const Eigen::Quaternion<T> error_r = q_current.conjugate() * q_target_final;
    const Eigen::AngleAxis<T> angle_axis(error_r);
    const Eigen::Vector3<T> rotation_residual = angle_axis.axis() * angle_axis.angle();

    Eigen::Vector3<T> translation_residual = t_current - t_target_final;

    for (int i = 0; i < 3; ++i) {
        residual[i] = static_cast<T>(rotation_weight_) * rotation_residual[i];
        residual[i + 3] = static_cast<T>(translation_weights_[i]) * translation_residual[i];
    }
    return true;
}

ceres::ResidualBlockId AbsolutePoseConstraintImpl::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    return problem.AddResidualBlock(this->create_cost_function(), loss_func,
                                    source_node_->mutable_rotation_coeffs_data(),
                                    source_node_->mutable_position_data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
