#include "ouster/impl/absolute_pose_constraint.h"

#include <ceres/ceres.h>

namespace ouster {
namespace mapping {
namespace impl {

AbsolutePoseConstraint::AbsolutePoseConstraint(
    const std::shared_ptr<Node> source_node,
    const std::shared_ptr<const Node>& target_node,
    const std::array<double, 3>& rotation_weights,
    const std::array<double, 3>& translation_weights)
    : node_source_(source_node),
      node_target_(target_node),
      diff_r_(1.0, 0.0, 0.0, 0.0),
      diff_t_(0.0, 0.0, 0.0),
      rotation_weights_(rotation_weights),
      translation_weights_(translation_weights) {
    // Ensure the size of the weight vectors is correct (e.g., 3).
    if (rotation_weights_.size() != 3) {
        throw std::runtime_error(
            "rotation_weights_ must have exactly 3 elements.");
    }
    if (translation_weights_.size() != 3) {
        throw std::runtime_error(
            "translation_weights_ must have exactly 3 elements.");
    }
}

AbsolutePoseConstraint::AbsolutePoseConstraint(
    const std::shared_ptr<Node> source_node,
    const std::shared_ptr<const Node>& target_node,
    const Eigen::Quaterniond& diff_r, const Eigen::Vector3d& diff_t,
    const std::array<double, 3>& rotation_weights,
    const std::array<double, 3>& translation_weights)
    : node_source_(source_node),
      node_target_(target_node),
      diff_r_(diff_r),
      diff_t_(diff_t),
      rotation_weights_(rotation_weights),
      translation_weights_(translation_weights) {
    // Ensure the size of the weight vectors is correct (e.g., 3).
    if (rotation_weights_.size() != 3) {
        throw std::runtime_error(
            "rotation_weights_ must have exactly 3 elements.");
    }
    if (translation_weights_.size() != 3) {
        throw std::runtime_error(
            "translation_weights_ must have exactly 3 elements.");
    }
}

ceres::CostFunction* AbsolutePoseConstraint::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<AbsolutePoseConstraint, 6, 4, 3>(
        new AbsolutePoseConstraint(node_source_, node_target_, diff_r_, diff_t_,
                                   rotation_weights_, translation_weights_));
}

template <typename T>
bool AbsolutePoseConstraint::operator()(const T* const rotation,
                                        const T* const translation,
                                        T* residual) const {
    Eigen::Quaternion<T> q_current(rotation[3], rotation[0], rotation[1],
                                   rotation[2]);
    Eigen::Matrix<T, 3, 1> t_current(translation[0], translation[1],
                                     translation[2]);

    Eigen::Quaternion<T> q_target = node_target_->rotation.template cast<T>();
    q_target.normalize();
    Eigen::Matrix<T, 3, 1> t_target(T(node_target_->position.x()),
                                    T(node_target_->position.y()),
                                    T(node_target_->position.z()));

    Eigen::Quaternion<T> q_diff(T(this->diff_r_.w()), T(this->diff_r_.x()),
                                T(this->diff_r_.y()), T(this->diff_r_.z()));
    q_diff.normalize();
    Eigen::Matrix<T, 3, 1> t_diff(T(this->diff_t_.x()), T(this->diff_t_.y()),
                                  T(this->diff_t_.z()));

    Eigen::Quaternion<T> q_target_final = q_target * q_diff;
    Eigen::Matrix<T, 3, 1> t_target_final = t_target + q_target * t_diff;

    Eigen::Quaternion<T> error_r = q_current.conjugate() * q_target_final;
    Eigen::AngleAxis<T> angle_axis(error_r);
    Eigen::Matrix<T, 3, 1> rotation_residual =
        angle_axis.axis() * angle_axis.angle();

    Eigen::Matrix<T, 3, 1> translation_residual = t_current - t_target_final;

    for (int i = 0; i < 3; ++i) {
        residual[i] = T(rotation_weights_[i]) * rotation_residual[i];
        residual[i + 3] = T(translation_weights_[i]) * translation_residual[i];
    }
    return true;
}

void AbsolutePoseConstraint::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    problem.AddResidualBlock(this->create_cost_function(), loss_func,
                             node_source_->rotation.coeffs().data(),
                             node_source_->position.data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace ouster
