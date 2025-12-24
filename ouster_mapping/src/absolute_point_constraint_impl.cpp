#include "ouster/impl/absolute_point_constraint_impl.h"

#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>

#include <memory>

#include "ouster/pose_optimizer_node.h"

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

AbsolutePointConstraintImpl::AbsolutePointConstraintImpl(
    const std::shared_ptr<Node> source_node,
    const Eigen::Vector3d& absolute_position,
    const Eigen::Array3d& translation_weights)
    : source_node_(source_node),
      absolute_position_(absolute_position),
      translation_weights_(translation_weights) {}

template <typename T>
bool AbsolutePointConstraintImpl::operator()(const T* const rotation,
                                             const T* const translation,
                                             T* residual) const {
    // Use the absolute point constraint selected point
    Eigen::Vector3d local_point = source_node_->ap_constraint_pt.row(0);

    Eigen::Vector3<T> local_point_v = local_point.cast<T>();

    Eigen::Quaternion<T> q(rotation[3], rotation[0], rotation[1], rotation[2]);
    q.normalize();

    Eigen::Vector3<T> global_translation(translation[0], translation[1],
                                         translation[2]);

    Eigen::Vector3<T> transformed_point =
        q * local_point_v + global_translation;

    Eigen::Vector3<T> target_position = absolute_position_.cast<T>();

    Eigen::Array<T, 3, 1> error = (transformed_point - target_position).array();

    Eigen::Array<T, 3, 1> weighted_error =
        error * translation_weights_.cast<T>();

    residual[0] = weighted_error[0];
    residual[1] = weighted_error[1];
    residual[2] = weighted_error[2];

    return true;
}

ceres::CostFunction* AbsolutePointConstraintImpl::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<AbsolutePointConstraintImpl, 3, 4,
                                           3>(new AbsolutePointConstraintImpl(
        source_node_, absolute_position_, translation_weights_));
}

ceres::ResidualBlockId AbsolutePointConstraintImpl::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    return problem.AddResidualBlock(this->create_cost_function(), loss_func,
                                    source_node_->rotation.coeffs().data(),
                                    source_node_->position.data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
