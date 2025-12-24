#include "ouster/impl/point_to_point_constraint_impl.h"

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
PointToPointConstraintImpl::PointToPointConstraintImpl(
    const std::shared_ptr<Node> node1, const std::shared_ptr<Node> node2,
    const Eigen::Array3d& translation_weights)
    : node1_(node1), node2_(node2), translation_weights_(translation_weights) {}

template <typename T>
bool PointToPointConstraintImpl::operator()(const T* const c_i_rotation,
                                            const T* const c_i_translation,
                                            const T* const c_j_rotation,
                                            const T* const c_j_translation,
                                            T* residual) const {
    // Extract corresponding points from ptp_constraint_pt in both nodes (in
    // global coordinates).
    Eigen::Vector3d pt_i =
        node1_->ptp_constraint_pt.row(0);  // Selected point of node1
    Eigen::Vector3d pt_j =
        node2_->ptp_constraint_pt.row(0);  // Selected point of node2

    // Convert the 3D points to T type for compatibility with Ceres
    Eigen::Vector3<T> pt_i_t = pt_i.cast<T>();
    Eigen::Vector3<T> pt_j_t = pt_j.cast<T>();

    // Apply the rotation and translation to the points of node1 and node2 to
    // transform them
    Eigen::Quaternion<T> rotation_i(c_i_rotation[3], c_i_rotation[0],
                                    c_i_rotation[1], c_i_rotation[2]);
    Eigen::Quaternion<T> rotation_j(c_j_rotation[3], c_j_rotation[0],
                                    c_j_rotation[1], c_j_rotation[2]);

    // Transform the points by applying the rotations and translations
    Eigen::Vector3<T> transformed_pt_i =
        rotation_i * pt_i_t + Eigen::Vector3<T>(c_i_translation[0],
                                                c_i_translation[1],
                                                c_i_translation[2]);
    Eigen::Vector3<T> transformed_pt_j =
        rotation_j * pt_j_t + Eigen::Vector3<T>(c_j_translation[0],
                                                c_j_translation[1],
                                                c_j_translation[2]);

    // Calculate the translation difference (translation error) between the
    // transformed points as an Array for elementwise operations
    Eigen::Array<T, 3, 1> error = (transformed_pt_j - transformed_pt_i).array();

    // Scale the error based on the translation weights using Array semantics
    Eigen::Array<T, 3, 1> weighted_error =
        error * translation_weights_.cast<T>();

    residual[0] = weighted_error[0];
    residual[1] = weighted_error[1];
    residual[2] = weighted_error[2];

    return true;
}

ceres::CostFunction* PointToPointConstraintImpl::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<PointToPointConstraintImpl, 3, 4, 3,
                                           4, 3>(
        new PointToPointConstraintImpl(node1_, node2_, translation_weights_));
}

ceres::ResidualBlockId PointToPointConstraintImpl::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    return problem.AddResidualBlock(
        this->create_cost_function(), loss_func,
        node1_->rotation.coeffs().data(), node1_->position.data(),
        node2_->rotation.coeffs().data(), node2_->position.data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
