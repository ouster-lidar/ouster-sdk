#include "ouster/impl/point_to_point_constraint.h"

#include <ceres/ceres.h>

namespace ouster {
namespace mapping {
namespace impl {
PointToPointConstraint::PointToPointConstraint(
    const std::shared_ptr<Node> node1, const std::shared_ptr<Node> node2,
    double translation_weight)
    : node1_(node1), node2_(node2), translation_weight_(translation_weight) {}

template <typename T>
bool PointToPointConstraint::operator()(const T* const c_i_rotation,
                                        const T* const c_i_translation,
                                        const T* const c_j_rotation,
                                        const T* const c_j_translation,
                                        T* residual) const {
    // Extract corresponding points from pts_ in both nodes (in global
    // coordinates).
    Eigen::Matrix<double, 3, 1> pt_i =
        node1_->pts.row(0);  // First point of node1
    Eigen::Matrix<double, 3, 1> pt_j =
        node2_->pts.row(0);  // First point of node2

    // Convert the 3D points to T type for compatibility with Ceres
    Eigen::Matrix<T, 3, 1> pt_i_t = pt_i.cast<T>();
    Eigen::Matrix<T, 3, 1> pt_j_t = pt_j.cast<T>();

    // Apply the rotation and translation to the points of node1 and node2 to
    // transform them
    Eigen::Quaternion<T> rotation_i(c_i_rotation[3], c_i_rotation[0],
                                    c_i_rotation[1], c_i_rotation[2]);
    Eigen::Quaternion<T> rotation_j(c_j_rotation[3], c_j_rotation[0],
                                    c_j_rotation[1], c_j_rotation[2]);

    // Transform the points by applying the rotations and translations
    Eigen::Matrix<T, 3, 1> transformed_pt_i =
        rotation_i * pt_i_t + Eigen::Matrix<T, 3, 1>(c_i_translation[0],
                                                     c_i_translation[1],
                                                     c_i_translation[2]);
    Eigen::Matrix<T, 3, 1> transformed_pt_j =
        rotation_j * pt_j_t + Eigen::Matrix<T, 3, 1>(c_j_translation[0],
                                                     c_j_translation[1],
                                                     c_j_translation[2]);

    // Calculate the translation difference (translation error) between the
    // transformed points
    Eigen::Matrix<T, 3, 1> error = transformed_pt_j - transformed_pt_i;

    // Scale the error based on the translation weight and store it in the
    // residual
    residual[0] = translation_weight_ * error[0];
    residual[1] = translation_weight_ * error[1];
    residual[2] = translation_weight_ * error[2];

    return true;
}

ceres::CostFunction* PointToPointConstraint::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 4, 3, 4,
                                           3>(
        new PointToPointConstraint(node1_, node2_, translation_weight_));
}

void PointToPointConstraint::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    problem.AddResidualBlock(
        this->create_cost_function(), loss_func,
        node1_->rotation.coeffs().data(), node1_->position.data(),
        node2_->rotation.coeffs().data(), node2_->position.data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace ouster
