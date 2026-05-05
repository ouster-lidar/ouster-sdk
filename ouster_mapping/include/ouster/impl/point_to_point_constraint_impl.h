#pragma once

#include <ceres/cost_function.h>
#include <ceres/problem.h>

#include <Eigen/Dense>
#include <memory>

#include "ouster/impl/constraint_impl.h"

namespace ouster {
namespace sdk {
namespace mapping {

class Node;

namespace impl {

class PointToPointConstraintImpl : public ConstraintImpl {
   public:
    PointToPointConstraintImpl(
        const std::shared_ptr<Node> node1, const std::shared_ptr<Node> node2,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones());

    // Create a cost function
    ceres::CostFunction* create_cost_function() const override;

    // Add the constraint to the optimization problem
    ceres::ResidualBlockId add_to_problem(
        ceres::Problem& problem, ceres::LossFunction* loss_func) const override;

    template <typename T>
    bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                    const T* const c_j_rotation, const T* const c_j_translation,
                    T* residual) const;

   private:
    const std::shared_ptr<Node> node1_;
    const std::shared_ptr<Node> node2_;
    const Eigen::Array3d translation_weights_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
