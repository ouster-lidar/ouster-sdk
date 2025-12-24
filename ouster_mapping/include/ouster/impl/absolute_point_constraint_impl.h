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

/**
 * @brief A cost function for AbsolutePointConstraint.
 */
class AbsolutePointConstraintImpl : public ConstraintImpl {
   public:
    AbsolutePointConstraintImpl(
        const std::shared_ptr<Node> source_node,
        const Eigen::Vector3d& absolute_position,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones());

    ceres::CostFunction* create_cost_function() const override;

    template <typename T>
    bool operator()(const T* const rotation, const T* const translation,
                    T* residual) const;

    ceres::ResidualBlockId add_to_problem(
        ceres::Problem& problem, ceres::LossFunction* loss_func) const override;

   private:
    const std::shared_ptr<Node> source_node_;
    Eigen::Vector3d absolute_position_;
    Eigen::Array3d translation_weights_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
