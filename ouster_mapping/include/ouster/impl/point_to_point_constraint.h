#pragma once

#include "ouster/impl/abstract_constraint.h"
#include "ouster/pose_optimizer_node.h"

namespace ouster {
namespace mapping {
namespace impl {

class PointToPointConstraint : public AbstractConstraint {
   public:
    PointToPointConstraint(const std::shared_ptr<Node> node1,
                           const std::shared_ptr<Node> node2,
                           double translation_weight = 1.0);

    // Create a cost function
    ceres::CostFunction* create_cost_function() const override;

    // Add the constraint to the optimization problem
    void add_to_problem(ceres::Problem& problem,
                        ceres::LossFunction* loss_func) const override;

    template <typename T>
    bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                    const T* const c_j_rotation, const T* const c_j_translation,
                    T* residual) const;

   private:
    const std::shared_ptr<Node> node1_;
    const std::shared_ptr<Node> node2_;
    const double translation_weight_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace ouster
