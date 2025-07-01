#pragma once

#include <ceres/cost_function.h>

#include "ceres/problem.h"

namespace ouster {
namespace mapping {
namespace impl {
/**
 * @class AbstractConstraint
 * @brief Abstract base class for defining constraints for optimization in
 * Ceres.
 */
class AbstractConstraint {
   public:
    virtual ~AbstractConstraint() = default;

    virtual ceres::CostFunction* create_cost_function() const = 0;

    virtual void add_to_problem(ceres::Problem& problem,
                                ceres::LossFunction* loss_func) const = 0;
};
}  // namespace impl
}  // namespace mapping
}  // namespace ouster
