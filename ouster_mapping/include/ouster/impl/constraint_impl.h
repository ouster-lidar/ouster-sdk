#pragma once

#include <ceres/cost_function.h>

#include "ceres/problem.h"

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {
/**
 * @class ConstraintImpl
 * @brief Abstract base class for defining constraints for optimization in
 * Ceres.
 */
class ConstraintImpl {
   public:
    virtual ~ConstraintImpl() = default;

    virtual ceres::CostFunction* create_cost_function() const = 0;

    virtual ceres::ResidualBlockId add_to_problem(
        ceres::Problem& problem, ceres::LossFunction* loss_func) const = 0;
};
}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
