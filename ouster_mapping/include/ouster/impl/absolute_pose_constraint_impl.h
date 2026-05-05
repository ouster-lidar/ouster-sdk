#pragma once

#include <ceres/cost_function.h>
#include <ceres/problem.h>

#include <Eigen/Dense>
#include <array>
#include <memory>

#include "ouster/impl/constraint_impl.h"

namespace ouster {
namespace sdk {
namespace mapping {

class Node;

namespace impl {

/**
 * @brief A cost function for AbsolutePose constraint.
 */
class AbsolutePoseConstraintImpl : public ConstraintImpl {
   public:
    AbsolutePoseConstraintImpl(
        const std::shared_ptr<Node> source_node,
        const std::shared_ptr<const Node>& target_node,
        double rotation_weight = 1.0,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones());

    AbsolutePoseConstraintImpl(
        const std::shared_ptr<Node> source_node,
        const std::shared_ptr<const Node>& target_node,
        const Eigen::Quaterniond& diff_r, const Eigen::Vector3d& diff_t,
        double rotation_weight = 1.0,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones());

    ceres::CostFunction* create_cost_function() const override;

    template <typename T>
    bool operator()(const T* const rotation, const T* const translation,
                    T* residual) const;

    ceres::ResidualBlockId add_to_problem(
        ceres::Problem& problem, ceres::LossFunction* loss_func) const override;

   private:
    const std::shared_ptr<Node> source_node_;
    const std::shared_ptr<const Node> target_node_;
    Eigen::Quaterniond diff_r_;
    Eigen::Vector3d diff_t_;
    double rotation_weight_;
    Eigen::Array3d translation_weights_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
