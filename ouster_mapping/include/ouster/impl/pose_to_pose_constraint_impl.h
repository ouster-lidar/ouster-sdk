#pragma once

#include <ceres/cost_function.h>
#include <ceres/problem.h>

#include <Eigen/Dense>
#include <memory>

#include "ouster/impl/constraint_impl.h"
#include "ouster/impl/transformation.h"

namespace ouster {
namespace sdk {
namespace mapping {

class Node;

namespace impl {

/**
 * @class PoseToPoseConstraintImpl
 * @brief Defines a cost function for 3D pose optimization using Ceres.
 */
class PoseToPoseConstraintImpl : public ConstraintImpl {
   public:
    PoseToPoseConstraintImpl(
        const std::shared_ptr<Node> node1, const std::shared_ptr<Node>& node2,
        const Eigen::Quaterniond& diff_r, const Eigen::Vector3d& diff_t,
        double rotation_weight = 1.0,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones());

    PoseToPoseConstraintImpl(
        const std::shared_ptr<Node> node1, const std::shared_ptr<Node>& node2,
        const ouster::sdk::core::impl::PoseH& diff,
        double rotation_weight = 1.0,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones());

    ceres::CostFunction* create_cost_function() const override;

    ceres::ResidualBlockId add_to_problem(
        ceres::Problem& problem, ceres::LossFunction* loss_func) const override;

    template <typename T>
    bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                    const T* const c_j_rotation, const T* const c_j_translation,
                    T* residual) const;

   private:
    const std::shared_ptr<Node> node1_;
    const std::shared_ptr<Node> node2_;
    Eigen::Quaterniond diff_r_;
    Eigen::Vector3d diff_t_;
    const double rotation_weight_;
    const Eigen::Array3d translation_weights_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
