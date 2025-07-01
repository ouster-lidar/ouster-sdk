#pragma once

#include "ouster/impl/abstract_constraint.h"
#include "ouster/impl/transformation.h"
#include "ouster/pose_optimizer_node.h"

namespace ouster {
namespace mapping {
namespace impl {

/**
 * @class PoseToPoseConstraint
 * @brief Defines a cost function for 3D pose optimization using Ceres.
 */
class PoseToPoseConstraint : public AbstractConstraint {
   public:
    PoseToPoseConstraint(const std::shared_ptr<Node> node1,
                         const std::shared_ptr<Node>& node2,
                         const Eigen::Quaterniond& diff_r,
                         const Eigen::Vector3d& diff_t,
                         double rotation_weight = 1.0,
                         double translation_weight = 1.0);

    PoseToPoseConstraint(const std::shared_ptr<Node> node1,
                         const std::shared_ptr<Node>& node2,
                         const ouster::impl::PoseH& diff,
                         double rotation_weight = 1.0,
                         double translation_weight = 1.0);

    ceres::CostFunction* create_cost_function() const override;

    void add_to_problem(ceres::Problem& problem,
                        ceres::LossFunction* loss_func) const override;

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
    const double translation_weight_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace ouster
