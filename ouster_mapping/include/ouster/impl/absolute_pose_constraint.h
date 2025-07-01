#pragma once

#include <array>

#include "ouster/impl/abstract_constraint.h"
#include "ouster/pose_optimizer_node.h"

namespace ouster {
namespace mapping {
namespace impl {

/**
 * @brief A cost function for AbsolutePose constraint.
 */
class AbsolutePoseConstraint : public AbstractConstraint {
   public:
    AbsolutePoseConstraint(
        const std::shared_ptr<Node> source_node,
        const std::shared_ptr<const Node>& target_node,
        const std::array<double, 3>& rotation_weights = {1.0, 1.0, 1.0},
        const std::array<double, 3>& translation_weights = {1.0, 1.0, 1.0});

    AbsolutePoseConstraint(
        const std::shared_ptr<Node> source_node,
        const std::shared_ptr<const Node>& target_node,
        const Eigen::Quaterniond& diff_r, const Eigen::Vector3d& diff_t,
        const std::array<double, 3>& rotation_weights = {1.0, 1.0, 1.0},
        const std::array<double, 3>& translation_weights = {1.0, 1.0, 1.0});

    ceres::CostFunction* create_cost_function() const override;

    template <typename T>
    bool operator()(const T* const rotation, const T* const translation,
                    T* residual) const;

    void add_to_problem(ceres::Problem& problem,
                        ceres::LossFunction* loss_func) const override;

   private:
    const std::shared_ptr<Node> node_source_;
    const std::shared_ptr<const Node> node_target_;
    Eigen::Quaterniond diff_r_;
    Eigen::Vector3d diff_t_;
    std::array<double, 3> rotation_weights_;
    std::array<double, 3> translation_weights_;
};

}  // namespace impl
}  // namespace mapping
}  // namespace ouster
