#include "ouster/impl/pose_to_pose_constraint_impl.h"

#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>

#include <array>
#include <memory>

#include "ouster/impl/utils.h"
#include "ouster/pose_optimizer_node.h"

using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseQ;

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

template <typename T>
Eigen::Vector3<T> rotation_quaternion_to_angle_axis(
    const Eigen::Quaternion<T>& quaternion) {
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    if (normalized_quaternion.w() < 0.) {
        normalized_quaternion.w() = -1. * normalized_quaternion.w();
        normalized_quaternion.x() = -1. * normalized_quaternion.x();
        normalized_quaternion.y() = -1. * normalized_quaternion.y();
        normalized_quaternion.z() = -1. * normalized_quaternion.z();
    }
    const T angle = 2. * atan2(normalized_quaternion.vec().norm(),
                               normalized_quaternion.w());
    constexpr double k_cut_off_angle = 1e-7;
    const T scale = angle < k_cut_off_angle ? T(2.) : angle / sin(angle / 2.);
    return Eigen::Vector3<T>(scale * normalized_quaternion.x(),
                             scale * normalized_quaternion.y(),
                             scale * normalized_quaternion.z());
}

template <typename T>
Eigen::Array<T, 6, 1> scale_error(const Eigen::Array<T, 6, 1>& error,
                                  double rotation_weight,
                                  const Eigen::Array3d& translation_weights) {
    Eigen::Array<T, 6, 1> weights_arr;
    weights_arr.template head<3>().setConstant(T(rotation_weight));
    weights_arr.template tail<3>() = translation_weights.template cast<T>();
    return error * weights_arr;
}

template <typename T>
Eigen::Array<T, 6, 1> compute_unscaled_error(
    const Eigen::Quaternion<double>& qua,
    const Eigen::Matrix<double, 3, 1>& pos, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation) {
    const Eigen::Quaternion<T> r_i_inverse(
        start_rotation[3], -start_rotation[0], -start_rotation[1],
        -start_rotation[2]);

    const Eigen::Vector3<T> delta(end_translation[0] - start_translation[0],
                                  end_translation[1] - start_translation[1],
                                  end_translation[2] - start_translation[2]);
    const Eigen::Vector3<T> h_translation = r_i_inverse * delta;

    const Eigen::Quaternion<T> h_rotation_inverse =
        Eigen::Quaternion<T>(end_rotation[3], -end_rotation[0],
                             -end_rotation[1], -end_rotation[2]) *
        Eigen::Quaternion<T>(start_rotation[3], start_rotation[0],
                             start_rotation[1], start_rotation[2]);

    const Eigen::Vector3<T> angle_axis_difference =
        rotation_quaternion_to_angle_axis(h_rotation_inverse * qua.cast<T>());

    Eigen::Array<T, 6, 1> error;
    error.template head<3>() = angle_axis_difference.template cast<T>().array();
    error.template tail<3>() = (pos.cast<T>() - h_translation).array();

    return error;
}

template <typename T>
bool PoseToPoseConstraintImpl::operator()(const T* const c_i_rotation,
                                          const T* const c_i_translation,
                                          const T* const c_j_rotation,
                                          const T* const c_j_translation,
                                          T* residual) const {
    const Eigen::Array<T, 6, 1> error_arr = scale_error(
        compute_unscaled_error(diff_r_, diff_t_, c_i_rotation, c_i_translation,
                               c_j_rotation, c_j_translation),
        rotation_weight_, translation_weights_);

    // Map the residual pointer to Eigen vector and assign from array (convert
    // to matrix)
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residual_map(residual);
    residual_map = error_arr.matrix();
    return true;
}

PoseToPoseConstraintImpl::PoseToPoseConstraintImpl(
    const std::shared_ptr<Node> node1, const std::shared_ptr<Node>& node2,
    const Eigen::Quaterniond& diff_r, const Eigen::Vector3d& diff_t,
    double rotation_weight, const Eigen::Array3d& translation_weights)
    : node1_(node1),
      node2_(node2),
      diff_r_(diff_r),
      diff_t_(diff_t),
      rotation_weight_(rotation_weight),
      translation_weights_(translation_weights) {}

PoseToPoseConstraintImpl::PoseToPoseConstraintImpl(
    const std::shared_ptr<Node> node1, const std::shared_ptr<Node>& node2,
    const PoseH& diff, double rotation_weight,
    const Eigen::Array3d& translation_weights)
    : node1_(node1),
      node2_(node2),
      rotation_weight_(rotation_weight),
      translation_weights_(translation_weights) {
    PoseQ diff_q = diff.log().q();
    diff_r_ = diff_q.r();
    diff_t_ = diff_q.t();
}

ceres::CostFunction* PoseToPoseConstraintImpl::create_cost_function() const {
    return new ceres::AutoDiffCostFunction<PoseToPoseConstraintImpl, 6, 4, 3, 4,
                                           3>(
        new PoseToPoseConstraintImpl(node1_, node2_, diff_r_, diff_t_,
                                     rotation_weight_, translation_weights_));
}

ceres::ResidualBlockId PoseToPoseConstraintImpl::add_to_problem(
    ceres::Problem& problem, ceres::LossFunction* loss_func) const {
    return problem.AddResidualBlock(
        this->create_cost_function(), loss_func,
        node1_->rotation.coeffs().data(), node1_->position.data(),
        node2_->rotation.coeffs().data(), node2_->position.data());
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
