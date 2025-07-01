#pragma once

#include <ceres/local_parameterization.h>

#include <sophus/ceres_typetraits.hpp>

namespace Sophus {

/// Templated local parameterization for LieGroup [with implemented
/// LieGroup::Dx_this_mul_exp_x_at_0() ]
template <template <typename, int = 0> class LieGroup>
class LocalParameterization : public ceres::LocalParameterization {
 public:
  using LieGroupd = LieGroup<double>;
  using Tangent = typename LieGroupd::Tangent;
  using TangentMap = typename Sophus::Mapper<Tangent>::ConstMap;
  static int constexpr DoF = LieGroupd::DoF;
  static int constexpr num_parameters = LieGroupd::num_parameters;

  /// LieGroup plus operation for Ceres
  ///
  ///  T * exp(x)
  ///
  bool Plus(double const* T_raw, double const* delta_raw,
            double* T_plus_delta_raw) const override {
    Eigen::Map<LieGroupd const> const T(T_raw);
    TangentMap delta = Sophus::Mapper<Tangent>::map(delta_raw);
    Eigen::Map<LieGroupd> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * LieGroupd::exp(delta);
    return true;
  }

  /// Jacobian of LieGroup plus operation for Ceres
  ///
  /// Dx T * exp(x)  with  x=0
  ///
  bool ComputeJacobian(double const* T_raw,
                       double* jacobian_raw) const override {
    Eigen::Map<LieGroupd const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, num_parameters, DoF,
                             DoF == 1 ? Eigen::ColMajor : Eigen::RowMajor>>
        jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  int GlobalSize() const override { return LieGroupd::num_parameters; }

  int LocalSize() const override { return LieGroupd::DoF; }
};

}  // namespace Sophus
