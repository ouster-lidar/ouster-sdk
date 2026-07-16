#include "ouster/mapping/adaptive_threshold.h"

#include <Eigen/Geometry>
#include <cmath>

namespace ouster {
namespace sdk {
namespace mapping {

AdaptiveThreshold::AdaptiveThreshold(double max_range, double initial_threshold,
                                     double min_motion_threshold)
    : min_motion_threshold_(min_motion_threshold),
      max_range_(max_range),
      model_sse_(initial_threshold * initial_threshold),
      num_samples_(1) {}

void AdaptiveThreshold::update_model_deviation(const core::Matrix4dR& current_deviation) {
    const Eigen::Matrix3d rotation = current_deviation.block<3, 3>(0, 0);
    const double theta = Eigen::AngleAxisd(rotation).angle();
    const double delta_rot = 2.0 * max_range_ * std::sin(theta / 2.0);
    const double delta_trans = current_deviation.block<3, 1>(0, 3).norm();
    const double model_error = delta_trans + delta_rot;
    if (model_error > min_motion_threshold_) {
        model_sse_ += model_error * model_error;
        num_samples_++;
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
