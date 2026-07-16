#pragma once

#include <ouster/core/typedefs.h>

#include <cmath>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/// @brief Adaptive threshold for ICP registration.
struct OUSTER_API_CLASS AdaptiveThreshold {
    /// @brief Constructor for Adaptive threshold.
    /// @param[in] max_range Maximum range of the sensor.
    /// @param[in] initial_threshold Initial threshold.
    /// @param[in] min_motion_threshold Minimum motion threshold.
    OUSTER_API_FUNCTION
    explicit AdaptiveThreshold(double max_range, double initial_threshold = 2.0,
                               double min_motion_threshold = 0.01);

    /// @brief Update the current belief of the deviation from the prediction model.
    /// @param[in] current_deviation Current deviation.
    OUSTER_API_FUNCTION
    void update_model_deviation(const core::Matrix4dR& current_deviation);

    /// @brief Returns the KISS-ICP adaptive threshold used in registration.
    /// @return The KISS-ICP adaptive threshold used in registration.
    OUSTER_API_FUNCTION
    inline double compute_threshold() const {
        return std::sqrt(model_sse_ / num_samples_);
    }

    /// @brief Minimum motion threshold.
    double min_motion_threshold_;
    /// @brief Maximum range.
    double max_range_;
    /// @brief Model SSE.
    double model_sse_;
    /// @brief Number of samples.
    int num_samples_;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
