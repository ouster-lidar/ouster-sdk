#pragma once

#include "ouster/visibility.h"

namespace ouster {
namespace mapping {

/**
 * @brief Sampling Mode for PoseOptimize get poses or timestamps.
 */
enum class SamplingMode {
    KEY_FRAMES,  ///< Key frames sampling mode, selects poses or timestamps of
                 ///< every key frame in Pose Optimize

    COLUMNS  ///< Selects poses or timestamps corresponding to every column in
             ///< each lidarscan.
};

/**
 * @brief Loss function types used in PoseOptimizer.
 */
enum class LossFunction {
    HuberLoss,
    CauchyLoss,
    SoftLOneLoss,
    ArctanLoss,
    TrivialLoss
};

/**
 * @brief Converts a string to the corresponding LossFunction enum value.
 *
 * This function takes a string representation of a loss function name
 * and returns the matching LossFunction enum.
 *
 * @param[in] name The string name of the loss function.
 * @return The corresponding LossFunction enum value.
 * @throws std::invalid_argument If the provided name does not match any known
 * LossFunction.
 */
inline LossFunction from_string(const std::string& name) {
    if (name == "HuberLoss") {
        return LossFunction::HuberLoss;
    } else if (name == "CauchyLoss") {
        return LossFunction::CauchyLoss;
    } else if (name == "SoftLOneLoss") {
        return LossFunction::SoftLOneLoss;
    } else if (name == "ArctanLoss") {
        return LossFunction::ArctanLoss;
    } else if (name == "TrivialLoss") {
        return LossFunction::TrivialLoss;
    }
    throw std::invalid_argument("Unknown LossFunction: " + name);
}

/**
 * @brief Converts a LossFunction enum value to its corresponding string.
 *
 * This function takes a LossFunction enum and returns a string representation
 * of that enum. Valid return values are: "HuberLoss", "CauchyLoss",
 * "SoftLOneLoss", "ArctanLoss", and "TrivialLoss".
 *
 * @param[in] lf The LossFunction enum to convert.
 * @return A std::string containing the name of the loss function.
 */
inline std::string to_string(const LossFunction lf) {
    switch (lf) {
        case LossFunction::HuberLoss:
            return "HuberLoss";
        case LossFunction::CauchyLoss:
            return "CauchyLoss";
        case LossFunction::SoftLOneLoss:
            return "SoftLOneLoss";
        case LossFunction::ArctanLoss:
            return "ArctanLoss";
        case LossFunction::TrivialLoss:
            return "TrivialLoss";
        default:
            // If you ever extend the enum and forget to update this,
            // throwing an exception helps catch it at runtime.
            throw std::invalid_argument("Unknown LossFunction enum value");
    }
}

}  // namespace mapping
}  // namespace ouster
