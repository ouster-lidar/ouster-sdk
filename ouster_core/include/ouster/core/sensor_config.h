/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_config.h
 * @brief Base class for Ouster sensor configuration information
 */

#pragma once

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Base class for sensor configuration parameters.
 *
 * Currently carries no attributes of its own; it exists so that concrete
 * configuration types (e.g. LidarConfig) share a common base, mirroring the
 * SensorInfo / LidarInfo hierarchy. APIs that accept a configuration take this
 * base type and downcast to the concrete type they require.
 */
struct OUSTER_API_CLASS SensorConfig {
    OUSTER_API_FUNCTION
    virtual ~SensorConfig() = default;

   protected:
    OUSTER_API_FUNCTION
    SensorConfig() = default;

    /// Copy constructor.
    OUSTER_API_FUNCTION
    SensorConfig(const SensorConfig&) = default;

    /// Copy assignment operator.
    OUSTER_API_FUNCTION
    SensorConfig& operator=(const SensorConfig&) = default;

    /// Move constructor.
    OUSTER_API_FUNCTION
    SensorConfig(SensorConfig&&) = default;

    /// Move assignment operator.
    OUSTER_API_FUNCTION
    SensorConfig& operator=(SensorConfig&&) = default;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
