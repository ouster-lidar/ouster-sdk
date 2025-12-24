/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "ouster/typedefs.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

// Forward declaration
template <typename T>
class XYZLutT;
using XYZLut = XYZLutT<double>;

/**
 * Beam configuration structure for LIDAR sensors.
 */
struct OUSTER_API_CLASS BeamConfig {
    uint32_t n_cols;  ///< Number of columns in the LIDAR data
    uint32_t n_rows;  ///< Number of rows in the LIDAR data

    mat4d beam_to_lidar_transform;    ///< Transformation matrix from beam to
                                      ///< LIDAR frame
    mat4d lidar_to_sensor_transform;  ///< Transformation matrix from LIDAR to
                                      ///< sensor frame
    mat4d sensor_to_body_transform;   ///< Extrinsic transformation matrix
    float m_per_zmbin;                ///< Scale factor for Zrb encoding -
                                      ///< sensor architecture and OS-dependent.
    uint64_t serial_number;           ///< Serial number of the sensor
    std::vector<double> px_altitudes;  ///< Pixel altitudes in degrees
    std::vector<double> px_azimuths;   ///< Pixel azimuths in degrees

    std::shared_ptr<XYZLut> lut;  ///< Lookup table for converting pixel
                                  ///< coordinates to Cartesian coordinates
    std::shared_ptr<XYZLut>
        lut_no_sensor_to_body_transform;  ///< Lookup table for converting
                                          ///< pixel coordinates to Cartesian
                                          ///< coordinates without
                                          ///< sensor_to_body_transform

    /**
     * Parameterized constructor
     * @param[in] n_cols_init Number of columns in the LIDAR data
     * @param[in] px_altitudes_init Vector of pixel altitudes
     * @param[in] px_azimuths_init Vector of pixel azimuths
     * @param[in] beam_to_lidar_transform_init Transformation matrix from beam
     * to LIDAR frame
     * @param[in] lidar_to_sensor_transform_init Transformation matrix from
     * LIDAR to sensor frame
     * @param[in] sensor_to_body_transform_init Extrinsic transformation
     * matrix
     * zone rendered binary format
     * @param[in] m_per_zmbin_init Scale factor for Zrb encoding
     * @param[in] serial_number_init Serial number of the sensor
     */
    OUSTER_API_FUNCTION
    BeamConfig(uint32_t n_cols_init,
               const std::vector<double>& px_altitudes_init,
               const std::vector<double>& px_azimuths_init,
               mat4d beam_to_lidar_transform_init,
               mat4d lidar_to_sensor_transform_init,
               mat4d sensor_to_body_transform_init, float m_per_zmbin_init,
               uint64_t serial_number_init);
    /**
     * Returns true if this BeamConfig equals another BeamConfig
     * @param[in] rhs The other BeamConfig to compare against
     * @return True if the two BeamConfigs are equal, false otherwise
     */
    OUSTER_API_FUNCTION
    bool operator==(BeamConfig const& rhs) const;
    /**
     * Returns true if this BeamConfig does not equal another BeamConfig
     * @param[in] rhs The other BeamConfig to compare against
     * @return True if the two BeamConfigs are not equal, false otherwise
     */
    OUSTER_API_FUNCTION
    bool operator!=(BeamConfig const& rhs) const;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
