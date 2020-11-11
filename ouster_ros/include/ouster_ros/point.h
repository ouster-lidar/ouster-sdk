/**
 * @file
 * @brief PCL point datatype for use with ouster sensors
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <chrono>
#include <functional>

#include "ouster/lidar_scan.h"

namespace ouster_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline std::function<
        Point(std::ptrdiff_t, std::ptrdiff_t, std::chrono::nanoseconds,
              std::chrono::nanoseconds, uint32_t, uint16_t, uint16_t, uint16_t)>
    get_from_pixel(const ouster::XYZLut& xyz_lut, size_t w, size_t h) {
        return [xyz_lut, w, h](std::ptrdiff_t u, std::ptrdiff_t v,
                               std::chrono::nanoseconds ts,
                               std::chrono::nanoseconds scan_ts, uint32_t range,
                               uint16_t intensity, uint16_t noise,
                               uint16_t reflectivity) -> Point {
            const auto xyz = xyz_lut.direction.row(u * w + v) * range +
                             xyz_lut.offset.row(u * w + v);
            return {static_cast<float>(xyz(0)),
                    static_cast<float>(xyz(1)),
                    static_cast<float>(xyz(2)),
                    0.0f,
                    static_cast<float>(intensity),
                    static_cast<uint32_t>((ts - scan_ts).count()),
                    static_cast<uint16_t>(reflectivity),
                    static_cast<uint8_t>(u),
                    static_cast<uint16_t>(noise),
                    static_cast<uint32_t>(range)};
        };
    }
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, t, t)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (uint16_t, noise, noise)
    (uint32_t, range, range)
)
// clang-format on
