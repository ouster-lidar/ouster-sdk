/**
 * @file
 * @brief PCL point datatype for use with the OS-1
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace ouster_ros {
namespace OS1 {

struct EIGEN_ALIGN16 PointOS1 {
    PCL_ADD_POINT4D;
    float intensity;
    float t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointOS1,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, t, t)
    (uint8_t, ring, ring)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, noise, noise)
    (uint32_t, range, range)
)
// clang-format on
