#pragma once

namespace ouster {
namespace OSF {

// TODO: does this ever change?? should be part of the profile
const int RANGE_MULTIPLIER_DEFAULT = 4;

enum OSF_VERSION {
    V_INVALID = 0,
    V_1_0,  // Original version of the OSF (2019/9/16)
    V_1_1,  // Add gps/imu/car trajectory to the OSF (2019/11/14) 
    V_1_2,  // Change gps_waypoint type to Table in order to
            // support Python language generator
    V_1_3,  // Add extension for Message in osfChunk
            // and for Session in osfSession (2020/03/18)
    V_1_4   // Gen2/128 support (2020/08/11)
};

}  // namespace OSF
}  // namespace ouster
