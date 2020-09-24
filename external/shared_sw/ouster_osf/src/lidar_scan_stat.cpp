#include "ouster/osf/lidar_scan_stat.h"

namespace ouster {

Eigen::Vector3d mean_direction(const LidarScan& ls,
                               const sensor::sensor_info& sinfo) {
    auto scan_lut = make_xyz_lut(sinfo);
    auto points_3d = cartesian(ls, scan_lut);
    return points_3d.colwise().mean();
}

}  // namespace ouster