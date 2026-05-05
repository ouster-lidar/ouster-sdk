#pragma once

#include <Eigen/Core>
#include <string>

#include "ouster/typedefs.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * [BETA] Load the 3D X Y and Z points from a PCD or PLY file and returns
 * them an Nx3 array.
 *
 * @throws std::runtime_error if unable to open or parse file
 * @return Nx3 array of the resulting points
 *
 * @remarks this is a beta feature and may change in future releases.
 */
OUSTER_API_FUNCTION PointCloudXYZf read_pointcloud(
    const std::string& filename  ///< [in] filename to load
);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
