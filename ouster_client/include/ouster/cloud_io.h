#pragma once

#include <Eigen/Core>
#include <string>

#include "ouster/visibility.h"

namespace ouster {
namespace core {

/**
 * [BETA] Load the 3D X Y and Z points from a PCD or PLY file and returns
 * them an Nx3 matrix.
 *
 * @throws std::runtime_error if unable to open or parse file
 * @return Nx3 matrix of the resulting points
 *
 * @remarks this is a beta feature and may change in future releases.
 */
OUSTER_API_FUNCTION Eigen::Matrix<float, Eigen::Dynamic, 3> read_pointcloud(
    const std::string& filename  ///< [in] filename to load
);

}  // namespace core
}  // namespace ouster
