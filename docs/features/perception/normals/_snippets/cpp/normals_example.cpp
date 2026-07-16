#include <cstdint>

#include "ouster/algorithm/normals.h"

using namespace ouster::sdk;

core::MatrixX3dR estimate_normals(const core::PointCloudXYZd& xyz_destaggered,
                                  const core::img_t<uint32_t>& range_destaggered) {
    // clang-format off
    //! [doc-stag-normals-api]
    auto sensor_origins = core::MatrixX3dR::Zero(
        range_destaggered.cols(), 3);

    auto normal_vectors = algorithm::normals(
        xyz_destaggered,
        range_destaggered,
        sensor_origins);
    //! [doc-etag-normals-api]
    // clang-format on

    return normal_vectors;
}
