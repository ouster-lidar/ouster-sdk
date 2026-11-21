#include <cmath>
#include <cstdint>

#include "ouster/core/camera_model.h"
#include "ouster/core/types.h"

using ouster::sdk::core::CameraModel;
using ouster::sdk::core::img_t;

img_t<uint32_t> depth_to_range_mm(const img_t<uint32_t>& depth_mm, const CameraModel& model) {
    //! [doc-stag-depth-to-range]
    // depth_mm is planar depth (distance along the optical axis) in
    // millimeters. RANGE is the distance from the camera origin, so each
    // pixel is scaled by the length of its unit ray.
    img_t<uint32_t> range_mm(depth_mm.rows(), depth_mm.cols());
    for (Eigen::Index v = 0; v < depth_mm.rows(); ++v) {
        for (Eigen::Index u = 0; u < depth_mm.cols(); ++u) {
            const double x = (static_cast<double>(u) - model.cx) / model.fx;
            const double y = (static_cast<double>(v) - model.cy) / model.fy;
            const double scale = std::sqrt(1.0 + (x * x) + (y * y));
            range_mm(v, u) = static_cast<uint32_t>((depth_mm(v, u) * scale) + 0.5);
        }
    }
    //! [doc-etag-depth-to-range]
    return range_mm;
}
