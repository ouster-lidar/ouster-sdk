
#include "ouster/core/xyzlut.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include "ouster/core/camera_info.h"
#include "ouster/core/camera_model.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/types.h"

namespace ouster {
namespace sdk {
namespace core {

namespace impl {
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit, const mat4d& beam_to_lidar_transform,
                    const mat4d& transform, const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg) {
    if (w <= 0 || h <= 0) {
        throw std::invalid_argument("lut dimensions must be greater than zero");
    }

    if ((azimuth_angles_deg.size() != h || altitude_angles_deg.size() != h) &&
        (azimuth_angles_deg.size() != w * h || altitude_angles_deg.size() != w * h)) {
        throw std::invalid_argument("unexpected frame dimensions");
    }

    double beam_to_lidar_euclidean_distance_mm = beam_to_lidar_transform(0, 3);
    if (beam_to_lidar_transform(2, 3) != 0) {
        beam_to_lidar_euclidean_distance_mm = std::sqrt(std::pow(beam_to_lidar_transform(0, 3), 2) +
                                                        std::pow(beam_to_lidar_transform(2, 3), 2));
    }

    Eigen::ArrayXd encoder(w * h);   // theta_e
    Eigen::ArrayXd azimuth(w * h);   // theta_a
    Eigen::ArrayXd altitude(w * h);  // phi

    if (azimuth_angles_deg.size() == h && altitude_angles_deg.size() == h) {
        // OS sensor
        const double azimuth_radians = M_PI * 2.0 / static_cast<double>(w);

        // populate angles for each pixel
        for (size_t col_idx = 0; col_idx < w; col_idx++) {
            for (size_t row_idx = 0; row_idx < h; row_idx++) {
                size_t i = (row_idx * w) + col_idx;
                encoder(static_cast<Eigen::Index>(i)) =
                    2.0 * M_PI - (static_cast<double>(col_idx) * azimuth_radians);
                azimuth(static_cast<Eigen::Index>(i)) = -azimuth_angles_deg[row_idx] * M_PI / 180.0;
                altitude(static_cast<Eigen::Index>(i)) =
                    altitude_angles_deg[row_idx] * M_PI / 180.0;
            }
        }

    } else if (azimuth_angles_deg.size() == w * h && altitude_angles_deg.size() == w * h) {
        // DF sensor
        // populate angles for each pixel
        for (size_t col_idx = 0; col_idx < w; col_idx++) {
            for (size_t row_idx = 0; row_idx < h; row_idx++) {
                size_t i = (row_idx * w) + col_idx;
                encoder(static_cast<Eigen::Index>(i)) = 0;
                azimuth(static_cast<Eigen::Index>(i)) = azimuth_angles_deg[i] * M_PI / 180.0;
                altitude(static_cast<Eigen::Index>(i)) = altitude_angles_deg[i] * M_PI / 180.0;
            }
        }
    }

    // unit vectors for each pixel
    ArrayX3R<double> direction = PointCloudXYZd{w * h, 3};
    direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
    direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
    direction.col(2) = altitude.sin();

    // offsets due to beam origin
    ArrayX3R<double> offset = PointCloudXYZd{w * h, 3};
    offset.col(0) = encoder.cos() * beam_to_lidar_transform(0, 3) -
                    direction.col(0) * beam_to_lidar_euclidean_distance_mm;
    offset.col(1) = encoder.sin() * beam_to_lidar_transform(0, 3) -
                    direction.col(1) * beam_to_lidar_euclidean_distance_mm;
    offset.col(2) =
        -direction.col(2) * beam_to_lidar_euclidean_distance_mm + beam_to_lidar_transform(2, 3);

    // apply the supplied transform
    auto rot = transform.topLeftCorner(3, 3).transpose();
    auto trans = transform.topRightCorner(3, 1).transpose();
    direction.matrix() *= rot;
    offset.matrix() *= rot;
    offset.matrix() += trans.replicate(static_cast<Eigen::Index>(w * h), 1);

    // apply scaling factor
    direction *= range_unit;
    offset *= range_unit;

    return XYZLut(direction, offset, h, w);
}

namespace {

// Build an XYZLut for a camera directly from its pinhole intrinsics.
//
// Per-pixel unit ray directions are computed from the CameraModel and expressed
// in the Ouster sensor frame (X forward, Y left, Z up). This is algebraically
// identical to deriving per-pixel beam angles
// (azimuth = atan2(oy, ox), altitude = asin(oz/|o|)) and rebuilding the unit
// vector, so the resulting LUT matches the previous beam-angle-based camera LUT
// exactly. The offset is zero (no beam origin); the sensor->body extrinsic
// is applied the same way as the lidar path.
XYZLut make_camera_xyz_lut(const CameraInfo& camera) {
    if (!camera.model) {
        // Substituting a plausible focal length would produce a cloud wrong by
        // an unknowable scale factor that still looks entirely reasonable, so
        // return an empty (0x0) LUT instead. Dewarp skips frames whose LUT does
        // not match their dimensions, so model-less cameras contribute no points.
        return {};
    }
    const size_t w = camera.w();
    const size_t h = camera.h();
    if (w == 0 || h == 0) {
        // Without an image size there is nothing to project either; treat it
        // like a missing model.
        return {};
    }

    const CameraModel& model = *camera.model;
    const double fx = model.fx;
    const double fy = model.fy;
    const double cx = model.cx;
    const double cy = model.cy;
    // CameraModel's constructor enforces this, but its members are
    // public, so a model can still be mutated into an unusable state after the
    // fact. Rejecting it here keeps the division below well defined. The
    // negated comparison also catches NaN.
    if (!(fx > 0.0) || !(fy > 0.0)) {
        std::stringstream error_stream;
        error_stream << "camera XYZLut requires focal lengths greater than zero, got fx=" << fx
                     << " fy=" << fy;
        throw std::invalid_argument(error_stream.str());
    }

    ArrayX3R<double> direction = PointCloudXYZd{w * h, 3};
    ArrayX3R<double> offset = PointCloudXYZd{w * h, 3};
    offset.setZero();
    for (size_t row_idx = 0; row_idx < h; row_idx++) {
        for (size_t col_idx = 0; col_idx < w; col_idx++) {
            const auto i = static_cast<Eigen::Index>((row_idx * w) + col_idx);
            const double ox = 1.0;
            const double oy = -((static_cast<double>(col_idx) - cx) / fx);
            const double oz = -((static_cast<double>(row_idx) - cy) / fy);
            const double magnitude = std::sqrt((ox * ox) + (oy * oy) + (oz * oz));
            direction(i, 0) = ox / magnitude;
            direction(i, 1) = oy / magnitude;
            direction(i, 2) = oz / magnitude;
        }
    }

    // Apply the sensor->body extrinsic so the LUT produces body-frame
    // coordinates; translation is converted to the LUT's scaled units, mirroring
    // the lidar path.
    mat4d transform = camera.sensor_to_body;
    transform(0, 3) /= RANGE_UNIT;
    transform(1, 3) /= RANGE_UNIT;
    transform(2, 3) /= RANGE_UNIT;
    auto rot = transform.topLeftCorner(3, 3).transpose();
    auto trans = transform.topRightCorner(3, 1).transpose();
    direction.matrix() *= rot;
    offset.matrix() *= rot;
    offset.matrix() += trans.replicate(static_cast<Eigen::Index>(w * h), 1);

    direction *= RANGE_UNIT;
    offset *= RANGE_UNIT;

    return XYZLut(direction, offset, h, w);
}

}  // namespace

XYZLut make_xyz_lut(const SensorInfo& sensor) {
    // Dispatch on the concrete sensor kind: cameras project from their pinhole
    // intrinsics, lidars from their per-row beam angles + lidar transforms
    // (both lidar-only fields, no longer on the SensorInfo base).
    if (const auto* camera = dynamic_cast<const CameraInfo*>(&sensor)) {
        return make_camera_xyz_lut(*camera);
    }
    const auto* lidar = dynamic_cast<const LidarInfo*>(&sensor);
    if (lidar == nullptr) {
        throw std::invalid_argument(
            "XYZLut can only be constructed from a LidarInfo or CameraInfo");
    }

    // apply extrinsics after lidar_to_sensor_transform so the
    // resulting LUT will produce the coordinates in
    // "extrinsics frame" instead of "sensor frame"
    mat4d ext_transform = lidar->sensor_to_body;
    ext_transform(0, 3) /= RANGE_UNIT;
    ext_transform(1, 3) /= RANGE_UNIT;
    ext_transform(2, 3) /= RANGE_UNIT;
    const mat4d transform = ext_transform * lidar->lidar_to_sensor_transform;
    return make_xyz_lut(lidar->w(), lidar->h(), RANGE_UNIT, lidar->beam_to_lidar_transform,
                        transform, lidar->beam_azimuth_angles, lidar->beam_altitude_angles);
}
}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
