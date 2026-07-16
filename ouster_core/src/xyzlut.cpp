
#include "ouster/core/xyzlut.h"

#include "ouster/core/lidar_frame.h"

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

XYZLut make_xyz_lut(const SensorInfo& sensor, bool use_extrinsics) {
    mat4d transform = sensor.lidar_to_sensor_transform;
    if (use_extrinsics) {
        // apply extrinsics after lidar_to_sensor_transform so the
        // resulting LUT will produce the coordinates in
        // "extrinsics frame" instead of "sensor frame"
        mat4d ext_transform = sensor.sensor_to_body;
        ext_transform(0, 3) /= RANGE_UNIT;
        ext_transform(1, 3) /= RANGE_UNIT;
        ext_transform(2, 3) /= RANGE_UNIT;
        transform = ext_transform * sensor.lidar_to_sensor_transform;
    }
    return make_xyz_lut(sensor.format.columns_per_frame, sensor.format.pixels_per_column,
                        RANGE_UNIT, sensor.beam_to_lidar_transform, transform,
                        sensor.beam_azimuth_angles, sensor.beam_altitude_angles);
}
}  // namespace impl

OUSTER_DIAGNOSTIC_PUSH
OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED
PointCloudXYZd cartesian(const LidarFrame& frame, const XYZLut& lut) {
    return cartesian(frame.field(ChanField::RANGE), lut);
}
OUSTER_DIAGNOSTIC_POP

PointCloudXYZd cartesian(const Eigen::Ref<const img_t<uint32_t>>& range, const XYZLut& lut) {
    if (range.cols() * range.rows() != lut.direction.rows()) {
        throw std::invalid_argument("unexpected image dimensions");
    }

    PointCloudXYZd points(range.rows() * range.cols(), 3);
    impl::cartesianT<double>(points, range, lut.direction, lut.offset);
    return points;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
