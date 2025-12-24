
#include "ouster/xyzlut.h"

#include "ouster/lidar_scan.h"

namespace ouster {
namespace sdk {
namespace core {

XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    const mat4d& beam_to_lidar_transform,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg) {
    if (w <= 0 || h <= 0) {
        throw std::invalid_argument("lut dimensions must be greater than zero");
    }

    if ((azimuth_angles_deg.size() != h || altitude_angles_deg.size() != h) &&
        (azimuth_angles_deg.size() != w * h ||
         altitude_angles_deg.size() != w * h)) {
        throw std::invalid_argument("unexpected scan dimensions");
    }

    double beam_to_lidar_euclidean_distance_mm = beam_to_lidar_transform(0, 3);
    if (beam_to_lidar_transform(2, 3) != 0) {
        beam_to_lidar_euclidean_distance_mm =
            std::sqrt(std::pow(beam_to_lidar_transform(0, 3), 2) +
                      std::pow(beam_to_lidar_transform(2, 3), 2));
    }

    XYZLut lut;

    Eigen::ArrayXd encoder(w * h);   // theta_e
    Eigen::ArrayXd azimuth(w * h);   // theta_a
    Eigen::ArrayXd altitude(w * h);  // phi

    if (azimuth_angles_deg.size() == h && altitude_angles_deg.size() == h) {
        // OS sensor
        const double azimuth_radians = M_PI * 2.0 / w;

        // populate angles for each pixel
        for (size_t col_idx = 0; col_idx < w; col_idx++) {
            for (size_t row_idx = 0; row_idx < h; row_idx++) {
                size_t i = (row_idx * w) + col_idx;
                encoder(i) = 2.0 * M_PI - (col_idx * azimuth_radians);
                azimuth(i) = -azimuth_angles_deg[row_idx] * M_PI / 180.0;
                altitude(i) = altitude_angles_deg[row_idx] * M_PI / 180.0;
            }
        }

    } else if (azimuth_angles_deg.size() == w * h &&
               altitude_angles_deg.size() == w * h) {
        // DF sensor
        // populate angles for each pixel
        for (size_t col_idx = 0; col_idx < w; col_idx++) {
            for (size_t row_idx = 0; row_idx < h; row_idx++) {
                size_t i = (row_idx * w) + col_idx;
                encoder(i) = 0;
                azimuth(i) = azimuth_angles_deg[i] * M_PI / 180.0;
                altitude(i) = altitude_angles_deg[i] * M_PI / 180.0;
            }
        }
    }

    // unit vectors for each pixel
    lut.direction = PointCloudXYZd{w * h, 3};
    lut.direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
    lut.direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
    lut.direction.col(2) = altitude.sin();

    // offsets due to beam origin
    lut.offset = PointCloudXYZd{w * h, 3};
    lut.offset.col(0) =
        encoder.cos() * beam_to_lidar_transform(0, 3) -
        lut.direction.col(0) * beam_to_lidar_euclidean_distance_mm;
    lut.offset.col(1) =
        encoder.sin() * beam_to_lidar_transform(0, 3) -
        lut.direction.col(1) * beam_to_lidar_euclidean_distance_mm;
    lut.offset.col(2) =
        -lut.direction.col(2) * beam_to_lidar_euclidean_distance_mm +
        beam_to_lidar_transform(2, 3);

    // apply the supplied transform
    auto rot = transform.topLeftCorner(3, 3).transpose();
    auto trans = transform.topRightCorner(3, 1).transpose();
    lut.direction.matrix() *= rot;
    lut.offset.matrix() *= rot;
    lut.offset.matrix() += trans.replicate(w * h, 1);

    // apply scaling factor
    lut.direction *= range_unit;
    lut.offset *= range_unit;

    return lut;
}

XYZLut make_xyz_lut(const SensorInfo& sensor, bool use_extrinsics) {
    mat4d transform = sensor.lidar_to_sensor_transform;
    if (use_extrinsics) {
        // apply extrinsics after lidar_to_sensor_transform so the
        // resulting LUT will produce the coordinates in
        // "extrinsics frame" instead of "sensor frame"
        mat4d ext_transform = sensor.extrinsic;
        ext_transform(0, 3) /= RANGE_UNIT;
        ext_transform(1, 3) /= RANGE_UNIT;
        ext_transform(2, 3) /= RANGE_UNIT;
        transform = ext_transform * sensor.lidar_to_sensor_transform;
    }
    return make_xyz_lut(
        sensor.format.columns_per_frame, sensor.format.pixels_per_column,
        RANGE_UNIT, sensor.beam_to_lidar_transform, transform,
        sensor.beam_azimuth_angles, sensor.beam_altitude_angles);
}

PointCloudXYZd cartesian(const LidarScan& scan, const XYZLut& lut) {
    return cartesian(scan.field(ChanField::RANGE), lut);
}

PointCloudXYZd cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                         const XYZLut& lut) {
    if (range.cols() * range.rows() != lut.direction.rows()) {
        throw std::invalid_argument("unexpected image dimensions");
    }

    PointCloudXYZd points(range.rows() * range.cols(), 3);
    cartesianT<double>(points, range, lut.direction, lut.offset);
    return points;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
