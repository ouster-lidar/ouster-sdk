/**
 * @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#if defined _WIN32
#pragma warning(push, 2)
#endif

#include <Eigen/Eigen>

#if defined _WIN32
#pragma warning(pop)
#endif

#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "ouster/types.h"

namespace ouster {

struct LidarScan {
    enum LidarScanIndex { RANGE, INTENSITY, NOISE, REFLECTIVITY };
    using ts_t = std::chrono::nanoseconds;
    using raw_t = uint32_t;
    using data_t = Eigen::Array<raw_t, Eigen::Dynamic, 4>;
    using index_t = std::ptrdiff_t;

    template <class T>
    using field_t = Eigen::Array<T, Eigen::Dynamic, 1>;

    using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

    index_t w;
    index_t h;

    data_t data;
    std::vector<ts_t> ts;

    uint32_t range(size_t u, size_t v) const {
        assert(u < (size_t)h && v < (size_t)w);
        return this->range()[this->ind(u, v)];
    }
    uint32_t intensity(size_t u, size_t v) const {
        assert(u < (size_t)h && v < (size_t)w);
        return this->intensity()[this->ind(u, v)];
    }

    LidarScan() : w(0), h(0), data{0, 4}, ts(0){};
    LidarScan(size_t w, size_t h) : w(w), h(h), data{w * h, 4}, ts(w){};

    // get ts in us. We may use us everywhere in our code. In that case, we will
    // remove this function. (Hao)
    std::chrono::microseconds ts_us(size_t v) const {
        assert(v < (size_t)w);
        return std::chrono::duration_cast<std::chrono::microseconds>(ts[v]);
    }

    inline index_t ind(const index_t u, const index_t v) const {
        return u * w + v;
    }

    data_t::ColXpr range() { return data.col(LidarScanIndex::RANGE); }
    data_t::ColXpr intensity() { return data.col(LidarScanIndex::INTENSITY); }
    data_t::ColXpr noise() { return data.col(LidarScanIndex::NOISE); }
    data_t::ColXpr reflectivity() {
        return data.col(LidarScanIndex::REFLECTIVITY);
    }

    data_t::ConstColXpr range() const {
        return data.col(LidarScanIndex::RANGE);
    }
    data_t::ConstColXpr intensity() const {
        return data.col(LidarScanIndex::INTENSITY);
    }
    data_t::ConstColXpr noise() const {
        return data.col(LidarScanIndex::NOISE);
    }
    data_t::ConstColXpr reflectivity() const {
        return data.col(LidarScanIndex::REFLECTIVITY);
    }
};

/**
 * Lookup table of beam directions and offsets
 */
struct XYZLut {
    LidarScan::Points direction;
    LidarScan::Points offset;
};

/**
 * Generate a matrix of unit vectors pointing radially outwards, useful for
 * efficiently computing cartesian coordinates from ranges.  The result is a
 * n x 3 array of doubles stored in column-major order where each row is the
 * unit vector corresponding to the nth point in a lidar scan, with 0 <= n <
 * h * w.
 * The ordering of the rows is consistent with LidarScan::ind(u, v) for the
 * 3D point corresponding to the pixel at row u, column v in the LidarScan.
 * @param w number of columns in the lidar scan. e.g. 512, 1024, or 2048.
 * @param h number of rows in the lidar scan
 * @param range_unit the unit, in meters, of the range,  e.g. sensor::range_unit
 * @param lidar_origin_to_beam_origin_mm the radius to the beam origin point of
 *  the unit, in millimeters
 * @param transform additional transformation to apply to resulting points
 * @param azimuth_angles_deg azimuth offsets in degrees for each of h beams
 * @param altitude_angles_Deg altitude in degrees for each of h beams
 * @return xyz direction unit vectors for each point in the lidar scan
 */
XYZLut make_xyz_lut(LidarScan::index_t w, LidarScan::index_t h,
                    double range_unit, double lidar_origin_to_beam_origin_mm,
                    const sensor::mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg);

/**
 * Convenient overload that uses parameters from the supplied sensor_info
 * @param sensor metadata returned from the client
 * @return xyz direction unit vectors for each point in the lidar scan
 */
inline XYZLut make_xyz_lut(const sensor::sensor_info& sensor) {
    return make_xyz_lut(
        sensor.format.columns_per_frame, sensor.format.pixels_per_column,
        sensor::range_unit, sensor.lidar_origin_to_beam_origin_mm,
        sensor.lidar_to_sensor_transform, sensor.beam_azimuth_angles,
        sensor.beam_altitude_angles);
}

/**
 * Generate a destaggered version of a field from LidarScan,
 * used for visualizing lidar data as an image view, or for algorithms
 * that exploit the structure of the lidar data, such as
 * beam_uniformity in ouster_viz, or computer vision algorithms.
 * You can re-stagger a destaggered image by setting template parameter
 * direction to be -1.
 *
 * For example:
 *     destagger(lidarscan.intensity())
 *     destagger(lidarscan.intensity().cast<double>())
 *
 * @param field
 * @param pixel_shift_by_row
 * @return destaggered version of field
 */
template <class T, int direction = 1>
inline LidarScan::field_t<T> destagger(
    const Eigen::Ref<const LidarScan::field_t<T>> field,
    const std::vector<int>& pixel_shift_by_row) {
    static_assert(direction == 1 || direction == -1,
                  "Destagger direction should be only +1 or -1");
    const LidarScan::index_t h = pixel_shift_by_row.size();
    const LidarScan::index_t w = field.size() / h;

    Eigen::Array<T, Eigen::Dynamic, 1> destaggered(h * w);
    for (LidarScan::index_t u = 0; u < h; u++) {
        const LidarScan::index_t offset =
            (direction * pixel_shift_by_row[u] + w) % w;

        destaggered.segment(u * w + offset, w - offset) =
            field.segment(u * w, w - offset);
        destaggered.segment(u * w, offset) =
            field.segment(u * w + w - offset, offset);
    }
    return destaggered;
}

/**
 * Convenient overload that uses parameters from the supplied sensor_info
 * @param field
 * @param sensor
 * @return destaggered version of field
 */
template <class T>
inline LidarScan::field_t<T> destagger(
    const Eigen::Ref<const LidarScan::field_t<T>> field,
    const sensor::sensor_info& sensor) {
    return destagger(field, sensor.format.pixel_shift_by_row);
}

/**
 * Convert LidarScan to cartesian points.
 * @param scan a LidarScan
 * @param xyz_lut a lookup table of unit vectors generated by make_xyz_lut
 * @return cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = scan.ind(u, v) for a pixel
 *         at row u, column v.
 */
inline LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut) {
    auto raw = lut.direction.colwise() * scan.range().cast<double>();
    return (raw.array() == 0.0).select(raw, raw + lut.offset);
}

}  // namespace ouster
