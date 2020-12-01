/**
 * @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <vector>

#include "ouster/types.h"

namespace ouster {

struct LidarScan {
    /**
     * XYZ coordinates with dimensions arranged contiguously in columns
     */
    using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

    using raw_t = uint32_t;

    using ts_t = std::chrono::nanoseconds;

    enum Field { RANGE, INTENSITY, NOISE, REFLECTIVITY };

    using data_t = Eigen::Array<raw_t, -1, 4>;

    struct BlockHeader {
        ts_t timestamp;
        uint32_t encoder;
        uint32_t status;
    };

    std::ptrdiff_t w;
    std::ptrdiff_t h;
    data_t data;
    std::vector<BlockHeader> headers;
    uint16_t frame_id;

    LidarScan() : w(0), h(0), data{0, 4}, headers(0), frame_id(0){};
    LidarScan(size_t w, size_t h)
        : w(w), h(h), data{w * h, 4}, headers(w), frame_id(0){};

    /** Access timestamps */
    std::vector<LidarScan::ts_t> timestamps() const {
        std::vector<LidarScan::ts_t> res;
        res.reserve(headers.size());
        for (const auto& h : headers) res.push_back(h.timestamp);
        return res;
    }

    /** Access azimuth block header fields */
    const BlockHeader& header(size_t m_id) const { return headers.at(m_id); }

    BlockHeader& header(size_t m_id) { return headers.at(m_id); }

    /** Access azimuth block data */
    Eigen::Map<data_t, 0, Eigen::Stride<-1, -1>> block(size_t m_id) {
        return Eigen::Map<data_t, 0, Eigen::Stride<-1, -1>>(
            data.row(m_id).data(), h, 4, {w * h, w});
    }

    Eigen::Map<const data_t, 0, Eigen::Stride<-1, -1>> block(
        size_t m_id) const {
        return Eigen::Map<const data_t, 0, Eigen::Stride<-1, -1>>(
            data.row(m_id).data(), h, 4, {w * h, w});
    }

    /** Access a channel field for the entire scan as a 2d array */
    Eigen::Map<img_t<raw_t>> field(Field f) {
        return Eigen::Map<img_t<raw_t>>(data.col(f).data(), h, w);
    }

    Eigen::Map<const img_t<raw_t>> field(Field f) const {
        return Eigen::Map<const img_t<raw_t>>(data.col(f).data(), h, w);
    }
};

/**
 * Equality for column headers.
 */
inline bool operator==(const LidarScan::BlockHeader& a,
                       const LidarScan::BlockHeader& b) {
    return a.timestamp == b.timestamp && a.encoder == b.encoder &&
           a.status == b.status;
}

/**
 * Equality for scans.
 */
inline bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.w == b.w && a.h == b.h && (a.data == b.data).all() &&
           a.headers == b.headers && a.frame_id && b.frame_id;
}

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
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm,
                    const mat4d& transform,
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
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row,
                          bool inverse = false) {
    const auto h = img.rows();
    const auto w = img.cols();

    // TODO: throw if pixels_shifts.size() != h

    img_t<T> destaggered{h, w};
    for (std::ptrdiff_t u = 0; u < h; u++) {
        const std::ptrdiff_t offset =
            ((inverse ? -1 : 1) * pixel_shift_by_row[u] + w) % w;

        destaggered.row(u).segment(offset, w - offset) =
            img.row(u).segment(0, w - offset);
        destaggered.row(u).segment(0, offset) =
            img.row(u).segment(w - offset, offset);
    }
    return destaggered;
}

template <typename T>
inline img_t<T> stagger(const Eigen::Ref<const img_t<T>>& img,
                        const std::vector<int>& pixel_shift_by_row) {
    return destagger(img, pixel_shift_by_row, true);
}

/**
 * Convenient overload that uses parameters from the supplied sensor_info
 * @param field
 * @param sensor
 * @return destaggered version of field
 */
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& field,
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
    img_t<double> range = scan.field(LidarScan::RANGE).cast<double>();
    auto raw = lut.direction.colwise() *
               Eigen::Map<Eigen::ArrayXd>(range.data(), range.size());
    return (raw.array() == 0.0).select(raw, raw + lut.offset);
}

}  // namespace ouster
