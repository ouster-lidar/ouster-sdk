/**
 * @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "ouster/types.h"

namespace ouster {

/**
 * Datastructure for efficient operations on aggregated lidar data.
 *
 * Stores each field (range, intensity, etc.) contiguously as a H x W block of
 * 4-byte unsigned integers, where H is the number of beams and W is the
 * horizontal resolution (e.g. 512, 1024, 2048).
 *
 * Note: this is the "staggered" representation where each column corresponds
 * to a single measurement in time. Use the destagger() function to create an
 * image where columns correspond to a single azimuth angle.
 */
class LidarScan {
   public:
    static constexpr int N_FIELDS = 4;

    using raw_t = uint32_t;
    using ts_t = std::chrono::nanoseconds;
    using data_t = Eigen::Array<raw_t, Eigen::Dynamic, N_FIELDS>;

    using DynStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;

    /** XYZ coordinates with dimensions arranged contiguously in columns */
    using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

    /** Data fields reported per channel */
    enum Field { RANGE, INTENSITY, AMBIENT, REFLECTIVITY };

    /** Measurement block information, other than the channel data */
    struct BlockHeader {
        ts_t timestamp;
        uint32_t encoder;
        uint32_t status;
    };

    /* Members variables: use with caution, some of these will become private */
    std::ptrdiff_t w{0};
    std::ptrdiff_t h{0};
    data_t data{};
    std::vector<BlockHeader> headers{};
    int32_t frame_id{-1};

    /** The default constructor creates an invalid 0 x 0 scan */
    LidarScan() = default;

    /**
     * Initialize an empty scan with the given horizontal / vertical resolution.
     *
     * @param w horizontal resoulution, i.e. the number of measurements per scan
     * @param h vertical resolution, i.e. the number of channels
     */
    LidarScan(size_t w, size_t h)
        : w{static_cast<std::ptrdiff_t>(w)},
          h{static_cast<std::ptrdiff_t>(h)},
          data{w * h, N_FIELDS},
          headers{w, BlockHeader{ts_t{0}, 0, 0}} {};

    /**
     * Access timestamps as a vector.
     *
     * @returns copy of the measurement timestamps as a vector
     */
    std::vector<LidarScan::ts_t> timestamps() const {
        std::vector<LidarScan::ts_t> res;
        res.reserve(headers.size());
        for (const auto& h : headers) res.push_back(h.timestamp);
        return res;
    }

    /**
     * Access measurement block header fields.
     *
     * @return the header values for the specified measurement id
     */
    BlockHeader& header(size_t m_id) { return headers.at(m_id); }

    /** @copydoc header(size_t m_id) */
    const BlockHeader& header(size_t m_id) const { return headers.at(m_id); }

    /**
     * Access measurement block data.
     *
     * @param m_id the measurement id of the desired block
     * @return a view of the measurement block data
     */
    Eigen::Map<data_t, Eigen::Unaligned, DynStride> block(size_t m_id) {
        return Eigen::Map<data_t, Eigen::Unaligned, DynStride>(
            data.row(m_id).data(), h, N_FIELDS, {w * h, w});
    }

    /** @copydoc block(size_t m_id) */
    Eigen::Map<const data_t, Eigen::Unaligned, DynStride> block(
        size_t m_id) const {
        return Eigen::Map<const data_t, Eigen::Unaligned, DynStride>(
            data.row(m_id).data(), h, N_FIELDS, {w * h, w});
    }

    /**
     * Access a lidar data field.
     *
     * @param f the field to view
     * @return a view of the field data
     */
    Eigen::Map<img_t<raw_t>> field(Field f) {
        return Eigen::Map<img_t<raw_t>>(data.col(f).data(), h, w);
    }

    /** @copydoc field(Field f) */
    Eigen::Map<const img_t<raw_t>> field(Field f) const {
        return Eigen::Map<const img_t<raw_t>>(data.col(f).data(), h, w);
    }
};

/** Equality for column headers. */
inline bool operator==(const LidarScan::BlockHeader& a,
                       const LidarScan::BlockHeader& b) {
    return a.timestamp == b.timestamp && a.encoder == b.encoder &&
           a.status == b.status;
}

/** Equality for scans. */
inline bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.w == b.w && a.h == b.h && (a.data == b.data).all() &&
           a.headers == b.headers && a.frame_id && b.frame_id;
}

/** Not Equality for scans. */
inline bool operator!=(const LidarScan& a, const LidarScan& b) {
    return !(a == b);
}

/** Lookup table of beam directions and offsets. */
struct XYZLut {
    LidarScan::Points direction;
    LidarScan::Points offset;
};

/**
 * Generate a set of lookup tables useful for computing cartesian coordinates
 * from ranges.
 *
 * The lookup tables are:
 * - direction: a matrix of unit vectors pointing radially outwards
 * - offset: a matrix of offsets dependent on beam origin distance from lidar
 *           origin
 *
 * Each table is an n x 3 array of doubles stored in column-major order where
 * each row corresponds to the nth point in a lidar scan, with 0 <= n < h*w.
 *
 * @param w number of columns in the lidar scan. e.g. 512, 1024, or 2048
 * @param h number of rows in the lidar scan
 * @param range_unit the unit, in meters, of the range,  e.g. sensor::range_unit
 * @param lidar_origin_to_beam_origin_mm the radius to the beam origin point of
 *        the unit, in millimeters
 * @param transform additional transformation to apply to resulting points
 * @param azimuth_angles_deg azimuth offsets in degrees for each of h beams
 * @param altitude_angles_deg altitude in degrees for each of h beams
 * @return xyz direction and offset vectors for each point in the lidar scan
 */
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg);

/**
 * Convenient overload that uses parameters from the supplied sensor_info.
 *
 * @param sensor metadata returned from the client
 * @return xyz direction and offset vectors for each point in the lidar scan
 */
inline XYZLut make_xyz_lut(const sensor::sensor_info& sensor) {
    return make_xyz_lut(
        sensor.format.columns_per_frame, sensor.format.pixels_per_column,
        sensor::range_unit, sensor.lidar_origin_to_beam_origin_mm,
        sensor.lidar_to_sensor_transform, sensor.beam_azimuth_angles,
        sensor.beam_altitude_angles);
}

/**
 * Convert LidarScan to cartesian points.
 *
 * @param scan a LidarScan
 * @param xyz_lut lookup tables generated by make_xyz_lut
 * @return cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col
 */
LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut);

/**
 * Generate a destaggered version of a channel field.
 *
 * In the default staggered representation, each column corresponds to a single
 * timestamp. In the destaggered representation, each column corresponds to a
 * single azimuth angle, compensating for the azimuth offset of each beam.
 *
 * Destaggering is used for visualizing lidar data as an image or for algorithms
 * that exploit the structure of the lidar data, such as beam_uniformity in
 * ouster_viz, or computer vision algorithms.
 *
 * For example:
 *     destagger(lidarscan.field(Field::INTENSITY))
 *     destagger(lidarscan.field(Field::INTENSITY).cast<double>())
 *
 * @param img the channel field
 * @param pixel_shift_by_row offsets, usually queried from the sensor
 * @param inverse perform the inverse operation
 * @return destaggered version of the image
 */
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row,
                          bool inverse = false) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    if (pixel_shift_by_row.size() != h)
        throw std::invalid_argument{"image height does not match shifts size"};

    img_t<T> destaggered{h, w};
    for (size_t u = 0; u < h; u++) {
        const std::ptrdiff_t offset =
            ((inverse ? -1 : 1) * pixel_shift_by_row[u] + w) % w;

        destaggered.row(u).segment(offset, w - offset) =
            img.row(u).segment(0, w - offset);
        destaggered.row(u).segment(0, offset) =
            img.row(u).segment(w - offset, offset);
    }
    return destaggered;
}

/**
 * Generate a staggered version of a channel field.
 *
 * @param img
 * @param pixel_shift_by_row
 * @return staggered version of the image
 */
template <typename T>
inline img_t<T> stagger(const Eigen::Ref<const img_t<T>>& img,
                        const std::vector<int>& pixel_shift_by_row) {
    return destagger(img, pixel_shift_by_row, true);
}

/**
 * Parse lidar packets into a LidarScan.
 *
 * Make a function that batches a single scan (revolution) of data to a
 * LidarScan.
 */
class ScanBatcher {
    std::ptrdiff_t w;
    std::ptrdiff_t h;
    uint16_t next_m_id;
    LidarScan ls_write;

   public:
    sensor::packet_format pf;

    /**
     * Create a batcher given information about the scan and packet format.
     *
     * @param w number of columns in the lidar scan. One of 512, 1024, or 2048
     * @param pf expected format of the incoming packets used for parsing
     */
    ScanBatcher(size_t w, const sensor::packet_format& pf);

    /**
     * Add a packet to the scan.
     *
     * @param packet_buf the lidar packet
     * @param lidar scan to populate
     * @return true when the provided lidar scan is ready to use
     */
    bool operator()(const uint8_t* packet_buf, LidarScan& ls);
};

}  // namespace ouster
