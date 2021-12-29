/**
 * @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "ouster/types.h"

namespace ouster {

// forward declarations
namespace impl {
struct FieldSlot;
}

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
    [[deprecated]] static constexpr int N_FIELDS = 4;

    using raw_t [[deprecated]] = uint32_t;
    using ts_t [[deprecated]] = std::chrono::nanoseconds;

    template <typename T>
    using Header = Eigen::Array<T, Eigen::Dynamic, 1>;

    /** XYZ coordinates with dimensions arranged contiguously in columns */
    using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

    /* Old names provided for compatibility, see sensor::ChanField */
    using Field [[deprecated]] = sensor::ChanField;
    [[deprecated]] static constexpr Field RANGE = sensor::RANGE;
    [[deprecated]] static constexpr Field INTENSITY = sensor::SIGNAL;
    [[deprecated]] static constexpr Field AMBIENT = sensor::NEAR_IR;
    [[deprecated]] static constexpr Field REFLECTIVITY = sensor::REFLECTIVITY;

    /** Measurement block information, other than the channel data */
    struct [[deprecated]] BlockHeader {
        ts_t timestamp;
        uint32_t encoder;
        uint32_t status;
    };

   private:
    Header<uint64_t> timestamp_;
    Header<uint16_t> measurement_id_;
    Header<uint32_t> status_;
    std::map<sensor::ChanField, impl::FieldSlot> fields_;
    std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>
        field_types_;

    LidarScan(size_t w, size_t h,
              std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>
                  field_types);

   public:
    /* Members variables: use with caution, some of these will become private */
    std::ptrdiff_t w{0};
    std::ptrdiff_t h{0};
    std::vector<BlockHeader> headers{};
    int32_t frame_id{-1};

    using FieldIter = decltype(field_types_)::const_iterator;

    /** The default constructor creates an invalid 0 x 0 scan */
    LidarScan();

    /**
     * Initialize a scan with fields configured for the LEGACY udp profile.
     *
     * @param w horizontal resoulution, i.e. the number of measurements per scan
     * @param h vertical resolution, i.e. the number of channels
     */
    LidarScan(size_t w, size_t h);

    /**
     * Initialize a scan with the default fields for a particular udp profile.
     *
     * @param w horizontal resoulution, i.e. the number of measurements per scan
     * @param h vertical resolution, i.e. the number of channels
     * @param profile udp profile for
     */
    LidarScan(size_t w, size_t h, sensor::UDPProfileLidar profile);

    /**
     * Initialize a scan with a custom set of fields.
     *
     * @param w horizontal resoulution, i.e. the number of measurements per scan
     * @param h vertical resolution, i.e. the number of channels
     * @param begin iterator of pairs of channel fields and types
     */
    template <typename Iterator>
    LidarScan(size_t w, size_t h, Iterator begin, Iterator end)
        : LidarScan(w, h, {begin, end}){};

    /*
     * Special member functions
     */
    LidarScan(const LidarScan& other);
    LidarScan(LidarScan&& other);
    LidarScan& operator=(const LidarScan& other);
    LidarScan& operator=(LidarScan&& other);
    ~LidarScan();

    /**
     * Access timestamps as a vector.
     *
     * @returns copy of the measurement timestamps as a vector
     */
    [[deprecated]] std::vector<LidarScan::ts_t> timestamps() const;

    /**
     * Access measurement block header fields.
     *
     * @return the header values for the specified measurement id
     */
    [[deprecated]] BlockHeader& header(size_t m_id);

    /** @copydoc header(size_t m_id) */
    [[deprecated]] const BlockHeader& header(size_t m_id) const;

    /**
     * Access a lidar data field.
     *
     * The type parameter T must match the dynamic type of the field. See the
     * constructor documentation for expected field types or query dynamically
     * for generic operations.
     *
     * @param f the field to view
     * @throw std::invalid_argument if T does not match the runtime field type
     * @return a view of the field data
     */
    template <typename T = uint32_t,
              typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
    Eigen::Ref<img_t<T>> field(sensor::ChanField f);

    /** @copydoc field(Field f) */
    template <typename T = uint32_t,
              typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
    Eigen::Ref<const img_t<T>> field(sensor::ChanField f) const;

    /**
     * Get the type of the specified field
     *
     * @param f the field to query
     * @return the type tag associated with the field
     */
    sensor::ChanFieldType field_type(sensor::ChanField) const;

    /**
     * A const forward iterator over field / type pairs
     */
    FieldIter begin() const;
    FieldIter end() const;

    /**
     * Access the measurement timestamp headers
     *
     * @return a view of timestamp as a w-element vector
     */
    Eigen::Ref<Header<uint64_t>> timestamp();

    /** @copydoc timestamp() */
    Eigen::Ref<const Header<uint64_t>> timestamp() const;

    /**
     * Access the measurement id headers
     *
     * @return a view of measurement ids as a w-element vector
     */
    Eigen::Ref<Header<uint16_t>> measurement_id();

    /** @copydoc measurement_id() */
    Eigen::Ref<const Header<uint16_t>> measurement_id() const;

    /**
     * Access the measurement status headers
     *
     * @return a view of measurement statuses as a w-element vector
     */
    Eigen::Ref<Header<uint32_t>> status();

    /** @copydoc status() */
    Eigen::Ref<const Header<uint32_t>> status() const;

    friend bool operator==(const LidarScan& a, const LidarScan& b);
};

/** Equality for column headers. */
bool operator==(const LidarScan::BlockHeader& a,
                const LidarScan::BlockHeader& b);

/** Equality for scans. */
bool operator==(const LidarScan& a, const LidarScan& b);

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
 * Generate a set of lookup tables useful for computing Cartesian coordinates
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
 * Convert LidarScan to Cartesian points.
 *
 * @param scan a LidarScan
 * @param xyz_lut lookup tables generated by make_xyz_lut
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col
 */
LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut);

/**
 * Convert a staggered range image to Cartesian points.
 *
 * @param a range image in the same format as the RANGE field of a LidarScan
 * @param xyz_lut lookup tables generated by make_xyz_lut
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col
 */
LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const XYZLut& lut);

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
 * @param img the channel field
 * @param pixel_shift_by_row offsets, usually queried from the sensor
 * @param inverse perform the inverse operation
 * @return destaggered version of the image
 */
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row,
                          bool inverse = false);

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
    std::vector<uint8_t> cache;
    bool cached_packet = false;

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
     * Create a batcher given information about the scan and packet format.
     *
     * @param info sensor metadata returned from the client
     */
    ScanBatcher(const sensor::sensor_info& info);

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

#include "ouster/impl/lidar_scan_impl.h"
