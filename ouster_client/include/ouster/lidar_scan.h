/**
 * Copyright (c) 2018, Ouster, Inc.  All rights reserved.  @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/defaults.h"
#include "ouster/field.h"
#include "ouster/types.h"

namespace ouster {

/*
 * Description for a field that we desire in a lidar scan
 */
struct FieldType {
    std::string name;                    ///< Name of the field
    sensor::ChanFieldType element_type;  ///< Type of field elements
    std::vector<size_t> extra_dims;      ///< Additional dimensions of the field
    FieldClass field_class =
        FieldClass::PIXEL_FIELD;  ///< Category of field, determines the
                                  ///< first dimensions of the field

    /**
     * Initialize a default FieldType with no name.
     */
    FieldType();

    /**
     * Initialize a lidar scan from another with only the indicated fields.
     * Casts, zero pads or removes fields from the original scan if necessary.
     *
     * @param[in] name_ Name of the field described by the type.
     * @param[in] element_type_ Primitive type for elements in the field.
     * @param[in] extra_dims_ Additional dimensions of the field
     * @param[in] class_ Category of field, determins the first dimensions of
                         the field
     */
    FieldType(const std::string& name_, sensor::ChanFieldType element_type_,
              const std::vector<size_t> extra_dims_ = {},
              FieldClass class_ = FieldClass::PIXEL_FIELD);

    /**
     * Less than operated needed to allow sorting FieldTypes by name
     *
     * @param[in] other The FieldType to compare with
     *
     * @return if the name is "less than" the provided FieldType
     */
    inline bool operator<(const FieldType& other) const {
        return name < other.name;
    }
};

/**
 * Get string representation of a FieldType.
 *
 * @param[in] field_type The field type to get the string representation of.
 *
 * @return string representation of the FieldType.
 */
std::string to_string(const FieldType& field_type);

/**
 * Equality for FieldTypes.
 *
 * @param[in] a The first type to compare.
 * @param[in] b The second type to compare.
 *
 * @return if a == b.
 */
bool operator==(const FieldType& a, const FieldType& b);

/**
 * Equality for FieldTypes.
 *
 * @param[in] a The first type to compare.
 * @param[in] b The second type to compare.
 *
 * @return if a != b.
 */
bool operator!=(const FieldType& a, const FieldType& b);

/**
 * Alias for the lidar scan field types
 */
using LidarScanFieldTypes = std::vector<FieldType>;

/**
 * Data structure for efficient operations on aggregated lidar data.
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
    template <typename T>
    using Header = Eigen::Array<T, Eigen::Dynamic, 1>;  ///< Header typedef

    /** XYZ coordinates with dimensions arranged contiguously in columns. */
    using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

   private:
    std::unordered_map<std::string, Field> fields_;

    // Required special case "fields"
    Field timestamp_;
    Field measurement_id_;
    Field status_;
    Field packet_timestamp_;
    Field pose_;

    /**
     * The alert flags field, from eUDP packet headers.
     */
    Field alert_flags_;

    LidarScan(size_t w, size_t h, LidarScanFieldTypes field_types,
              size_t columns_per_packet);

    /**
     * The number of packets used to produce a full scan given the width in
     * pixels and the number of columns per packet.
     */
    size_t packet_count_{0};

   public:
    /**
     * Pointer offsets to deal with strides.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    size_t w{0};

    /**
     * Pointer offsets to deal with strides.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    size_t h{0};

    /**
     * Number of columns contained in each packet making up the scan.
     */
    size_t columns_per_packet_{DEFAULT_COLUMNS_PER_PACKET};

    /**
     * Frame status - information from the packet header which corresponds to a
     * frame
     *
     * @warning Member variables: use with caution, some of these will become
     * private.
     */
    uint64_t frame_status{0};

    /**
     * Thermal shutdown counter. Please refer to the firmware documentation for
     * more information.
     */
    uint8_t shutdown_countdown{0};

    /**
     * Shot limiting counter. Please refer to the firmware documentation for
     * more information.
     */
    uint8_t shot_limiting_countdown{0};

    /**
     * The current frame ID.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    int64_t frame_id{-1};

    /** The default constructor creates an invalid 0 x 0 scan. */
    LidarScan();

    /**
     * Initialize a scan with fields configured for the LEGACY udp profile.
     *
     * @param[in] w horizontal resolution, i.e. the number of measurements per
     *              scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     *
     * Note, the number of columns per packet is set to the default
     * (DEFAULT_COLUMNS_PER_PACKET).
     */
    LidarScan(size_t w, size_t h);

    /**
     * Initialize a scan with fields configured as default for the provided
     * SensorInfo's configuration
     *
     * @param[in] sensor_info description of sensor to create scan for
     */
    LidarScan(const sensor::sensor_info& sensor_info);

    /**
     * Initialize a scan with the default fields for a particular udp profile.
     *
     * @param[in] w horizontal resolution, i.e. the number of measurements per
     *              scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] profile udp profile.
     * @param[in] columns_per_packet The number of columns per packet,
     *                               this argument is optional.
     */
    LidarScan(size_t w, size_t h, sensor::UDPProfileLidar profile,
              size_t columns_per_packet = DEFAULT_COLUMNS_PER_PACKET);

    /**
     * Initialize a scan with a custom set of fields.
     *
     * @tparam Iterator A standard template iterator for the custom fields.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     *              scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] begin begin iterator of pairs of channel fields and types.
     * @param[in] end end iterator of pairs of channel fields and types.
     * @param[in] columns_per_packet The number of columns per packet,
     *                               this argument is optional.
     */
    template <typename Iterator>
    LidarScan(size_t w, size_t h, Iterator begin, Iterator end,
              size_t columns_per_packet = DEFAULT_COLUMNS_PER_PACKET)
        : LidarScan(w, h, {begin, end}, columns_per_packet) {}

    /**
     * Initialize a lidar scan from another lidar scan.
     *
     * @param[in] other The other lidar scan to initialize from.
     */
    LidarScan(const LidarScan& other);

    /**
     * Initialize a lidar scan from another with only the indicated fields.
     * Casts, zero pads or removes fields from the original scan if necessary.
     *
     * @throw std::invalid_argument if field dimensions are incompatible
     *
     * @param[in] other The other lidar scan to initialize from.
     * @param[in] fields Fields to have in new lidar scan.
     */
    LidarScan(const LidarScan& other, const LidarScanFieldTypes& fields);

    /** @copydoc LidarScan(const LidarScan& other) */
    LidarScan(LidarScan&& other);

    /**
     * Copy.
     *
     * @param[in] other The lidar scan to copy from.
     */
    LidarScan& operator=(const LidarScan& other);

    /**
     * Copy via Move semantic.
     *
     * @param[in] other The lidar scan to copy from.
     */
    LidarScan& operator=(LidarScan&& other);

    /**
     * Lidar scan destructor.
     */
    ~LidarScan();

    /**
     * Get frame shot limiting status
     *
     * @return true if sensor is shot limiting
     */
    sensor::ShotLimitingStatus shot_limiting() const;

    /**
     * Get frame thermal shutdown status
     *
     * @return true if sensor is in thermal shutdown state.
     */
    sensor::ThermalShutdownStatus thermal_shutdown() const;

    /**
     * @defgroup ClientLidarScanField Access fields in a lidar scan
     * Access a lidar data field.
     *
     * @throw std::invalid_argument if T does not match the runtime field type.
     *
     * @tparam T The type parameter T must match the dynamic type of the field.
     * See the constructor documentation for expected field types or query
     * dynamically for generic operations.
     *
     * @param[in] f the field to view.
     *
     * @return a view of the field data.
     */

    /**
     * @copydoc ClientLidarScanField
     */
    template <typename T>
    Eigen::Ref<img_t<T>> field(const std::string& f);

    /**
     * @copydoc ClientLidarScanField
     */
    template <typename T>
    Eigen::Ref<const img_t<T>> field(const std::string& f) const;

    /**
     * @defgroup ClientLidarScanFieldString Access fields in a lidar scan
     * Access a lidar data field.
     *
     * @param[in] name string key of the field to access
     *
     * @return Field reference of the requested field
     */

    /**
     * @copydoc ClientLidarScanFieldString
     */
    Field& field(const std::string& name);

    /**
     * @copydoc ClientLidarScanFieldString
     */
    const Field& field(const std::string& name) const;

    /**
     * Check if a field exists
     *
     * @param[in] name string key of the field to check
     *
     * @return true if the lidar scan has the field, else false
     */
    bool has_field(const std::string& name) const;

    /**
     * Add a new zero-filled field to lidar scan.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field, or
     *        if flags dimensional requirements are not met
     *
     * @param[in] name string key of the field to add
     * @param[in] d descriptor of the field to add
     * @param[in] field_class class to be assigned to the field, e.g.
     * PIXEL_FIELD
     *
     * @return field
     */
    Field& add_field(const std::string& name, FieldDescriptor d,
                     FieldClass field_class = FieldClass::PIXEL_FIELD);

    /**
     * Add a new zero-filled field to lidar scan.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field
     *
     * @param[in] type Descriptor of the field to add.
     *
     * @return field The value of the field added.
     */
    Field& add_field(const FieldType& type);

    /**
     * Release the field and remove it from lidar scan
     *
     * @throw std::invalid_argument if field under key does not exist
     *
     * @param[in] name string key of the field to remove
     *
     * @return field The deleted field.
     */
    Field del_field(const std::string& name);

    /**
     * Get the type of the specified field.
     *
     * @param[in] name the string key of the field to query.
     *
     * @return the type associated with the field.
     */
    FieldType field_type(const std::string& name) const;

    /**
     * Get the FieldType of all fields in the scan
     *
     * @return The type associated with every field in the scan.
     */
    LidarScanFieldTypes field_types() const;

    /**
     * Reference to the internal fields map
     *
     * @return The unordered map of field type and field.
     */
    std::unordered_map<std::string, Field>& fields();

    /** @copydoc fields() */
    const std::unordered_map<std::string, Field>& fields() const;

    /**
     * Access the measurement timestamp headers.
     *
     * @return a view of timestamp as a w-element vector.
     */
    Eigen::Ref<Header<uint64_t>> timestamp();

    /**
     * @copydoc timestamp()
     */
    Eigen::Ref<const Header<uint64_t>> timestamp() const;

    /**
     * Access the packet timestamp headers (usually host time).
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    Eigen::Ref<Header<uint64_t>> packet_timestamp();

    /**
     * Access the host timestamp headers (usually host time).
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    Eigen::Ref<const Header<uint64_t>> packet_timestamp() const;

    /**
     * Access the packet alert flags headers.
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    Eigen::Ref<Header<uint8_t>> alert_flags();

    /**
     * Access the packet alert flags headers.
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    Eigen::Ref<const Header<uint8_t>> alert_flags() const;

    /**
     * Return the first valid packet timestamp
     *
     * @return the first valid packet timestamp, 0 if none available
     */
    uint64_t get_first_valid_packet_timestamp() const;

    /**
     * Return the first valid column timestamp
     *
     * @return the first valid column timestamp, 0 if none available
     */
    uint64_t get_first_valid_column_timestamp() const;

    /**
     * Access the measurement id headers.
     *
     * @return a view of measurement ids as a w-element vector.
     */
    Eigen::Ref<Header<uint16_t>> measurement_id();

    /** @copydoc measurement_id() */
    Eigen::Ref<const Header<uint16_t>> measurement_id() const;

    /**
     * Access the measurement status headers.
     *
     * @return a view of measurement statuses as a w-element vector.
     */
    Eigen::Ref<Header<uint32_t>> status();

    /** @copydoc status() */
    Eigen::Ref<const Header<uint32_t>> status() const;

    /**
     * Access the array of poses (per each timestamp). Cast to
     * ArrayView3<double> in order to access as 3d
     * @return 3d field of homogenous pose matrices, shaped (w, 4, 4).
     */
    Field& pose();

    /** @copydoc pose() */
    const Field& pose() const;

    /**
     * Assess completeness of scan.
     * @param[in] window The column window to use for validity assessment
     * @return whether all columns within given column window were valid
     */
    bool complete(sensor::ColumnWindow window) const;

    /**
     * Returns the number of lidar packets used to produce this scan.
     *
     * @return the number of packets
     */
    size_t packet_count() const;

    friend bool operator==(const LidarScan& a, const LidarScan& b);
};

/**
 * Get string representation of lidar scan field types.
 *
 * @param[in] field_types The field types to get the string representation of.
 *
 * @return string representation of the lidar scan field types.
 */
std::string to_string(const LidarScanFieldTypes& field_types);

/**
 * Get the lidar scan field types from lidar profile
 *
 * @param[in] udp_profile_lidar lidar profile
 *
 * @return The lidar scan field types
 */
LidarScanFieldTypes get_field_types(sensor::UDPProfileLidar udp_profile_lidar);

/**
 * Get the lidar scan field types from sensor info
 *
 * @param[in] info The sensor info to get the lidar scan field types from.
 *
 * @return The lidar scan field types
 */
LidarScanFieldTypes get_field_types(const sensor::sensor_info& info);

/**
 * Get string representation of a lidar scan.
 *
 * @param[in] ls The lidar scan to get the string representation of.
 *
 * @return string representation of the lidar scan.
 */
std::string to_string(const LidarScan& ls);

/** \defgroup ouster_client_lidar_scan_operators Ouster Client lidar_scan.h
 * Operators
 * @{
 */

/**
 * Equality for scans.
 *
 * @param[in] a The first scan to compare.
 * @param[in] b The second scan to compare.
 *
 * @return if a == b.
 */
bool operator==(const LidarScan& a, const LidarScan& b);

/**
 * NOT Equality for scans.
 *
 * @param[in] a The first scan to compare.
 * @param[in] b The second scan to compare.
 *
 * @return if a != b.
 */
inline bool operator!=(const LidarScan& a, const LidarScan& b) {
    return !(a == b);
}
/** @}*/

/** Lookup table of beam directions and offsets. */
struct XYZLut {
    LidarScan::Points direction;  ///< Lookup table of beam directions
    LidarScan::Points offset;     ///< Lookup table of beam offsets
};

/**
 * Generate a set of lookup tables useful for computing Cartesian coordinates
 * from ranges.
 *
 * The lookup tables are:
 * - direction: a matrix of unit vectors pointing radially outwards.
 * - offset: a matrix of offsets dependent on beam origin distance from lidar
 *           origin.
 *
 * Each table is an n x 3 array of doubles stored in column-major order where
 * each row corresponds to the nth point in a lidar scan, with 0 <= n < h*w.
 *
 * Projections to XYZ made with this XYZLut will be in the coordinate frame
 * defined by transform*beam_to_lidar_transform.
 *
 * @param[in] w number of columns in the lidar scan. e.g. 512, 1024, or 2048.
 * @param[in] h number of rows in the lidar scan.
 * @param[in] range_unit the unit, in meters, of the range,  e.g.
 * sensor::range_unit.
 * @param[in] beam_to_lidar_transform transform between beams and
 * lidar origin. Translation portion is in millimeters.
 * @param[in] transform additional transformation to apply to resulting points.
 * @param[in] azimuth_angles_deg azimuth offsets in degrees for each of h beams.
 * @param[in] altitude_angles_deg altitude in degrees for each of h beams.
 *
 * @return xyz direction and offset vectors for each point in the lidar scan.
 */
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    const mat4d& beam_to_lidar_transform,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg);

/**
 * Convenient overload that uses parameters from the supplied sensor_info.
 * Projections to XYZ made with this XYZLut will be in the sensor coordinate
 * frame defined in the sensor documentation unless use_extrinics is true.
 * Then the projections will be in the coordinate frame defined by the provided
 * extrinsics in the metadata.
 *
 * @param[in] sensor metadata returned from the client.
 * @param[in] use_extrinsics if true, applies the ``sensor.extrinsic`` transform
 *                           to the resulting "sensor frame" coordinates
 *
 * @return xyz direction and offset vectors for each point in the lidar scan.
 */
XYZLut make_xyz_lut(const sensor::sensor_info& sensor, bool use_extrinsics);

/** \defgroup ouster_client_lidar_scan_cartesian Ouster Client lidar_scan.h
 * XYZLut related items.
 * @{
 */
/**
 * Convert LidarScan to Cartesian points.
 *
 * @param[in] scan a LidarScan.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut);

/**
 * Convert a staggered range image to Cartesian points.
 *
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const XYZLut& lut);
/** @}*/

/** \defgroup ouster_client_destagger Ouster Client lidar_scan.h
 * @{
 */
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
 * @tparam T the datatype of the channel field.
 *
 * @param[in] img the channel field.
 * @param[in] pixel_shift_by_row offsets, usually queried from the sensor.
 * @param[in] inverse perform the inverse operation.
 *
 * @return destaggered version of the image.
 */
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row,
                          bool inverse = false);

/**
 * Generate a staggered version of a channel field.
 *
 * @tparam T the datatype of the channel field.
 *
 * @param[in] img the channel field.
 * @param[in] pixel_shift_by_row offsets, usually queried from the sensor.
 *
 * @return staggered version of the image.
 */
template <typename T>
inline img_t<T> stagger(const Eigen::Ref<const img_t<T>>& img,
                        const std::vector<int>& pixel_shift_by_row) {
    return destagger(img, pixel_shift_by_row, true);
}
/** @}*/
/**
 * Parse lidar packets into a LidarScan.
 *
 * Make a function that batches a single scan (revolution) of data to a
 * LidarScan.
 */
class ScanBatcher {
    size_t w;
    size_t h;
    uint16_t next_valid_m_id;
    uint16_t next_headers_m_id;
    uint16_t next_valid_packet_id;
    std::vector<uint8_t> cache;
    uint64_t cache_packet_ts;
    bool cached_packet = false;
    int64_t finished_scan_id = -1;
    size_t expected_packets;
    size_t batched_packets = 0;

    void _parse_by_col(const uint8_t* packet_buf, LidarScan& ls);
    void _parse_by_block(const uint8_t* packet_buf, LidarScan& ls);
    void finalize_scan(LidarScan& ls, bool raw_headers);

   public:
    sensor::packet_format pf;  ///< The packet format object used for decoding

    // clang-format off
    /**
     * Create a batcher given information about the scan and packet format.
     * @deprecated Use ScanBatcher::ScanBatcher(const sensor_info&) instead.
     *
     * @param[in] w number of columns in the lidar scan. One of 512, 1024, or
     * 2048.
     * @param[in] pf expected format of the incoming packets used for parsing.
     */
    ScanBatcher(size_t w, const sensor::packet_format& pf);
    // clang-format on

    /**
     * Create a batcher given information about the scan and packet format.
     *
     * @param[in] info sensor metadata returned from the client.
     */
    ScanBatcher(const sensor::sensor_info& info);

    /**
     * Add a packet to the scan.
     * @deprecated this method is deprecated in favor of one that accepts a
     * reference to a LidarPacket.
     *
     * @param[in] packet_buf a buffer containing raw bytes from a lidar packet.
     * @param[in] ls lidar scan to populate.
     *
     * @return true when the provided lidar scan is ready to use.
     */
    [[deprecated(
        "Use ScanBatcher::operator(LidarPacket&, LidarScan&) instead.")]] bool
    operator()(const uint8_t* packet_buf, LidarScan& ls);

    /**
     * Add a packet to the scan.
     *
     * @param[in] packet a LidarPacket.
     * @param[in] ls lidar scan to populate.
     *
     * @return true when the provided lidar scan is ready to use.
     */
    bool operator()(const ouster::sensor::LidarPacket& packet, LidarScan& ls);

    /**
     * Add a packet to the scan.
     *
     * @param[in] packet_buf a buffer containing raw bytes from a lidar packet.
     * @param[in] packet_ts timestamp of the packet (usually HOST time on
     * receive).
     * @param[in] ls lidar scan to populate.
     *
     * @return true when the provided lidar scan is ready to use.
     */
    bool operator()(const uint8_t* packet_buf, uint64_t packet_ts,
                    LidarScan& ls);
};

namespace pose_util {
typedef Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> Points;
typedef Eigen::Matrix<double, Eigen::Dynamic, 16, Eigen::RowMajor> Poses;
typedef Eigen::Matrix<double, 1, 16, Eigen::RowMajor> Pose;

/**
 * This function takes in a set of 3D points and a set of 4x4 pose matrices
 *
 * @param[out] dewarped An eigen matrix of shape (N, 3) to hold the dewarped
 * 3D points, where the same number of points are transformed by each
 * corresponding pose matrix.
 * @param[in] points A Eigen matrix of shape (N, 3) representing the 3D points.
 * Each row corresponds to a point in 3D space.
 * @param[in] poses A Eigen matrix of shape (W, 16) representing W 4x4
 * transformation matrices. Each row is a flattened 4x4 pose matrix
 */

void dewarp(Eigen::Ref<Points> dewarped, const Eigen::Ref<const Points> points,
            const Eigen::Ref<const Poses> poses);

/**
 * This function takes in a set of 3D points and a set of 4x4 pose matrices
 *
 * @param[in] points A Eigen matrix of shape (N, 3) representing the 3D points.
 * Each row corresponds to a point in 3D space.
 * @param[in] poses A Eigen matrix of shape (W, 16) representing W 4x4
 * transformation matrices. Each row is a flattened 4x4 pose matrix
 *
 * @return A matrix of shape (N, 3) containing the dewarped 3D points,
 * where the same number of points are transformed by each corresponding pose
 * matrix.
 */

Points dewarp(const Eigen::Ref<const Points> points,
              const Eigen::Ref<const Poses> poses);

/**
 *  Applies a single 4x4 pose transformation to a set of 3D points.
 *
 * This function takes in a set of 3D points and applies a single 4x4
 * transformation matrix (Pose) to all points.
 *
 * @param[out] transformed A matrix of shape (N, 3) containing the transformed
 * 3D points, where each point is rotated and translated by the given pose.
 * @param[in] points A matrix of shape (N, 3) representing the 3D points.
 *               Each row corresponds to a point in 3D space.
 * @param[in] pose A vector of 16 elements representing a flattened 4x4
 * transformation matrix.
 */

void transform(Eigen::Ref<Points> transformed,
               const Eigen::Ref<const Points> points,
               const Eigen::Ref<const Pose> pose);
/**
 *  Applies a single 4x4 pose transformation to a set of 3D points.
 *
 * This function takes in a set of 3D points and applies a single 4x4
 * transformation matrix (Pose) to all points.
 *
 * @param[in] points A matrix of shape (N, 3) representing the 3D points.
 *               Each row corresponds to a point in 3D space.
 * @param[in] pose A vector of 16 elements representing a flattened 4x4
 * transformation matrix.
 *
 * @return A matrix of shape (N, 3) containing the transformed 3D points,
 *         where each point is rotated and translated by the given pose.
 */

Points transform(const Eigen::Ref<const Points> points,
                 const Eigen::Ref<const Pose> pose);

}  // namespace pose_util
}  // namespace ouster

#include "ouster/impl/cartesian.h"
#include "ouster/impl/lidar_scan_impl.h"
