/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster client datatypes and constants
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/core/chanfield.h"
#include "ouster/core/data_format.h"
#include "ouster/core/deprecation.h"
#include "ouster/core/impl/cache_ptr.h"
#include "ouster/core/sensor_config.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zone_monitor.h"
#include "ouster/core/zone_state.h"
#include "version.h"

namespace ouster {
namespace sdk {
namespace core {

using nonstd::optional;

/** Forward declaration for Field */
class Field;

/** Unit of range from sensor packet, in meters. */
constexpr double RANGE_UNIT = 0.001;

/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> GEN1_ALTITUDE_ANGLES;
/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> GEN1_AZIMUTH_ANGLES;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d DEFAULT_IMU_TO_SENSOR_TRANSFORM;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d DEFAULT_LIDAR_TO_SENSOR_TRANSFORM;

/*
 * Constants used for configuration. Refer to the sensor documentation for the
 * meaning of each option.
 */

/**
 * Get number of columns in a frame for a lidar mode.
 *
 * @param[in] mode LidarMode to get the number of columns for.
 *
 * @return number of columns per rotation for the mode.
 */
OUSTER_DEPRECATED_MSG("LidarMode.columns", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_API_FUNCTION
uint32_t n_cols_of_lidar_mode(LidarMode mode);

/**
 * Get the lidar rotation frequency from lidar mode.
 *
 * @param[in] mode Lidar mode to get the rotation frequency from.
 *
 * @return lidar rotation frequency in Hz.
 */
OUSTER_DEPRECATED_MSG("LidarMode.fps", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_API_FUNCTION
unsigned int frequency_of_lidar_mode(LidarMode mode);

/**
 * Get client version.
 *
 * @return client version string
 */
OUSTER_API_FUNCTION
std::string client_version();

// clang-format off

/**
 * Table of accessors for extracting data from imu and lidar packets.
 *
 * In the user guide, refer to section 9 for the lidar packet format and section
 * 10 for imu packets.
 *
 * For 0 <= n < columns_per_packet, nth_col(n, packet_buf) returns a pointer to
 * the nth measurement block. For 0 <= m < pixels_per_column, nth_px(m, col_buf)
 * returns the mth channel data block.
 *
 * Use imu_la_{x,y,z} to access the acceleration in the corresponding
 * direction. Use imu_av_{x,y,z} to read the angular velocity.
 */
class OUSTER_API_CLASS PacketFormat {
   protected:
    struct Impl;
    std::shared_ptr<const Impl> impl_;

    std::vector<std::pair<std::string, std::pair<ChanFieldType, int>>> field_types_;

   public:
    /**
     * Construct packet format from data format.
     *
     * @param[in] format data format
     */
    OUSTER_API_FUNCTION
    PacketFormat(const DataFormat& format);

    /**
     * Construct packet format from sensor info
     *
     * @param[in] info sensor info
     */
    OUSTER_API_FUNCTION
    PacketFormat(const SensorInfo& info);

    using FieldIter =
        decltype(field_types_)::const_iterator;  ///< iterator over field types
                                                 ///< of packet

    const UDPProfileLidar
        udp_profile_lidar;  ///< udp lidar profile of packet format
    const UDPProfileIMU udp_profile_imu;  ///< udp imu profile of packet format
    const HeaderType header_type;         ///< header type of packets
    const size_t lidar_packet_size;       ///< lidar packet size
    const size_t imu_packet_size;         ///< imu packet size
    const size_t zone_packet_size;        ///< zone monitoring packet size
    const uint32_t columns_per_packet;         ///< columns per lidar packet
    const uint32_t pixels_per_column;          ///< pixels per column for lidar

    const size_t imu_measurements_per_packet;  ///< number of accel/gyro
                                               ///< measurements per imu packet
    const size_t imu_packets_per_frame;        ///< number of imu packets per
                                               ///< frame

    const size_t packet_header_size;  ///< Size in bytes of the packet header.
    const size_t col_header_size;     ///< Size in bytes of the column header.
    const size_t col_footer_size;     ///< Size in bytes of the column footer
    const size_t col_size;  ///< Total size in bytes of a single column block
                            ///< (header + data + footer).
    const size_t packet_footer_size;  ///< Size in bytes of the packet footer

    const uint32_t max_frame_id;   ///< maximum frame id for this packet format
    bool zone_monitoring_enabled;  ///< if yes, zone monitoring is on

    /**
     * Read the packet type packet header.
     * Applicable to non-legacy Lidar and IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the packet type.
     */
    OUSTER_API_FUNCTION
    uint16_t packet_type(const uint8_t* packet_buf) const;

    /**
     * Read the frame_id packet header.
     * Applicable to lidar packets and non-legacy IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the frame id.
     */
    OUSTER_API_FUNCTION
    uint32_t frame_id(const uint8_t* packet_buf) const;

    /**
     * Read the initialization id packet header.
     * Applicable to non-legacy Lidar and IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the init id.
     */
    OUSTER_API_FUNCTION
    uint32_t init_id(const uint8_t* packet_buf) const;

    /**
     * Read the packet serial number header.
     * Applicable to non-legacy Lidar and IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the serial number.
     */
    OUSTER_API_FUNCTION
    uint64_t prod_sn(const uint8_t* packet_buf) const;

    /**
     * Read the alert flags.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the alert flags byte.
     */
    OUSTER_API_FUNCTION
    uint8_t alert_flags(const uint8_t* lidar_buf) const;

    /**
     * Read the packet thermal shutdown countdown
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the thermal shutdown countdown.
     */
    OUSTER_API_FUNCTION
    uint16_t countdown_thermal_shutdown(const uint8_t* lidar_buf) const;

    /**
     * Read the packet shot limiting countdown
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the shot limiting countdown.
     */
    OUSTER_API_FUNCTION
    uint16_t countdown_shot_limiting(const uint8_t* lidar_buf) const;

    /**
     * Read the packet thermal shutdown header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the thermal shutdown status
     */
    OUSTER_API_FUNCTION
    ThermalShutdownStatus thermal_shutdown(const uint8_t* lidar_buf) const;

    /**
     * Read the packet shot limiting header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the shot limiting status
     */
    OUSTER_API_FUNCTION
    ShotLimitingStatus shot_limiting(const uint8_t* lidar_buf) const;

    /**
     * Get the bit width of the specified channel field.
     *
     * @param[in] f the channel field to query.
     *
     * @return a type tag specifying the bitwidth of the requested field or
     * ChannelFieldType::VOID if it is not supported by the packet format.
     */
    OUSTER_API_FUNCTION
    ChanFieldType field_type(const std::string& f) const;

    /**
     * A const forward iterator over field / type pairs.
     *
     * @return Iterator pointing to the first element in the field type of
     * packets.
     *
     */
    OUSTER_API_FUNCTION
    FieldIter begin() const;

    /**
     * A const forward iterator over field / type pairs.
     *
     * @return Iterator pointing to the last element in the field type of
     * packets.
     */
    OUSTER_API_FUNCTION
    FieldIter end() const;

    /**
     * Get pointer to the packet footer of a lidar buffer.
     *
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to packet footer of lidar buffer, can be nullptr if
     * packet format doesn't have packet footer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* footer(const uint8_t* lidar_buf) const;

    // Measurement block accessors

    /**
     * Get pointer to nth column of a lidar buffer.
     *
     * @param[in] col_idx which column.
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to nth column of lidar buffer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* nth_col(size_t col_idx, const uint8_t* lidar_buf) const;

    /**
     * Read column timestamp from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column timestamp.
     */
    OUSTER_API_FUNCTION
    uint64_t col_timestamp(const uint8_t* col_buf) const;

    /**
     * Read measurement id from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column measurement id.
     */
    OUSTER_API_FUNCTION
    uint16_t col_measurement_id(const uint8_t* col_buf) const;

    /**
     * Read column status from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column status.
     */
    OUSTER_API_FUNCTION
    uint32_t col_status(const uint8_t* col_buf) const;

    /**
     * @brief Encodes the column value.
     *
     * This function encodes the column encoder value. It is only present in
     * LEGACY lidar packets and col_measurement_id() should generally be used
     * instead.
     *
     * @param[in] col_buf A measurement block pointer returned by `nth_col()`.
     *
     * @return Encoded column value.
     */
    OUSTER_API_FUNCTION
    uint32_t col_encoder(const uint8_t* col_buf) const;

    /**
     * @brief Retrieves the current frame id
     *
     * This function returns the frame id of a column. It is only present in
     * LEGACY lidar packets and frame_id() should generally be used instead.
     *
     * @param[in] col_buf A measurement block pointer returned by `nth_col()`.
     *
     * @return The current frame id.
     */
    OUSTER_API_FUNCTION
    uint16_t col_frame_id(const uint8_t* col_buf) const;

    /**
     * Copy the specified channel field out of a packet measurement block.
     *
     * @tparam T T should be a numeric type large enough to store
     * values of the specified field. Otherwise, data will be truncated.
     *
     * @param[in] col_buf a measurement block pointer returned by `nth_col()`.
     * @param[in] field_name the channel field to copy.
     * @param[out] dst destination array of size pixels_per_column * dst_stride.
     * @param[in] dst_stride stride for writing to the destination array.
     */
    template <typename T>
    OUSTER_API_FUNCTION
    void col_field(const uint8_t* col_buf, const std::string& field_name,
                   T* dst, int dst_stride = 1) const;

    /**
     * Returns maximum available size of parsing block usable with block_field
     *
     * @return if packet format does not allow for block parsing, returns 0
     */
    OUSTER_API_FUNCTION
    int block_parsable() const;

    /**
     * Copy the specified channel field out of a packet measurement block.
     * Faster traversal than col_field, but has to copy the entire packet all at
     * once.
     *
     * @tparam T T should be a numeric type large enough to store
     * values of the specified field. Otherwise, data will be truncated.
     *
     * @param[out] data destination array to copy field data into.
     * @param[in] cols number of columns in the destination array.
     * @param[in] field_name the channel field to copy.
     * @param[in] lidar_buf the lidar buffer.
     */
    template <typename T, int BlockDim>
    OUSTER_API_FUNCTION
    void block_field(T* data, int cols, const std::string& field_name,
                     const uint8_t* lidar_buf) const;

    // Per-pixel channel data block accessors

    /**
     * Get pointer to nth pixel of a column buffer.
     *
     * @param[in] px_idx which pixel.
     * @param[in] col_buf the column buffer.
     *
     * @return pointer to nth pixel of a column buffer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* nth_px(size_t px_idx, const uint8_t* col_buf) const;

    // IMU packet accessors

    /**
     * Get pointer to nth measurement of an IMU buffer.
     * This applies to PROFILE_IMU_ACCEL32_GYRO32_NMEA which contains multiple
     * measurements within each packet.
     * In PROFILE_IMU_LEGACY profile, each packet only contains one measurement.
     *
     * @param[in] meas_idx which measurement.
     * @param[in] imu_buf the imu buffer.
     *
     * @return pointer to nth measurement of an imu buffer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* imu_nth_measurement(size_t meas_idx,
                                       const uint8_t* imu_buf) const;

    /**
     * Read NMEA timestamp from imu packet buffer.
     * Only available in PROFILE_ACCEL32_GYRO32_NMEA, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return NMEA timestamp from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_nmea_ts(const uint8_t* imu_buf) const;

    // uint32_t imu_nmea_status(const uint8_t* imu_buf) const;

    // TODO: would have been std::string_view if we had access to cpp17
    /**
     * Read NMEA sentence from an IMU buffer.
     * Only available in PROFILE_ACCEL32_GYRO32_NMEA.
     *
     * @param[in] imu_buf the imu buffer.
     *
     * @return NMEA sentence string
     */
    OUSTER_API_FUNCTION
    std::string imu_nmea_sentence(const uint8_t* imu_buf) const;

    /**
     * Read sys ts from imu packet buffer.
     * Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return sys ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_sys_ts(const uint8_t* imu_buf) const;

    /**
     * Read acceleration timestamp.
     * Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_accel_ts(const uint8_t* imu_buf) const;

    /**
     * Read gyro timestamp.
     * Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return gyro ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_gyro_ts(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in x.
     * Acceleration unit is g.
     *
     * @param[in] imu_buf pointer to the imu buffer containing the measurement
     *
     * @return acceleration in x.
     */
    OUSTER_API_FUNCTION
    float imu_la_x(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in y.
     * Acceleration unit is g.
     *
     * @param[in] imu_buf pointer to the imu buffer containing the measurement
     *
     * @return acceleration in y.
     */
    OUSTER_API_FUNCTION
    float imu_la_y(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in z.
     * Acceleration unit is g.
     *
     * @param[in] imu_buf pointer to the imu buffer containing the measurement
     *
     * @return acceleration in z.
     */
    OUSTER_API_FUNCTION
    float imu_la_z(const uint8_t* imu_buf) const;

    /**
     * Read angular velocity in x.
     * Angular velocity unit is degrees/second.
     *
     * @param[in] imu_buf pointer to the imu buffer containing the measurement
     *
     * @return angular velocity in x.
     */
    OUSTER_API_FUNCTION
    float imu_av_x(const uint8_t* imu_buf) const;

    /**
     * Read angular velocity in y.
     * Angular velocity unit is degrees/second.
     *
     * @param[in] imu_buf pointer to the imu buffer containing the measurement
     *
     * @return angular velocity in y.
     */
    OUSTER_API_FUNCTION
    float imu_av_y(const uint8_t* imu_buf) const;

    /**
     * Read angular velocity in z.
     * Angular velocity unit is degrees/second.
     *
     * @param[in] imu_buf pointer to the imu buffer containing the measurement
     *
     * @return angular velocity in z.
     */
    OUSTER_API_FUNCTION
    float imu_av_z(const uint8_t* imu_buf) const;

    /**
     * Parse imu measurements of acceleration.
     *
     * Does not work with non-legacy IMU format.
     *
     * @param[in] col_offset offset to the first column to fill data in
     * @param[in] imu_buf pointer to imu packet data
     * @param[out] accel field to store acceleration data, which must be of
     *             shape (N,3) and float type
     */
    OUSTER_API_FUNCTION
    void parse_accel(size_t col_offset, const uint8_t* imu_buf, Field& accel);

    /**
     * Parse imu measurements of angular velocity.
     *
     * Does not work with non-legacy IMU format.
     *
     * @param[in] col_offset offset to the first column to fill data in
     * @param[in] imu_buf pointer to imu packet data
     * @param[out] gyro field to store angular velocity data, which must be of
     *             shape (N,3) and float type
     */
    OUSTER_API_FUNCTION
    void parse_gyro(size_t col_offset, const uint8_t* imu_buf, Field& gyro);

    /**
     * Get the mask of possible values that can be parsed by the channel field
     *
     * @param[in] f the channel field
     *
     * @return mask of possible values
     */
    OUSTER_API_FUNCTION
    uint64_t field_value_mask(const std::string& f) const;

    /**
     * Get number of bits in the channel field
     *
     * @param[in] f the channel field
     *
     * @return number of bits
     */
    OUSTER_API_FUNCTION
    int field_bitness(const std::string& f) const;

    /**
     * Return the CRC contained in the packet if present
     *
     * @param[in] buf the packet buffer.
     * @param[in] buffer_size size of buffer
     *
     * @return crc contained in the packet if present
     */
    OUSTER_API_FUNCTION
    optional<uint64_t> crc(const uint8_t* buf, size_t buffer_size) const;

    /**
     * Calculate the CRC for the given packet.
     *
     * @param[in] buf the packet buffer.
     * @param[in] buffer_size size of buffer
     *
     * @return calculated crc of the packet
     */
    OUSTER_API_FUNCTION
    uint64_t calculate_crc(const uint8_t* buf, size_t buffer_size) const;

    /**
     * Get zone monitoring timestamp.
     *
     * @param[in] zone_packet zone monitoring packet
     * @return zone monitoring timestamp
     */
    OUSTER_API_FUNCTION
    uint64_t zone_timestamp(const uint8_t* zone_packet) const;

    /**
     * Get live zoneset hash.
     *
     * @param[in] zone_packet zone monitoring packet
     * @return 256bit hash of zone config
     */
    OUSTER_API_FUNCTION
    std::array<uint8_t, 32> live_zoneset_hash(const uint8_t* zone_packet) const;

    /**
     * Get pointer to nth measurement of a Zone Monitoring packet.
     *
     * @param[in] meas_idx which measurement
     * @param[in] zone_packet zone monitoring packet
     *
     * @return pointer to nth measurement of a zone monitoring packet.
     */
    OUSTER_API_FUNCTION
    const uint8_t* zone_nth_measurement(size_t meas_idx,
                                        const uint8_t* zone_packet) const;

    /**
     * Get zone status (live or not live).
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return true if the zone is live
     */
    OUSTER_API_FUNCTION
    bool zone_live(const uint8_t* zone_buffer) const;

    /**
     * Get zone id.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone id
     */
    OUSTER_API_FUNCTION
    uint8_t zone_id(const uint8_t* zone_buffer) const;

    /**
     * Get zone error flags.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone error flags
     */
    OUSTER_API_FUNCTION
    uint8_t zone_error_flags(const uint8_t* zone_buffer) const;

    /**
     * Get zone trigger type (Occupancy / Non-occupancy).
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone trigger type
     */
    OUSTER_API_FUNCTION
    uint8_t zone_trigger_type(const uint8_t* zone_buffer) const;

    /**
     * Get zone trigger status.
     * 0x0 deasserted, 0x1 asserted
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone trigger status
     */
    OUSTER_API_FUNCTION
    uint8_t zone_trigger_status(const uint8_t* zone_buffer) const;

    /**
     * Get the count of frames triggered consecutively for the zone.
     * Resets on deassertion.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone triggered frames count
     */
    OUSTER_API_FUNCTION
    uint32_t zone_triggered_frames(const uint8_t* zone_buffer) const;

    /**
     * Get the count of points in the zone.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return count of points in the zone
     */
    OUSTER_API_FUNCTION
    uint32_t zone_points_count(const uint8_t* zone_buffer) const;

    /**
     * Return the number of measurements that return a range below the zone's
     * minimum range, i.e. the number of measurements that occlude the zone.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return the count of measurements that occlude the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_occlusion_count(const uint8_t* zone_buffer) const;

    /**
     * Get the number of pixels that overlap the zone for which an invalid or
     * undetectable range was returned.
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return the count of measurements with undetected or invalid range for
     * the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_invalid_count(const uint8_t* zone_buffer) const;

    /**
     * Get the maximum number of points that can be detected in the zone.
     * @param[in] zone_buffer zone monitoring measurement
     * @return maximum number of points that can be detected in the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_max_count(const uint8_t* zone_buffer) const;

    /**
     * Get the minimum range value detected in a zone from the provided zone
     * buffer.
     *
     * @param[in] zone_buffer Pointer to the buffer containing zone monitoring
     * data.
     * @return Minimum range in millimeters for the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_min_range(const uint8_t* zone_buffer) const;

    /**
     * Get the maximum range value detected in a zone from the provided zone
     * buffer.
     *
     * @param[in] zone_buffer Pointer to the buffer containing zone monitoring
     * data.
     * @return Maximum range in millimeters for the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_max_range(const uint8_t* zone_buffer) const;

    /**
     * Get the mean (average) range value detected in a zone from the provided
     * zone buffer.
     *
     * @param[in] zone_buffer Pointer to the buffer containing zone monitoring
     * data.
     * @return Mean range in millimeters for the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_mean_range(const uint8_t* zone_buffer) const;

    /**
     * Get pointer to nth column of a lidar buffer.
     *
     * @param[in] col_idx which column.
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to nth column of lidar buffer.
     */
    OUSTER_API_FUNCTION
    uint8_t* nth_col(size_t col_idx, uint8_t* lidar_buf) const;

    /**
     * Get pointer to nth pixel of a column buffer.
     *
     * @param[in] px_idx which pixel.
     * @param[in] col_buf the column buffer.
     *
     * @return pointer to nth pixel of a column buffer.
     */
    OUSTER_API_FUNCTION
    uint8_t* nth_px(size_t px_idx, uint8_t* col_buf) const;

    /**
     * Get pointer to the packet footer of a lidar buffer.
     *
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to packet footer of lidar buffer, can be nullptr if
     * packet format doesn't have packet footer.
     */
    OUSTER_API_FUNCTION
    uint8_t* footer(uint8_t* lidar_buf) const;

    /**
     * Get pointer to nth measurement of an IMU buffer.
     * This applies to PROFILE_IMU_ACCEL32_GYRO32_NMEA which contains multiple
     * measurements within each packet.
     * In PROFILE_IMU_LEGACY profile, each packet only contains one measurement.
     *
     * @param[in] meas_idx which measurement.
     * @param[in] imu_buf the imu buffer.
     *
     * @return pointer to nth measurement of an imu buffer.
     */
    OUSTER_API_FUNCTION
    uint8_t* imu_nth_measurement(size_t meas_idx, uint8_t* imu_buf) const;

    /**
     * Get pointer to nth measurement of a Zone Monitoring packet.
     *
     * @param[in] meas_idx which measurement
     * @param[in] zone_packet zone monitoring packet
     *
     * @return pointer to nth measurement of a zone monitoring packet.
     */
    OUSTER_API_FUNCTION
    uint8_t* zone_nth_measurement(size_t meas_idx, uint8_t* zone_packet) const;

    /**
     * Set the alert flags.
     *
     * @param[in,out] lidar_buf the lidar buf.
     * @param[in] alert_flags the alert flags byte to set.
     */
    OUSTER_API_FUNCTION
    void set_alert_flags(uint8_t* lidar_buf, uint8_t alert_flags) const;

    /**
     * Set the column status.
     * @param[in,out] col_buf the column buf.
     * @param[in] status the column status to set.
     */
    OUSTER_API_FUNCTION
    void set_col_status(uint8_t* col_buf, uint32_t status) const;

    /**
     * Set the column timestamp.
     *
     * @param[in,out] col_buf the column buf.
     * @param[in] timestamp the column timestamp to set.
     */
    OUSTER_API_FUNCTION
    void set_col_timestamp(uint8_t* col_buf, uint64_t timestamp) const;

    /**
     * Set the column measurement id.
     *
     * @param[in,out] col_buf the column buf.
     * @param[in] m_id the column measurement id to set.
     */
    OUSTER_API_FUNCTION
    void set_col_measurement_id(uint8_t* col_buf, uint16_t m_id) const;

    /**
     * Set the frame_id header.
     *
     * @param[in,out] lidar_buf the lidar buffer.
     * @param[in] frame_id the frame id to set.
     */
    OUSTER_API_FUNCTION
    void set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const;

    /**
     * Set the init id header.
     *
     * @param[in,out] lidar_buf the lidar buffer.
     * @param[in] init_id the init id to set.
     */
    OUSTER_API_FUNCTION
    void set_init_id(uint8_t* lidar_buf, uint32_t init_id) const;

    /**
     * Set the packet type header.
     *
     * @param[in,out] packet_buf the packet buffer.
     * @param[in] packet_type the packet type to set.
     */
    OUSTER_API_FUNCTION
    void set_packet_type(uint8_t* packet_buf, uint16_t packet_type) const;

    /**
     * Set the serial number header.
     *
     * @param[in,out] lidar_buf the lidar buffer.
     * @param[in] serial_number the serial number to set.
     */
    OUSTER_API_FUNCTION
    void set_prod_sn(uint8_t* lidar_buf, uint64_t serial_number) const;

    /**
     * Set the shot limiting header.
     *
     * @param[in,out] lidar_buf the lidar buf.
     * @param[in] status the shot limiting status to set.
     */
    OUSTER_API_FUNCTION
    void set_shot_limiting(uint8_t* lidar_buf, uint8_t status) const;

    /**
     * Set the shot limiting countdown header.
     *
     * @param[in,out] lidar_buf the lidar buf.
     * @param[in] shot_limiting_countdown the shot limiting countdown to set.
     */
    OUSTER_API_FUNCTION
    void set_shot_limiting_countdown(uint8_t* lidar_buf,
                                     uint8_t shot_limiting_countdown) const;
    /**
     * Set the thermal shutdown header.
     *
     * @param[in,out] lidar_buf the lidar buf.
     * @param[in] status the thermal shutdown status to set.
     */
    OUSTER_API_FUNCTION
    void set_shutdown(uint8_t* lidar_buf, uint8_t status) const;

    /**
     * Set the thermal shutdown countdown header.
     *
     * @param[in,out] lidar_buf the lidar buf.
     * @param[in] shutdown_countdown the thermal shutdown countdown to set.
     */
    OUSTER_API_FUNCTION
    void set_shutdown_countdown(uint8_t* lidar_buf,
                                uint8_t shutdown_countdown) const;

    /**
     * Set the specified channel field into a packet measurement block.
     * @tparam T The field type.
     * @param[in] data source array containing field data to write.
     * @param[in] cols number of columns in the source array.
     * @param[in] field_name the name of the channel field to set.
     * @param[in,out] lidar_buf the lidar buffer.
     */
    template <typename T>
    OUSTER_API_FUNCTION
    void set_block(const T* data, int cols,
                   const std::string& field_name, uint8_t* lidar_buf) const;

    /**
     * Unpack the RAW_HEADERS field from a frame into the lidar packet buffer.
     * Note: measurement_id indices will be taken from lidar_buf itself,
     * so this method expects those to be pre-filled
     * @tparam T The field type.
     * @param[in] field source eigen array
     * @param[in,out] lidar_buf the lidar buffer.
     */
    // NOTE: defined inline after the class. Apple Clang 13 has a mangling
    // bug where explicit instantiation of this member function template
    // produces an ABI-incompatible mangle for the dependent default
    // StrideType expression in Eigen::Ref. Defining inline forces every
    // TU to implicitly instantiate from the same header, which keeps
    // mangling consistent across the library and its consumers.
    template <typename T>
    OUSTER_API_FUNCTION
    void unpack_raw_headers(Eigen::Ref<const img_t<T>> field,
                            uint8_t* lidar_buf) const;

    /**
     * Set NMEA timestamp into imu packet buffer.
     *
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] timestamp the NMEA timestamp to set.
     */
    OUSTER_API_FUNCTION
    void set_imu_nmea_ts(uint8_t* imu_buf, uint64_t timestamp) const;

    /**
     * Set NMEA sentence into imu packet buffer.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] sentence the NMEA sentence to set.
     */
    OUSTER_API_FUNCTION
    void set_imu_nmea_sentence(uint8_t* imu_buf,
                               const std::string& sentence) const;
    /**
     * Set NMEA sentence into imu packet buffer.
     * @note will copy 85 bytes, beware.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] ptr pointer to char array containing NMEA sentence.
     */
    OUSTER_API_FUNCTION
    void set_imu_nmea_sentence(uint8_t* imu_buf, const char* ptr) const;

    /**
     * Set the imu acceleration in x.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] la_x the acceleration in x to set.
     * @note the unit depends on the imu profile.
     */
    OUSTER_API_FUNCTION
    void set_imu_la_x(uint8_t* imu_buf, float la_x) const;

    /**
     * Set the imu acceleration in y.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] la_y the acceleration in y to set.
     * @note the unit depends on the imu profile.
     */

    OUSTER_API_FUNCTION
    void set_imu_la_y(uint8_t* imu_buf, float la_y) const;

    /**
     * Set the imu acceleration in z.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] la_z the acceleration in z to set.
     * @note the unit depends on the imu profile.
     */
    OUSTER_API_FUNCTION
    void set_imu_la_z(uint8_t* imu_buf, float la_z) const;

    /**
     * Set the imu angular velocity around the x axis.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] av_x the angular velocity about x to set.
     * @note the unit depends on the imu profile.
     */
    OUSTER_API_FUNCTION
    void set_imu_av_x(uint8_t* imu_buf, float av_x) const;

    /**
     * Set the imu angular velocity around the y axis.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] av_y the angular velocity about y to set.
     * @note the unit depends on the imu profile.
     */
    OUSTER_API_FUNCTION
    void set_imu_av_y(uint8_t* imu_buf, float av_y) const;

    /**
     * Set the imu angular velocity around the z axis.
     * @param[in,out] imu_buf the imu packet buffer.
     * @param[in] av_z the angular velocity about z to set.
     * @note the unit depends on the imu profile.
     */
    OUSTER_API_FUNCTION
    void set_imu_av_z(uint8_t* imu_buf, float av_z) const;

    /**
     * Set zone monitoring timestamp.
     *
     * @param[in,out] zone_buf zone monitoring packet
     * @param[in] timestamp the zone monitoring timestamp to set
     */
    OUSTER_API_FUNCTION
    void set_zone_timestamp(uint8_t* zone_buf, uint64_t timestamp) const;

    /**
     * Set live zoneset hash.
     *
     * @note the hash should be 32 bytes (256 bits)
     * @param[in,out] zone_buf zone monitoring packet
     * @param[in] hash 256bit hash of zone config
     */
    OUSTER_API_FUNCTION
    void set_live_zoneset_hash(uint8_t* zone_buf, const uint8_t* hash) const;

    /**
     * Set zone state.
     *
     * @param[in,out] zone_measurement zone monitoring measurement
     * @param[in] zone_state the zone state to set
     */
    OUSTER_API_FUNCTION
    void set_zone_state(uint8_t* zone_measurement,
                        const ZoneState& zone_state) const;

    /**
        * Calculate the difference between two frame ids, accounting for rollover.
        *
        * @param[in] current the current frame id.
        * @param[in] other the other frame id.
        *
        * @return the difference between the two frame ids. The value is positive if
        * other is ahead of current, negative if other is behind current.
        */
    OUSTER_API_FUNCTION
    int frame_id_difference(uint32_t current,
                       uint32_t other) const;
};

// Inline definition of PacketFormat::unpack_raw_headers<T>. Defined here
// (rather than in parsing.cpp with explicit instantiations) to work around
// an Apple Clang mangling bug for the dependent default StrideType
// expression in Eigen::Ref. See the note on the declaration above.
template <typename T>
inline void PacketFormat::unpack_raw_headers(Eigen::Ref<const img_t<T>> field,
                                             uint8_t* lidar_buf) const {
    using ColMajorView = Eigen::Map<Eigen::Array<T, -1, 1, Eigen::ColMajor>>;

    if (sizeof(T) > 4) {
        throw std::invalid_argument(
            "RAW_HEADERS field should be of type"
            "uint32_t or smaller to work correctly");
    }

    uint8_t* col_zero = nth_col(0, lidar_buf);
    uint16_t m_id = col_measurement_id(col_zero);

    size_t ch_size = col_header_size / sizeof(T);
    size_t cf_size = col_footer_size / sizeof(T);
    size_t ph_size = packet_header_size / sizeof(T);
    size_t pf_size = packet_footer_size / sizeof(T);

    size_t ch_offset = 0;
    size_t cf_offset = ch_offset + ch_size;
    size_t ph_offset = cf_offset + cf_size;
    size_t pf_offset = ph_offset + ph_size;

    // fill in header and footer, col0 is sufficient for that
    ColMajorView ph_view(reinterpret_cast<T*>(lidar_buf), ph_size);
    ColMajorView pf_view(reinterpret_cast<T*>(footer(lidar_buf)), pf_size);
    ph_view = field.block(ph_offset, m_id, ph_size, 1);
    pf_view = field.block(pf_offset, m_id, pf_size, 1);

    for (uint32_t icol = 0; icol < columns_per_packet; ++icol) {
        uint8_t* col_buf = nth_col(icol, lidar_buf);
        uint8_t* colf_ptr = col_buf + col_size - col_footer_size;

        ColMajorView colh_view(reinterpret_cast<T*>(col_buf), ch_size);
        ColMajorView colf_view(reinterpret_cast<T*>(colf_ptr), cf_size);

        m_id = col_measurement_id(col_buf);

        colh_view = field.block(ch_offset, m_id, ch_size, 1);
        colf_view = field.block(cf_offset, m_id, cf_size, 1);
    }
}

/** @defgroup OusterCoreTypeGetFormat Get Packet Format functions */

/**
 * Get a packet parser for a particular data format.
 *
 * @ingroup OusterCoreTypeGetFormat
 *
 * @param[in] info parameters provided by the sensor.
 *
 * @return a PacketFormat suitable for parsing UDP packets sent by the sensor.
 */
OUSTER_API_FUNCTION
const PacketFormat& get_format(const SensorInfo& info);

/**
 * Get a packet parser for a particular data format.
 *
 * @ingroup OusterCoreTypeGetFormat
 *
 * @param[in] format DataFormat
 *
 * @return a PacketFormat suitable for parsing UDP packets sent by the sensor.
 */
OUSTER_API_FUNCTION
const PacketFormat& get_format(const DataFormat& format);

/**
 * Parse latitude and longitude from an NMEA sentence.
 *
 * @param[in] nmea_sentence pointer to the beginning of nmea string
 * @param[out] latitude output parameter for parsed latitude value
 * @param[out] longitude output parameter for parsed longitude value
 *
 * @return true if sentence is successfully parsed, otherwise false
 */
OUSTER_API_FUNCTION
bool parse_lat_long(const std::string& nmea_sentence, double& latitude,
                    double& longitude);

/** Maximum supported NMEA sentence length */
constexpr size_t NMEA_SENTENCE_LENGTH = 85;

/**
 * The type to represent json data in string form.
 */
using json_string = std::string;

}  // namespace core
}  // namespace sdk
}  // namespace ouster
