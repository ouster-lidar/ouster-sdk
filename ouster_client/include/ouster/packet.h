/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster client packet datatypes
 */

#pragma once

#include <cstdint>
#include <vector>

#include "types.h"

namespace ouster {
namespace sensor {

/// Types of Ouster packets that can come from a sensor
enum class PacketType {
    Unknown,  ///< Packet type could not or has not been determined
    Lidar,    ///< Pointcloud data packet
    Imu       ///< IMU data packet
};

/// Encapsulate a packet buffer and attributes associated with it.
struct OUSTER_API_CLASS Packet {
   private:
    /// Type of the packet
    PacketType type_;

   public:
    /// Timestamp in nanoseconds of packet capture
    uint64_t host_timestamp;

    /// Packet data
    std::vector<uint8_t> buf;

    /// packet_format associated with this packet
    std::shared_ptr<ouster::sensor::packet_format> format;

    /// Returns the type of the packet
    /// @return the packet type
    OUSTER_API_FUNCTION PacketType type() const { return type_; }

   protected:
    /// Construct an empty packet with a given type
    Packet(PacketType type  ///< [in] type packet type (imu or lidar)
    );

    /// Construct a packet with given type and a pre-allocated size
    Packet(PacketType type,  ///< [in] type packet type (imu or lidar)
           int size          ///< [in] size in bytes to allocate
    );

   public:
    /// Attempt to cast the packet to the desired concrete PacketType.
    /// @throw runtime_error if packet is not that of that type
    /// @tparam Type Type of packet to cast to. Either LidarPacker or ImuPacket.
    /// @return reference to the packet as the desired type
    template <typename Type>
    Type& as() {
        if (type() != Type::MyType) {
            throw std::runtime_error("Tried to cast packet to incorrect type.");
        }
        return static_cast<Type&>(*this);
    }
};

/// Reasons for failure of packet validation.
enum class PacketValidationFailure {
    NONE = 0,         ///< No validation errors were found
    PACKET_SIZE = 1,  ///< The packet size does not match the expected size
    ID = 2            ///< The prod_sn or init_id does not match the metadata
};

/// Validate a packet buffer against a given type.
///
/// @param[in] info The sensor info to try to check the buffer against.
/// @param[in] format The packet format to try to check the buffer against.
/// @param[in] buf The packet buffer to validate.
/// @param[in] buf_size The size of the packet buffer.
/// @param[in] type Optional type of packet to try and validate as. Unknown will
///                 try and guess the packet type
/// @return Result of the validation
OUSTER_API_FUNCTION
PacketValidationFailure validate_packet(
    const sensor_info& info, const ouster::sensor::packet_format& format,
    const uint8_t* buf, uint64_t buf_size,
    PacketType type = PacketType::Unknown);

/// Encapsulate a lidar packet buffer and attributes associated with it.
struct OUSTER_API_CLASS LidarPacket : public Packet {
    using Packet::Packet;

    /// PacketType enum for this packet type
    const static PacketType MyType = PacketType::Lidar;

    /// Construct a new empty Lidar Packet
    OUSTER_API_FUNCTION LidarPacket();

    /// Construct a new Lidar packet with a pre-allocated size
    OUSTER_API_FUNCTION LidarPacket(
        int size  ///< [in] size in bytes to allocate
    );

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const sensor_info& info,  ///< [in] expected sensor_metadata
        const ouster::sensor::packet_format&
            format  ///< [in] expected packet_format
    ) const;

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const sensor_info& info  ///< [in] expected sensor_metadata
    ) const;

    /// Get pointer to the nth column in the packet.
    /// @return pointer to the nth column in this packet
    OUSTER_API_FUNCTION inline auto nth_col(int n  ///< [in] which column
    ) const {
        return format->nth_col(n, buf.data());
    }

    /// Get pointer to the nth pixel of a column buffer
    /// @return pointer to the nth pixel of a column buffer
    OUSTER_API_FUNCTION inline auto nth_px(
        int n,                  ///< [in] which pixel
        const uint8_t* col_buf  ///< [in] pointer to column data
    ) const {
        return format->nth_px(n, col_buf);
    }

    /// Read column timestamp from a column buffer
    /// @return column timestamp
    OUSTER_API_FUNCTION inline auto col_timestamp(
        const uint8_t* col_buf  ///< [in] pointer to column data
    ) const {
        return format->col_timestamp(col_buf);
    }

    /// Read column measurement id from a column buffer
    /// @return column measurement id
    OUSTER_API_FUNCTION inline auto col_measurement_id(
        const uint8_t* col_buf  ///< [in] pointer to column data
    ) const {
        return format->col_measurement_id(col_buf);
    }

    /// Read column status from a column buffer
    /// @return column status
    OUSTER_API_FUNCTION inline auto col_status(
        const uint8_t* col_buf  ///< [in] pointer to column data
    ) const {
        return format->col_status(col_buf);
    }

    /// Copy the specified channel field out of a packet measurement block.
    ///
    /// @tparam T T should be a numeric type large enough to store
    /// values of the specified field. Otherwise, data will be truncated.
    ///
    /// @param[in] col_buf a measurement block pointer returned by `nth_col()`.
    /// @param[in] f the channel field to copy.
    /// @param[out] dst destination array of size pixels_per_column *
    /// dst_stride.
    /// @param[in] dst_stride stride for writing to the destination array.
    template <typename T>
    void col_field(const uint8_t* col_buf, const std::string& f, T* dst,
                   int dst_stride = 1) const {
        format->col_field<T>(col_buf, f, dst, dst_stride);
    }

    /// Read the packet type from the packet header.
    /// @return packet type
    OUSTER_API_FUNCTION inline auto packet_type() const {
        return format->packet_type(buf.data());
    }

    /// Read the frame id from the packet header.
    /// @return frame id
    OUSTER_API_FUNCTION inline auto frame_id() const {
        return format->frame_id(buf.data());
    }

    /// Read the init id from the packet header.
    /// @return init id
    OUSTER_API_FUNCTION inline auto init_id() const {
        return format->init_id(buf.data());
    }

    /// Read the product serial number from the packet header.
    /// @return product serial number
    OUSTER_API_FUNCTION inline auto prod_sn() const {
        return format->prod_sn(buf.data());
    }

    /// Read the alert flags from the packet header.
    /// @return alert flags
    OUSTER_API_FUNCTION inline auto alert_flags() const {
        return format->alert_flags(buf.data());
    }

    /// Read the thermal shutdown countdown from the packet header.
    /// @return thermal shutdown countdown
    OUSTER_API_FUNCTION inline auto countdown_thermal_shutdown() const {
        return format->countdown_thermal_shutdown(buf.data());
    }

    /// Read the shot limiting countdown from the packet header.
    /// @return shot limiting countdown
    OUSTER_API_FUNCTION inline auto countdown_shot_limiting() const {
        return format->countdown_shot_limiting(buf.data());
    }

    /// Read the thermal shutdown state from the packet header.
    /// @return thermal shutdown state
    OUSTER_API_FUNCTION inline auto thermal_shutdown() const {
        return format->thermal_shutdown(buf.data());
    }

    /// Read the shot limiting state from the packet header.
    /// @return shot limiting state
    OUSTER_API_FUNCTION inline auto shot_limiting() const {
        return format->shot_limiting(buf.data());
    }

    /// Get a pointer to the packet footer.
    /// @return  pointer to packet footer of lidar buffer, can be nullptr if
    ///          packet format doesn't have packet footer.
    OUSTER_API_FUNCTION inline auto footer() const {
        return format->footer(buf.data());
    }

    /// Return the CRC contained in the packet footer if present.
    /// @return crc contained in the packet if present
    OUSTER_API_FUNCTION inline auto crc() const {
        return format->crc(buf.data());
    }

    /// Calculate the CRC for the given packet data.
    /// @return calculated crc of the packet
    OUSTER_API_FUNCTION inline auto calculate_crc() const {
        return format->calculate_crc(buf.data());
    }

    /// Returns maximum available size of parsing block usable with block_field
    /// @return if packet format does not allow for block parsing, returns 0
    OUSTER_API_FUNCTION inline auto block_parsable() const {
        return format->block_parsable();
    }

    /// Copy the specified channel field out of a packet measurement block.
    /// Faster traversal than col_field, but has to copy the entire packet all
    /// at once.
    ///
    /// @tparam T T should be a numeric type large enough to store
    /// values of the specified field. Otherwise, data will be truncated.
    template <typename T, int BlockDim>
    void block_field(
        Eigen::Ref<img_t<T>> field,  ///< [out] destination eigen array
        const std::string& f         ///< [in] the channel field to copy
    ) const {
        format->block_field<T, BlockDim>(field, f, buf.data());
    }
};

/// Encapsulate an imu packet buffer and attributes associated with it.
struct OUSTER_API_CLASS ImuPacket : public Packet {
    using Packet::Packet;

    /// PacketType enum for this packet type
    const static PacketType MyType = PacketType::Imu;

    /// Construct a new empty Imu Packet
    OUSTER_API_FUNCTION ImuPacket();

    /// Construct a new Imu packet with a pre-allocated size
    OUSTER_API_FUNCTION ImuPacket(int size  ///< [in] size in bytes to allocate
    );

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const sensor_info& info,  ///< [in] expected sensor_metadata
        const ouster::sensor::packet_format&
            format  ///< [in] expected packet_format
    ) const;

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const sensor_info& info  ///< [in] expected sensor_metadata
    ) const;

    /// Read system timestamp from the packet.
    /// @return system timestamp
    OUSTER_API_FUNCTION inline auto sys_ts() const {
        return format->imu_sys_ts(buf.data());
    }

    /// Read accelerometer timestamp from the packet.
    /// @return accelerometer timestamp
    OUSTER_API_FUNCTION inline auto accel_ts() const {
        return format->imu_accel_ts(buf.data());
    }

    /// Read gyroscope timestamp from the packet.
    /// @return gyroscope timestamp
    OUSTER_API_FUNCTION inline auto gyro_ts() const {
        return format->imu_gyro_ts(buf.data());
    }

    /// Read acceleration in X direction from the packet.
    /// @return acceleration in the X direction
    OUSTER_API_FUNCTION inline auto la_x() const {
        return format->imu_la_x(buf.data());
    }

    /// Read acceleration in Y direction from the packet.
    /// @return acceleration in the Y direction
    OUSTER_API_FUNCTION inline auto la_y() const {
        return format->imu_la_y(buf.data());
    }

    /// Read acceleration in Z direction from the packet.
    /// @return acceleration in the Z direction
    OUSTER_API_FUNCTION inline auto la_z() const {
        return format->imu_la_z(buf.data());
    }

    /// Read angular velocity on the X axis from the packet.
    /// @return angular velocity on the X axis
    OUSTER_API_FUNCTION inline auto av_x() const {
        return format->imu_av_x(buf.data());
    }

    /// Read angular velocity on the Y axis from the packet.
    /// @return angular velocity on the Y axis
    OUSTER_API_FUNCTION inline auto av_y() const {
        return format->imu_av_y(buf.data());
    }

    /// Read angular velocity on the Z axis from the packet.
    /// @return angular velocity on the Z axis
    OUSTER_API_FUNCTION inline auto av_z() const {
        return format->imu_av_z(buf.data());
    }
};

}  // namespace sensor
}  // namespace ouster
