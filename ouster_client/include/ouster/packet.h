/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster client packet datatypes
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/deprecation.h"
#include "ouster/zone_state.h"
#include "types.h"

namespace ouster {
namespace sdk {
namespace core {

/// Types of Ouster packets that can come from a sensor
enum class PacketType {
    Unknown,  ///< Packet type could not or has not been determined
    Lidar,    ///< Pointcloud data packet
    Imu,      ///< IMU data packet
    Zone      ///< Zone Monitoring packet
};

/// Reasons for failure of packet validation.
enum class PacketValidationFailure {
    NONE = 0,         ///< No validation errors were found
    PACKET_SIZE = 1,  ///< The packet size does not match the expected size
    ID = 2            ///< The prod_sn or init_id does not match the metadata
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

    /// PacketFormat associated with this packet
    std::shared_ptr<PacketFormat> format;

    /// Returns the type of the packet
    /// @return the packet type
    OUSTER_API_FUNCTION PacketType type() const { return type_; }

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION
    PacketValidationFailure validate(
        const SensorInfo& info  ///< [in] expected sensor_metadata
    ) const;

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const SensorInfo& info,     ///< [in] expected sensor_metadata
        const PacketFormat& format  ///< [in] expected PacketFormat
    ) const;

    /// Read the packet type from the packet header.
    /// @return packet type
    OUSTER_API_FUNCTION inline auto packet_type() const {
        header_check();
        return format->packet_type(buf.data());
    }

    /// Read the frame id from the packet header.
    /// @return frame id
    OUSTER_API_FUNCTION inline auto frame_id() const {
        header_check();
        return format->frame_id(buf.data());
    }

    /// Read the init id from the packet header.
    /// @return init id
    OUSTER_API_FUNCTION inline auto init_id() const {
        header_check();
        return format->init_id(buf.data());
    }

    /// Read the product serial number from the packet header.
    /// @return product serial number
    OUSTER_API_FUNCTION inline auto prod_sn() const {
        header_check();
        return format->prod_sn(buf.data());
    }

    /// Read the alert flags from the packet header.
    /// @return alert flags
    OUSTER_API_FUNCTION inline auto alert_flags() const {
        header_check();
        return format->alert_flags(buf.data());
    }

    /// Read the thermal shutdown countdown from the packet header.
    /// @return thermal shutdown countdown
    OUSTER_API_FUNCTION inline auto countdown_thermal_shutdown() const {
        header_check();
        return format->countdown_thermal_shutdown(buf.data());
    }

    /// Read the shot limiting countdown from the packet header.
    /// @return shot limiting countdown
    OUSTER_API_FUNCTION inline auto countdown_shot_limiting() const {
        header_check();
        return format->countdown_shot_limiting(buf.data());
    }

    /// Read the thermal shutdown state from the packet header.
    /// @return thermal shutdown state
    OUSTER_API_FUNCTION inline auto thermal_shutdown() const {
        header_check();
        return format->thermal_shutdown(buf.data());
    }

    /// Read the shot limiting state from the packet header.
    /// @return shot limiting state
    OUSTER_API_FUNCTION inline auto shot_limiting() const {
        header_check();
        return format->shot_limiting(buf.data());
    }

    /// Return the CRC contained in the packet footer if present.
    /// @return crc contained in the packet if present
    OUSTER_API_FUNCTION inline auto crc() const {
        header_check();
        return format->crc(buf.data(), buf.size());
    }

    /// Calculate the CRC for the given packet data.
    /// @return calculated crc of the packet
    OUSTER_API_FUNCTION inline auto calculate_crc() const {
        header_check();
        return format->calculate_crc(buf.data(), buf.size());
    }

   protected:
    /// Construct an empty packet with a given type
    Packet(PacketType type  ///< [in] type packet type (imu or lidar)
    );

    /// Construct a packet with given type and a pre-allocated size
    Packet(PacketType type,  ///< [in] type packet type (imu or lidar)
           int size          ///< [in] size in bytes to allocate
    );

    // Check if this packet has a header/footer, otherwise throw
    inline void header_check() const {
        if (type_ == PacketType::Imu &&
            format->udp_profile_imu ==
                ouster::sdk::core::UDPProfileIMU::LEGACY) {
            throw std::runtime_error("Legacy IMU packets lack this field.");
        }
    }

   public:
    /// Attempt to cast the packet to the desired concrete PacketType.
    /// @throw runtime_error if packet is not that of that type
    /// @tparam Type Type of packet to cast to. Either LidarPacker or ImuPacket.
    /// @return reference to the packet as the desired type
    template <typename Type>
    Type& as() {
        if (type() != Type::MY_TYPE) {
            throw std::runtime_error("Tried to cast packet to incorrect type.");
        }
        return static_cast<Type&>(*this);
    }
    /** @copybrief as
     * Const overload.
     * @copydetails as
     */
    template <typename Type>
    const Type& as() const {
        if (type() != Type::MY_TYPE) {
            throw std::runtime_error("Tried to cast packet to incorrect type.");
        }
        return static_cast<const Type&>(*this);
    }
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
PacketValidationFailure validate_packet(const SensorInfo& info,
                                        const PacketFormat& format,
                                        const uint8_t* buf, uint64_t buf_size,
                                        PacketType type = PacketType::Unknown);

/// Encapsulate a lidar packet buffer and attributes associated with it.
struct OUSTER_API_CLASS LidarPacket : public Packet {
    using Packet::Packet;

    /// PacketType enum for this packet type
    const static PacketType MY_TYPE = PacketType::Lidar;
    OUSTER_DIAGNOSTIC_PUSH
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED
    /**
     * @deprecated Use MY_TYPE instead.
     * This will be removed in a future release.
     */
    OUSTER_DEPRECATED_MSG(MY_TYPE, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
    const static PacketType my_type = PacketType::Lidar;
    OUSTER_DIAGNOSTIC_POP

    /// Construct a new empty Lidar Packet
    OUSTER_API_FUNCTION LidarPacket();

    /// Construct a new Lidar packet with a pre-allocated size
    OUSTER_API_FUNCTION LidarPacket(
        int size  ///< [in] size in bytes to allocate
    );

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const SensorInfo& info,     ///< [in] expected sensor_metadata
        const PacketFormat& format  ///< [in] expected PacketFormat
    ) const;

    /// Validates that the packet matches the expected format and metadata.
    /// @return a PacketValdationFailure with either NONE or a failure reason.
    OUSTER_API_FUNCTION PacketValidationFailure validate(
        const SensorInfo& info  ///< [in] expected sensor_metadata
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

    /// Get a pointer to the packet footer.
    /// @return  pointer to packet footer of lidar buffer, can be nullptr if
    ///          packet format doesn't have packet footer.
    OUSTER_API_FUNCTION inline auto footer() const {
        return format->footer(buf.data());
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
    const static PacketType MY_TYPE = PacketType::Imu;
    OUSTER_DIAGNOSTIC_PUSH
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED
    /**
     * @deprecated Use MY_TYPE instead.
     * This will be removed in a future release.
     */
    OUSTER_DEPRECATED_MSG(MY_TYPE, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
    const static PacketType my_type = PacketType::Imu;
    OUSTER_DIAGNOSTIC_POP

    /// Construct a new empty Imu Packet
    OUSTER_API_FUNCTION ImuPacket();

    /// Construct a new Imu packet with a pre-allocated size
    OUSTER_API_FUNCTION ImuPacket(int size  ///< [in] size in bytes to allocate
    );

    /// Read system timestamp from the packet.
    /// Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
    /// @return system timestamp
    OUSTER_API_FUNCTION inline auto sys_ts() const {
        return format->imu_sys_ts(buf.data());
    }

    /// Read accelerometer timestamp from the packet.
    /// Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
    /// @return accelerometer timestamp
    OUSTER_API_FUNCTION inline auto accel_ts() const {
        return format->imu_accel_ts(buf.data());
    }

    /// Read gyroscope timestamp from the packet.
    /// Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
    /// @return gyroscope timestamp
    OUSTER_API_FUNCTION inline auto gyro_ts() const {
        return format->imu_gyro_ts(buf.data());
    }

    /// Read NMEA sentence from an IMU buffer.
    /// Only available in PROFILE_ACCEL32_GYRO32_NMEA.
    /// @return nmea sentence string
    OUSTER_API_FUNCTION
    std::string nmea_sentence() const {
        return format->imu_nmea_sentence(buf.data());
    }

    /// Read nmea timestamp from the packet.
    /// Only available in PROFILE_ACCEL32_GYRO32_NMEA, otherwise returns 0.
    /// @return nmea timestamp
    OUSTER_API_FUNCTION
    uint64_t nmea_ts() const { return format->imu_nmea_ts(buf.data()); }

    /// Get acceleration reads from imu packet.
    /// @return 2d array of accel readings in m/s^2
    OUSTER_API_FUNCTION Eigen::Array<float, Eigen::Dynamic, 3> accel() const;

    /// Get angular velocity reads from imu packet.
    /// @return 2d array of angular velocity readings in rad/sec
    OUSTER_API_FUNCTION Eigen::Array<float, Eigen::Dynamic, 3> gyro() const;

    /// Get status of reads from imu packet. 0x1 for valid.
    /// Always returns (1,1) array of 0x1 with UDPProfileIMU::LEGACY.
    /// @return 1d array of statuses
    OUSTER_API_FUNCTION Eigen::Array<uint16_t, Eigen::Dynamic, 1> status()
        const;

    /// Get read timestamps from imu packet.
    /// Always returns (1,1) array of 0 with UDPProfileIMU::LEGACY.
    /// @return 1d array of timestamps
    OUSTER_API_FUNCTION Eigen::Array<uint64_t, Eigen::Dynamic, 1> timestamp()
        const;

    /// Get correlated measurement ids from imu packet.
    /// Always returns (1,1) array of 0 with UDPProfileIMU::LEGACY.
    /// @return 1d array of measurement ids
    OUSTER_API_FUNCTION Eigen::Array<uint16_t, Eigen::Dynamic, 1>
    measurement_id() const;

    /// Read acceleration in X direction from the packet.
    /// Only works with UDPProfileIMU::LEGACY.
    /// @return acceleration in the X direction
    [[deprecated("Use accel() instead")]] OUSTER_API_FUNCTION inline auto la_x()
        const {
        if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
            return format->imu_la_x(buf.data());
        } else {
            return 0.f;
        }
    }

    /// Read acceleration in Y direction from the packet.
    /// Only works with UDPProfileIMU::LEGACY.
    /// @return acceleration in the Y direction
    [[deprecated("Use accel() instead")]] OUSTER_API_FUNCTION inline auto la_y()
        const {
        if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
            return format->imu_la_y(buf.data());
        } else {
            return 0.f;
        }
    }

    /// Read acceleration in Z direction from the packet.
    /// Only works with UDPProfileIMU::LEGACY.
    /// @return acceleration in the Z direction
    [[deprecated("Use accel() instead")]] OUSTER_API_FUNCTION inline auto la_z()
        const {
        if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
            return format->imu_la_z(buf.data());
        } else {
            return 0.f;
        }
    }

    /// Read angular velocity on the X axis from the packet.
    /// Only works with UDPProfileIMU::LEGACY.
    /// @return angular velocity on the X axis
    [[deprecated("Use gyro() instead")]] OUSTER_API_FUNCTION inline auto av_x()
        const {
        if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
            return format->imu_av_x(buf.data());
        } else {
            return 0.f;
        }
    }

    /// Read angular velocity on the Y axis from the packet.
    /// Only works with UDPProfileIMU::LEGACY.
    /// @return angular velocity on the Y axis
    [[deprecated("Use gyro() instead")]] OUSTER_API_FUNCTION inline auto av_y()
        const {
        if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
            return format->imu_av_y(buf.data());
        } else {
            return 0.f;
        }
    }

    /// Read angular velocity on the Z axis from the packet.
    /// Only works with PROFILE_IMU_LEGACY.
    /// @return angular velocity on the Z axis
    [[deprecated("Use gyro() instead")]] OUSTER_API_FUNCTION inline auto av_z()
        const {
        if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
            return format->imu_av_z(buf.data());
        } else {
            return 0.f;
        }
    }
};

/// Encapsulate an imu packet buffer and attributes associated with it.
struct OUSTER_API_CLASS ZonePacket : public Packet {
    using Packet::Packet;

    /// PacketType enum for this packet type
    const static PacketType MY_TYPE = PacketType::Zone;
    OUSTER_DIAGNOSTIC_PUSH
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED
    /**
     * @deprecated Use MY_TYPE instead.
     * This will be removed in a future release.
     */
    OUSTER_DEPRECATED_MSG(MY_TYPE, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
    const static PacketType my_type = PacketType::Zone;
    OUSTER_DIAGNOSTIC_POP

    /// Construct a new empty Zone Packet
    OUSTER_API_FUNCTION ZonePacket();

    /// Construct a new Imu packet with a pre-allocated size
    OUSTER_API_FUNCTION ZonePacket(int size  ///< [in] size in bytes to allocate
    );

    /// Get live zoneset hash.
    /// @return 256bit hash of zone config
    std::array<uint8_t, 32> OUSTER_API_FUNCTION live_zoneset_hash() const {
        return format->live_zoneset_hash(buf.data());
    }

    /// Get zone timestamp.
    /// @return zone timestamp
    uint64_t OUSTER_API_FUNCTION timestamp() const {
        return format->zone_timestamp(buf.data());
    }

    /// Get array of zone states.
    /// @return 1d array of zone states
    Eigen::Array<ZoneState, Eigen::Dynamic, 1> OUSTER_API_FUNCTION
    zone_states() const;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
