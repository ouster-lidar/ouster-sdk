#include "ouster/packet.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace ouster {
namespace sdk {
namespace core {

Packet::Packet(PacketType type) : type_(type), host_timestamp{0} {}

Packet::Packet(PacketType type, int size) : type_(type), host_timestamp{0} {
    // this is necessary due to how client works - it may read size() + 1
    // bytes into the packet in case of rogue packet coming through
    buf.reserve(size + 1);
    buf.resize(size, 0);
}

PacketValidationFailure Packet::validate(const SensorInfo& info) const {
    return ouster::sdk::core::validate_packet(info, *format, buf.data(),
                                              buf.size(), type_);
}

PacketValidationFailure Packet::validate(const SensorInfo& info,
                                         const PacketFormat& format) const {
    return ouster::sdk::core::validate_packet(info, format, buf.data(),
                                              buf.size(), type_);
}

PacketValidationFailure validate_packet(const SensorInfo& info,
                                        const PacketFormat& format,
                                        const uint8_t* buf, uint64_t buf_size,
                                        PacketType type) {
    // Check if we need to guess the type
    if (type == PacketType::Unknown) {
        if (buf_size == format.imu_packet_size) {
            type = PacketType::Imu;
        } else if (buf_size == format.zone_packet_size) {
            type = PacketType::Zone;
        } else {
            type = PacketType::Lidar;
        }
    }

    if (type == PacketType::Lidar) {
        if (buf_size != format.lidar_packet_size) {
            return PacketValidationFailure::PACKET_SIZE;
        }
    } else if (type == PacketType::Imu) {
        if (buf_size != format.imu_packet_size) {
            return PacketValidationFailure::PACKET_SIZE;
        }
    } else if (type == PacketType::Zone) {
        if (buf_size != format.zone_packet_size) {
            return PacketValidationFailure::PACKET_SIZE;
        }
    }

    // early exit if legacy IMU since we cant check the ids
    if (type == PacketType::Imu &&
        format.udp_profile_imu == UDPProfileIMU::LEGACY) {
        return PacketValidationFailure::NONE;
    }

    // validate the init id and serial number
    auto init_id = format.init_id(buf);
    if (info.init_id != 0 && init_id != 0 && init_id != info.init_id) {
        return PacketValidationFailure::ID;
    }

    if (info.sn != 0) {
        uint64_t p_sn = format.prod_sn(buf);
        if ((p_sn != 0) && (p_sn != info.sn)) {
            return PacketValidationFailure::ID;
        }
    }
    return PacketValidationFailure::NONE;
};

LidarPacket::LidarPacket() : Packet{MY_TYPE} {}

LidarPacket::LidarPacket(int size) : Packet{MY_TYPE, size} {}

ImuPacket::ImuPacket() : Packet{MY_TYPE} {}

ImuPacket::ImuPacket(int size) : Packet{MY_TYPE, size} {}

ZonePacket::ZonePacket() : Packet{MY_TYPE} {}

ZonePacket::ZonePacket(int size) : Packet{MY_TYPE, size} {}

Eigen::Array<float, Eigen::Dynamic, 3> ImuPacket::accel() const {
    // The legacy profile reports raw data, so we need to convert it to m/s^2
    if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
        // The acceleration of gravity in m/s^2
        constexpr float gravity = 9.80665;
        Eigen::Array<float, Eigen::Dynamic, 3> out(1, 3);
        out(0, 0) = format->imu_la_x(buf.data()) * gravity;
        out(0, 1) = format->imu_la_y(buf.data()) * gravity;
        out(0, 2) = format->imu_la_z(buf.data()) * gravity;
        return out;
    }

    // The non-legacy profile reports data in m/s^2, so no conversion needed
    size_t measurements_per_packet = format->imu_measurements_per_packet;
    Eigen::Array<float, Eigen::Dynamic, 3> out(measurements_per_packet, 3);
    for (size_t i = 0; i < measurements_per_packet; ++i) {
        const uint8_t* ptr = format->imu_nth_measurement(i, buf.data());
        out(i, 0) = format->imu_la_x(ptr);
        out(i, 1) = format->imu_la_y(ptr);
        out(i, 2) = format->imu_la_z(ptr);
    }
    return out;
}

Eigen::Array<float, Eigen::Dynamic, 3> ImuPacket::gyro() const {
    // The legacy profile reports raw data, so we need to convert it to rad/sec
    if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
        // Conversion factor from degrees to radians
        constexpr float degrees_to_radians = M_PI / 180.0;
        Eigen::Array<float, Eigen::Dynamic, 3> out(1, 3);
        out(0, 0) = format->imu_av_x(buf.data()) * degrees_to_radians;
        out(0, 1) = format->imu_av_y(buf.data()) * degrees_to_radians;
        out(0, 2) = format->imu_av_z(buf.data()) * degrees_to_radians;
        return out;
    }

    // The non-legacy profile reports data in rad/sec, so no conversion needed
    size_t measurements_per_packet = format->imu_measurements_per_packet;
    Eigen::Array<float, Eigen::Dynamic, 3> out(measurements_per_packet, 3);
    for (size_t i = 0; i < measurements_per_packet; ++i) {
        const uint8_t* ptr = format->imu_nth_measurement(i, buf.data());
        out(i, 0) = format->imu_av_x(ptr);
        out(i, 1) = format->imu_av_y(ptr);
        out(i, 2) = format->imu_av_z(ptr);
    }
    return out;
}

Eigen::Array<uint16_t, Eigen::Dynamic, 1> ImuPacket::status() const {
    if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
        Eigen::Array<uint16_t, Eigen::Dynamic, 1> out(1, 1);
        out(0, 0) = 0x1;
        return out;
    }

    size_t measurements_per_packet = format->imu_measurements_per_packet;
    Eigen::Array<uint16_t, Eigen::Dynamic, 1> out(measurements_per_packet, 1);
    for (size_t i = 0; i < measurements_per_packet; ++i) {
        const uint8_t* ptr = format->imu_nth_measurement(i, buf.data());
        out(i, 0) = format->col_status(ptr) & 0x1;
    }
    return out;
}

Eigen::Array<uint64_t, Eigen::Dynamic, 1> ImuPacket::timestamp() const {
    if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
        Eigen::Array<uint64_t, Eigen::Dynamic, 1> out(1, 1);
        out(0, 0) = 0;
        return out;
    }

    size_t measurements_per_packet = format->imu_measurements_per_packet;
    Eigen::Array<uint64_t, Eigen::Dynamic, 1> out(measurements_per_packet, 1);
    for (size_t i = 0; i < measurements_per_packet; ++i) {
        const uint8_t* ptr = format->imu_nth_measurement(i, buf.data());
        out(i, 0) = format->col_timestamp(ptr);
    }
    return out;
}

Eigen::Array<uint16_t, Eigen::Dynamic, 1> ImuPacket::measurement_id() const {
    if (format->udp_profile_imu == UDPProfileIMU::LEGACY) {
        Eigen::Array<uint16_t, Eigen::Dynamic, 1> out(1, 1);
        out(0, 0) = 0;
        return out;
    }

    size_t measurements_per_packet = format->imu_measurements_per_packet;
    Eigen::Array<uint16_t, Eigen::Dynamic, 1> out(measurements_per_packet, 1);
    for (size_t i = 0; i < measurements_per_packet; ++i) {
        const uint8_t* ptr = format->imu_nth_measurement(i, buf.data());
        out(i, 0) = format->col_measurement_id(ptr);
    }
    return out;
}

Eigen::Array<ZoneState, Eigen::Dynamic, 1> ZonePacket::zone_states() const {
    Eigen::Array<ZoneState, Eigen::Dynamic, 1> out(16, 1);
    for (size_t i = 0; i < 16; ++i) {
        const uint8_t* zone_ptr = format->zone_nth_measurement(i, buf.data());

        out(i, 0).live = static_cast<uint8_t>(format->zone_live(zone_ptr));
        out(i, 0).id = format->zone_id(zone_ptr);
        out(i, 0).error_flags = format->zone_error_flags(zone_ptr);
        out(i, 0).trigger_type = format->zone_trigger_type(zone_ptr);
        out(i, 0).trigger_status = format->zone_trigger_status(zone_ptr);
        out(i, 0).triggered_frames = format->zone_triggered_frames(zone_ptr);
        out(i, 0).count = format->zone_points_count(zone_ptr);
        out(i, 0).occlusion_count = format->zone_occlusion_count(zone_ptr);
        out(i, 0).invalid_count = format->zone_invalid_count(zone_ptr);
        out(i, 0).max_count = format->zone_max_count(zone_ptr);
        out(i, 0).min_range = format->zone_min_range(zone_ptr);
        out(i, 0).max_range = format->zone_max_range(zone_ptr);
        out(i, 0).mean_range = format->zone_mean_range(zone_ptr);
    }
    return out;
}
}  // namespace core
}  // namespace sdk
}  // namespace ouster
