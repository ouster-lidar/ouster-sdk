#include "ouster/packet.h"

namespace ouster {
namespace sensor {
Packet::Packet(PacketType type) : type_(type), host_timestamp{0} {}

Packet::Packet(PacketType type, int size) : type_(type), host_timestamp{0} {
    // this is necessary due to how client works - it may read size() + 1
    // bytes into the packet in case of rogue packet coming through
    buf.reserve(size + 1);
    buf.resize(size, 0);
}

PacketValidationFailure validate_packet(
    const sensor_info& info, const ouster::sensor::packet_format& format,
    const uint8_t* buf, uint64_t buf_size, PacketType type) {
    // Check if we need to guess the type
    if (type == PacketType::Unknown) {
        if (buf_size == format.imu_packet_size) {
            type = PacketType::Imu;
        } else {
            type = PacketType::Lidar;
        }
    }

    if (type == PacketType::Lidar) {
        if (buf_size != format.lidar_packet_size) {
            return PacketValidationFailure::PACKET_SIZE;
        }

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
    } else if (type == PacketType::Imu) {
        if (buf_size != format.imu_packet_size) {
            return PacketValidationFailure::PACKET_SIZE;
        }
        return PacketValidationFailure::NONE;
    }
    return PacketValidationFailure::NONE;
};

LidarPacket::LidarPacket() : Packet{MyType} {}

LidarPacket::LidarPacket(int size) : Packet{MyType, size} {}

PacketValidationFailure LidarPacket::validate(
    const sensor_info& info,
    const ouster::sensor::packet_format& format) const {
    return validate_packet(info, format, buf.data(), buf.size(),
                           PacketType::Lidar);
}

PacketValidationFailure LidarPacket::validate(const sensor_info& info) const {
    return validate_packet(info, *format, buf.data(), buf.size(),
                           PacketType::Lidar);
}

ImuPacket::ImuPacket() : Packet{MyType} {}

ImuPacket::ImuPacket(int size) : Packet{MyType, size} {}

PacketValidationFailure ImuPacket::validate(
    const sensor_info& info,
    const ouster::sensor::packet_format& format) const {
    return validate_packet(info, format, buf.data(), buf.size(),
                           PacketType::Imu);
}

PacketValidationFailure ImuPacket::validate(const sensor_info& info) const {
    return validate_packet(info, *format, buf.data(), buf.size(),
                           PacketType::Imu);
}
}  // namespace sensor
}  // namespace ouster
