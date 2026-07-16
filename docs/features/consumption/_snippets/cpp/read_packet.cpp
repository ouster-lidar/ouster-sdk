#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/sensor/sensor_packet_source.h"

//! [doc-stag-pcap-record-imports]
#include "ouster/pcap/pcap_packet_source.h"
#include "ouster/sensor/sensor_frame_set_source.h"
using namespace ouster::sdk;
//! [doc-etag-pcap-record-imports]

namespace ouster {
namespace sdk {
namespace docs {

inline const char* packet_type_name(core::PacketType type) {
    switch (type) {
        case core::PacketType::Lidar:
            return "Lidar";
        case core::PacketType::Imu:
            return "Imu";
        default:
            return "Unknown";
    }
}
namespace {

constexpr float kPollTimeoutSec = 1.0F;
constexpr std::size_t kPreviewPackets = 5;

auto make_time_limited_iter(int n_seconds) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(n_seconds);
    return [deadline]() mutable { return std::chrono::steady_clock::now() < deadline; };
}

}  // namespace

void record_sensor_session(const std::string& source, int lidar_port = 7502, int imu_port = 7503,
                           int n_seconds = 10) {
    // clang-format off
    //! [doc-stag-pcap-record-setup]
    auto packet_source = open_packet_source(source, [&](auto& options) {
                                            options.lidar_port = lidar_port;
                                            options.imu_port = imu_port;
                                            options.buffer_time_sec = 1.0F; });
    //! [doc-etag-pcap-record-setup]
    // clang-format on
    const auto& infos = packet_source.sensor_info();
    if (infos.empty() || !infos.front()) {
        throw std::runtime_error("record_sensor_session requires sensor metadata");
    }
    std::cout << "Connected to source with " << infos.size()
              << " sensor(s). Serial=" << infos.front()->sn << '\n';

    auto sensor_packets =
        std::dynamic_pointer_cast<sensor::SensorPacketSource>(packet_source.child());
    if (!sensor_packets) {
        throw std::runtime_error("record_sensor_session expects a SensorPacketSource child");
    }

    auto keep_running = make_time_limited_iter(n_seconds);
    std::size_t lidar_packets = 0;
    std::size_t imu_packets = 0;
    std::size_t printed_packets = 0;

    while (keep_running()) {
        auto event = sensor_packets->get_packet(kPollTimeoutSec);

        if (event.type == sensor::ClientEvent::EXIT) {
            break;
        }

        if (event.type != sensor::ClientEvent::PACKET) {
            continue;
        }

        auto& packet = event.packet();
        if (packet.type() == core::PacketType::Lidar) {
            ++lidar_packets;
        } else if (packet.type() == core::PacketType::Imu) {
            ++imu_packets;
        }

        if (printed_packets < kPreviewPackets) {
            std::cout << "Packet " << printed_packets + 1
                      << ": type=" << packet_type_name(packet.type())
                      << ", bytes=" << packet.buf.size() << ", host_ts=" << packet.host_timestamp
                      << '\n';
            ++printed_packets;
        }
    }

    std::cout << "Captured " << lidar_packets << " lidar packet(s) and " << imu_packets
              << " imu packet(s)." << std::endl;
}

}  // namespace docs
}  // namespace sdk
}  // namespace ouster
