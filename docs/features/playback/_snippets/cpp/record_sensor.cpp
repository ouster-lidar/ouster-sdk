#include <chrono>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/types.h"
#include "ouster/pcap/os_pcap.h"
#include "ouster/sensor/sensor_packet_source.h"

//! [doc-stag-pcap-record-imports]
#include "ouster/core/open_source.h"
#include "ouster/pcap/pcap_packet_source.h"
#include "ouster/sensor/sensor_frame_set_source.h"
using namespace ouster::sdk;
//! [doc-etag-pcap-record-imports]

//! [doc-stag-osf-write-imports]
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/osf/writer.h"
using namespace ouster::sdk;
//! [doc-etag-osf-write-imports]

namespace ouster {
namespace docs {

inline std::string packet_type_name(core::PacketType type) {
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

constexpr int kWriteFragSize = 1500;

std::string current_time_part() {
    const auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string format_fname_base(const core::SensorInfo& meta, const std::string& time_part) {
    std::ostringstream oss;
    oss << meta.prod_line << '_' << meta.sn << '_';
    if (meta.config.lidar_mode) {
        oss << core::to_string(*meta.config.lidar_mode);
    } else {
        oss << "UNKNOWN";
    }
    oss << '_' << time_part;
    return oss.str();
}

uint64_t now_in_microseconds() {
    const auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
}

auto make_time_limited_iter(int n_seconds) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(n_seconds);
    return [deadline]() mutable { return std::chrono::steady_clock::now() < deadline; };
}

core::AnyPacketSource open_pcap_source_with_metadata(const std::string& pcap_file,
                                                     const std::vector<std::string>& meta_files) {
    return open_packet_source(pcap_file, [&](auto& options) { options.meta = meta_files; });
}

}  // namespace

void record_sensor_session(const std::string& hostname, int lidar_port = 7502, int imu_port = 7503,
                           int n_seconds = 10) {
    // make a descriptive filename for metadata/pcap files
    auto get_fname_base = [](const std::shared_ptr<core::SensorInfo>& info) {
        return format_fname_base(*info, current_time_part());
    };
    // clang-format off
    //! [doc-stag-pcap-record-setup]
    // connect to sensor and record lidar/imu packets
    auto source = open_packet_source(hostname, 
                                    [&](auto& options) {
                                        options.lidar_port = lidar_port;
                                        options.imu_port = imu_port;
                                        options.buffer_time_sec = 1.0F;});

    const auto& info = source.sensor_info()[0];

    // make a descriptive filename for metadata/pcap files
    auto fname_base = get_fname_base(info);
    
    std::cout << "Saving sensor info to: " << fname_base << ".json" << std::endl;
    std::ofstream(fname_base + ".json") << info->to_json_string();
    
    std::cout << "Writing to: " << fname_base << ".pcap (Ctrl-C to stop early)" << std::endl;
    //! [doc-etag-pcap-record-setup]
    //! [doc-stag-pcap-record]
    // int n_seconds = 10
    // Yield items from iterable until n_seconds have passed.
    auto source_it = make_time_limited_iter(n_seconds);
    auto handle = pcap::record_initialize(fname_base + ".pcap", kWriteFragSize);
    std::uint64_t n_packets = 0;

    auto sensor_src = std::dynamic_pointer_cast<sensor::SensorPacketSource>(source.child());
    
    while (source_it()) {
        auto event = sensor_src->get_packet(1.0);
        
        if (event.type != sensor::ClientEvent::PACKET) continue;
        
        auto& packet = event.packet();
        const int port = packet.type() == core::PacketType::Lidar ? lidar_port : imu_port;
        auto timestamp = packet.host_timestamp != 0 ? packet.host_timestamp / 1000 : now_in_microseconds();
        
        pcap::record_packet(*handle, "127.0.0.1", "127.0.0.1", port,
                            port, packet.buf.data(), packet.buf.size(), timestamp);
        ++n_packets;
    }

    pcap::record_uninitialize(*handle);
    std::cout << "Captured " << n_packets << " packets" << std::endl;
    //! [doc-etag-pcap-record]
    // clang-format on
}

void record_sensor_session_save_osf(const std::string& hostname, const std::string& output_file,
                                    int n_frames = 100) {
    // clang-format off
    //! [doc-stag-osf-write-setup]
    auto source = open_source(hostname);
    const auto& infos = source.sensor_info()[0];
    //! [doc-etag-osf-write-setup]
    //! [doc-stag-osf-write]
    osf::Writer writer(output_file, *infos);
    int written = 0;

    for (const auto& frame_set : source) {
        for (size_t idx = 0; idx < frame_set.size(); ++idx) {
            const auto& frame = frame_set[idx];
            if (!frame) { continue; }
            writer.save(static_cast<int>(idx), *frame); }
        if (++written >= n_frames) { break; }}
    //! [doc-etag-osf-write]
    // clang-format on

    writer.close();
}

void osf_slice_frames(const std::string& osf_file) {
    auto make_sliced_fname = [](const std::string& f) {
        auto base = f;
        auto idx = base.find_last_of('.');
        if (idx != std::string::npos) {
            base = base.substr(0, idx);
        }
        return base + "_sliced.osf";
    };

    auto dereference_infos = [](const auto& ptrs) {
        std::vector<core::SensorInfo> infos;
        for (const auto& ptr : ptrs) {
            if (ptr) infos.push_back(*ptr);
        }
        return infos;
    };
    // clang-format off
    //! [doc-stag-osf-slice-frames]
    osf::OsfFrameSetSource source(osf_file);
    std::vector<std::string> fields_to_write = {
        "RANGE", 
        "SIGNAL", 
        "REFLECTIVITY"};
    
    osf::Writer writer(make_sliced_fname(osf_file),
                       dereference_infos(source.sensor_info()),
                       fields_to_write);
    // Read frames and write back
    for (const auto& frame_set : source) {
        for (size_t idx = 0; idx < frame_set.size(); ++idx) {
            const auto& frame = frame_set[idx];
            if (!frame) { continue; }
            std::cout << "writing sliced frame with ts = "
                      << frame->get_first_valid_packet_timestamp() << std::endl;
            writer.save(static_cast<int>(idx), *frame); } }
    writer.close();
    //! [doc-etag-osf-slice-frames]
    // clang-format on
}

}  // namespace docs
}  // namespace ouster
