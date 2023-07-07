/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file pcap_source.h
 * @brief Pcap raw data source
 *
 */
#pragma once

#include <chrono>
#include <map>

#include "ouster/lidar_scan.h"
#include "ouster/os_pcap.h"
#include "ouster/osf/basics.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

/**
 * Wrapper to process pcap files with lidar packet sensor data.
 * Currently supports a single sensor, but can be extended easily for
 * multisensor pcaps.
 */
class PcapRawSource {
   public:
    using ts_t = std::chrono::nanoseconds;

    /// Lidar data callbacks
    using LidarDataHandler =
        std::function<void(const ts_t, const ouster::LidarScan&)>;

    /// General pcap packet handler
    using PacketHandler = std::function<void(
        const ouster::sensor_utils::packet_info&, const uint8_t*)>;

    // Predicate to control the bag run loop
    using PacketInfoPredicate =
        std::function<bool(const ouster::sensor_utils::packet_info&)>;

    /**
     * Opens pcap file and checks available packets inside with
     * heuristics applied to guess Ouster lidar port with data.
     */
    PcapRawSource(const std::string& filename);

    /**
     * Attach lidar data handler to the port that receives already
     * batched LidarScans with a timestamp of the first UDP lidar packet.
     * LidarScan uses default field types by the profile
     */
    void addLidarDataHandler(int dst_port,
                             const ouster::sensor::sensor_info& info,
                             LidarDataHandler&& lidar_handler);

    /**
     * The addLidarDataHandler() function overload. In this function, LidarScan
     * uses user modified field types
     */
    void addLidarDataHandler(int dst_port,
                             const ouster::sensor::sensor_info& info,
                             const LidarScanFieldTypes& ls_field_types,
                             LidarDataHandler&& lidar_handler);

    /**
     * Read all packets from pcap and pass data to the attached handlers
     * based on `dst_port` from pcap packets.
     */
    void runAll();

    /**
     * Run the internal loop through all packets while the
     * `pred(pinfo) == true`.
     * `pred` function called before reading packet buffer and passing to the
     * appropriate handlers.
     */
    void runWhile(const PacketInfoPredicate& pred);

    /**
     * Close pcap file
     */
    ~PcapRawSource();

   private:
    PcapRawSource(const PcapRawSource&) = delete;
    PcapRawSource& operator=(const PcapRawSource&) = delete;

    void handleCurrentPacket(const sensor_utils::packet_info& pinfo);

    std::string pcap_filename_;
    ouster::sensor::sensor_info info_;
    std::shared_ptr<ouster::sensor_utils::playback_handle> pcap_handle_{
        nullptr};
    std::map<int, PacketHandler> packet_handlers_{};
};

}  // namespace osf
}  // namespace ouster
