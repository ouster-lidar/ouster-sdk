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

    /**
     * Lidar data callbacks
     *
     * @param timestamp[in] The timestamp for the scan.
     * @param scan[in] The LidarScan object.
     */
    using LidarDataHandler = std::function<void(const ts_t timestamp,
                                                const ouster::LidarScan& scan)>;

    /**
     * General pcap packet handler
     *
     * @param info[in] The sensor_info for the packet.
     * @param buf[in] The raw buffer for the packet.
     */
    using PacketHandler = std::function<void(
        const ouster::sensor_utils::packet_info& info, const uint8_t* buf)>;

    /**
     * Predicate to control the bag run loop
     *
     * @param info[in] The sensor_info for the packet.
     * @return True if the loop should continue, False if the loop should halt.
     */
    using PacketInfoPredicate =
        std::function<bool(const ouster::sensor_utils::packet_info& info)>;

    /**
     * Opens pcap file and checks available packets inside with
     * heuristics applied to guess Ouster lidar port with data.
     *
     * @param filename[in] The filename of the pcap file to open.
     */
    PcapRawSource(const std::string& filename);

    /**
     * Attach lidar data handler to the port that receives already
     * batched LidarScans with a timestamp of the first UDP lidar packet.
     * LidarScan uses default field types by the profile
     *
     * @param dst_port[in] The destination port for the target lidar stream.
     * @param info[in] The sensor info for the stream.
     * @param lidar_handler[in] The callback to call on packet.
     */
    void addLidarDataHandler(int dst_port,
                             const ouster::sensor::sensor_info& info,
                             LidarDataHandler&& lidar_handler);

    /**
     * @copydoc addLidarDataHandler
     * @param ls_field_types[in] The LidarScan field types to use.
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
     *
     * @param[in] pred The predicate function to decide whether to continue or
     *                 not.
     */
    void runWhile(const PacketInfoPredicate& pred);

    /**
     * Close the pcap file.
     */
    ~PcapRawSource();

   private:
    /// Remove some stuff
    PcapRawSource(const PcapRawSource&) = delete;
    PcapRawSource& operator=(const PcapRawSource&) = delete;

    /**
     * Read current packet and dispatch handlers accordingly.
     *
     * @param pinfo[in] The new packet info.
     */
    void handleCurrentPacket(const sensor_utils::packet_info& pinfo);

    /**
     * The pcap file path.
     */
    std::string pcap_filename_;

    /**
     * The associated sensor_info.
     */
    ouster::sensor::sensor_info info_;

    /**
     * The internal pcap file handler.
     */
    std::shared_ptr<ouster::sensor_utils::playback_handle> pcap_handle_{
        nullptr};

    /**
     * Map containing a 'destination port' to 'handler' mapping.
     */
    std::map<int, PacketHandler> packet_handlers_{};
};

}  // namespace osf
}  // namespace ouster
