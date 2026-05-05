/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <ouster/indexed_pcap_reader.h>
#include <ouster/open_source.h>
#include <ouster/packet_source.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ouster {
namespace sdk {
namespace pcap {

/// Finds associated metadata json files for the given data source filename
/// @param[in] data_path Path to find metadata for.
/// @return list of metadata filenames
OUSTER_API_FUNCTION
std::vector<std::string> resolve_metadata_multi(const std::string& data_path);

/// Options for PcapPacketSource
struct OUSTER_API_CLASS PcapPacketSourceOptions
    : private ouster::sdk::PacketSourceOptions {
    using PacketSourceOptions::check;
    using PacketSourceOptions::extrinsics;
    using PacketSourceOptions::extrinsics_file;
    using PacketSourceOptions::index;
    using PacketSourceOptions::meta;
    using PacketSourceOptions::sensor_info;
    using PacketSourceOptions::soft_id_check;

    /**
     * Construct PcapPacketSourceOptions from a PacketSourceOptions object.
     *
     * Allows the Pcap-specific options to be initialized from a more general
     * PacketSourceOptions instance, enabling reuse of configuration values
     * across multiple packet source types.
     *
     * @param[in] o A PacketSourceOptions object to copy configuration from.
     */
    OUSTER_API_FUNCTION
    PcapPacketSourceOptions(const PacketSourceOptions& o);

    OUSTER_API_FUNCTION
    PcapPacketSourceOptions();
};

class PcapScanSource;
class PcapPacketIteratorImpl;
class PcapScanIteratorImpl;
/// PacketSource that produces packets from a given PCAP file
class OUSTER_API_CLASS PcapPacketSource
    : public ouster::sdk::core::PacketSource,
      ouster::sdk::impl::PacketSourceBuilder<ouster::sdk::core::IoType::PCAP,
                                             PcapPacketSource> {
    std::unique_ptr<ouster::sdk::pcap::IndexedPcapReader> reader_;
    friend class PcapPacketIteratorImpl;
    friend class PcapScanSource;
    friend class PcapScanIteratorImpl;

    int64_t start_location_;
    bool index_;
    bool soft_id_check_;

    uint64_t size_error_count_ = 0;
    uint64_t id_error_count_ = 0;

    // maps from port + serial number to sensor index
    std::map<uint16_t, std::map<uint64_t, int>> port_info_;
    std::vector<std::shared_ptr<ouster::sdk::core::PacketFormat>>
        packet_formats_;
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_info_;

    ouster::sdk::core::PacketIterator begin_scan(uint64_t scan_index) const;

   public:
    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    PcapPacketSource(
        const std::string& file,  ///< [in] pcap file to open
        const std::function<void(PcapPacketSourceOptions&)>& options = {}
        ///< [in] scan source options
    );

    /// open source compatible constructor
    OUSTER_API_FUNCTION
    PcapPacketSource(
        const std::string& file,         ///< [in] pcap file to open
        PcapPacketSourceOptions options  ///< [in] scan source options
    );

    OUSTER_API_FUNCTION
    ouster::sdk::core::PacketIterator begin() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    /// Get number of id errors that occurred while retrieving packets
    /// @return numer of id errors
    OUSTER_API_FUNCTION
    uint64_t id_error_count() const;

    /// Get number of size errors that occurred while retrieving packets
    /// @return numer of size errors
    OUSTER_API_FUNCTION
    uint64_t size_error_count() const;

   protected:
    OUSTER_API_FUNCTION
    void close() override;
};

}  // namespace pcap
}  // namespace sdk
}  // namespace ouster
