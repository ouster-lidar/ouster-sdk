/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <ouster/indexed_pcap_reader.h>
#include <ouster/open_source.h>
#include <ouster/packet_source.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace ouster {
namespace pcap {

/// Finds associated metadata json files for the given data source filename
/// @return list of metadata filenames
OUSTER_API_FUNCTION
std::vector<std::string> resolve_metadata_multi(
    const std::string& data_path  /// <[in] path to find metadata for
);

/// Options for PcapPacketSource
struct OUSTER_API_CLASS PcapPacketSourceOptions
    : private ouster::PacketSourceOptions {
    using ouster::PacketSourceOptions::check;
    using ouster::PacketSourceOptions::extrinsics;
    using ouster::PacketSourceOptions::extrinsics_file;
    using ouster::PacketSourceOptions::index;
    using ouster::PacketSourceOptions::meta;
    using ouster::PacketSourceOptions::sensor_info;
    using ouster::PacketSourceOptions::soft_id_check;

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
    : public ouster::core::PacketSource,
      ouster::impl::PacketSourceBuilder<ouster::core::IoType::PCAP,
                                        PcapPacketSource> {
    std::unique_ptr<ouster::sensor_utils::IndexedPcapReader> reader_;
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
    std::vector<std::shared_ptr<ouster::sensor::packet_format>> packet_formats_;
    std::vector<std::shared_ptr<ouster::sensor::sensor_info>> sensor_info_;

    ouster::core::PacketIterator begin_scan(uint64_t scan_index) const;

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
    ouster::core::PacketIterator begin() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
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
}  // namespace ouster
