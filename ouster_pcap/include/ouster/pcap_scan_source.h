/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/pcap_packet_source.h"
#include "ouster/scan_source.h"
#include "ouster/types.h"

namespace ouster {
namespace pcap {

/// Options for the PcapScanSource
struct OUSTER_API_CLASS PcapScanSourceOptions : private ScanSourceOptions {
    using ScanSourceOptions::extrinsics;
    using ScanSourceOptions::extrinsics_file;
    using ScanSourceOptions::field_names;
    using ScanSourceOptions::index;
    using ScanSourceOptions::meta;
    using ScanSourceOptions::raw_fields;
    using ScanSourceOptions::raw_headers;
    using ScanSourceOptions::sensor_info;
    using ScanSourceOptions::soft_id_check;

    using ScanSourceOptions::check;

    OUSTER_API_FUNCTION
    PcapScanSourceOptions(const ScanSourceOptions& o);

    OUSTER_API_FUNCTION
    PcapScanSourceOptions();
};

class PcapScanIteratorImpl;
/// ScanSource that produces LidarScans from a given PCAP file
class OUSTER_API_CLASS PcapScanSource
    : public ouster::core::ScanSource,
      ouster::impl::ScanSourceBuilder<ouster::core::IoType::PCAP,
                                      PcapScanSource> {
    friend class PcapScanIteratorImpl;
    ouster::pcap::PcapPacketSource packets_;

    size_t num_scans_;
    std::vector<size_t> scans_count_;
    std::vector<std::vector<ouster::FieldType>> field_types_;

    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> index_;
    std::vector<std::pair<uint64_t, uint64_t>> real_index_;

    bool indexed_ = false;

    void assert_indexed(const char* function) const;

    // registers this with the open_source factory
    inline void dummy() { (void)registered_; }

   protected:
    void close() override;

   public:
    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    PcapScanSource(
        const std::string& n,  ///< [in] sensor hostnames to connect to
        const std::function<void(PcapScanSourceOptions&)>& options = {}
        ///< [in] scan source options
    );

    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    PcapScanSource(
        const std::string& source,     ///< [in] sensor hostnames to connect to,
                                       ///< for multiple comma separate
        PcapScanSourceOptions options  ///< [in] scan source options
    );

    OUSTER_API_FUNCTION
    ouster::core::ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::core::ScanIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& scans_num() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override;

    /// Return the number of id errors that occurred while building scans
    /// @return count of id errors
    OUSTER_API_FUNCTION
    uint64_t id_error_count() const;

    /// Return the number of size errors that occurred while building scans
    /// @return count of size errors
    OUSTER_API_FUNCTION
    uint64_t size_error_count() const;
};
}  // namespace pcap
}  // namespace ouster
