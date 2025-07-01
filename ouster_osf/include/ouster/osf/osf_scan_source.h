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
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/scan_source.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

/// Options for the OsfScanSource
class OUSTER_API_CLASS OsfScanSourceOptions
    : private ouster::ScanSourceOptions {
   public:
    using ouster::ScanSourceOptions::check;
    using ouster::ScanSourceOptions::error_handler;
    using ouster::ScanSourceOptions::extrinsics;
    using ouster::ScanSourceOptions::extrinsics_file;
    using ouster::ScanSourceOptions::field_names;
    using ouster::ScanSourceOptions::index;

    OUSTER_API_FUNCTION
    OsfScanSourceOptions(const ScanSourceOptions& o);

    OUSTER_API_FUNCTION
    OsfScanSourceOptions();
};

/// ScanSource that produces LidarScans from a given OSF file
class OUSTER_API_CLASS OsfScanSource
    : public ouster::core::ScanSource,
      ouster::impl::ScanSourceBuilder<ouster::core::IoType::OSF,
                                      OsfScanSource> {
    friend class OsfScanIteratorImpl;
    std::unique_ptr<ouster::osf::Reader> reader_;
    // should match stream id to sensor info
    std::map<uint32_t, std::shared_ptr<ouster::sensor::sensor_info>>
        sensor_info_;
    // map stream id to sensor index
    std::map<int, uint32_t> sensor_ids_;
    std::vector<uint32_t> valid_ids_;
    std::vector<std::shared_ptr<ouster::sensor::sensor_info>> sensor_infos_;
    std::vector<size_t> scans_num_;
    std::vector<std::pair<uint64_t, uint64_t>> index_;
    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> real_index_;
    nonstd::optional<std::vector<std::string>> desired_fields_;
    size_t scan_count_;
    bool indexed_;

    // registers this with the open_source factory
    inline void dummy() { (void)registered_; }

   protected:
    void close() override;

   public:
    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    OsfScanSource(const std::string& file,  ///< [in] OSF file to open
                  const std::function<void(OsfScanSourceOptions&)>& options = {}
                  ///< [in] scan source options
    );

    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    OsfScanSource(const std::string& file,      ///< [in] OSF file to open
                  OsfScanSourceOptions options  ///< [in] scan source options
    );

    OUSTER_API_FUNCTION
    ouster::core::ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::core::ScanIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& scans_num() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
    individual_index() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index()
        const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override;
};
}  // namespace osf
}  // namespace ouster
