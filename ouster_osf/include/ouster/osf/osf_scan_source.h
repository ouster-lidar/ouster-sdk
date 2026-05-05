/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

#include "ouster/impl/build.h"
#include "ouster/impl/open_source_impl.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/osf/reader.h"
#include "ouster/scan_source.h"
#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace osf {

/// Options for the OsfScanSource
class OUSTER_API_CLASS OsfScanSourceOptions
    : private ouster::sdk::ScanSourceOptions {
   public:
    using ScanSourceOptions::check;
    using ScanSourceOptions::error_handler;
    using ScanSourceOptions::extrinsics;
    using ScanSourceOptions::extrinsics_file;
    using ScanSourceOptions::field_names;
    using ScanSourceOptions::index;

    /**
     * @brief Construct OsfScanSourceOptions from a ScanSourceOptions object.
     *
     * @param[in] o The ScanSourceOptions object to initialize from.
     */
    OUSTER_API_FUNCTION
    OsfScanSourceOptions(const ScanSourceOptions& o);

    OUSTER_API_FUNCTION
    OsfScanSourceOptions();
};

/// ScanSource that produces LidarScans from a given OSF file
class OUSTER_API_CLASS OsfScanSource
    : public ouster::sdk::core::ScanSource,
      ouster::sdk::impl::ScanSourceBuilder<ouster::sdk::core::IoType::OSF,
                                           OsfScanSource> {
    friend class OsfScanIteratorImpl;
    friend class OsfCollationIteratorImpl;
    std::unique_ptr<ouster::sdk::osf::Reader> reader_;
    // should match stream id to sensor info
    std::map<uint32_t, std::shared_ptr<ouster::sdk::core::SensorInfo>>
        sensor_info_;
    // map stream id to sensor index
    std::map<int, uint32_t> sensor_ids_;
    std::vector<uint32_t> valid_ids_;
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_infos_;
    std::vector<size_t> scans_num_;
    std::vector<std::pair<uint64_t, uint64_t>> index_;
    std::vector<uint64_t> collation_index_;
    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> real_index_;
    nonstd::optional<std::vector<std::string>> desired_fields_;
    size_t scan_count_;
    bool indexed_;

    // registers this with the open_source factory
    inline void dummy() { (void)REGISTERED; }

    OsfScanSource(const std::string& file,       ///< [in] OSF file to open
                  OsfScanSourceOptions options,  ///< [in] scan source options
                  bool read_collations           ///< [in] read collations from
                                                 ///<      osf file if present
    );

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
    ouster::sdk::core::ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::sdk::core::ScanIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

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
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
    sensor_info() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<ScanSource> move() override;

    /**
     * Check if the OsfScanSource has collations available and enabled
     *
     * @return true if collations are available and enabled
     */
    OUSTER_API_FUNCTION
    bool is_collated() const;

    /**
     * open_source compatible factory that handles different collating
     * behaviours.
     *
     * @relates ouster::open_source
     *
     * @param[in] sources source filenames
     * @param[in] options source options
     * @param[in] collate whether to collate the source or not
     * @param[in] sensor_idx access specific sensor index in the osf
     * @return unique_ptr of ScanSource type
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<core::ScanSource> create(
        const std::vector<std::string>& sources,
        const ScanSourceOptions& options, bool collate, int sensor_idx = -1);
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
