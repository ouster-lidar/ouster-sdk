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

#include "ouster/core/frame_set_source.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/impl/open_source_impl.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/reader.h"

namespace ouster {
namespace sdk {
namespace osf {

/// Options for the OsfFrameSetSource
class OUSTER_API_CLASS OsfFrameSetSourceOptions : private ouster::sdk::FrameSetSourceOptions {
   public:
    using FrameSetSourceOptions::check;
    using FrameSetSourceOptions::error_handler;
    using FrameSetSourceOptions::extrinsics;
    using FrameSetSourceOptions::extrinsics_file;
    using FrameSetSourceOptions::field_names;
    using FrameSetSourceOptions::index;

    /**
     * @brief Construct OsfFrameSetSourceOptions from a FrameSetSourceOptions
     * object.
     *
     * @param[in] o The FrameSetSourceOptions object to initialize from.
     */
    OUSTER_API_FUNCTION
    OsfFrameSetSourceOptions(const FrameSetSourceOptions& o);

    OUSTER_API_FUNCTION
    OsfFrameSetSourceOptions();
};

/// FrameSetSource that produces LidarFrames from a given OSF file
class OUSTER_API_CLASS OsfFrameSetSource
    : public ouster::sdk::core::FrameSetSource,
      ouster::sdk::impl::FrameSetSourceBuilder<ouster::sdk::core::IoType::OSF, OsfFrameSetSource> {
    friend class OsfFrameSetIteratorImpl;
    friend class OsfCollationIteratorImpl;
    std::unique_ptr<ouster::sdk::osf::Reader> reader_;
    // should match stream id to sensor info
    std::map<uint32_t, std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_info_;
    // map stream id to sensor index
    std::map<int, uint32_t> sensor_ids_;
    std::vector<uint32_t> valid_ids_;
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_infos_;
    std::vector<size_t> frames_num_;
    std::vector<std::pair<uint64_t, uint64_t>> index_;
    std::vector<uint64_t> collation_index_;
    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> real_index_;
    nonstd::optional<std::vector<std::string>> desired_fields_;
    size_t frame_count_;
    bool indexed_;
    bool had_collations_;

    // registers this with the open_source factory
    inline void dummy() {
        (void)REGISTERED;
    }

    OsfFrameSetSource(const std::string& file,           ///< [in] OSF file to open
                      OsfFrameSetSourceOptions options,  ///< [in] frame set source options
                      bool read_collations               ///< [in] read collations from
                                                         ///<      osf file if present
    );

   protected:
    OUSTER_API_FUNCTION void close() override;

   public:
    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    OsfFrameSetSource(const std::string& file,  ///< [in] OSF file to open
                      const std::function<void(OsfFrameSetSourceOptions&)>& options = {}
                      ///< [in] frame set source options
    );

    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    OsfFrameSetSource(const std::string& file,          ///< [in] OSF file to open
                      OsfFrameSetSourceOptions options  ///< [in] frame set source options
    );

    OUSTER_API_FUNCTION
    ouster::sdk::core::FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::sdk::core::FrameSetIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& frames_num() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_info() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override;

    OUSTER_API_FUNCTION
    bool is_collated() const override;

    OUSTER_API_FUNCTION
    bool contains_collations() const override;

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
     * @return unique_ptr of FrameSetSource type
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<core::FrameSetSource> create(const std::vector<std::string>& sources,
                                                        const FrameSetSourceOptions& options,
                                                        bool collate, int sensor_idx = -1);
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace osf {

OUSTER_DEPRECATED_TYPE(OsfScanSource, OsfFrameSetSource, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(OsfScanSourceOptions, OsfFrameSetSourceOptions,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
