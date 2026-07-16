/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/frame_set_source.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/pcap/pcap_packet_source.h"

namespace ouster {
namespace sdk {
namespace pcap {

/// Options for the PcapFrameSetSource
struct OUSTER_API_CLASS PcapFrameSetSourceOptions : private ouster::sdk::FrameSetSourceOptions {
    using FrameSetSourceOptions::extrinsics;
    using FrameSetSourceOptions::extrinsics_file;
    using FrameSetSourceOptions::field_names;
    using FrameSetSourceOptions::index;
    using FrameSetSourceOptions::meta;
    using FrameSetSourceOptions::raw_fields;
    using FrameSetSourceOptions::raw_headers;
    using FrameSetSourceOptions::sensor_info;
    using FrameSetSourceOptions::soft_id_check;

    using FrameSetSourceOptions::check;

    /**
     * @brief Construct PcapFrameSetSourceOptions from a FrameSetSourceOptions
     * object.
     *
     * Initializes the Pcap-specific frame set source options by copying from a
     * `FrameSetSourceOptions` instance. This enables reuse of configuration
     * parameters like `sensor_info`, `meta`, `extrinsics`, and others across
     * multiple frame set source types.
     *
     * @param[in] o The FrameSetSourceOptions object to initialize from.
     */
    OUSTER_API_FUNCTION
    PcapFrameSetSourceOptions(const FrameSetSourceOptions& o);

    OUSTER_API_FUNCTION
    PcapFrameSetSourceOptions();
};

class PcapFrameSetIteratorImpl;
/// FrameSetSource that produces LidarFrames from a given PCAP file
class OUSTER_API_CLASS PcapFrameSetSource
    : public ouster::sdk::core::FrameSetSource,
      ouster::sdk::impl::FrameSetSourceBuilder<ouster::sdk::core::IoType::PCAP,
                                               PcapFrameSetSource> {
    friend class PcapFrameSetIteratorImpl;
    PcapPacketSource packets_;

    size_t num_frames_;
    std::vector<size_t> frames_count_;
    std::vector<std::vector<ouster::sdk::core::FieldType>> field_types_;

    std::vector<std::vector<std::pair<uint64_t, uint64_t>>> index_;
    std::vector<std::pair<uint64_t, uint64_t>> real_index_;

    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> internal_sensor_info_;

    bool indexed_ = false;

    size_t size_hint_ = 0;

    void assert_indexed(const char* function) const;

    // registers this with the open_source factory
    inline void dummy() {
        (void)REGISTERED;
    }

   protected:
    OUSTER_API_FUNCTION void close() override;

   public:
    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    PcapFrameSetSource(const std::string& source,  ///< [in] sensor hostnames to connect to
                       const std::function<void(PcapFrameSetSourceOptions&)>& options = {}
                       ///< [in] frame set source options
    );

    /// open_source compatible constructor
    OUSTER_API_FUNCTION
    PcapFrameSetSource(const std::string& source,         ///< [in] sensor hostnames to connect to,
                                                          ///< for multiple comma separate
                       PcapFrameSetSourceOptions options  ///< [in] frame set source options
    );

    OUSTER_API_FUNCTION
    ouster::sdk::core::FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::sdk::core::FrameSetIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_info() const override;

    OUSTER_API_FUNCTION
    size_t size() const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    bool is_indexed() const override;

    OUSTER_API_FUNCTION
    const std::vector<size_t>& frames_num() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& individual_index()
        const override;

    OUSTER_API_FUNCTION
    const std::vector<std::pair<uint64_t, uint64_t>>& full_index() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<ouster::sdk::core::FrameSetSource> move() override;

    /// Return the number of id errors that occurred while building frames
    /// @return count of id errors
    OUSTER_API_FUNCTION
    uint64_t id_error_count() const;

    /// Return the number of size errors that occurred while building frames
    /// @return count of size errors
    OUSTER_API_FUNCTION
    uint64_t size_error_count() const;

    /**
     * open_source compatible factory.
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
    static std::unique_ptr<ouster::sdk::core::FrameSetSource> create(
        const std::vector<std::string>& sources, const FrameSetSourceOptions& options, bool collate,
        int sensor_idx = -1);
};
}  // namespace pcap
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace pcap {

OUSTER_DEPRECATED_TYPE(PcapScanSource, PcapFrameSetSource, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(PcapScanSourceOptions, PcapFrameSetSourceOptions,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace pcap
}  // namespace sdk
}  // namespace ouster
