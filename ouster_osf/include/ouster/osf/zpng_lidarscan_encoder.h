/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/osf/lidarscan_encoder.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Default compression level for ZPNG-encoded lidar scan fields.
 */
static constexpr int DEFAULT_ZPNG_OSF_COMPRESSION_LEVEL = 1;

/**
 * @brief Zlib-compressed PNG encoder for lidar scan fields.
 *
 * This is a more performant variant of `PngLidarScanEncoder.
 */
class OUSTER_API_CLASS ZPngLidarScanEncoder
    : public ouster::sdk::osf::LidarScanEncoder {
   public:
    /**
     * @brief Construct a ZPngLidarScanEncoder with a specified compression
     * level.
     *
     * @param[in] compression_amount Compression level passed to zlib (0 = none,
     * 9 = max).
     */
    OUSTER_API_FUNCTION
    ZPngLidarScanEncoder(int compression_amount)
        : compression_amount_{compression_amount} {}

    // This method is for standard destaggered fields.
    // FIXME[tws] method should be private, but "friend class/FRIEND_TEST" for
    // the unit test isn't working for some reason
    OUSTER_API_IGNORE
    ScanChannelData encode_field(
        const ouster::sdk::core::Field& field,
        const std::vector<int>& px_offset = {}) const override;

   private:
    int compression_amount_{0};
};
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
