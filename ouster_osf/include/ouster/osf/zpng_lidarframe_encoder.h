/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <vector>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/lidarframe_encoder.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Default compression level for ZPNG-encoded lidar frame fields.
 */
static constexpr int DEFAULT_ZPNG_OSF_COMPRESSION_LEVEL = 1;

/**
 * @brief Zlib-compressed PNG encoder for lidar frame fields.
 *
 * This is a more performant variant of `PngLidarFrameEncoder.
 */
class OUSTER_API_CLASS ZPngLidarFrameEncoder : public ouster::sdk::osf::LidarFrameEncoder {
   public:
    /**
     * @brief Construct a ZPngLidarFrameEncoder with a specified compression
     * level.
     *
     * @param[in] compression_amount Compression level passed to zlib (0 = none,
     * 9 = max).
     */
    OUSTER_API_FUNCTION
    ZPngLidarFrameEncoder(int compression_amount) : compression_amount_{compression_amount} {}

    // This method is for standard destaggered fields.
    // FIXME[tws] method should be private, but "friend class/FRIEND_TEST" for
    // the unit test isn't working for some reason
    OUSTER_API_FUNCTION
    FrameChannelData encode_field(const ouster::sdk::core::Field& field,
                                  const std::vector<int>& px_offset = {}) const override;

   private:
    int compression_amount_{0};
};
}  // namespace osf
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace osf {

OUSTER_DEPRECATED_TYPE(ZPngLidarScanEncoder, ZPngLidarFrameEncoder,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
