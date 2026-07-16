/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <vector>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

/** Encoded single PNG buffer */
using FrameChannelData = std::vector<uint8_t>;

/** Encoded PNG buffers */
using FrameData = std::vector<FrameChannelData>;

/**
 * @brief Interface for encoding lidar frame fields.
 *
 * This interface defines a method to encode a field of a lidar frame,
 * optionally destaggering it based on pixel offsets.
 */
class OUSTER_API_CLASS LidarFrameEncoder {
   public:
    OUSTER_API_FUNCTION
    virtual ~LidarFrameEncoder() = default;

    /** This method encodes a field, if px_offset is provided it is destaggered
     * before encoding
     * @param[in] field The field to encode (e.g., RANGE, SIGNAL).
     * @param[in] px_offset Optional pixel offsets for destaggering the data.
     * @return Encoded binary data representing the field.
     *
     * @throws std::runtime_error If encoding fails (e.g., invalid field shape,
     *         unsupported field type, or backend encoding errors).
     */
    OUSTER_API_FUNCTION
    virtual FrameChannelData encode_field(const ouster::sdk::core::Field& field,
                                          const std::vector<int>& px_offset = {}) const = 0;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace osf {

OUSTER_DEPRECATED_TYPE(LidarScanEncoder, LidarFrameEncoder, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanChannelData, FrameChannelData, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanData, FrameData, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
