/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

/** Encoded single PNG buffer */
using ScanChannelData = std::vector<uint8_t>;

/** Encoded PNG buffers */
using ScanData = std::vector<ScanChannelData>;

/**
 * @brief Interface for encoding lidar scan fields.
 *
 * This interface defines a method to encode a field of a lidar scan,
 * optionally destaggering it based on pixel offsets.
 */
class OUSTER_API_CLASS LidarScanEncoder {
   public:
    OUSTER_API_FUNCTION
    virtual ~LidarScanEncoder() = default;

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
    virtual ScanChannelData encode_field(
        const ouster::sdk::core::Field& field,
        const std::vector<int>& px_offset = {}) const = 0;

    /**
     * @deprecated
     * @param[in] field deprecated
     * @param[in] px_offset deprecated
     * @return deprecated
     */
    OUSTER_DIAGNOSTIC_PUSH
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED
    OUSTER_DEPRECATED_MSG(encode_field, OUSTER_DEPRECATED_LAST_SUPPORTED_0_16)
    OUSTER_API_FUNCTION
    virtual ScanChannelData encodeField(
        const ouster::sdk::core::Field& field,
        const std::vector<int>& px_offset = {}) const {
        return encode_field(field, px_offset);
    };
    OUSTER_DIAGNOSTIC_POP
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
