/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <vector>

#include "ouster/core/deprecation.h"
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
 * @brief Interface for encoding frame fields.
 *
 * This interface defines a method to encode a field of a frame, optionally
 * destaggering it based on pixel offsets. It serves every frame kind written to
 * an OSF, lidar and camera alike.
 */
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class OUSTER_API_CLASS FrameEncoder {
   public:
    OUSTER_API_FUNCTION
    virtual ~FrameEncoder() = default;

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
    // NOLINTNEXTLINE(google-default-arguments)
    virtual FrameChannelData encode_field(const ouster::sdk::core::Field& field,
                                          const std::vector<int>& px_offset = {}) const = 0;
};

/** @brief Deprecated alias for FrameEncoder. */
OUSTER_DEPRECATED_TYPE(LidarFrameEncoder, FrameEncoder, OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
