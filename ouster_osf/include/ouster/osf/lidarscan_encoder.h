/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

// Encoded single PNG buffer
using ScanChannelData = std::vector<uint8_t>;

// Encoded PNG buffers
using ScanData = std::vector<ScanChannelData>;

class OUSTER_API_CLASS LidarScanEncoder {
   public:
    OUSTER_API_FUNCTION
    virtual ~LidarScanEncoder() = default;

   private:
    // This method encodes a field, if px_offset is provided it is destaggered
    // before encoding
    virtual ScanChannelData encodeField(
        const ouster::Field& field,
        const std::vector<int>& px_offset = {}) const = 0;

    friend class LidarScanStream;
};

}  // namespace osf
}  // namespace ouster
