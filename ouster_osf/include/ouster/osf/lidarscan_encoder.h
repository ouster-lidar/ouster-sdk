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
    // TODO these methods do essentially the same thing and should be
    // deduplicated. Standard fields are stored in destaggered form, but this
    // should be a detail specific to and hidden by the encoder.

    // This method is for standard destaggered fields.
    virtual bool fieldEncode(const LidarScan& lidar_scan,
                             const ouster::FieldType& field_type,
                             const std::vector<int>& px_offset,
                             ScanData& scan_data, size_t scan_idx) const = 0;

    // This method is for custom fields.
    virtual ScanChannelData encodeField(const ouster::Field& field) const = 0;
    friend class LidarScanStream;
};

}  // namespace osf
}  // namespace ouster
