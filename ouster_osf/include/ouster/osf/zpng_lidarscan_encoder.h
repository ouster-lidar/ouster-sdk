/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include "ouster/lidar_scan.h"
#include "ouster/osf/lidarscan_encoder.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

static constexpr int DEFAULT_ZPNG_OSF_COMPRESSION_LEVEL = 1;

class OUSTER_API_CLASS ZPngLidarScanEncoder
    : public ouster::osf::LidarScanEncoder {
   public:
    OUSTER_API_FUNCTION
    ZPngLidarScanEncoder(int compression_amount)
        : compression_amount_{compression_amount} {}

    // TODO these methods do essentially the same thing and should be
    // deduplicated. Standard fields are stored in destaggered form, but this
    // should be a detail specific to and hidden by the encoder.

    // This method is for standard destaggered fields.
    // FIXME[tws] method should be private, but "friend class/FRIEND_TEST" for
    // the unit test isn't working for some reason
    OUSTER_API_IGNORE
    bool fieldEncode(const LidarScan& lidar_scan,
                     const ouster::FieldType& field_type,
                     const std::vector<int>& px_offset, ScanData& scan_data,
                     size_t scan_idx) const override;

    // This method is for custom fields.
    // FIXME[tws] method should be private, but "friend class/FRIEND_TEST" for
    // the unit test isn't working for some reason
    OUSTER_API_IGNORE
    ScanChannelData encodeField(const ouster::Field& field) const override;

   private:
    int compression_amount_{0};
};
}  // namespace osf
}  // namespace ouster
