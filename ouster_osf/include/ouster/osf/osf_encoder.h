/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include "ouster/osf/lidarscan_encoder.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

/**
 * @brief used to configure the osf::Writer class.
 *
 * Right now it only contains a shared ptr to a LidarScanEncoder,
 * but in the future it may contain other items to allow parts of the OSF
 * encoding to vary independently.
 */
class OUSTER_API_CLASS Encoder {
   public:
    OUSTER_API_FUNCTION
    Encoder(const std::shared_ptr<LidarScanEncoder>& lidar_scan_encoder)
        : lidar_scan_encoder_{lidar_scan_encoder} {}

    OUSTER_API_FUNCTION
    LidarScanEncoder& lidar_scan_encoder() const {
        return *lidar_scan_encoder_;
    }

   private:
    std::shared_ptr<LidarScanEncoder> lidar_scan_encoder_;
};

}  // namespace osf
}  // namespace ouster
