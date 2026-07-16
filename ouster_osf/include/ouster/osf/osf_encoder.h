/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <memory>

#include "ouster/core/visibility.h"
#include "ouster/osf/lidarframe_encoder.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * @brief used to configure the osf::Writer class.
 *
 * Right now it only contains a shared ptr to a LidarFrameEncoder,
 * but in the future it may contain other items to allow parts of the OSF
 * encoding to vary independently.
 */
class OUSTER_API_CLASS Encoder {
   public:
    /**
     * @brief Construct an Encoder with a given LidarFrameEncoder.
     *
     * @param[in] lidar_frame_encoder Shared pointer to the frame encoder
     * instance.
     */
    OUSTER_API_FUNCTION
    Encoder(const std::shared_ptr<LidarFrameEncoder>& lidar_frame_encoder)
        : lidar_frame_encoder_{lidar_frame_encoder} {}

    /**
     * @brief Access the internal LidarFrameEncoder instance.
     *
     * @return Reference to the LidarFrameEncoder.
     */
    OUSTER_API_FUNCTION
    LidarFrameEncoder& lidar_frame_encoder() const {
        return *lidar_frame_encoder_;
    }

   private:
    std::shared_ptr<LidarFrameEncoder> lidar_frame_encoder_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
