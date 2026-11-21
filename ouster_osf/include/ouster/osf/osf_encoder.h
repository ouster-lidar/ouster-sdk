/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <memory>

#include "ouster/core/deprecation.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/frame_encoder.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * @brief used to configure the osf::Writer class.
 *
 * Right now it only contains a shared ptr to a FrameEncoder,
 * but in the future it may contain other items to allow parts of the OSF
 * encoding to vary independently.
 */
class OUSTER_API_CLASS Encoder {
   public:
    /**
     * @brief Construct an Encoder with a given FrameEncoder.
     *
     * @param[in] frame_encoder Shared pointer to the frame encoder instance.
     */
    OUSTER_API_FUNCTION
    explicit Encoder(const std::shared_ptr<FrameEncoder>& frame_encoder)
        : frame_encoder_{frame_encoder} {}

    /**
     * @brief Access the internal FrameEncoder instance.
     *
     * @return Reference to the FrameEncoder.
     */
    OUSTER_API_FUNCTION
    FrameEncoder& frame_encoder() const {
        return *frame_encoder_;
    }

    /**
     * @brief Deprecated alias for frame_encoder().
     *
     * @return Reference to the FrameEncoder.
     */
    OUSTER_DEPRECATED_MSG(frame_encoder(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    FrameEncoder& lidar_frame_encoder() const {
        return *frame_encoder_;
    }

   private:
    std::shared_ptr<FrameEncoder> frame_encoder_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
