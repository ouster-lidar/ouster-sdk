/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/camera_frame.h"

#include <cstdint>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include "ouster/core/camera_info.h"
#include "ouster/core/field.h"
#include "ouster/core/frame.h"

namespace ouster {
namespace sdk {
namespace core {

namespace {

// A CameraFrame has no packets. It carries a single packet timestamp (used for
// per-frame timing, e.g. visualization and collation) plus a single status
// word defaulted to valid so the shared Frame packet-timestamp accessors treat
// the frame's timestamp as valid, exactly like a lidar frame with valid
// columns.
void init_camera_headers(Field& receive_timestamp, Field& status) {
    receive_timestamp = Field{fd_array<uint64_t>(1), FieldClass::FRAME_FIELD};
    status = Field{fd_array<uint32_t>(1), FieldClass::FRAME_FIELD};
    status.get<uint32_t>()[0] = 1u;  // bit 0 = valid
}

}  // namespace

CameraFrame::CameraFrame() {
    init_timestamps_and_poses(1, FieldClass::FRAME_FIELD, FieldClass::FRAME_FIELD);
    init_camera_headers(receive_timestamp_, status_);
}

CameraFrame::CameraFrame(std::shared_ptr<CameraInfo> info) {
    set_sensor_info(std::move(info));
    init_timestamps_and_poses(1, FieldClass::FRAME_FIELD, FieldClass::FRAME_FIELD);
    init_camera_headers(receive_timestamp_, status_);
}

std::shared_ptr<Frame> CameraFrame::clone() const {
    return std::make_shared<CameraFrame>(*this);
}

std::string CameraFrame::to_string() const {
    std::stringstream string_stream;
    // A CameraFrame has no frame id (that is a LidarFrame member), so identify
    // the frame by its single timestamp instead.
    string_stream << "CameraFrame: {h = " << h << ", w = " << w
                  << ", packets_per_frame = " << receive_timestamp().size()
                  << ", ts = " << timestamp()[0] << "," << std::endl;
    impl::append_frame_fields(fields(), string_stream);
    string_stream << "}";
    return string_stream.str();
}

bool operator==(const CameraFrame& a, const CameraFrame& b) {
    return a.equals(b);
}

bool operator!=(const CameraFrame& a, const CameraFrame& b) {
    return !a.equals(b);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
