/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/stream_camera_frame.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <nonstd/optional.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fb_common.h"
#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/field.h"
#include "ouster/core/impl/logging.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/frame_encoder.h"
#include "ouster/osf/impl/basics.h"  // NOLINT(misc-include-cleaner)
#include "ouster/osf/meta_camera_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

// Auto-generated flatbuffer headers.
#include "os_sensor/camera_frame_stream_generated.h"

using ouster::sdk::core::CameraFrame;
using ouster::sdk::core::CameraInfo;
using ouster::sdk::core::Field;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldDescriptor;
using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace osf {

// ============== CameraFrameStreamMeta ============================

CameraFrameStreamMeta::CameraFrameStreamMeta(uint32_t sensor_meta_id)
    : sensor_meta_id_{sensor_meta_id} {}

uint32_t CameraFrameStreamMeta::sensor_meta_id() const {
    return sensor_meta_id_;
}

std::vector<uint8_t> CameraFrameStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);
    auto cfs_offset = ouster::sdk::osf::impl::gen::CreateCameraFrameStream(fbb, sensor_meta_id_);
    fbb.FinishSizePrefixed(cfs_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<MetadataEntry> CameraFrameStreamMeta::from_buffer(const OsfBuffer& buf) {
    auto camera_frame_stream = impl::gen::GetSizePrefixedCameraFrameStream(buf.data());
    return std::make_unique<CameraFrameStreamMeta>(camera_frame_stream->sensor_id());
}

std::string CameraFrameStreamMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "CameraFrameStreamMeta: sensor_id = " << sensor_meta_id_;
    return string_stream.str();
}

// ============== CameraFrameStream ================================

CameraFrameStream::CameraFrameStream(Writer& writer, uint32_t sensor_meta_id,
                                     std::vector<std::string> fields_to_write)
    : writer_{writer}, meta_{sensor_meta_id}, desired_fields_{std::move(fields_to_write)} {
    writer_.add_metadata(meta_);
}

namespace {

flatbuffers::Offset<impl::gen::CameraFrameMsg> create_camera_frame_msg(
    flatbuffers::FlatBufferBuilder& fbb, const FrameEncoder& encoder,
    const CameraFrame& camera_frame, const std::vector<std::string>& desired_fields) {
    // Select the fields to write. An empty desired_fields means "all fields"
    // (matching the lidar path); otherwise only the named fields are written,
    // silently skipping any that the frame doesn't have.
    std::vector<std::pair<std::string, const Field*>> selected_fields;
    if (desired_fields.empty()) {
        for (const auto& field : camera_frame.fields()) {
            selected_fields.emplace_back(field.first, &field.second);
        }
    } else {
        for (const auto& name : desired_fields) {
            if (camera_frame.has_field(name)) {
                selected_fields.emplace_back(name, &camera_frame.field(name));
            }
        }
    }

    auto fields_off = fb_save_fields(fbb, encoder, selected_fields);
    auto obj_lists_off = fb_save_object_lists(fbb, camera_frame.objects());
    auto timestamp_off = fbb.CreateVector<uint64_t>(camera_frame.timestamp().data(),
                                                    camera_frame.timestamp().size());
    auto pose_off = fbb.CreateVector<double>(camera_frame.body_to_world().get<double>(),
                                             camera_frame.body_to_world().bytes() / sizeof(double));
    auto receive_timestamp_off = fbb.CreateVector<uint64_t>(
        camera_frame.receive_timestamp().data(), camera_frame.receive_timestamp().size());

    return impl::gen::CreateCameraFrameMsg(
        fbb, static_cast<uint64_t>(camera_frame.w), static_cast<uint64_t>(camera_frame.h),
        fields_off, obj_lists_off, timestamp_off, pose_off, receive_timestamp_off);
}

std::unique_ptr<CameraFrame> restore_camera_frame(
    const MessageRef& msg, std::shared_ptr<CameraInfo> info,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode) {
    auto cf_msg = flatbuffers::GetSizePrefixedRoot<impl::gen::CameraFrameMsg>(msg.data());

    auto camera_frame = std::make_unique<CameraFrame>();
    camera_frame->w = static_cast<std::size_t>(cf_msg->w());
    camera_frame->h = static_cast<std::size_t>(cf_msg->h());
    if (info) {
        camera_frame->set_sensor_info(std::move(info));
    }

    auto add_field = [&camera_frame](const std::string& name, const FieldDescriptor& desc,
                                     FieldClass field_class) -> Field& {
        return camera_frame->add_field(name, desc, field_class, false);
    };

    // make sure to keep objects in scope until futures are resolved
    auto fields = cf_msg->fields();
    auto futures = fb_restore_fields(fields, fields_to_decode, add_field, msg.error_handler());

    fb_restore_object_lists(cf_msg->objects(), camera_frame->objects(), msg.error_handler());

    auto ts_vec = cf_msg->timestamp();
    if (ts_vec != nullptr && ts_vec->size() == 1) {
        camera_frame->timestamp()[0] = ts_vec->Get(0);
    } else if (ts_vec != nullptr && ts_vec->size() != 0) {
        logger().error("ERROR: CameraFrameMsg has timestamp of length: {}, expected: 1",
                       ts_vec->size());
    }
    if (camera_frame->timestamp()[0] == 0) {
        camera_frame->timestamp()[0] = static_cast<uint64_t>(msg.ts().count());
    }

    // Restore per-frame packet timestamps if present.
    auto packet_ts_vec = cf_msg->packet_timestamp();
    if (packet_ts_vec != nullptr && packet_ts_vec->size() != 0) {
        if (static_cast<uint32_t>(camera_frame->receive_timestamp().size()) ==
            packet_ts_vec->size()) {
            for (uint32_t i = 0; i < packet_ts_vec->size(); ++i) {
                camera_frame->receive_timestamp()[i] = packet_ts_vec->Get(i);
            }
        } else {
            logger().error(
                "ERROR: CameraFrameMsg has receive_timestamp of length: {}, expected: {}",
                packet_ts_vec->size(), camera_frame->receive_timestamp().size());
        }
    }
    // A CameraFrame has no packets; if no meaningful packet timestamp was
    // serialized (e.g. legacy files), carry the message (host) timestamp so
    // downstream consumers (e.g. the visualizer, which uses packet timestamps to
    // pace frame presentation) have timing.
    if (camera_frame->receive_timestamp().rows() > 0 && camera_frame->receive_timestamp()[0] == 0) {
        camera_frame->receive_timestamp()[0] = static_cast<uint64_t>(msg.ts().count());
    }

    auto pose_vec = cf_msg->pose();
    if (pose_vec != nullptr) {
        auto& pose = camera_frame->body_to_world();
        if (pose.size() == pose_vec->size()) {
            std::memcpy(pose.get<double>(), pose_vec->Data(), pose.bytes());
        } else if (pose_vec->size() != 0) {
            logger().error("ERROR: CameraFrameMsg has pose of length: {}, expected: {}",
                           pose_vec->size(), pose.size());
        }
    }

    // Finally wait for the futures and catch any exceptions, passing them to the error handler
    for (auto& future : futures) {
        try {
            future.get();
        } catch (std::runtime_error& error) {
            msg.error_handler()(Severity::OUSTER_WARNING, error.what());
        }
    }

    // error if any of the requested fields did not end up in the camera frame
    if (fields_to_decode) {
        for (const auto& field : fields_to_decode.value()) {
            if (!camera_frame->has_field(field)) {
                throw std::runtime_error("Requested field '" + field + "' does not exist in OSF.");
            }
        }
    }

    return camera_frame;
}

}  // namespace

void CameraFrameStream::save(const ouster::sdk::osf::ts_t receive_ts,
                             const ouster::sdk::osf::ts_t sensor_ts,
                             const CameraFrame& camera_frame) {
    auto msg_buf = make_msg(camera_frame);
    writer_.save_message(meta_.id(), receive_ts, sensor_ts, msg_buf,
                         MetadataTraits<CameraFrameStreamMeta>::type());
}

std::vector<uint8_t> CameraFrameStream::make_msg(const CameraFrame& camera_frame) {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    const auto& encoder = writer_.encoder().frame_encoder();
    auto msg_offset = create_camera_frame_msg(fbb, encoder, camera_frame, desired_fields_);
    fbb.FinishSizePrefixed(msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<CameraFrameStream::obj_type> CameraFrameStream::decode_msg(
    const MessageRef& msg, const CameraFrameStream::meta_type& meta,
    const MetadataStore& meta_provider, const nonstd::optional<std::vector<std::string>>& fields) {
    // A camera's calibration lives in a CameraSensor footer entry, not the
    // LidarSensor one, so that is what this stream's sensor id refers to.
    std::shared_ptr<CameraInfo> info;
    if (auto sensor = meta_provider.get<CameraSensor>(meta.sensor_meta_id())) {
        info = std::dynamic_pointer_cast<CameraInfo>(sensor->sensor_info());
    }
    return restore_camera_frame(msg, std::move(info), fields);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
