/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <nonstd/optional.hpp>
#include <vector>

#include "os_sensor/common_generated.h"
#include "os_sensor/lidar_scan_stream_generated.h"
#include "ouster/core/error_handler.h"
#include "ouster/core/field.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object.h"
#include "ouster/core/pose.h"
#include "ouster/core/types.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/impl/png_tools.h"
#include "ouster/osf/lidarframe_encoder.h"

namespace ouster {
namespace sdk {
namespace osf {

// After Flatbuffers >= 22.9.24 the alignment bug was introduced in the #7520
// https://github.com/google/flatbuffers/pull/7520
// That is manifested in the additional 2 bytes written to the vector buffer
// before adding the vector size that are not properly handled by the reading
// the buffer back since vector is the 4 bytes length with the data starting
// right after the size. And with those additional 2 zero bytes it's just
// ruining the vector data.
// It started happening because of alignment rules changed in #7520 and it's
// triggering for our code because we have a small structs of just 2 bytes
// which can result in not 4 bytes aligned memory that is min requirement
// for storing the subsequent vector length in uoffset_t type)
// FIX[pb]: We are changing the original create_vector_of_structs implementation
//      with CreateUninitializedVectorOfStructs impl that is different in
//      a way how it's using StartVector underneath.
template <typename T>
flatbuffers::Offset<flatbuffers::Vector<const T*>> create_vector_of_structs(
    flatbuffers::FlatBufferBuilder& fbb, const T* v, size_t len) {
    T* buf_to_write;
    auto res_off = fbb.CreateUninitializedVectorOfStructs(len, &buf_to_write);
    if (len > 0) {
        memcpy(buf_to_write, reinterpret_cast<const uint8_t*>(v), sizeof(T) * len);
    }
    return res_off;
}

impl::gen::CHAN_FIELD_TYPE to_osf_enum(ouster::sdk::core::ChanFieldType field_type);

ouster::sdk::core::ChanFieldType from_osf_enum(impl::gen::CHAN_FIELD_TYPE field_type);

impl::gen::FIELD_CLASS to_osf_enum(ouster::sdk::core::FieldClass field_class);

ouster::sdk::core::FieldClass from_osf_enum(impl::gen::FIELD_CLASS field_class);

nonstd::optional<impl::gen::CHAN_FIELD> to_osf_enum(const std::string& f);

std::string from_osf_enum(impl::gen::CHAN_FIELD f);

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>> fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
    const std::vector<std::pair<std::string, const ouster::sdk::core::Field*>>& fields);

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>> fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
    const std::unordered_map<std::string, ouster::sdk::core::Field>& fields);

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>>
fb_save_frame_channels(flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
                       const ouster::sdk::core::LidarFrame& frame,
                       const ouster::sdk::core::LidarFrameFieldTypes& field_types,
                       const std::vector<int>& px_offset);

using AddFieldFn = std::function<ouster::sdk::core::Field&(
    const std::string&, const ouster::sdk::core::FieldDescriptor&, ouster::sdk::core::FieldClass)>;

void fb_restore_fields(const flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>* fb_fields,
                       const nonstd::optional<std::vector<std::string>>& fields_to_decode,
                       const AddFieldFn& add_field, const core::error_handler_t& error_handler);

void fb_restore_channels(
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>* fb_channels,
    const ouster::sdk::core::LidarFrameFieldTypes& field_types, const std::vector<int>& px_offset,
    ouster::sdk::core::LidarFrame& frame, const core::error_handler_t& error_handler);

flatbuffers::Offset<impl::gen::Pose> fb_save_pose(flatbuffers::FlatBufferBuilder& fbb,
                                                  const core::Pose& pose);

bool fb_restore_pose(const impl::gen::Pose* fb_pose, core::Pose& pose,
                     std::vector<std::pair<core::Severity, std::string>>& errors,
                     const std::string& context);

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::ObjectList>>>
fb_save_object_lists(
    flatbuffers::FlatBufferBuilder& fbb,
    const std::unordered_map<std::string, std::vector<core::Object>>& object_lists);

void fb_restore_object_lists(
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::ObjectList>>* fb_obj_lists,
    std::unordered_map<std::string, std::vector<core::Object>>& object_lists,
    const core::error_handler_t& error_handler);

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
