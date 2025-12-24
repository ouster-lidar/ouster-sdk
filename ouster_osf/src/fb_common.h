/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <nonstd/optional.hpp>
#include <vector>

#include "os_sensor/common_generated.h"
#include "os_sensor/lidar_scan_stream_generated.h"
#include "ouster/error_handler.h"
#include "ouster/field.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/impl/png_tools.h"
#include "ouster/osf/lidarscan_encoder.h"
#include "ouster/types.h"

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
        memcpy(buf_to_write, reinterpret_cast<const uint8_t*>(v),
               sizeof(T) * len);
    }
    return res_off;
}

impl::gen::CHAN_FIELD_TYPE to_osf_enum(ouster::sdk::core::ChanFieldType f);

ouster::sdk::core::ChanFieldType from_osf_enum(impl::gen::CHAN_FIELD_TYPE ft);

impl::gen::FIELD_CLASS to_osf_enum(ouster::sdk::core::FieldClass ff);

ouster::sdk::core::FieldClass from_osf_enum(impl::gen::FIELD_CLASS ff);

nonstd::optional<impl::gen::CHAN_FIELD> to_osf_enum(const std::string& f);

std::string from_osf_enum(impl::gen::CHAN_FIELD f);

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>>
fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScanEncoder& encoder,
    const std::vector<std::pair<std::string, const ouster::sdk::core::Field*>>&
        fields);

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>>
fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScanEncoder& encoder,
    const std::unordered_map<std::string, ouster::sdk::core::Field>& fields);

flatbuffers::Offset<
    flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>>
fb_save_scan_channels(flatbuffers::FlatBufferBuilder& fbb,
                      const LidarScanEncoder& encoder,
                      const ouster::sdk::core::LidarScan& scan,
                      const ouster::sdk::core::LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset);

using AddFieldFn = std::function<ouster::sdk::core::Field&(
    const std::string&, const ouster::sdk::core::FieldDescriptor&,
    ouster::sdk::core::FieldClass)>;

void fb_restore_fields(
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>* fb_fields,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode,
    AddFieldFn add_field, const core::error_handler_t& error_handler);

void fb_restore_channels(
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>*
        fb_channels,
    const ouster::sdk::core::LidarScanFieldTypes& field_types,
    const std::vector<int>& px_offset, ouster::sdk::core::LidarScan& scan,
    const core::error_handler_t& error_handler);

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
