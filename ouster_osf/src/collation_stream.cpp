/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/collation_stream.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fb_common.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/impl/logging.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/frame_encoder.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

// Auto-generated flatbuffer headers.
#include "os_sensor/collation_stream_generated.h"

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace osf {

CollationStreamMeta::CollationStreamMeta() = default;

std::vector<uint8_t> CollationStreamMeta::buffer() const {
    // CollationStreamMeta is a marker entry: its mere presence flags the file as
    // containing collations. The per-slot frame ordering is reconstructed on
    // read (see reconstruct_collation_stream_ids()), so the table is empty.
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(64);
    auto cs_offset = ouster::sdk::osf::impl::gen::CreateCollationStream(fbb);
    fbb.FinishSizePrefixed(cs_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> CollationStreamMeta::from_buffer(const OsfBuffer& /*buf*/) {
    return std::make_unique<CollationStreamMeta>();
}

std::string CollationStreamMeta::repr() const {
    return "CollationStreamMeta (collation marker)";
}

namespace {

flatbuffers::Offset<impl::gen::CollationMsg> create_collation_msg(
    flatbuffers::FlatBufferBuilder& fbb, const FrameEncoder& encoder,
    const ouster::sdk::core::FrameSet& collation, const std::vector<uint64_t>& frame_indices,
    const std::vector<FrameUniqueId>& legacy_scan_ids) {
    auto fields_off = fb_save_fields(fbb, encoder, collation.fields());

    auto obj_lists_off = fb_save_object_lists(fbb, collation.objects());

    // Legacy, lidar-only list (scan_ids): one ScanID per lidar slot, addressed
    // by per-lidar index, with null slots encoded as LEGACY_INVALID_SCAN_ID.
    // This reproduces exactly what a pre-camera SDK would have written for a
    // lidar-only file, so such readers reconstruct the lidar FrameSet and
    // ignore the (unknown) cameras instead of crashing.
    std::vector<impl::gen::ScanID> legacy_ids;
    legacy_ids.reserve(legacy_scan_ids.size());
    for (const auto& id : legacy_scan_ids) {
        legacy_ids.emplace_back(id.first, id.second);
    }
    auto scan_ids_off = fbb.CreateVectorOfStructs(legacy_ids);

    // Unified list (frame_ids): one ScanID per collation slot, where sensor_id
    // is the unified sensor registration index (== slot position) and scan_idx
    // is the message index within that sensor's frame stream (or UINT64_MAX for
    // a null slot). Lidar and camera slots are represented identically; this is
    // the authoritative order preferred by camera-aware readers.
    std::vector<impl::gen::ScanID> frame_ids;
    frame_ids.reserve(frame_indices.size());
    for (size_t slot = 0; slot < frame_indices.size(); ++slot) {
        frame_ids.emplace_back(static_cast<uint32_t>(slot), frame_indices[slot]);
    }
    auto frame_ids_off = fbb.CreateVectorOfStructs(frame_ids);

    return impl::gen::CreateCollationMsg(fbb, scan_ids_off, fields_off, obj_lists_off,
                                         frame_ids_off);
}

}  // namespace

CollationStream::CollationStream(Writer& writer) : writer_{writer}, meta_{} {
    writer_.add_metadata(meta_);
}

void CollationStream::save(const ouster::sdk::osf::ts_t receive_ts,
                           const ouster::sdk::osf::ts_t /*sensor_ts*/,
                           const ouster::sdk::core::FrameSet& collation,
                           const std::vector<uint64_t>& frame_indices,
                           const std::vector<FrameUniqueId>& legacy_scan_ids) {
    auto msg_buf = make_msg(collation, frame_indices, legacy_scan_ids);
    writer_.save_message(meta_.id(), receive_ts, {}, msg_buf,
                         MetadataTraits<CollationStreamMeta>::type());
}

std::vector<uint8_t> CollationStream::make_msg(const ouster::sdk::core::FrameSet& collation,
                                               const std::vector<uint64_t>& frame_indices,
                                               const std::vector<FrameUniqueId>& legacy_scan_ids) {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    const auto& encoder = writer_.encoder().frame_encoder();
    auto msg_offset = create_collation_msg(fbb, encoder, collation, frame_indices, legacy_scan_ids);
    fbb.FinishSizePrefixed(msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<ouster::sdk::core::FrameSet> CollationStream::decode_msg(
    const MessageRef& msg, const CollationStream::meta_type& /*meta*/,
    const MetadataStore& /*meta_provider*/, const ResolveFrameFn& resolve_frame) {
    const auto& buf = msg.buffer();
    auto collation_msg = flatbuffers::GetSizePrefixedRoot<impl::gen::CollationMsg>(buf.data());

    std::vector<std::shared_ptr<ouster::sdk::core::Frame>> all_frames;

    // Index-based collation: each ScanID names a sensor and the message index
    // within that sensor's frame stream, and the list order is the collation
    // slot order, so lidar and camera slots are resolved uniformly through
    // resolve_frame.
    //
    // Prefer the unified `frame_ids` field (written by camera-aware SDKs), which
    // addresses every slot by unified sensor index and covers cameras. Fall back
    // to the legacy `scan_ids` field for files written before `frame_ids`
    // existed; there sensor_id is the per-lidar index, which for those
    // (lidar-only) files equals the unified index, so it resolves through the
    // same path. A null slot is UINT64_MAX in `frame_ids` and the
    // {sensor_id=0xFFFF, scan_idx=0} sentinel in legacy `scan_ids` (the latter
    // resolves to nullptr via resolve_frame's out-of-range guard).
    auto frame_ids = collation_msg->frame_ids();
    auto scan_ids = collation_msg->scan_ids();
    const auto* ids = (frame_ids != nullptr) ? frame_ids : scan_ids;
    if (ids == nullptr) {
        logger().error("ERROR: collation msg doesn't have frame ids.");
        return std::make_unique<ouster::sdk::core::FrameSet>();
    }
    all_frames.reserve(ids->size());
    for (size_t i = 0; i < ids->size(); ++i) {
        auto msg_id = ids->Get(i);
        if (msg_id->scan_idx() == INVALID_COLLATION_FRAME_INDEX || !resolve_frame) {
            all_frames.push_back(nullptr);
            continue;
        }
        FrameUniqueId uid{msg_id->sensor_id(), msg_id->scan_idx()};
        all_frames.push_back(resolve_frame(uid));
    }

    auto out = std::make_unique<ouster::sdk::core::FrameSet>(std::move(all_frames));
    auto add_field =
        [&out](const std::string& name, const ouster::sdk::core::FieldDescriptor& desc,
               ouster::sdk::core::FieldClass /*field_class*/) -> ouster::sdk::core::Field& {
        return out->add_field(name, desc, false);
    };

    // make sure to keep objects in scope until futures are resolved
    auto fields = collation_msg->fields();
    auto futures = fb_restore_fields(fields, {}, add_field, msg.error_handler());

    fb_restore_object_lists(collation_msg->objects(), out->objects(), msg.error_handler());

    // Finally wait for the futures and catch any exceptions, passing them to the error handler
    for (auto& future : futures) {
        try {
            future.get();
        } catch (std::runtime_error& error) {
            msg.error_handler()(Severity::OUSTER_WARNING, error.what());
        }
    }

    return out;
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
