/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/collation_stream.h"

#include <algorithm>
#include <iterator>

#include "fb_common.h"
#include "ouster/core/impl/logging.h"
#include "ouster/osf/buffer.h"

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace osf {

CollationStreamMeta::CollationStreamMeta() = default;

std::vector<uint8_t> CollationStreamMeta::buffer() const {
    // not sure if the following are necessary
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);
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
    return "CollationStreamMeta";
}

flatbuffers::Offset<impl::gen::CollationMsg> create_collation_msg(
    flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
    const ouster::sdk::core::FrameSet& collation, const std::vector<FrameUniqueId>& frame_uids) {
    std::vector<impl::gen::ScanID> fbb_frame_uids;
    fbb_frame_uids.reserve(frame_uids.size());
    for (const auto& frame_uid : frame_uids) {
        fbb_frame_uids.emplace_back(frame_uid.first, frame_uid.second);
    }
    auto ids_off = osf::create_vector_of_structs<impl::gen::ScanID>(fbb, fbb_frame_uids.data(),
                                                                    fbb_frame_uids.size());

    auto fields_off = fb_save_fields(fbb, encoder, collation.fields());

    auto obj_lists_off = fb_save_object_lists(fbb, collation.objects());

    return impl::gen::CreateCollationMsg(fbb, ids_off, fields_off, obj_lists_off);
}

CollationStream::CollationStream(Writer& writer) : writer_{writer}, meta_{} {
    writer_.add_metadata(meta_);
}

void CollationStream::save(const ouster::sdk::osf::ts_t receive_ts,
                           const ouster::sdk::osf::ts_t /*sensor_ts*/,
                           const ouster::sdk::core::FrameSet& collation,
                           const std::vector<FrameUniqueId>& frame_uids) {
    auto msg_buf = make_msg(collation, frame_uids);
    writer_.save_message(meta_.id(), receive_ts, {}, msg_buf,
                         MetadataTraits<CollationStreamMeta>::type());
}

std::vector<uint8_t> CollationStream::make_msg(const ouster::sdk::core::FrameSet& collation,
                                               const std::vector<FrameUniqueId>& frame_uids) {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    const auto& encoder = writer_.encoder().lidar_frame_encoder();
    auto msg_offset = create_collation_msg(fbb, encoder, collation, frame_uids);
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

    auto msg_frame_uids = collation_msg->scan_ids();
    if (msg_frame_uids == nullptr) {
        logger().error("ERROR: collation msg doesn't have frame ids.");
        return std::make_unique<ouster::sdk::core::FrameSet>();
    }
    std::vector<FrameUniqueId> frame_uids;
    for (size_t i = 0; i < msg_frame_uids->size(); ++i) {
        auto msg = msg_frame_uids->Get(i);
        frame_uids.emplace_back(msg->sensor_id(), msg->scan_idx());
    }

    std::vector<std::shared_ptr<ouster::sdk::core::LidarFrame>> frames;
    std::transform(frame_uids.begin(), frame_uids.end(), std::back_inserter(frames), resolve_frame);

    auto out = std::make_unique<ouster::sdk::core::FrameSet>(frames);
    auto add_field =
        [&out](const std::string& name, const ouster::sdk::core::FieldDescriptor& desc,
               ouster::sdk::core::FieldClass /*field_class*/) -> ouster::sdk::core::Field& {
        return out->add_field(name, desc);
    };

    fb_restore_fields(collation_msg->fields(), {}, add_field, msg.error_handler());

    fb_restore_object_lists(collation_msg->objects(), out->objects(), msg.error_handler());

    return out;
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
