/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/collation_stream.h"

#include <algorithm>
#include <iterator>

#include "fb_common.h"
#include "ouster/impl/logging.h"
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

std::unique_ptr<MetadataEntry> CollationStreamMeta::from_buffer(
    const OsfBuffer) {
    return std::make_unique<CollationStreamMeta>();
}

std::string CollationStreamMeta::repr() const { return "CollationStreamMeta"; }

flatbuffers::Offset<impl::gen::CollationMsg> create_collation_msg(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScanEncoder& encoder,
    const ouster::sdk::core::LidarScanSet& collation,
    const std::vector<ScanId>& scan_ids) {
    std::vector<impl::gen::ScanID> fbb_scan_ids;
    fbb_scan_ids.reserve(scan_ids.size());
    for (const auto& scan_id : scan_ids) {
        fbb_scan_ids.emplace_back(scan_id.first, scan_id.second);
    }
    auto ids_off = osf::create_vector_of_structs<impl::gen::ScanID>(
        fbb, fbb_scan_ids.data(), fbb_scan_ids.size());

    auto fields_off = fb_save_fields(fbb, encoder, collation.fields());

    return impl::gen::CreateCollationMsg(fbb, ids_off, fields_off);
}

CollationStream::CollationStream(Token /*key*/, Writer& writer)
    : writer_{writer}, meta_{} {
    writer_.add_metadata(meta_);
}

void CollationStream::save(const ouster::sdk::osf::ts_t receive_ts,
                           const ouster::sdk::osf::ts_t /*sensor_ts*/,
                           const ouster::sdk::core::LidarScanSet& collation,
                           const std::vector<ScanId>& scan_ids) {
    auto msg_buf = make_msg(collation, scan_ids);
    writer_.save_message(meta_.id(), receive_ts, {}, msg_buf,
                         MetadataTraits<CollationStreamMeta>::type());
}

std::vector<uint8_t> CollationStream::make_msg(
    const ouster::sdk::core::LidarScanSet& collation,
    const std::vector<ScanId>& scan_ids) {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    const auto& encoder = writer_.encoder().lidar_scan_encoder();
    auto msg_offset = create_collation_msg(fbb, encoder, collation, scan_ids);
    fbb.FinishSizePrefixed(msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<ouster::sdk::core::LidarScanSet> CollationStream::decode_msg(
    const MessageRef& msg, const CollationStream::meta_type& /*meta*/,
    const MetadataStore& /*meta_provider*/, const ResolveScanFn& resolve_scan) {
    const auto& buf = msg.buffer();
    auto collation_msg =
        flatbuffers::GetSizePrefixedRoot<impl::gen::CollationMsg>(buf.data());

    auto msg_scan_ids = collation_msg->scan_ids();
    if (!msg_scan_ids) {
        logger().error("ERROR: collation msg doesn't have scan ids.");
        return nullptr;
    }
    std::vector<ScanId> scan_ids;
    for (size_t i = 0; i < msg_scan_ids->size(); ++i) {
        auto msg = msg_scan_ids->Get(i);
        scan_ids.emplace_back(msg->sensor_id(), msg->scan_idx());
    }

    std::vector<std::shared_ptr<ouster::sdk::core::LidarScan>> scans;
    std::transform(scan_ids.begin(), scan_ids.end(), std::back_inserter(scans),
                   resolve_scan);

    auto out = std::make_unique<ouster::sdk::core::LidarScanSet>(scans);
    auto add_field = [&out](const std::string& name,
                            const ouster::sdk::core::FieldDescriptor& desc,
                            ouster::sdk::core::FieldClass /*field_class*/)
        -> ouster::sdk::core::Field& { return out->add_field(name, desc); };

    fb_restore_fields(collation_msg->fields(), {}, add_field,
                      msg.error_handler());

    return out;
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
