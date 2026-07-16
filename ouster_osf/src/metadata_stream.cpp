#include "ouster/osf/metadata_stream.h"

#include "os_sensor/sensor_info_stream_generated.h"
#include "ouster/core/class_map.h"
#include "ouster/core/impl/logging.h"

using namespace std::literals::chrono_literals;

using ouster::sdk::core::ClassMapSet;

namespace ouster {
namespace sdk {
namespace osf {

class MessageRef;

std::vector<uint8_t> FrameSetSourceMetadataStream::make_msg(
    const ouster::sdk::core::FrameSetSourceMetadataSet& frame_set_source_metadata_set) {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);

    std::vector<flatbuffers::Offset<ouster::sdk::osf::v2::ScanSourceMetadataEntry>> entries;
    for (const auto& kv : frame_set_source_metadata_set.entries) {
        const auto& key = kv.first;
        const auto& value = kv.second;
        // TODO[tws] visitor
        if (value.is<std::string>()) {
            auto string = fbb.CreateString(value.as<std::string>());
            auto enum_value = ouster::sdk::osf::v2::ScanSourceMetadataValue::Generic;
            auto entry_offset = ouster::sdk::osf::v2::CreateScanSourceMetadataEntryDirect(
                fbb, key.c_str(), enum_value, string.Union());
            entries.push_back(entry_offset);
        } else if (value.is<ClassMapSet>()) {
            std::vector<flatbuffers::Offset<ouster::sdk::osf::v2::ClassMap>> class_map_offsets;

            // For each ClassMap in the ClassMapSet
            for (const auto& class_map : value.as<ClassMapSet>().class_maps) {
                std::vector<flatbuffers::Offset<ouster::sdk::osf::v2::ClassMapEntry>>
                    class_map_entry_offsets;

                // For each entry in the ClassMap
                for (const auto& kv : class_map.second.class_map) {
                    const auto& class_id = kv.first;
                    const auto& class_name = kv.second;
                    auto cme_offset = ouster::sdk::osf::v2::CreateClassMapEntryDirect(
                        fbb, class_id, class_name.c_str());
                    class_map_entry_offsets.push_back(cme_offset);
                }
                auto cm_offset = ouster::sdk::osf::v2::CreateClassMapDirect(
                    fbb, class_map.first.c_str(), &class_map_entry_offsets);
                class_map_offsets.push_back(cm_offset);
            }
            auto class_map_vector = fbb.CreateVector(class_map_offsets);
            // TODO[tws] simplify
            ouster::sdk::osf::v2::ClassMapSetBuilder class_map_set_builder(fbb);
            class_map_set_builder.add_items(class_map_vector);
            auto class_map_set_offset = class_map_set_builder.Finish();
            auto enum_value = ouster::sdk::osf::v2::ScanSourceMetadataValue::ClassMapSet;
            auto entry_offset = ouster::sdk::osf::v2::CreateScanSourceMetadataEntryDirect(
                fbb, key.c_str(), enum_value, class_map_set_offset.Union());
            entries.push_back(entry_offset);
        }
    }
    auto offset = ouster::sdk::osf::v2::CreateScanSourceMetadataSetDirect(fbb, &entries);
    fbb.FinishSizePrefixed(offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const uint32_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<FrameSetSourceMetadataSet> FrameSetSourceMetadataStream::from_buffer(
    const std::vector<uint8_t>& buf) {
    auto msg =
        flatbuffers::GetSizePrefixedRoot<ouster::sdk::osf::v2::ScanSourceMetadataSet>(buf.data());
    auto out = std::make_unique<ouster::sdk::core::FrameSetSourceMetadataSet>();

    for (size_t i = 0; i < msg->entries()->size(); ++i) {
        const auto& entry = msg->entries()->Get(i);
        std::string key = entry->key()->str();
        switch (entry->value_type()) {
            case ouster::sdk::osf::v2::ScanSourceMetadataValue::Generic: {
                const auto* string_value = entry->value_as_Generic();
                if (string_value != nullptr) {
                    out->entries.emplace(key, string_value->str());
                }
                break;
            }
            case ouster::sdk::osf::v2::ScanSourceMetadataValue::ClassMapSet: {
                const auto* cmc = entry->value_as_ClassMapSet();
                if (cmc != nullptr) {
                    ClassMapSet class_maps;
                    for (size_t j = 0; j < cmc->items()->size(); ++j) {
                        const auto& class_map = cmc->items()->Get(j);
                        core::ClassMap class_map_obj;
                        for (size_t k = 0; k < class_map->items()->size(); ++k) {
                            const auto& cme = class_map->items()->Get(k);
                            class_map_obj.class_map.emplace(cme->key(), cme->value()->str());
                        }
                        class_maps.class_maps.emplace(class_map->key()->str(), class_map_obj);
                    }
                    out->entries.emplace(key, class_maps);
                }
                break;
            }
            default:
                ouster::sdk::core::logger().warn("Skipping unknown ScanSourceMetadataEntry type");
                break;
        }
    }
    return out;
}

std::unique_ptr<FrameSetSourceMetadataStream::obj_type> FrameSetSourceMetadataStream::decode_msg(
    const MessageRef& msg, const meta_type& /*meta*/, const MetadataStore& /*meta_provider*/) {
    const auto& buf = msg.buffer();
    return from_buffer(buf);
}

void FrameSetSourceMetadataStream::save(
    const FrameSetSourceMetadataSet& frame_set_source_metadata_set, ts_t timestamp) {
    const auto& msg_buf = make_msg(frame_set_source_metadata_set);
    ts_t sensor_ts{0ns};
    writer_.save_message(meta_.id(), timestamp, sensor_ts, msg_buf,
                         MetadataTraits<FrameSetSourceMetadataStreamMeta>::type());
}

FrameSetSourceMetadataStreamMeta::FrameSetSourceMetadataStreamMeta() = default;

std::vector<uint8_t> FrameSetSourceMetadataStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    auto lss_offset = ouster::sdk::osf::v2::CreateScanSourceMetadataStream(fbb);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<MetadataEntry> FrameSetSourceMetadataStreamMeta::from_buffer(
    const ouster::sdk::osf::OsfBuffer& /*buf*/) {
    // the metadata is empty, dont bother trying to decode
    return std::make_unique<FrameSetSourceMetadataStreamMeta>();
}

std::string FrameSetSourceMetadataStreamMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "FrameSetSourceMetadataStreamMeta";
    return string_stream.str();
}

FrameSetSourceMetadataStream::FrameSetSourceMetadataStream(Writer& writer)
    : writer_{writer}, stream_meta_id_{writer_.add_metadata(meta_)} {}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
