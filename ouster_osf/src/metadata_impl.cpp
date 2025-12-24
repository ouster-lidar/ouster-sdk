#include <vector>

#include "ouster/osf/impl/fb_utils.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace impl {
// NOLINTBEGIN(misc-include-cleaner)
flatbuffers::Offset<gen::MetadataEntry> make_entry(
    const MetadataEntry& entry, flatbuffers::FlatBufferBuilder& fbb) {
    auto buf = entry.buffer();
    return gen::CreateMetadataEntryDirect(fbb, entry.id(), entry.type().c_str(),
                                          &buf);
}

std::vector<flatbuffers::Offset<gen::MetadataEntry>> make_entries(
    const MetadataStore& store, flatbuffers::FlatBufferBuilder& fbb) {
    using FbEntriesVector =
        std::vector<flatbuffers::Offset<gen::MetadataEntry>>;
    FbEntriesVector entries;
    for (const auto& entry : store.entries()) {
        auto entry_offset = make_entry(*entry.second, fbb);
        entries.push_back(entry_offset);
    }
    return entries;
}
// NOLINTEND(misc-include-cleaner)
}  // namespace impl
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
