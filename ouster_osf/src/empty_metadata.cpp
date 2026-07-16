#include "ouster/osf/empty_metadata.h"

namespace ouster {
namespace sdk {
namespace osf {

EmptyMeta::EmptyMeta() {}

std::vector<uint8_t> EmptyMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    auto lss_offset = ouster::sdk::osf::v2::CreateEmpty(fbb);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<MetadataEntry> EmptyMeta::from_buffer(const ouster::sdk::osf::OsfBuffer& /*buf*/) {
    // the metadata is empty, dont bother trying to decode
    return std::make_unique<EmptyMeta>();
}

std::string EmptyMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "EmptyMeta";
    return string_stream.str();
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
