/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/meta_extrinsics.h"

#include "flatbuffers/flatbuffers.h"
#include "os_sensor/extrinsics_generated.h"

namespace ouster {
namespace osf {

std::vector<uint8_t> Extrinsics::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(256);
    std::vector<double> extrinsic_vec(16);

    // Changing column-major data layout to row-major before storing to OSF buf
    // Not using internal mat4d::data() reprensentation here because we
    // don't always control how it is created.
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            extrinsic_vec[4 * i + j] = extrinsics_(i, j);
        }
    }
    auto ext_offset = osf::gen::CreateExtrinsicsDirect(
        fbb, &extrinsic_vec, ref_meta_id_,
        name_.empty() ? nullptr : name_.c_str());
    osf::gen::FinishSizePrefixedExtrinsicsBuffer(fbb, ext_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const uint32_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> Extrinsics::from_buffer(
    const std::vector<uint8_t>& buf) {
    auto ext_fb = gen::GetSizePrefixedExtrinsics(buf.data());
    if (!ext_fb) return nullptr;
    std::string name;
    if (ext_fb->name()) {
        name = ext_fb->name()->str();
    }
    mat4d ext_mat = mat4d::Identity();
    if (ext_fb->extrinsics() && ext_fb->extrinsics()->size() == 16) {
        for (uint32_t i = 0; i < ext_fb->extrinsics()->size(); ++i) {
            uint32_t r = i / 4;
            uint32_t c = i % 4;
            ext_mat(r, c) = ext_fb->extrinsics()->Get(i);
        }
    }
    return std::make_unique<osf::Extrinsics>(ext_mat, ext_fb->ref_id(), name);
}

std::string Extrinsics::repr() const {
    std::stringstream ss;
    ss << "ExtrinsicsMeta: ref_id = " << ref_meta_id_ << ", name = " << name_
       << ", extrinsics =";
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            ss << " " << extrinsics_(i, j);
        }
    }
    return ss.str();
};

}  // namespace osf
}  // namespace ouster
