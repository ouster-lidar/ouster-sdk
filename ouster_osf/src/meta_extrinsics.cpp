/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/meta_extrinsics.h"

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "os_sensor/extrinsics_generated.h"
#include "ouster/osf/metadata.h"
// These headers are indeed actually used.
#include "ouster/osf/buffer.h"
#include "ouster/osf/impl/basics.h"  // NOLINT(misc-include-cleaner)
#include "ouster/typedefs.h"
#include "ouster/types.h"  // NOLINT(misc-include-cleaner)

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace osf {

Extrinsics::Extrinsics(const mat4d& extrinsics, uint32_t ref_meta_id,
                       const std::string& name)
    : extrinsics_{extrinsics}, ref_meta_id_{ref_meta_id}, name_{name} {}

const mat4d& Extrinsics::extrinsics() const { return extrinsics_; }

const std::string& Extrinsics::name() const { return name_; }

uint32_t Extrinsics::ref_meta_id() const { return ref_meta_id_; }

std::vector<uint8_t> Extrinsics::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(256);
    std::vector<double> extrinsic_vec(16);

    // Changing column-major data layout to row-major before storing to OSF buf
    // Not using internal mat4d::data() reprensentation here because we
    // don't always control how it is created.
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            extrinsic_vec[(4 * i) + j] = extrinsics_(i, j);
        }
    }
    auto ext_offset = osf::impl::gen::CreateExtrinsicsDirect(
        fbb, &extrinsic_vec, ref_meta_id_,
        name_.empty() ? nullptr : name_.c_str());
    osf::impl::gen::FinishSizePrefixedExtrinsicsBuffer(fbb, ext_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const uint32_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> Extrinsics::from_buffer(const OsfBuffer buf) {
    auto ext_fb = impl::gen::GetSizePrefixedExtrinsics(buf.data());
    if (ext_fb == nullptr) {
        return nullptr;
    }
    std::string name;
    if (ext_fb->name() != nullptr) {
        name = ext_fb->name()->str();
    }
    mat4d ext_mat = mat4d::Identity();
    if ((ext_fb->extrinsics() != nullptr) &&
        ext_fb->extrinsics()->size() == 16) {
        for (uint32_t i = 0; i < ext_fb->extrinsics()->size(); ++i) {
            uint32_t row = i / 4;
            uint32_t col = i % 4;
            ext_mat(row, col) = ext_fb->extrinsics()->Get(i);
        }
    }
    return std::make_unique<osf::Extrinsics>(ext_mat, ext_fb->ref_id(), name);
}

std::string Extrinsics::repr() const {
    std::stringstream stream;
    stream << "ExtrinsicsMeta: ref_id = " << ref_meta_id_
           << ", name = " << name_ << ", extrinsics =";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            stream << " " << extrinsics_(i, j);
        }
    }
    return stream.str();
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
