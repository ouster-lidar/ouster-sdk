#pragma once

#include "osfChunk_generated.h"
#include "ouster/osf/util.h"

namespace ouster {
namespace OSF {

inline bool is_first_chunk_ok(const ouster::OSF::OsfBufferOpener& opener) {
    size_t chunk_size = ouster::OSF::readPrefixedSizeFromOffset(
        opener.osf_file, opener.chunks_offset);
    auto verifier = flatbuffers::Verifier(
        opener.osf_file + opener.chunks_offset + 4,  // + 4 => size prefixed
        chunk_size);
    return ouster::OSF::VerifyosfChunkBuffer(verifier);
}

}  // namespace OSF
}  // namespace ouster