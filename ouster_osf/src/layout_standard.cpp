/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/layout_standard.h"

#include <vector>

#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

StandardLayoutCW::StandardLayoutCW(Writer& writer, uint32_t chunk_size)
    : chunk_size_{chunk_size ? chunk_size : STANDARD_DEFAULT_CHUNK_SIZE},
      writer_{writer} {}

void StandardLayoutCW::saveMessage(const uint32_t stream_id, const ts_t ts,
                                   const std::vector<uint8_t>& msg_buf) {
    if (chunk_builder_.size() + msg_buf.size() > chunk_size_) {
        finish_chunk();
    }

    chunk_builder_.saveMessage(stream_id, ts, msg_buf);
}

void StandardLayoutCW::finish_chunk() {
    std::vector<uint8_t> bb = chunk_builder_.finish();
    if (!bb.empty()) {
        writer_.emit_chunk(chunk_builder_.start_ts(), chunk_builder_.end_ts(),
                           bb);
    }

    // Prepare for the new chunk messages
    chunk_builder_.reset();
}

void StandardLayoutCW::finish() { finish_chunk(); }

}  // namespace osf
}  // namespace ouster