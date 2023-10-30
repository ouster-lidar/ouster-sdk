/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/layout_streaming.h"

#include <vector>

#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

StreamingLayoutCW::StreamingLayoutCW(Writer& writer, uint32_t chunk_size)
    : chunk_size_{chunk_size ? chunk_size : STREAMING_DEFAULT_CHUNK_SIZE},
      writer_{writer} {}

void StreamingLayoutCW::saveMessage(const uint32_t stream_id, const ts_t ts,
                                    const std::vector<uint8_t>& msg_buf) {
    if (!chunk_builders_.count(stream_id)) {
        chunk_builders_.insert({stream_id, std::make_shared<ChunkBuilder>()});
    }

    auto chunk_builder = chunk_builders_[stream_id];

    // checking non-decreasing invariant of chunks and messages
    if (chunk_builder->end_ts() > ts) {
        std::stringstream err;
        err << "ERROR: Can't write wirh a decreasing timestamp: " << ts.count()
            << " for stream_id: " << stream_id
            << " ( previous recorded timestamp: "
            << chunk_builder->end_ts().count() << ")";
        throw std::logic_error(err.str());
    }

    if (chunk_builder->size() + msg_buf.size() > chunk_size_) {
        finish_chunk(stream_id, chunk_builder);
    }

    chunk_builder->saveMessage(stream_id, ts, msg_buf);

    // update running statistics per stream
    stats_message(stream_id, ts, msg_buf);
}

void StreamingLayoutCW::stats_message(const uint32_t stream_id, const ts_t ts,
                                      const std::vector<uint8_t>& msg_buf) {
    auto msg_size = static_cast<uint32_t>(msg_buf.size());
    auto stats_it = stream_stats_.find(stream_id);
    if (stats_it == stream_stats_.end()) {
        stream_stats_.insert({stream_id, StreamStats(stream_id, ts, msg_size)});
    } else {
        stats_it->second.update(ts, msg_size);
    }
}

void StreamingLayoutCW::finish_chunk(
    uint32_t stream_id, const std::shared_ptr<ChunkBuilder>& chunk_builder) {
    std::vector<uint8_t> bb = chunk_builder->finish();
    if (!bb.empty()) {
        uint64_t chunk_offset = writer_.emit_chunk(chunk_builder->start_ts(),
                                                   chunk_builder->end_ts(), bb);
        chunk_stream_id_.emplace_back(
            chunk_offset, ChunkInfo{chunk_offset, stream_id,
                                    chunk_builder->messages_count()});
    }

    // Prepare for the new chunk messages
    chunk_builder->reset();
}

void StreamingLayoutCW::finish() {
    for (auto& cb_it : chunk_builders_) {
        finish_chunk(cb_it.first, cb_it.second);
    }

    writer_.addMetadata(StreamingInfo{
        chunk_stream_id_, {stream_stats_.begin(), stream_stats_.end()}});
}

}  // namespace osf
}  // namespace ouster