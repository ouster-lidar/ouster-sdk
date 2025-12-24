/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/layout_streaming.h"

#include <cstdint>
#include <memory>
#include <sstream>
#include <vector>

#include "ouster/osf/basics.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {

StreamingLayoutCW::StreamingLayoutCW(Writer& writer, uint32_t chunk_size)
    : chunk_size_{(chunk_size != 0u) ? chunk_size
                                     : STREAMING_DEFAULT_CHUNK_SIZE},
      writer_{writer} {}

void StreamingLayoutCW::save_message(const uint32_t stream_id,
                                     const ts_t receive_ts,
                                     const ts_t sensor_ts,
                                     const std::vector<uint8_t>& msg_buf,
                                     const std::string& type) {
    if (chunk_builders_.count(stream_id) == 0u) {
        chunk_builders_.insert({stream_id, std::make_shared<ChunkBuilder>()});
    }

    auto chunk_builder = chunk_builders_[stream_id];

    // checking non-decreasing invariant of chunks and messages
    if (chunk_builder->end_ts() > receive_ts) {
        std::stringstream err;
        err << "ERROR: Can't write with a decreasing timestamp: "
            << receive_ts.count() << " for stream_id: " << stream_id
            << " ( previous recorded timestamp: "
            << chunk_builder->end_ts().count() << ")";
        throw std::logic_error(err.str());
    }

    if (chunk_builder->size() + msg_buf.size() > chunk_size_) {
        finish_chunk(stream_id, chunk_builder);
    }

    chunk_builder->save_message(stream_id, receive_ts, sensor_ts, msg_buf,
                                type);

    // update running statistics per stream
    stats_message(stream_id, receive_ts, sensor_ts, msg_buf);
}

void StreamingLayoutCW::finish() {
    for (auto& cb_it : chunk_builders_) {
        finish_chunk(cb_it.first, cb_it.second);
    }

    writer_.add_metadata(StreamingInfo{
        chunk_stream_id_, {stream_stats_.begin(), stream_stats_.end()}});
}

void StreamingLayoutCW::flush(uint32_t stream_id) {
    finish_chunk(stream_id, chunk_builders_[stream_id]);
}

uint32_t StreamingLayoutCW::chunk_size() const { return chunk_size_; }

const StreamStats& StreamingLayoutCW::get_stats(uint32_t stream_id) const {
    auto stats_it = stream_stats_.find(stream_id);
    if (stats_it == stream_stats_.end()) {
        throw std::invalid_argument(
            "StreamingLayoutCW::get_stats error: could not find stream_id");
    }

    return stats_it->second;
}

void StreamingLayoutCW::stats_message(const uint32_t stream_id,
                                      const ts_t receive_ts,
                                      const ts_t sensor_ts,
                                      const std::vector<uint8_t>& msg_buf) {
    auto msg_size = static_cast<uint32_t>(msg_buf.size());
    auto stats_it = stream_stats_.find(stream_id);
    if (stats_it == stream_stats_.end()) {
        stream_stats_.insert({stream_id, StreamStats(stream_id, receive_ts,
                                                     sensor_ts, msg_size)});
    } else {
        stats_it->second.update(receive_ts, sensor_ts, msg_size);
    }
}

void StreamingLayoutCW::finish_chunk(
    uint32_t stream_id, const std::shared_ptr<ChunkBuilder>& chunk_builder) {
    std::vector<uint8_t> builder = chunk_builder->finish();
    if (!builder.empty()) {
        uint64_t chunk_offset = writer_.emit_chunk(
            chunk_builder->start_ts(), chunk_builder->end_ts(), builder);
        chunk_stream_id_.emplace_back(
            chunk_offset, ChunkInfo{chunk_offset, stream_id,
                                    chunk_builder->messages_count()});
    }

    // Prepare for the new chunk messages
    chunk_builder->reset();
}
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
