/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file layout_streaming.h
 * @brief OSF Streaming Layout
 *
 */
#pragma once

#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

constexpr uint32_t STREAMING_DEFAULT_CHUNK_SIZE =
    2 * 1024 * 1024;  // not strict ...

/**
 * Sreaming Layout chunking strategy
 *
 * TODO[pb]: sorting TBD as in RFC0018 but first pass should be good enough
 * because usually messages from the same source and of the same type comes
 * sorted from pcap/bag sources.
 *
 * When messages laid out into chunks in an ordered as they come (not full
 * RFC0018 compliant, see TODO below), with every chunk holding messages
 * exclusively of a single stream_id. Tries not to exceede `chunk_size` (if
 * possible). However if a single message size is bigger than specified
 * `chunk_size` it's still recorded.
 */
class StreamingLayoutCW : public ChunksWriter {
   public:
    StreamingLayoutCW(Writer& writer,
                      uint32_t chunk_size = STREAMING_DEFAULT_CHUNK_SIZE);
    void saveMessage(const uint32_t stream_id, const ts_t ts,
                     const std::vector<uint8_t>& msg_buf) override;

    void finish() override;

    uint32_t chunk_size() const override { return chunk_size_; }

   private:
    void stats_message(const uint32_t stream_id, const ts_t ts,
                       const std::vector<uint8_t>& msg_buf);
    void finish_chunk(uint32_t stream_id,
                      const std::shared_ptr<ChunkBuilder>& chunk_builder);

    const uint32_t chunk_size_;
    std::map<uint32_t, std::shared_ptr<ChunkBuilder>> chunk_builders_{};
    std::vector<std::pair<uint64_t, ChunkInfo>> chunk_stream_id_{};
    std::map<uint32_t, StreamStats> stream_stats_{};
    Writer& writer_;
};

}  // namespace osf
}  // namespace ouster