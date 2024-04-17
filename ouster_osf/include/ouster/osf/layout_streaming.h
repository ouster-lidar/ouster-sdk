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

/** @defgroup OSFStreamingDefaultSize OSF Streaming Default Size. */

/**
 * Default Streaming Chunk Size.
 * This is used in StreamingLayoutCW
 *
 * @ingroup OSFStreamingDefaultSize
 * @relates StreamingLayoutCW
 */
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
    /**
     * @param[in] writer Writer object for use when writing messages
     * @param[in] chunk_size The chunk size to use, this arg is optional.
     */
    StreamingLayoutCW(Writer& writer,
                      uint32_t chunk_size = STREAMING_DEFAULT_CHUNK_SIZE);

    /**
     * @copydoc ChunksWriter::save_message
     *
     * @throws std::logic_error Exception on inconsistent timestamps.
     */
    void save_message(const uint32_t stream_id, const ts_t ts,
                      const std::vector<uint8_t>& buf) override;

    /**
     * @copydoc ChunksWriter::finish
     */
    void finish() override;

    /**
     * @copydoc ChunksWriter::chunk_size
     */
    uint32_t chunk_size() const override;

   private:
    /**
     * Internal method to calculate and append the stats
     * for a specific set of new messages.
     *
     * @param[in] stream_id The stream id to associate with the message.
     * @param[in] ts The timestamp for the messages.
     * @param[in] msg_buf A vector of message buffers to gather stats about.
     */
    void stats_message(const uint32_t stream_id, const ts_t ts,
                       const std::vector<uint8_t>& msg_buf);

    /**
     *  Finish out a chunk and write the chunk to the writer.
     *
     * @param[in] stream_id The stream id finish up.
     * @param[in] chunk_builder The chunk builder to use for formulating the
     *                          chunk.
     */
    void finish_chunk(uint32_t stream_id,
                      const std::shared_ptr<ChunkBuilder>& chunk_builder);

    /**
     * Chunk size to use for writing.
     */
    const uint32_t chunk_size_;

    /**
     * Per stream_id chunk builders.
     * Map Format: <stream_id, chunk builder>
     */
    std::map<uint32_t, std::shared_ptr<ChunkBuilder>> chunk_builders_{};

    /**
     * Vector pairs for chunk info/stream_id
     * Pair Format: <stream_id, chunk info>
     */
    std::vector<std::pair<uint64_t, ChunkInfo>> chunk_stream_id_{};

    /**
     * Per stream_id stats.
     * Map Format: <stream_id, stream stats>
     */
    std::map<uint32_t, StreamStats> stream_stats_{};

    /**
     * Internal writer object to use for writing.
     */
    Writer& writer_;
};

}  // namespace osf
}  // namespace ouster
