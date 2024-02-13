/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file meta_streaming_info.h
 * @brief Metadata entry StreamingInfo
 *
 */
#pragma once

#include <iostream>
#include <memory>

#include "ouster/osf/metadata.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

struct ChunkInfo {
    uint64_t offset;
    uint32_t stream_id;
    uint32_t message_count;
};

struct StreamStats {
    uint32_t stream_id;
    ts_t start_ts;
    ts_t end_ts;
    uint64_t message_count;
    uint32_t message_avg_size;
    StreamStats() = default;
    StreamStats(uint32_t s_id, ts_t t, uint32_t msg_size);
    void update(ts_t t, uint32_t msg_size);
};

std::string to_string(const ChunkInfo& chunk_info);
std::string to_string(const StreamStats& stream_stats);

/**
 * Metadata entry to store StreamingInfo, to support StreamingLayout (RFC 0018)
 *
 * @verbatim
 * Fields:
 *   chunks_info: chunk -> stream_id map
 *   stream_stats: stream statistics of messages in file
 *
 * OSF type:
 *   ouster/v1/streaming/StreamingInfo
 *
 * Flatbuffer definition file:
 *   fb/streaming/streaming_info.fbs
 * @endverbatim
 *
 */
class StreamingInfo : public MetadataEntryHelper<StreamingInfo> {
   public:
    StreamingInfo() {}

    StreamingInfo(
        const std::vector<std::pair<uint64_t, ChunkInfo>>& chunks_info,
        const std::vector<std::pair<uint32_t, StreamStats>>& stream_stats);

    std::map<uint64_t, ChunkInfo>& chunks_info();
    std::map<uint32_t, StreamStats>& stream_stats();

    std::vector<uint8_t> buffer() const override final;
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);
    std::string repr() const override;

   private:
    std::map<uint64_t, ChunkInfo> chunks_info_{};
    std::map<uint32_t, StreamStats> stream_stats_{};
};

template <>
struct MetadataTraits<StreamingInfo> {
    static const std::string type() {
        return "ouster/v1/streaming/StreamingInfo";
    }
};

}  // namespace osf
}  // namespace ouster
