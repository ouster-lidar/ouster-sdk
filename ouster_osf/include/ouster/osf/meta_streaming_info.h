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
    StreamStats(uint32_t s_id, ts_t t, uint32_t msg_size)
        : stream_id{s_id},
          start_ts{t},
          end_ts{t},
          message_count{1},
          message_avg_size{msg_size} {};
    void update(ts_t t, uint32_t msg_size) {
        if (start_ts > t) start_ts = t;
        if (end_ts < t) end_ts = t;
        ++message_count;
        int avg_size = static_cast<int>(message_avg_size);
        avg_size = avg_size + (static_cast<int>(msg_size) - avg_size) /
                                  static_cast<int>(message_count);
        message_avg_size = static_cast<uint32_t>(avg_size);
    }
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
        const std::vector<std::pair<uint32_t, StreamStats>>& stream_stats)
        : chunks_info_{chunks_info.begin(), chunks_info.end()},
          stream_stats_{stream_stats.begin(), stream_stats.end()} {}

    std::map<uint64_t, ChunkInfo>& chunks_info() { return chunks_info_; }
    std::map<uint32_t, StreamStats>& stream_stats() { return stream_stats_; }

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