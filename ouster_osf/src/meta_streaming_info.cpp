/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/meta_streaming_info.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <jsoncons/json.hpp>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ouster/osf/basics.h"
// We need this for the v2 namespace.
#include "ouster/osf/buffer.h"
#include "ouster/osf/impl/basics.h"  // NOLINT(misc-include-cleaner)
#include "ouster/osf/metadata.h"
#include "streaming/streaming_info_generated.h"

namespace ouster {
namespace sdk {
namespace osf {

namespace {
flatbuffers::Offset<impl::gen::StreamingInfo> create_streaming_info(
    flatbuffers::FlatBufferBuilder& fbb,
    const std::map<uint64_t, ChunkInfo>& chunks_info,
    const std::map<uint32_t, StreamStats>& stream_stats) {
    // Pack chunks vector
    std::vector<flatbuffers::Offset<impl::gen::ChunkInfo>> chunks_info_vec;
    for (const auto& chunk_info_pair : chunks_info) {
        const auto& chunk = chunk_info_pair.second;
        auto ci_offset = impl::gen::CreateChunkInfo(
            fbb, chunk.offset, chunk.stream_id, chunk.message_count);
        chunks_info_vec.push_back(ci_offset);
    }

    // Pack stream_stats vector
    std::vector<flatbuffers::Offset<impl::gen::StreamStats>> stream_stats_vec;
    for (const auto& stream_stat_pair : stream_stats) {
        auto stat = stream_stat_pair.second;
        auto ss_offset = impl::gen::CreateStreamStats(
            fbb, stat.stream_id, stat.start_ts.count(), stat.end_ts.count(),
            stat.message_count, stat.message_avg_size,
            fbb.CreateVector<uint64_t>(stat.receive_timestamps),
            fbb.CreateVector<uint64_t>(stat.sensor_timestamps));
        stream_stats_vec.push_back(ss_offset);
    }

    auto si_offset = impl::gen::CreateStreamingInfoDirect(fbb, &chunks_info_vec,
                                                          &stream_stats_vec);
    return si_offset;
}

};  // namespace

std::string to_string(const ChunkInfo& chunk_info) {
    std::stringstream string_stream;
    string_stream << "{offset = " << chunk_info.offset
                  << ", stream_id = " << chunk_info.stream_id
                  << ", message_count = " << chunk_info.message_count << "}";
    return string_stream.str();
}

StreamStats::StreamStats(uint32_t s_id, ts_t receive_ts, ts_t sensor_ts,
                         uint32_t msg_size)
    : stream_id{s_id},
      start_ts{receive_ts},
      end_ts{receive_ts},
      message_count{1},
      message_avg_size{msg_size} {
    receive_timestamps.push_back(receive_ts.count());
    sensor_timestamps.push_back(sensor_ts.count());
}

void StreamStats::update(ts_t receive_ts, ts_t sensor_ts, uint32_t msg_size) {
    if (start_ts > receive_ts) {
        start_ts = receive_ts;
    }
    if (end_ts < receive_ts) {
        end_ts = receive_ts;
    }
    ++message_count;
    int avg_size = static_cast<int>(message_avg_size);
    avg_size = avg_size + (static_cast<int>(msg_size) - avg_size) /
                              static_cast<int>(message_count);
    message_avg_size = static_cast<uint32_t>(avg_size);
    receive_timestamps.push_back(receive_ts.count());
    sensor_timestamps.push_back(sensor_ts.count());
}

std::string to_string(const StreamStats& stream_stats) {
    std::stringstream string_stream;
    string_stream << "{stream_id = " << stream_stats.stream_id
                  << ", start_ts = " << stream_stats.start_ts.count()
                  << ", end_ts = " << stream_stats.end_ts.count()
                  << ", message_count = " << stream_stats.message_count
                  << ", message_avg_size = " << stream_stats.message_avg_size
                  << ", host_timestamps = [";
    for (const auto& timestamp : stream_stats.receive_timestamps) {
        string_stream << timestamp << ", ";
    }
    string_stream << "], sensor_timestamps = [";
    for (const auto& timestamp : stream_stats.sensor_timestamps) {
        string_stream << timestamp << ", ";
    }
    string_stream << "]}";
    return string_stream.str();
}

StreamingInfo::StreamingInfo(
    const std::vector<std::pair<uint64_t, ChunkInfo>>& chunks_info,
    const std::vector<std::pair<uint32_t, StreamStats>>& stream_stats)
    : chunks_info_{chunks_info.begin(), chunks_info.end()},
      stream_stats_{stream_stats.begin(), stream_stats.end()} {}

StreamingInfo::StreamingInfo(
    const std::map<uint64_t, ChunkInfo>& chunks_info,
    const std::map<uint32_t, StreamStats>& stream_stats)
    : chunks_info_(chunks_info), stream_stats_(stream_stats) {}

std::map<uint64_t, ChunkInfo>& StreamingInfo::chunks_info() {
    return chunks_info_;
}

std::map<uint32_t, StreamStats>& StreamingInfo::stream_stats() {
    return stream_stats_;
}

std::vector<uint8_t> StreamingInfo::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto si_offset = create_streaming_info(fbb, chunks_info_, stream_stats_);
    fbb.FinishSizePrefixed(si_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> StreamingInfo::from_buffer(const OsfBuffer buf) {
    auto streaming_info = impl::gen::GetSizePrefixedStreamingInfo(buf.data());

    auto new_streaming_info = std::make_unique<StreamingInfo>();

    auto& chunks_info = new_streaming_info->chunks_info();
    if ((streaming_info->chunks() != nullptr) &&
        (streaming_info->chunks()->size() != 0u)) {
        std::transform(streaming_info->chunks()->begin(),
                       streaming_info->chunks()->end(),
                       std::inserter(chunks_info, chunks_info.end()),
                       [](const impl::gen::ChunkInfo* chunk_info_ptr) {
                           return std::make_pair(
                               chunk_info_ptr->offset(),
                               ChunkInfo{chunk_info_ptr->offset(),
                                         chunk_info_ptr->stream_id(),
                                         chunk_info_ptr->message_count()});
                       });
    }

    auto& stream_stats = new_streaming_info->stream_stats();
    if ((streaming_info->stream_stats() != nullptr) &&
        (streaming_info->stream_stats()->size() != 0u)) {
        std::transform(
            streaming_info->stream_stats()->begin(),
            streaming_info->stream_stats()->end(),
            std::inserter(stream_stats, stream_stats.end()),
            [](const impl::gen::StreamStats* stat) {
                StreamStats stream_stats_obj{};
                stream_stats_obj.stream_id = stat->stream_id();
                stream_stats_obj.start_ts = ts_t{stat->start_ts()};
                stream_stats_obj.end_ts = ts_t{stat->end_ts()};
                stream_stats_obj.message_count = stat->message_count();
                stream_stats_obj.message_avg_size = stat->message_avg_size();
                if (stat->receive_timestamps()) {
                    for (auto value : *stat->receive_timestamps()) {
                        stream_stats_obj.receive_timestamps.push_back(value);
                    }
                }
                if (stat->sensor_timestamps()) {
                    for (auto value : *stat->sensor_timestamps()) {
                        stream_stats_obj.sensor_timestamps.push_back(value);
                    }
                }
                return std::make_pair(stat->stream_id(), stream_stats_obj);
            });
    }

    return new_streaming_info;
}

std::string StreamingInfo::repr() const {
    jsoncons::json streaming_info_obj;

    jsoncons::json chunks(jsoncons::json_array_arg);
    for (const auto& chunk_info_pair : chunks_info_) {
        jsoncons::json chunk_info;
        chunk_info["offset"] =
            static_cast<uint64_t>(chunk_info_pair.second.offset);
        chunk_info["stream_id"] = chunk_info_pair.second.stream_id;
        chunk_info["message_count"] = chunk_info_pair.second.message_count;
        chunks.emplace_back(chunk_info);
    }
    streaming_info_obj["chunks"] = chunks;

    jsoncons::json stream_stats(jsoncons::json_array_arg);
    for (const auto& stat : stream_stats_) {
        jsoncons::json stream_stat_json{};
        stream_stat_json["stream_id"] = stat.first;
        stream_stat_json["start_ts"] =
            static_cast<uint64_t>(stat.second.start_ts.count());
        stream_stat_json["end_ts"] =
            static_cast<uint64_t>(stat.second.end_ts.count());
        stream_stat_json["message_count"] =
            static_cast<uint64_t>(stat.second.message_count);
        stream_stat_json["message_avg_size"] = stat.second.message_avg_size;
        jsoncons::json sensor_timestamps_json(jsoncons::json_array_arg);
        jsoncons::json receive_timestamps_json(jsoncons::json_array_arg);
        for (const auto& timestamp : stat.second.sensor_timestamps) {
            sensor_timestamps_json.emplace_back(
                static_cast<uint64_t>(timestamp));
        }
        for (const auto& timestamp : stat.second.receive_timestamps) {
            receive_timestamps_json.emplace_back(
                static_cast<uint64_t>(timestamp));
        }
        stream_stat_json["sensor_timestamps"] = sensor_timestamps_json;
        stream_stat_json["receive_timestamps"] = receive_timestamps_json;
        stream_stats.emplace_back(stream_stat_json);
    }
    streaming_info_obj["stream_stats"] = stream_stats;

    std::string out;
    streaming_info_obj.dump(out);
    return out;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
