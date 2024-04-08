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

/**
 * Class for keeping track of OSF chunks.
 *
 * Flat Buffer Reference:
 *   fb/streaming/streaming_info.fbs :: ChunkInfo
 */
struct ChunkInfo {
    /**
     * The offset in the flatbuffer where
     * the chunk is located.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: offset
     */
    uint64_t offset;

    /**
     * The specific stream the chunk is associated with.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: stream_id
     */
    uint32_t stream_id;

    /**
     * The number of messages in the chunk
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: message_count
     */
    uint32_t message_count;
};

/**
 * Class for keeping track of OSF stream stats.
 *
 * Flat Buffer Reference:
 *   fb/streaming/streaming_info.fbs :: StreamStats
 */
struct StreamStats {
    /**
     * The specific stream the chunk is associated with.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: stream_id
     */
    uint32_t stream_id;

    /**
     * The first timestamp in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: start_ts
     */
    ts_t start_ts;

    /**
     * The last timestamp in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: end_ts
     */
    ts_t end_ts;

    /**
     * The number of messages in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: message_count
     */
    uint64_t message_count;

    /**
     * The average size of the messages in the stream.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamStats :: message_avg_size
     */
    uint32_t message_avg_size;

    /**
     * Default constructor, sets everthing to 0.
     */
    StreamStats() = default;

    /**
     * Construct a StreamStats with the specified values
     *
     * @param[in] s_id Specify the stream_id to use.
     * @param[in] t Set the start and end timestamps to the specified value.
     * @param[in] msg_size Set the average message size to the specified value.
     */
    StreamStats(uint32_t s_id, ts_t t, uint32_t msg_size);

    /**
     * Update values within the StreamStats
     *
     * @param[in] t Add another timestamp and calculate the start and end
     *              values.
     * @param[in] msg_size Add another message size and calculate the average.
     */
    void update(ts_t t, uint32_t msg_size);
};

/**
 * Get the string representation for a ChunkInfo object.
 *
 * @return The string representation for a ChunkInfo object.
 */
std::string to_string(const ChunkInfo& chunk_info);

/**
 * Get the string representation for a StreamStats object.
 *
 * @return The string representation for a StreamStats object.
 */
std::string to_string(const StreamStats& stream_stats);

/**
 * Metadata entry to store StreamingInfo, to support StreamingLayout (RFC 0018)
 *
 * OSF type:
 *   ouster/v1/streaming/StreamingInfo
 *
 * Flat Buffer Reference:
 *   fb/streaming/streaming_info.fbs :: StreamingInfo
 */
class StreamingInfo : public MetadataEntryHelper<StreamingInfo> {
   public:
    StreamingInfo() {}

    /**
     * @param[in] chunks_info Vector containing pairs of
     *                        stream_id/ChunkInfo
     *                        to be used to generate a stream_id/ChunkInfo
     *                        map.
     * @param[in] stream_stats Vector containing pairs of
     *                         stream_id/StreamStats
     *                         to be used to generate a
     *                         stream_id/StreamStats map.
     */
    StreamingInfo(
        const std::vector<std::pair<uint64_t, ChunkInfo>>& chunks_info,
        const std::vector<std::pair<uint32_t, StreamStats>>& stream_stats);

    /**
     * @param[in] chunks_info ///< Map containing stream_id/ChunkInfo data.
     * @param[in] stream_stats ///< Map containing stream_id/StreamStats data.
     */
    StreamingInfo(const std::map<uint64_t, ChunkInfo>& chunks_info,
                  const std::map<uint32_t, StreamStats>& stream_stats);

    /**
     * Return the chunk_info map. stream_id/ChunkInfo data.
     *
     * @return The chunk_info map. stream_id/ChunkInfo data.
     */
    std::map<uint64_t, ChunkInfo>& chunks_info();

    /**
     * Return the stream stat map. stream_id/StreamStats data.
     *
     * @return The stream stat map. stream_id/StreamStats data.
     */
    std::map<uint32_t, StreamStats>& stream_stats();

    /**
     * @copydoc MetadataEntry::buffer
     */
    std::vector<uint8_t> buffer() const override final;

    /**
     * Create a StreamingInfo object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new StreamingInfo cast as a MetadataEntry
     */
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    /**
     * Get the string representation for the LidarSensor object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the LidarSensor object.
     */
    std::string repr() const override;

   private:
    /**
     * The internal stream_id to ChunkInfo map.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamingInfo :: chunks
     */
    std::map<uint64_t, ChunkInfo> chunks_info_{};

    /**
     * The internal stream_id to StreamStats map.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: StreamingInfo :: stream_stats
     */
    std::map<uint32_t, StreamStats> stream_stats_{};
};

/** @defgroup OSFTraitsStreamingInfo Templated struct for traits. */
/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsStreamingInfo
 */
template <>
struct MetadataTraits<StreamingInfo> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/streaming/StreamingInfo".
     */
    static const std::string type() {
        return "ouster/v1/streaming/StreamingInfo";
    }
};

}  // namespace osf
}  // namespace ouster
