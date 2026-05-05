/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file chunk.h
 * @brief data structures related to the OSF file format
 *
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/osf/metadata.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Enumerator for dealing with chunk validity.
 *
 * This is synthesized and thus does not have a reference in the Flat Buffer.
 * Value is set in Reader::verify_chunk
 */
enum class ChunkValidity {
    UNKNOWN = 0,  ///< Validity can not be ascertained.
    VALID,        ///< Chunk is valid.
    INVALID       ///< Chunk is invalid.
};

/**
 * The structure for representing chunk information and
 * for forward iteration.
 *
 * This struct is partially mapped to the Flat Buffer data.
 * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset
 */
struct OUSTER_API_CLASS ChunkState {
    /**
     * The current chunk's offset from the begining of the chunks section.
     *
     * @param[in] offset the chunk position represented as an offset in bytes
     * relative to the end of the file header.
     * @param[in] start the earliest timestamp in a message within the chunk.
     * @param[in] end the latest timestamp in a message within the chunk.
     * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset :: offset
     */
    OUSTER_API_FUNCTION
    ChunkState(uint64_t offset, ts_t start, ts_t end);

    uint64_t offset;  ///< The file offset of the chunk (in number of bytes
                      // relative to the header.)

    uint64_t size;  ///< The size of this chunk in bytes.

    /**
     * The next chunk's offset for forward iteration.
     * Should work like a linked list.
     *
     * This is partially synthesized from the Flat Buffers.
     * This will link up with the next chunks offset.
     * Value is set in ChunksPile::link_stream_chunks
     * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset :: offset
     */
    uint64_t next_offset{std::numeric_limits<uint64_t>::max()};

    /**
     * The first timestamp in the chunk in ordinality.
     *
     * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset :: start_ts
     */
    ts_t start_ts;

    /**
     * The last timestamp in the chunk in ordinality.
     *
     * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset :: end_ts
     */
    ts_t end_ts;

    /**
     * The validity of the current chunk
     *
     * This is synthesized and thus does not have a reference in the Flat
     * Buffers. Value is set in Reader::verify_chunk
     */
    ChunkValidity status{ChunkValidity::UNKNOWN};
};

/**
 * The structure for representing streaming information.
 *
 * This struct is partially mapped to the Flat Buffer data.
 */
struct OUSTER_API_CLASS ChunkInfoNode {
    /**
     * The chunk offset from the begining of the chunks section.
     *
     * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset :: offset
     */
    uint64_t offset;

    /**
     * The next chunk's offset for forward iteration.
     * Should work like a linked list.
     *
     * This is partially synthesized from the Flat Buffers.
     * This will link up with the next chunks offset.
     * Value is set in ChunksPile::link_stream_chunks
     * Flat Buffer Reference: fb/metadata.fbs :: ChunkOffset :: offset
     */
    uint64_t next_offset;

    /**
     * The stream this is associated with.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: stream_id
     */
    uint32_t stream_id;

    /**
     * Total number of messages in a `stream_id` in the whole OSF file
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs :: ChunkInfo :: message_count
     */
    uint32_t message_count;

    /**
     * The index of the start of the message.
     * @todo try to describe this better
     *
     * This is partially synthesized from the Flat Buffers.
     * Value is set in ChunksPile::link_stream_chunks
     * Synthesized from Flat Buffer Reference:
     *   fb/metadata.fbs :: ChunkOffset :: message_count
     */
    uint32_t message_start_idx;
};

/**
 * Chunks state map. Validity info and next offset.
 */
class OUSTER_API_CLASS ChunksPile {
   public:
    /**
     * stream_id to offset map.
     */
    using StreamChunksMap =
        std::unordered_map<uint32_t, std::shared_ptr<std::vector<uint64_t>>>;

    /**
     * Default blank constructor.
     */
    OUSTER_API_FUNCTION
    ChunksPile() = default;  // TODO[tws] delete

    /**
     * Construct a ChunksPile from a vector of ChunkState and the offset of the
     * end of the last chunk (in bytes relative to the header.)
     * @param[in] chunks a vector of chunks
     * @param[in] end_of_chunks_offset the offset of the end of the last chunk
     * in bytes relative to the header.
     */
    OUSTER_API_FUNCTION
    ChunksPile(
        const std::vector<ChunkState>& chunks,
        uint64_t end_of_chunks_offset);  // TODO[tws] replace this constructor
    // with one that initializes the entire ChunksPile (including with stream
    // info.)

    /**
     * Return the chunk associated with an offset.
     *
     * @param[in] offset The offset to return the chunk for.
     * @return The chunk if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkState* get(uint64_t offset);

    /**
     * Add a new streaming info to the ChunkPile.
     *
     * @param[in] offset The offset for the chunk.
     * @param[in] stream_id The stream_id associated.
     * @param[in] message_count The number of messages.
     */
    OUSTER_API_FUNCTION
    void add_info(uint64_t offset, uint32_t stream_id, uint32_t message_count);

    /**
     * Return the streaming info associated with an offset.
     *
     * @param[in] offset The offset to return the streaming info for.
     * @return The streaming info if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkInfoNode* get_info(uint64_t offset);

    /**
     * Return the streaming info associated with a message_idx.
     *
     * @param[in] stream_id The stream to look for infos in.
     * @param[in] message_idx The specific message index to look for.
     * @return The streaming info if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkInfoNode* get_info_by_message_idx(uint32_t stream_id,
                                           uint32_t message_idx);

    /**
     * Return the chunk associated with a lower bound timestamp.
     *
     * @param[in] stream_id The stream to look for chunks in.
     * @param[in] ts The lower bound for the chunk.
     * @return The chunk if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkState* get_by_lower_bound_ts(uint32_t stream_id, const ts_t ts);

    /**
     * Return the next chunk identified by the offset.
     *
     * @param[in] offset The offset to return the next chunk for.
     * @return The chunk if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkState* next(uint64_t offset);

    /**
     * Return the next chunk identified by the offset per stream.
     *
     * @param[in] offset The offset to return the next chunk for.
     * @return The chunk if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkState* next_by_stream(uint64_t offset);

    /**
     * Return the first chunk.
     *
     * @return The chunk if found, or nullptr.
     */
    OUSTER_API_FUNCTION
    ChunkState* first();

    /**
     * Return the size of the chunk pile.
     *
     * @return The size of the chunk pile.
     */
    OUSTER_API_FUNCTION
    size_t size() const;

    /**
     * Return if there is a message index.
     *
     * @return If there is  a message index.
     */
    OUSTER_API_FUNCTION
    bool has_message_idx() const;

    /**
     * Return the stream_id to chunk offset map.
     *
     * @return The stream_id to chunk offset map.
     */
    OUSTER_API_FUNCTION
    StreamChunksMap& stream_chunks();

    /**
     * Return the chunks offset vector.
     *
     * @return The chunks offset vector.
     */
    OUSTER_API_FUNCTION
    std::vector<uint64_t> chunk_offsets() const;

    /**
     * Builds internal links between ChunkInfoNode per stream.
     *
     * @throws std::logic_error exception on non increasing timestamps.
     * @throws std::logic_error exception on non existent info.
     */
    OUSTER_API_FUNCTION
    void link_stream_chunks();

   protected:
    OUSTER_API_FUNCTION
    void link_chunks(uint64_t last_chunk_buf_offset);

   private:
    /**
     * The offset to chunk state map.
     */
    std::unordered_map<uint64_t, ChunkState> pile_{};

    /**
     * The offset to stream info map.
     */
    std::unordered_map<uint64_t, ChunkInfoNode> pile_info_{};

    /**
     * Ordered list of chunks offsets per stream id (only when ChunkInfo
     * is present).
     */
    StreamChunksMap stream_chunks_{};

    std::vector<uint64_t> chunk_offsets_{};
};

/**
 * To String Functionality For ChunkState
 *
 * @param[in] chunk_state The data to get the string representation for
 * @return The string representation
 */
OUSTER_API_FUNCTION
std::string to_string(const ChunkState& chunk_state);

/**
 * To String Functionality For ChunkInfoNode
 *
 * @param[in] chunk_info The data to get the string representation format
 * @return The string representation
 */
OUSTER_API_FUNCTION
std::string to_string(const ChunkInfoNode& chunk_info);

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
