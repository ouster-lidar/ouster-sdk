/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file reader.h
 * @brief OSF file Reader
 *
 */
#pragma once

#include <queue>
#include <unordered_map>

#include "ouster/error_handler.h"
#include "ouster/osf/file.h"
#include "ouster/osf/metadata.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

using ouster::core::default_error_handler;
using ouster::core::error_handler_t;
using ouster::core::Severity;

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
    ChunkValidity status;
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
    ChunksPile();

    /**
     * Add a new chunk to the ChunkPile.
     *
     * @param[in] offset The offset for the chunk.
     * @param[in] start_ts The first timestamp in the chunk.
     * @param[in] end_ts The first timestamp in the chunk.
     */
    OUSTER_API_FUNCTION
    void add(uint64_t offset, ts_t start_ts, ts_t end_ts);

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
     * Builds internal links between ChunkInfoNode per stream.
     *
     * @throws std::logic_error exception on non increasing timestamps.
     * @throws std::logic_error exception on non existent info.
     */
    OUSTER_API_FUNCTION
    void link_stream_chunks();

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

// Forward Decls
class Reader;
class MessageRef;
class ChunkRef;
class ChunksPile;
class ChunksRange;
struct MessagesStreamingIter;
struct MessagesChunkIter;
class MessagesStreamingRange;

/**
 * Chunk forward iterator in order of offset.
 */
struct OUSTER_API_CLASS ChunksIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const ChunkRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<ChunkRef>;
    using reference = const ChunkRef&;

    /**
     * Default construction that zeros out member variables.
     */
    OUSTER_API_FUNCTION
    ChunksIter();

    /**
     * Initialize from another ChunksIter object.
     *
     * @param[in] other The other ChunksIter object to initalize from.
     */
    OUSTER_API_FUNCTION
    ChunksIter(const ChunksIter& other);

    /**
     * Default assign operator.
     *
     * @param[in] other The other ChunksIter to assign to this.
     */
    OUSTER_API_FUNCTION
    ChunksIter& operator=(const ChunksIter& other) = default;

    /**
     * Return a ChunkRef object associated with this ChunksIter object.
     *
     * @throws std::logic_error Exception on end of iteration.
     *
     * @return The ChunkRef object associated with this ChunksIter object.
     */
    OUSTER_API_FUNCTION
    const ChunkRef operator*() const;

    /**
     * Return a ChunkRef pointer associated with this ChunksIter object.
     *
     * @return The ChunkRef pointer associated with this ChunksIter object.
     */
    OUSTER_API_FUNCTION
    const std::unique_ptr<ChunkRef> operator->() const;

    /**
     * Increment the ChunksIter iterator and return *this.
     *
     * @return The current ChunksIter object.
     */
    OUSTER_API_FUNCTION
    ChunksIter& operator++();

    /**
     * Equality operator to compare two ChunksIter objects.
     *
     * @param[in] other The other object to compare.
     * @return Whether the two ChunksIter objects are the same.
     */
    OUSTER_API_FUNCTION
    bool operator==(const ChunksIter& other) const;

    /**
     * Equality operator to compare two ChunksIter objects.
     *
     * @relates operator==(const ChunksIter& other)
     * @param[in] other The other object to compare.
     * @return Whether the two ChunksIter objects are not the same.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const ChunksIter& other) const;

    /**
     * To String Functionality For ChunksIter.
     *
     * @return The string representation
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

   private:
    /**
     * Internal constructor.
     *
     * @param[in] begin_addr The offset in the chunks to start at.
     * @param[in] end_addr The offset in the chunks that they end at.
     * @param[in] reader The reader object to use for reading.
     */
    ChunksIter(const uint64_t begin_addr, const uint64_t end_addr,
               Reader* reader);

    /**
     * Move iterator to the next chunk of "verified" chunks.
     */
    void next();

    /**
     * Move iterator to the next chunk.
     */
    void next_any();

    /**
     * Verify that the ChunksIter is not at the end,
     * and that the current chunk is valid.
     */
    bool is_cleared();

    /**
     * The current offset in the chunks.
     */
    uint64_t current_addr_;

    /**
     * The offset in the chunks that they end at.
     */
    uint64_t end_addr_;

    /**
     * The internal reader object to use for reading.
     */
    Reader* reader_;

    friend class ChunksRange;
};  // ChunksIter

/**
 * std iterator class for iterating through chunks.
 */
class OUSTER_API_CLASS ChunksRange {
   public:
    /**
     * Begin function for std iterator support.
     *
     * @return A ChunksIter object for iteration.
     */
    OUSTER_API_FUNCTION
    ChunksIter begin() const;

    /**
     * End function for std iterator support.
     *
     * @return A ChunksIter object for signifying
     *         the end of iteration.
     */
    OUSTER_API_FUNCTION
    ChunksIter end() const;

    /**
     * To String Functionality For ChunksRange.
     *
     * @return The string representation.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

   private:
    /**
     * Constructor for Reader to call for creating the ChunksRange object.
     *
     * @param[in] begin_addr The beginning offset into the chunks buffer.
     * @param[in] end_addr The end offset into the chunks buffer.
     * @param[in,out] reader The Reader object to use for reading.
     */
    ChunksRange(const uint64_t begin_addr, const uint64_t end_addr,
                Reader* reader);

    /**
     * The internal store of the begining offset into the chunks.
     */
    uint64_t begin_addr_;

    /**
     * The internal store of the ending offset into the chunks.
     */
    uint64_t end_addr_;

    /**
     * The internal store of the Reader object used for reading.
     */
    Reader* reader_;
    friend class Reader;
};  // ChunksRange

/**
 * %OSF Reader that simply reads sequentially messages from the OSF file.
 *
 * @todo Add filtered reads, and other nice things...
 */
class OUSTER_API_CLASS Reader {
   public:
    /**
     * Creates reader from %OSF file resource.
     *
     * @param[in] osf_file The OsfFile object to use to read from.
     * @param[in] error_handler An optional callback that serves as an error
     * handler.
     */
    OUSTER_API_FUNCTION
    Reader(OsfFile& osf_file,
           const error_handler_t& error_handler = default_error_handler);

    /**
     * Creates reader from %OSF file name.
     *
     * @param[in] file The OSF file path to read from.
     * @param[in] error_handler An optional callback that serves as an error
     * handler.
     */
    OUSTER_API_FUNCTION
    Reader(const std::string& file,
           const error_handler_t& error_handler = default_error_handler);

    /**
     * Reads the messages from the first OSF chunk in sequental order
     * till the end. Doesn't support RandomAccess.
     *
     * @throws std::logic_error Exception on not having sensor_info.
     *
     * @return The MessageStreamingRange object to iterate
     *         through the messages.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingRange messages();

    /**
     * @copydoc messages()
     * @param[in] start_ts Specify the start of the timestamps that
     *                     should be iterated through.
     * @param[in] end_ts Specify the end of the timestamps that
     *                   should be iterated through.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingRange messages(const ts_t start_ts, const ts_t end_ts);

    /**
     * @copydoc messages()
     * @param[in] stream_ids Filter the message iteration to specific streams.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingRange messages(const std::vector<uint32_t>& stream_ids);

    /**
     * @copydoc messages(const ts_t start_ts, const ts_t end_ts)
     * @param[in] stream_ids Filter the message iteration to specific streams.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingRange messages(const std::vector<uint32_t>& stream_ids,
                                    const ts_t start_ts, const ts_t end_ts);

    /**
     * Find the timestamp of the message by its index and stream_id.
     *
     * Requires the OSF with message_counts inside, i.e. has_message_idx()
     * return ``True``, otherwise return value is always empty (nullopt).
     *
     * @throws std::logic_error Exception on not having sensor_info.
     *
     * @param[in] stream_id stream id on which the message_idx search is
     *                      performed
     * @param[in] message_idx the message index (i.e. rank/number) to search for
     * @return message timestamp that corresponds to the message_idx in the
     *                 stream_id
     */
    OUSTER_API_FUNCTION
    nonstd::optional<ts_t> ts_by_message_idx(uint32_t stream_id,
                                             uint32_t message_idx);

    /**
     * Whether OSF contains the message counts that are needed for
     * ``ts_by_message_idx()``
     *
     * Message counts was added a bit later to the OSF core
     * (ChunkInfo struct), so this function will be obsolete over time.
     *
     * @return Whether OSF contains the message counts that are needed for
     *         ``ts_by_message_idx()``
     */
    OUSTER_API_FUNCTION
    bool has_message_idx() const;

    /**
     * Whether OSF contains the message timestamp index in the metadata
     * necessary to quickly collate and jump to a specific message time."
     *
     * @return Whether OSF contains the message timestamp index
     */
    OUSTER_API_FUNCTION
    bool has_timestamp_idx() const;

    /**
     * Reads chunks and returns the iterator to valid chunks only.
     * NOTE: Every chunk is read in full and validated. (i.e. it's not just
     * iterator over chunks index)
     *
     * @return The iterator to valid chunks only.
     */
    OUSTER_API_FUNCTION
    ChunksRange chunks();

    /**
     * Return the metadata id.
     *
     * @return The metadata id.
     */
    OUSTER_API_FUNCTION
    std::string metadata_id() const;

    /**
     * Return the lowest timestamp in the ChunksIter.
     *
     * @return The lowest timestamp in the ChunksIter.
     */
    OUSTER_API_FUNCTION
    ts_t start_ts() const;

    /**
     * Return the highest timestamp in the ChunksIter.
     *
     * @return The highest timestamp in the ChunksIter.
     */
    OUSTER_API_FUNCTION
    ts_t end_ts() const;

    /**
     * Return all metadata entries as a MetadataStore
     *
     * @return All of the metadata entries as a MetadataStore.
     */
    OUSTER_API_FUNCTION
    const MetadataStore& meta_store() const;

    /**
     * If the chunks can be read by stream and in non-decreasing timestamp
     * order.
     *
     * @return The chunks can be read by stream and timestamps are sane.
     */
    OUSTER_API_FUNCTION
    bool has_stream_info() const;

    /**
     * Get the OSF file format version.
     *
     * @return The OSF file format version of this file.
     */
    OUSTER_API_FUNCTION ouster::util::version version() const;

   private:
    /**
     * Read, parse and store all of the flatbuffer related metadata.
     *
     * @throws std::logic_error Exception on invalid metadata block.
     */
    void read_metadata();

    /**
     * Verify, store and link all streaming info indicies
     * i.e. StreamingInfo.chunks[] information
     *
     * @throws std::logic_error Exception on invalid chunk size.
     */
    void read_chunks_info();

    /**
     * Checks the flatbuffers validity of a chunk by chunk offset.
     *
     * @param[in] chunk_offset Specify the chunk to verify via offset.
     * @return The validity of the chunk.
     */
    bool verify_chunk(uint64_t chunk_offset);

    /**
     * Internal OsfFile object used to read the OSF file.
     */
    OsfFile file_;

    /**
     * File version.
     */
    ouster::util::version version_;

    /**
     * Internal MetadataStore object to hold all of the
     * metadata entries.
     */
    MetadataStore meta_store_{};

    /**
     * Internal ChunksPile object to hold all of the
     * chunks.
     */
    ChunksPile chunks_{};

    /**
     * Internal indicator of if this file has streaming info
     */
    bool has_streaming_info_{false};

    /**
     * Absolute offset to the beginning of the chunks in a file.
     */
    uint64_t chunks_base_offset_{0};

    /**
     * Internal byte vector containing the raw flatbuffer
     * metadata data.
     */
    std::vector<uint8_t> metadata_buf_{};

    /**
     * A function that can serve as a user-provided error handler.
     */
    error_handler_t error_handler_;

    // NOTE: These classes need an access to private member `chunks_` ...
    friend class ChunkRef;
    friend struct ChunksIter;
    friend struct MessagesStreamingIter;
};  // Reader

/**
 * Thin interface class that holds the pointer to the message
 * and reconstructs underlying data to the corresponding object type given
 * the Stream type.
 */
class OUSTER_API_CLASS MessageRef {
   public:
    using ts_t = osf::ts_t;

    /**
     * The only way to create the MessageRef is to point to the corresponding
     * byte buffer of the message in OSF file.
     *
     * @param[in] buf The buffer to use to make a MessageRef object.
     * @param[in] meta_provider The metadata store that is used in types
     *                          reconstruction
     * @param[in] error_handler An optional callback that serves as an error
     * handler.
     */
    OUSTER_API_FUNCTION
    MessageRef(const uint8_t* buf, const MetadataStore& meta_provider,
               const error_handler_t& error_handler);

    /**
     * The only way to create the MessageRef is to point to the corresponding
     * byte buffer of the message in OSF file.
     *
     * @param[in] buf The buffer to use to make a MessageRef object.
     * @param[in] meta_provider The metadata store that is used in types
     *                          reconstruction
     * @param[in,out] chunk_buf The pre-existing chunk buffer to use.
     * @param[in] error_handler An optional callback that serves as an error
     * handler.
     */
    OUSTER_API_FUNCTION
    MessageRef(const uint8_t* buf, const MetadataStore& meta_provider,
               std::shared_ptr<std::vector<uint8_t>> chunk_buf,
               const error_handler_t& error_handler);

    /**
     * Get the message stream id.
     *
     * @return The message stream id.
     */
    OUSTER_API_FUNCTION
    uint32_t id() const;

    /**
     * Get the timestamp of the message.
     *
     * @return The timestamp of the message.
     */
    OUSTER_API_FUNCTION
    ts_t ts() const;

    /// @todo [pb] Type of the stored data (meta of the stream?)
    // std::string stream_type() const;

    /**
     * Get the pointer to the underlying data.
     *
     * @return The pointer to the underlying data.
     */
    OUSTER_API_FUNCTION
    const uint8_t* buf() const;

    /**
     * Debug string representation.
     *
     * @return The string representation of a MessageRef.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

    /**
     * Checks whether the message belongs to the specified Stream type.
     *
     * @tparam Stream The cpp data type to check against.
     * @return If the current MessageRef is of type [Stream].
     */
    template <typename Stream>
    bool is() const {
        auto meta = meta_provider_.get<typename Stream::meta_type>(id());
        return (meta != nullptr);
    }

    /**
     * Checks whether the message belongs to the specified Strean type.
     *
     * @param[in] type_str The data type in string form to check against.
     * @return If the current MessageRef is of type type_str.
     */
    OUSTER_API_FUNCTION
    bool is(const std::string& type_str) const;

    /**
     * Reconstructs the underlying data to the class (copies data).
     *
     * @tparam Stream The type of the target data.
     * @return A smart pointer to the new object.
     */
    template <typename Stream>
    std::unique_ptr<typename Stream::obj_type> decode_msg() const {
        auto meta = meta_provider_.get<typename Stream::meta_type>(id());

        if (meta == nullptr) {
            // Stream and metadata entry id is inconsistent
            return nullptr;
        }

        try {
            return Stream::decode_msg(*this, *meta, meta_provider_);
        } catch (const std::runtime_error& error) {
            error_handler_(Severity::OUSTER_WARNING, error.what());
            return nullptr;
        }
    }

    template <typename Stream, typename T>
    std::unique_ptr<typename Stream::obj_type> decode_msg(T& t) const {
        auto meta = meta_provider_.get<typename Stream::meta_type>(id());

        if (meta == nullptr) {
            // Stream and metadata entry id is inconsistent
            return nullptr;
        }

        try {
            return Stream::decode_msg(*this, *meta, meta_provider_, t);
        } catch (const std::runtime_error& error) {
            error_handler_(Severity::OUSTER_WARNING, error.what());
            return nullptr;
        }
    }

    /**
     * Get the underlying raw message byte vector.
     *
     * @return Return the underlying raw message byte vector.
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const;

    /**
     * Check if two MessageRefs are equal.
     *
     * @param[in] other The other MessageRef to check against.
     * @return If the two MessageRefs are equal.
     */
    OUSTER_API_FUNCTION
    bool operator==(const MessageRef& other) const;

    /**
     * Check if two MessageRefs are not equal.
     *
     * @param[in] other The other MessageRef to check against.
     * @return If the two MessageRefs are not equal.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const MessageRef& other) const;

    /**
     * Get the error handler associated with this message.
     *
     * @return A constant reference to the error handler.
     */
    OUSTER_API_FUNCTION
    const error_handler_t& error_handler() const;

   private:
    /**
     * The internal raw byte array.
     */
    const uint8_t* buf_;

    /**
     * The internal store for all of the metadata entries.
     */
    const MetadataStore& meta_provider_;

    /**
     * The internal chunk buffer to use.
     */
    std::shared_ptr<ChunkBuffer> chunk_buf_;

    /**
     * A function can serve as a user-provided error handler.
     */
    error_handler_t error_handler_;
};  // MessageRef

/**
 * Thin interface class that holds the pointer to the chunk and hides the
 * messages reading routines. It expects that Chunk was "verified" before
 * creating a ChunkRef.
 */
class OUSTER_API_CLASS ChunkRef {
   public:
    /**
     * Default ChunkRef constructor that just zeros the internal fields.
     */
    OUSTER_API_FUNCTION
    ChunkRef();

    /**
     * @param[in] offset The offset into the chunk array for the specified
     *                   chunk.
     * @param[in] reader The reader object to use for reading.
     */
    OUSTER_API_FUNCTION
    ChunkRef(const uint64_t offset, Reader* reader);

    /**
     * Check if two ChunkRefs are equal.
     *
     * @param[in] other The other ChunkRef to check against.
     * @return If the two ChunkRef are equal.
     */
    OUSTER_API_FUNCTION
    bool operator==(const ChunkRef& other) const;

    /**
     * Check if two ChunkRefs are not equal.
     *
     * @param[in] other The other ChunkRef to check against.
     * @return If the two ChunkRef are not equal.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const ChunkRef& other) const;

    /**
     * Get the ChunkState for the chunk associated with this ChunkRef.
     *
     * @relates ChunkState
     *
     * @return The ChunkState associated with this ChunkRef.
     */
    OUSTER_API_FUNCTION
    ChunkState* state();

    /**
     * @copydoc state()
     */
    OUSTER_API_FUNCTION
    const ChunkState* state() const;

    /**
     * Get the ChunkInfoNode for the chunk associated with this ChunkRef.
     *
     * @relates ChunkInfoNode
     *
     * @return The ChunkInfoNode associated with this ChunkRef.
     */
    OUSTER_API_FUNCTION
    ChunkInfoNode* info();

    /**
     * @copydoc info()
     */
    OUSTER_API_FUNCTION
    const ChunkInfoNode* info() const;

    /**
     * Begin function for std iterator support.
     *
     * @return A MessagesChunkIter object for iteration.
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter begin() const;

    /**
     * End function for std iterator support.
     *
     * @return A MessagesChunkIter object for signifying
     *         the end of iteration.
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter end() const;

    /**
     * Get the message at a specific index.
     *
     * @todo Simplify this and any other instance of this
     *
     * @param[in] msg_idx The message index to get.
     * @return The resulting message.
     */
    OUSTER_API_FUNCTION
    const MessageRef operator[](size_t msg_idx) const;

    /**
     * Get the message smart pointer at a specific index.
     *
     * @todo Simplify this and any other instance of this
     *
     * @param[in] msg_idx The message index to get.
     * @return The resulting message smart pointer,
     *         returns nullptr if non existent.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<const MessageRef> messages(size_t msg_idx) const;

    /**
     * Debug string representation.
     *
     * @return The string representation of a ChunkRef.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

    /**
     * Return the chunk offset in the larger flatbuffer array.
     *
     * @return The chunk offset in the larger flatbuffer array.
     */
    OUSTER_API_FUNCTION
    uint64_t offset() const;

    /**
     * The lowest timestamp in the chunk.
     * A shortcut for state()->start_ts
     *
     * @relates state
     * @return starting timestamp in the received chunk
     */
    OUSTER_API_FUNCTION
    ts_t start_ts() const;

    /**
     * The highest timestamp in the chunk.
     * A shortcut for state()->end_ts
     *
     * @relates state
     * @return last timestamp in the received chunk
     */
    OUSTER_API_FUNCTION
    ts_t end_ts() const;

    /**
     * Returns the summation of the sizes of the chunks messages
     *
     * @return The summation of the sizes of the chunks messages,
     *         0 on chunk invalidity.
     */
    OUSTER_API_FUNCTION
    size_t size() const;

    /**
     * Get the validity of the chunk.
     *
     * @return The validity of the chunk.
     */
    OUSTER_API_FUNCTION
    bool valid() const;

   private:
    /**
     * Helper method to get the raw chunk pointer.
     * Deals with the chunk being memory mapped or not.
     *
     * @return The raw chunk pointer.
     */
    const uint8_t* get_chunk_ptr() const;

    /**
     * The internal chunk offset variable.
     */
    uint64_t chunk_offset_;

    /**
     * Reader object to use for reading.
     */
    Reader* reader_;

    /**
     * Chunk buffer to use for reading.
     */
    std::shared_ptr<ChunkBuffer> chunk_buf_;
};  // ChunkRef

/**
 * Convenient iterator class to go over all of the
 * messages in a chunk.
 */
struct OUSTER_API_CLASS MessagesChunkIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const MessageRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<MessageRef>;
    using reference = const MessageRef&;

    /**
     * Default MessagesChunkIter constructor that just zeros
     * the internal fields.
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter();

    /**
     * Initialize the MessagesChunkIter from another
     * MessageChunkIter object.
     *
     * @param[in] other The other MessagesChunkIter to initalize from.
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter(const MessagesChunkIter& other);

    /**
     * Default assignment operation.
     *
     * @param[in] other The other MessageChunkIter to assign to.
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter& operator=(const MessagesChunkIter& other) = default;

    /**
     * Gets the current ChunkRef via value.
     *
     * @return The current ChunkRef value.
     */
    OUSTER_API_FUNCTION
    const MessageRef operator*() const;

    /**
     * Gets the current ChunkRef via smart pointer.
     *
     * @return The current ChunkRef smart pointer.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<const MessageRef> operator->() const;

    /**
     * Advance to the next message in the chunk.
     *
     * @return *this
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter& operator++();

    /**
     * @copydoc operator++()
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter operator++(int);

    /**
     * Regress to the previous message in the chunk.
     *
     * @return *this
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter& operator--();

    /**
     * @copydoc operator--()
     */
    OUSTER_API_FUNCTION
    MessagesChunkIter operator--(int);

    /**
     * Check if two MessagesChunkIter are equal.
     *
     * @param[in] other The other MessagesChunkIter to check against.
     * @return If the two MessagesChunkIter are equal.
     */
    OUSTER_API_FUNCTION
    bool operator==(const MessagesChunkIter& other) const;

    /**
     * Check if two MessagesChunkIter are not equal.
     *
     * @param[in] other The other MessagesChunkIter to check against.
     * @return If the two MessagesChunkIter are not equal.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const MessagesChunkIter& other) const;

    /**
     * Debug string representation.
     *
     * @return The string representation of a MessagesChunkIter.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

   private:
    /**
     * Internal constructor for initializing a MessageChunkIter
     * off of a ChunkRef and message index.
     *
     * @param[in] chunk_ref The ChunkRef to use when initializing.
     * @param[in] msg_idx The message index in the ChunkRef
     *                    to use when initializing.
     */
    MessagesChunkIter(const ChunkRef chunk_ref, const size_t msg_idx);

    /**
     * Advance to the next message in the chunk.
     */
    void next();

    /**
     * Regress to the previous message in the chunk.
     */
    void prev();

    /**
     * The internal ChunkRef var.
     */
    ChunkRef chunk_ref_;

    /**
     * The current message index.
     */
    size_t msg_idx_;

    friend class ChunkRef;
};  // MessagesChunkIter

class OUSTER_API_CLASS MessagesStreamingRange {
   public:
    /**
     * Begin function for std iterator support.
     *
     * @return A MessagesStreamingIter object for iteration.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter begin() const;

    /**
     * End function for std iterator support.
     *
     * @return A MessagesStreamingIter object for signifying
     *         the end of iteration.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter end() const;

    /**
     * Debug string representation.
     *
     * @return The string representation of a MessagesStreamingRange.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

   private:
    /**
     * Initialize from Reader on a set of filters..
     * //using range [start_ts, end_ts] <---- not inclusive .... !!!
     *
     * @param[in] start_ts The lowest timestamp to start with.
     * @param[in] end_ts The highest timestamp to end with.
     * @param[in] stream_ids The stream indicies to use with the streaming
     *                       range.
     * @param[in] reader The reader object to use for reading the OSF file.
     */
    MessagesStreamingRange(const ts_t start_ts, const ts_t end_ts,
                           const std::vector<uint32_t>& stream_ids,
                           Reader* reader);

    /**
     * The lowest timestamp for the range.
     */
    ts_t start_ts_;

    /**
     * The highest timestamp for the range.
     */
    ts_t end_ts_;

    /**
     * The set of stream indicies in the range.
     */
    std::vector<uint32_t> stream_ids_;

    /**
     * The reader object to use to read the OSF file.
     */
    Reader* reader_;
    friend class Reader;
};  // MessagesStreamingRange

/**
 * Iterator over all messages in Streaming Layout order for specified
 * timestamp range.
 */
struct OUSTER_API_CLASS MessagesStreamingIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const MessageRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<MessageRef>;
    using reference = const MessageRef&;

    using opened_chunk_type = std::pair<ChunkRef, uint64_t>;

    /**
     * Comparison struct used for determining which chunk is greater.
     */
    struct OUSTER_API_CLASS greater_chunk_type {
        /**
         * Comparison operator used for determining if the first is greater
         * than the second. The comparison is based on the timestamps.
         *
         * @param[in] a The first chunk to compare.
         * @param[in] b The second chunk to compare.
         * @return If the first chunk is greater than the second chunk.
         */
        OUSTER_API_FUNCTION
        bool operator()(const opened_chunk_type& a, const opened_chunk_type& b);
    };

    /**
     * Default MessagesStreamingIter constructor that just zeros
     * the internal fields.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter();

    /**
     * Initialize the MessagesStreamingIter from another
     * MessagesStreamingIter object.
     *
     * @param[in] other The other MessagesStreamingIter to initalize from.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter(const MessagesStreamingIter& other);

    /**
     * Default assignment operation.
     *
     * @param[in] other The other MessagesStreamingIter to assign to.
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter& operator=(const MessagesStreamingIter& other) =
        default;

    /**
     * Gets the current MessageRef via value.
     *
     * @return The current MessageRef value.
     */
    OUSTER_API_FUNCTION
    const MessageRef operator*() const;

    /**
     * Gets the current MessageRef via smart pointer.
     *
     * @return The current MessageRef smart pointer.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<const MessageRef> operator->() const;

    /**
     * Advance to the next message.
     *
     * @return *this
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter& operator++();

    /**
     * @copydoc operator++()
     */
    OUSTER_API_FUNCTION
    MessagesStreamingIter operator++(int);

    /**
     * Check if two MessagesStreamingIter are equal.
     *
     * @param[in] other The other MessagesStreamingIter to check against.
     * @return If the two MessagesStreamingIter are equal.
     */
    OUSTER_API_FUNCTION
    bool operator==(const MessagesStreamingIter& other) const;

    /**
     * Check if two MessagesStreamingIter are not equal.
     *
     * @param[in] other The other MessagesStreamingIter to check against.
     * @return If the two MessagesStreamingIter are not equal.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const MessagesStreamingIter& other) const;

    /**
     * Debug string representation.
     *
     * @return The string representation of a MessagesStreamingIter.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const;

   private:
    /**
     * Initialize from Reader on a set of filters..
     * //using range [start_ts, end_ts] <---- not inclusive .... !!!
     *
     * @param[in] start_ts The lowest timestamp to start with.
     * @param[in] end_ts The highest timestamp to end with.
     * @param[in] stream_ids The stream indicies to use with the streaming
     *                       range.
     * @param[in] reader The reader object to use for reading the OSF file.
     */
    MessagesStreamingIter(const ts_t start_ts, const ts_t end_ts,
                          const std::vector<uint32_t>& stream_ids,
                          Reader* reader);

    /**
     * Advance to the next message.
     */
    void next();

    /**
     * The current timestamp.
     */
    ts_t curr_ts_;

    /**
     * The last timestamp.
     */
    ts_t end_ts_;

    /**
     * The streams to iterate on.
     */
    std::vector<uint32_t> stream_ids_;

    /**
     * Used to hash the set of stream indexes.
     *
     * @todo Look at possibly removing this.
     */
    uint32_t stream_ids_hash_;

    /**
     * The reader object used to read the OSF file.
     */
    Reader* reader_;

    /**
     * Priority queue used to hold the chunks in timestamp order.
     *
     * @relates greater_chunk_type
     */
    std::priority_queue<opened_chunk_type, std::vector<opened_chunk_type>,
                        greater_chunk_type>
        curr_chunks_{};
    friend class Reader;
    friend class MessagesStreamingRange;
};  // MessagesStreamingIter

}  // namespace osf
}  // namespace ouster
