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

#include "ouster/osf/file.h"
#include "ouster/osf/metadata.h"
#include "ouster/types.h"

namespace ouster {
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
 * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset
 */
struct ChunkState {
    /**
     * The current chunk's offset from the begining of the chunks section.
     *
     * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset::offset
     */
    uint64_t offset;

    /**
     * The next chunk's offset for forward iteration.
     * Should work like a linked list.
     *
     * This is partially synthesized from the Flat Buffers.
     * This will link up with the next chunks offset.
     * Value is set in ChunksPile::link_stream_chunks
     * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset::offset
     */
    uint64_t next_offset;

    /**
     * The first timestamp in the chunk in ordinality.
     *
     * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset::start_ts
     */
    ts_t start_ts;

    /**
     * The last timestamp in the chunk in ordinality.
     *
     * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset::end_ts
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
struct ChunkInfoNode {
    /**
     * The chunk offset from the begining of the chunks section.
     *
     * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset::offset
     */
    uint64_t offset;

    /**
     * The next chunk's offset for forward iteration.
     * Should work like a linked list.
     *
     * This is partially synthesized from the Flat Buffers.
     * This will link up with the next chunks offset.
     * Value is set in ChunksPile::link_stream_chunks
     * Flat Buffer Reference: fb/metadata.fbs::ChunkOffset::offset
     */
    uint64_t next_offset;

    /**
     * The stream this is associated with.
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs::ChunkInfo::stream_id
     */
    uint32_t stream_id;

    /**
     * Total number of messages in a `stream_id` in the whole OSF file
     *
     * Flat Buffer Reference:
     *   fb/streaming/streaming_info.fbs::ChunkInfo::message_count
     */
    uint32_t message_count;

    /**
     * The index of the start of the message.
     * @todo try to describe this better
     *
     * This is partially synthesized from the Flat Buffers.
     * Value is set in ChunksPile::link_stream_chunks
     * Synthesized from Flat Buffer Reference:
     *   fb/metadata.fbs::ChunkOffset::message_count
     */
    uint32_t message_start_idx;
};

/**
 * Chunks state map. Validity info and next offset.
 */
class ChunksPile {
   public:
    /**
     * stream_id to offset map.
     */
    using StreamChunksMap =
        std::unordered_map<uint32_t, std::shared_ptr<std::vector<uint64_t>>>;

    /**
     * Default blank constructor.
     */
    ChunksPile();

    /**
     * Add a new chunk to the ChunkPile.
     *
     * @param[in] offset The offset for the chunk.
     * @param[in] start_ts The first timestamp in the chunk.
     * @param[in] end_ts The first timestamp in the chunk.
     */
    void add(uint64_t offset, ts_t start_ts, ts_t end_ts);

    /**
     * Return the chunk associated with an offset.
     *
     * @param[in] offset The offset to return the chunk for.
     * @return The chunk if found, or nullptr.
     */
    ChunkState* get(uint64_t offset);

    /**
     * Add a new streaming info to the ChunkPile.
     *
     * @param[in] offset The offset for the chunk.
     * @param[in] stream_id The stream_id associated.
     * @param[in] message_count The number of messages.
     */
    void add_info(uint64_t offset, uint32_t stream_id, uint32_t message_count);

    /**
     * Return the streaming info associated with an offset.
     *
     * @param[in] offset The offset to return the streaming info for.
     * @return The streaming info if found, or nullptr.
     */
    ChunkInfoNode* get_info(uint64_t offset);

    /**
     * Return the streaming info associated with a message_idx.
     *
     * @param[in] stream_id The stream to look for infos in.
     * @param[in] message_idx The specific message index to look for.
     * @return The streaming info if found, or nullptr.
     */
    ChunkInfoNode* get_info_by_message_idx(uint32_t stream_id,
                                           uint32_t message_idx);

    /**
     * Return the chunk associated with a lower bound timestamp.
     *
     * @param[in] stream_id The stream to look for chunks in.
     * @param[in] ts The lower bound for the chunk.
     * @return The chunk if found, or nullptr.
     */
    ChunkState* get_by_lower_bound_ts(uint32_t stream_id, const ts_t ts);

    /**
     * Return the next chunk identified by the offset.
     *
     * @param[in] offset The offset to return the next chunk for.
     * @return The chunk if found, or nullptr.
     */
    ChunkState* next(uint64_t offset);

    /**
     * Return the next chunk identified by the offset per stream.
     *
     * @param[in] offset The offset to return the next chunk for.
     * @return The chunk if found, or nullptr.
     */
    ChunkState* next_by_stream(uint64_t offset);

    /**
     * Return the first chunk.
     *
     * @return The chunk if found, or nullptr.
     */
    ChunkState* first();

    /**
     * Return the size of the chunk pile.
     *
     * @return The size of the chunk pile.
     */
    size_t size() const;

    /**
     * Return if there is stream info.
     *
     * @return If there is stream info.
     */
    bool has_info() const;

    /**
     * Return if there is a message index.
     *
     * @return If there is  a message index.
     */
    bool has_message_idx() const;

    /**
     * Return the stream_id to chunk offset map.
     *
     * @return The stream_id to chunk offset map.
     */
    StreamChunksMap& stream_chunks();

    /**
     * Builds internal links between ChunkInfoNode per stream
     */
    void link_stream_chunks();

   private:
    /**
     * The offset to chunk state map
     */
    std::unordered_map<uint64_t, ChunkState> pile_{};

    /**
     * The offset to stream info map
     */
    std::unordered_map<uint64_t, ChunkInfoNode> pile_info_{};

    /**
     * Ordered list of chunks offsets per stream id (only when ChunkInfo
     * is present)
     */
    StreamChunksMap stream_chunks_{};
};

std::string to_string(const ChunkState& chunk_state);
std::string to_string(const ChunkInfoNode& chunk_info);

class Reader;
class MessageRef;
class ChunkRef;
class ChunksPile;
class ChunksRange;
struct MessagesStandardIter;
struct MessagesStreamingIter;
struct MessagesChunkIter;

class MessagesStreamingRange;

/**
 * Chunks forward iterator in order of offset.
 */
struct ChunksIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const ChunkRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<ChunkRef>;
    using reference = const ChunkRef&;

    ChunksIter();
    ChunksIter(const ChunksIter& other);
    ChunksIter& operator=(const ChunksIter& other) = default;

    const ChunkRef operator*() const;
    const std::unique_ptr<ChunkRef> operator->() const;
    ChunksIter& operator++();
    bool operator==(const ChunksIter& other) const;
    bool operator!=(const ChunksIter& other) const;

    std::string to_string() const;

   private:
    ChunksIter(const uint64_t begin_addr, const uint64_t end_addr,
               Reader* reader);
    // move iterator to the next chunk of "verified" chunks set
    void next();
    void next_any();
    bool is_cleared();

    uint64_t current_addr_;
    uint64_t end_addr_;
    Reader* reader_;
    friend class ChunksRange;
};  // ChunksIter

/**
 * Chunks range
 */
class ChunksRange {
   public:
    ChunksIter begin() const;
    ChunksIter end() const;

    std::string to_string() const;

   private:
    ChunksRange(const uint64_t begin_addr, const uint64_t end_addr,
                Reader* reader);

    uint64_t begin_addr_;
    uint64_t end_addr_;
    Reader* reader_;
    friend class Reader;
};  // ChunksRange

/**
 * Messages range.
 */
class MessagesStandardRange {
   public:
    MessagesStandardIter begin() const;
    MessagesStandardIter end() const;

    std::string to_string() const;

   private:
    MessagesStandardRange(const ChunksIter begin_it, const ChunksIter end_it);

    ChunksIter begin_chunk_it_;
    ChunksIter end_chunk_it_;
    friend class Reader;
};  // MessagesStandardRange

/**
 * Messages forward iterator to read all messages across chunks span.
 */
struct MessagesStandardIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const MessageRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<MessageRef>;
    using reference = const MessageRef&;

    MessagesStandardIter();
    MessagesStandardIter(const MessagesStandardIter& other);
    MessagesStandardIter& operator=(const MessagesStandardIter& other) =
        default;

    const MessageRef operator*() const;
    std::unique_ptr<const MessageRef> operator->() const;
    MessagesStandardIter& operator++();
    bool operator==(const MessagesStandardIter& other) const;
    bool operator!=(const MessagesStandardIter& other) const;

    std::string to_string() const;

   private:
    MessagesStandardIter(const ChunksIter begin_it, const ChunksIter end_it,
                         const size_t msg_idx);
    // move iterator to the next msg that passes is_cleared()
    void next();
    void next_any();
    // true if the current msg pointer passes is_cleared() test (i.e. valid)
    bool is_cleared();

    ChunksIter current_chunk_it_;
    ChunksIter end_chunk_it_;
    size_t msg_idx_;
    friend class MessagesStandardRange;
};  // MessagesStandardIter

/**
 * %OSF Reader that simply reads sequentially messages from the OSF file.
 *
 * @todo Add filtered reads, and other nice things...
 */
class Reader {
   public:
    /**
     * Creates reader from %OSF file resource.
     */
    Reader(OsfFile& osf_file);

    /**
     * Creates reader from %OSF file name.
     */
    Reader(const std::string& file);

    /**
     * Reads the messages from the first OSF chunk in sequental order
     * till the end. Doesn't support RandomAccess.
     */
    MessagesStreamingRange messages();

    MessagesStreamingRange messages(const ts_t start_ts, const ts_t end_ts);

    MessagesStreamingRange messages(const std::vector<uint32_t>& stream_ids);

    MessagesStreamingRange messages(const std::vector<uint32_t>& stream_ids,
                                    const ts_t start_ts, const ts_t end_ts);

    /**
     * Find the timestamp of the message by its index and stream_id.
     *
     * Requires the OSF with message_counts inside, i.e. has_message_idx()
     * return ``True``, otherwise return value is always empty (nullopt).
     *
     * @param[in] stream_id stream id on which the message_idx search is
     * performed
     * @param[in] message_idx the message index (i.e. rank/number) to search for
     * @return message timestamp that corresponds to the message_idx in the
     *         stream_id
     */
    nonstd::optional<ts_t> ts_by_message_idx(uint32_t stream_id,
                                             uint32_t message_idx);

    /**
     * Whether OSF contains the message counts that are needed for
     * ``ts_by_message_idx()``
     *
     * Message counts was added a bit later to the OSF core
     * (ChunkInfo struct), so this function will be obsolete over time.
     */
    bool has_message_idx() const;

    MessagesStandardRange messages_standard();

    /**
     * Reads chunks and returns the iterator to valid chunks only.
     * NOTE: Every chunk is read in full and validated. (i.e. it's not just
     * iterator over chunks index)
     */
    ChunksRange chunks();

    /** metadata id */
    std::string id() const;

    /** metadata start ts */
    ts_t start_ts() const;

    /** metadata end ts */
    ts_t end_ts() const;

    /** Metadata store to get access to all metadata entries. */
    const MetadataStore& meta_store() const;

    /** if it can be read by stream and in non-decreasing timestamp order. */
    bool has_stream_info() const;

   private:
    void read_metadata();
    void read_chunks_info();  // i.e. StreamingInfo.chunks[] information

    // Checks the flatbuffers validity of a chunk by chunk offset.
    bool verify_chunk(uint64_t chunk_offset);

    OsfFile file_;

    MetadataStore meta_store_{};

    ChunksPile chunks_{};

    // absolute offset to the beginning of the chunks in a file.
    uint64_t chunks_base_offset_{0};
    std::vector<uint8_t> metadata_buf_{};

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
class MessageRef {
   public:
    using ts_t = osf::ts_t;

    /**
     * The only way to create the MessageRef is to point to the corresponding
     * byte buffer of the message in OSF file.
     * @param meta_provider the metadata store that is used in types
     *                      reconstruction
     */
    MessageRef(const uint8_t* buf, const MetadataStore& meta_provider);

    MessageRef(const uint8_t* buf, const MetadataStore& meta_provider,
               std::shared_ptr<std::vector<uint8_t>> chunk_buf);

    /** Message stream id */
    uint32_t id() const;

    /** Timestamp of the message */
    ts_t ts() const;

    /// @todo [pb] Type of the stored data (meta of the stream?)
    // std::string stream_type() const;

    /** Pointer to the underlying data */
    const uint8_t* buf() const;

    /** Debug string representation */
    std::string to_string() const;

    /** Checks whether the message belongs to the specified Stream type */
    template <typename Stream>
    bool is() const {
        auto meta = meta_provider_.get<typename Stream::meta_type>(id());
        return (meta != nullptr);
    }

    bool is(const std::string& type_str) const;

    /** Reconstructs the underlying data to the class (copies data) */
    template <typename Stream>
    std::unique_ptr<typename Stream::obj_type> decode_msg() const {
        auto meta = meta_provider_.get<typename Stream::meta_type>(id());

        if (meta == nullptr) {
            // Stream and metadata entry id is inconsistent
            return nullptr;
        }

        return Stream::decode_msg(buffer(), *meta, meta_provider_);
    }

    std::vector<uint8_t> buffer() const;

    bool operator==(const MessageRef& other) const;
    bool operator!=(const MessageRef& other) const;

   private:
    const uint8_t* buf_;
    const MetadataStore& meta_provider_;

    std::shared_ptr<ChunkBuffer> chunk_buf_;
};  // MessageRef

/**
 * Thin interface class that holds the pointer to the chunk and hides the
 * messages reading routines. It expects that Chunk was "verified" before
 * creating a ChunkRef.
 *
 */
class ChunkRef {
   public:
    ChunkRef();
    ChunkRef(const uint64_t offset, Reader* reader);

    bool operator==(const ChunkRef& other) const;
    bool operator!=(const ChunkRef& other) const;

    ChunkState* state();
    const ChunkState* state() const;

    ChunkInfoNode* info();
    const ChunkInfoNode* info() const;

    MessagesChunkIter begin() const;
    MessagesChunkIter end() const;

    const MessageRef operator[](size_t msg_idx) const;

    std::unique_ptr<const MessageRef> messages(size_t msg_idx) const;

    /** Debug string representation */
    std::string to_string() const;

    uint64_t offset() const;
    ts_t start_ts() const;
    ts_t end_ts() const;

    size_t size() const;

    bool valid() const;

   private:
    const uint8_t* get_chunk_ptr() const;

    uint64_t chunk_offset_;
    Reader* reader_;

    std::shared_ptr<ChunkBuffer> chunk_buf_;
};  // ChunkRef

/**
 * Convenienv iterator over all messages in a chunk.
 */
struct MessagesChunkIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const MessageRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<MessageRef>;
    using reference = const MessageRef&;

    MessagesChunkIter();
    MessagesChunkIter(const MessagesChunkIter& other);
    MessagesChunkIter& operator=(const MessagesChunkIter& other) = default;

    const MessageRef operator*() const;
    std::unique_ptr<const MessageRef> operator->() const;
    MessagesChunkIter& operator++();
    MessagesChunkIter operator++(int);
    MessagesChunkIter& operator--();
    MessagesChunkIter operator--(int);
    bool operator==(const MessagesChunkIter& other) const;
    bool operator!=(const MessagesChunkIter& other) const;

    std::string to_string() const;

   private:
    MessagesChunkIter(const ChunkRef chunk_ref, const size_t msg_idx);
    void next();
    void prev();

    ChunkRef chunk_ref_;
    size_t msg_idx_;
    friend class ChunkRef;
};  // MessagesChunkIter

class MessagesStreamingRange {
   public:
    MessagesStreamingIter begin() const;
    MessagesStreamingIter end() const;

    std::string to_string() const;

   private:
    // using range [start_ts, end_ts] <---- not inclusive .... !!!
    MessagesStreamingRange(const ts_t start_ts, const ts_t end_ts,
                           const std::vector<uint32_t>& stream_ids,
                           Reader* reader);

    ts_t start_ts_;
    ts_t end_ts_;
    std::vector<uint32_t> stream_ids_;
    Reader* reader_;
    friend class Reader;
};  // MessagesStreamingRange

/**
 * Iterator over all messages in Streaming Layout order for specified
 * timestamp range.
 */
struct MessagesStreamingIter {
    using iterator_category = std::forward_iterator_tag;
    using value_type = const MessageRef;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::unique_ptr<MessageRef>;
    using reference = const MessageRef&;

    using opened_chunk_type = std::pair<ChunkRef, uint64_t>;

    struct greater_chunk_type {
        bool operator()(const opened_chunk_type& a, const opened_chunk_type& b);
    };

    MessagesStreamingIter();
    MessagesStreamingIter(const MessagesStreamingIter& other);
    MessagesStreamingIter& operator=(const MessagesStreamingIter& other) =
        default;

    const MessageRef operator*() const;
    std::unique_ptr<const MessageRef> operator->() const;
    MessagesStreamingIter& operator++();
    MessagesStreamingIter operator++(int);
    bool operator==(const MessagesStreamingIter& other) const;
    bool operator!=(const MessagesStreamingIter& other) const;

    std::string to_string() const;

   private:
    // using range [start_ts, end_ts) <---- not inclusive .... !!!
    MessagesStreamingIter(const ts_t start_ts, const ts_t end_ts,
                          const std::vector<uint32_t>& stream_ids,
                          Reader* reader);
    void next();

    ts_t curr_ts_;
    ts_t end_ts_;
    std::vector<uint32_t> stream_ids_;
    uint32_t stream_ids_hash_;
    Reader* reader_;
    std::priority_queue<opened_chunk_type, std::vector<opened_chunk_type>,
                        greater_chunk_type>
        curr_chunks_{};
    friend class Reader;
    friend class MessagesStreamingRange;
};  // MessagesStreamingIter

}  // namespace osf
}  // namespace ouster
