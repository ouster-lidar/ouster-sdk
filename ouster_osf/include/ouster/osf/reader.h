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

enum class ChunkValidity { UNKNOWN = 0, VALID, INVALID };

/**
 * Chunks state. Validity info and next offset for forward iteration.
 */
struct ChunkState {
    uint64_t offset;
    uint64_t next_offset;
    ts_t start_ts;
    ts_t end_ts;
    ChunkValidity status;
};

struct ChunkInfoNode {
    uint64_t offset;
    uint64_t next_offset;
    uint32_t stream_id;
    uint32_t message_count;
    uint32_t message_start_idx;
};

/**
 * Chunks state map. Validity info and next offset.
 */
class ChunksPile {
   public:
    using ChunkStateIter = std::unordered_map<uint64_t, ChunkState>::iterator;
    using ChunkInfoIter = std::unordered_map<uint64_t, ChunkInfoNode>::iterator;
    using StreamChunksMap =
        std::unordered_map<uint32_t, std::shared_ptr<std::vector<uint64_t>>>;

    ChunksPile(){};

    void add(uint64_t offset, ts_t start_ts, ts_t end_ts);
    ChunkState* get(uint64_t offset);
    void add_info(uint64_t offset, uint32_t stream_id, uint32_t message_count);
    ChunkInfoNode* get_info(uint64_t offset);
    ChunkInfoNode* get_info_by_message_idx(uint32_t stream_id,
                                           uint32_t message_idx);
    ChunkState* get_by_lower_bound_ts(uint32_t stream_id, const ts_t ts);
    ChunkState* next(uint64_t offset);
    ChunkState* next_by_stream(uint64_t offset);

    ChunkState* first();

    ChunkStateIter begin();
    ChunkStateIter end();

    size_t size() const;

    bool has_info() const;
    bool has_message_idx() const;

    StreamChunksMap& stream_chunks() { return stream_chunks_; }

    // builds internal links between ChunkInfoNode per stream
    void link_stream_chunks();

   private:
    std::unordered_map<uint64_t, ChunkState> pile_{};
    std::unordered_map<uint64_t, ChunkInfoNode> pile_info_{};

    // ordered list of chunks offsets per stream id (only when ChunkInfo
    // is present)
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
    ChunksIter operator++(int);
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
    MessagesStandardIter operator++(int);
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
     * @param stream_id[in] stream id on which the message_idx search is
     * performed
     * @param message_idx[in] the message index (i.e. rank/number) to search for
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
    bool has_message_idx() const { return chunks_.has_message_idx(); };

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
    const MetadataStore& meta_store() const { return meta_store_; }

    /** if it can be read by stream and in non-decreasing timestamp order. */
    bool has_stream_info() const;

   private:
    void read_metadata();
    void read_chunks_info();  // i.e. StreamingInfo.chunks[] information

    void print_metadata_entries();

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
    MessageRef(const uint8_t* buf, const MetadataStore& meta_provider)
        : buf_(buf), meta_provider_(meta_provider), chunk_buf_{nullptr} {}

    MessageRef(const uint8_t* buf, const MetadataStore& meta_provider,
               std::shared_ptr<std::vector<uint8_t>> chunk_buf)
        : buf_(buf), meta_provider_(meta_provider), chunk_buf_{chunk_buf} {}

    /** Message stream id */
    uint32_t id() const;

    /** Timestamp of the message */
    ts_t ts() const;

    /// @todo [pb] Type of the stored data (meta of the stream?)
    // std::string stream_type() const;

    /** Pointer to the underlying data */
    const uint8_t* buf() const { return buf_; }

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

    ChunkState* state() { return reader_->chunks_.get(chunk_offset_); }
    const ChunkState* state() const {
        return reader_->chunks_.get(chunk_offset_);
    }

    ChunkInfoNode* info() { return reader_->chunks_.get_info(chunk_offset_); }
    const ChunkInfoNode* info() const {
        return reader_->chunks_.get_info(chunk_offset_);
    }

    MessagesChunkIter begin() const;
    MessagesChunkIter end() const;

    const MessageRef operator[](size_t msg_idx) const;

    std::unique_ptr<const MessageRef> messages(size_t msg_idx) const;

    /** Debug string representation */
    std::string to_string() const;

    uint64_t offset() const { return chunk_offset_; }
    ts_t start_ts() const { return state()->start_ts; }
    ts_t end_ts() const { return state()->end_ts; }

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
        bool operator()(const opened_chunk_type& a,
                        const opened_chunk_type& b) {
            return a.first[a.second].ts() > b.first[b.second].ts();
        }
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

    void print_and_finish();

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
