/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file writer.h
 * @brief OSF file Writer
 *
 */
#pragma once

#include <string>

#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace osf {

/**
 * Chunks writing strategy that decides when and how exactly write chunks
 * to a file. See RFC 0018 for Standard and Streaming Layout description.
 */
class ChunksWriter {
   public:
    virtual void saveMessage(const uint32_t stream_id, const ts_t ts,
                             const std::vector<uint8_t>& buf) = 0;
    virtual void finish() = 0;
    virtual uint32_t chunk_size() const = 0;
    virtual ~ChunksWriter() = default;
};

/**
 * %OSF Writer provides the base universal interface to store the collection
 * of metadata entries, streams and corresponding objects.
 *
 * Examples:
 *  @ref writer_test.cpp, writer_custom_test.cpp
 *
 */
class Writer {
   public:
    explicit Writer(const std::string& file_name);

    Writer(const std::string& file_name, const std::string& metadata_id,
           uint32_t chunk_size = 0);

    template <typename MetaType, typename... MetaParams>
    uint32_t addMetadata(MetaParams&&... params) {
        MetaType entry(std::forward<MetaParams>(params)...);
        return meta_store_.add(entry);
    }

    uint32_t addMetadata(MetadataEntry&& entry) { return addMetadata(entry); }

    uint32_t addMetadata(MetadataEntry& entry) {
        return meta_store_.add(entry);
    }

    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> getMetadata(
        const uint32_t metadata_id) const {
        return meta_store_.get<MetadataEntryClass>(metadata_id);
    }

    std::shared_ptr<MetadataEntry> getMetadata(
        const uint32_t metadata_id) const {
        return meta_store_.get(metadata_id);
    }

    /**
     * Creating streams by passing itself as first argument of the ctor and
     * following the all other parameters.
     */
    template <typename Stream, typename... StreamParams>
    Stream createStream(StreamParams&&... params) {
        return Stream(*this, std::forward<StreamParams>(params)...);
    }

    /**
     * %Writer accepts messages in the form of bytes buffers with linked meta_id
     * and timestamp.
     * @todo [pb]: It should be hidden into private/protected, but I don't see
     * yet how to do it and give an access to every derived Stream objects.
     */
    void saveMessage(const uint32_t stream_id, const ts_t ts,
                     const std::vector<uint8_t>& buf);

    const MetadataStore& meta_store() const { return meta_store_; }

    const std::string& metadata_id() const { return metadata_id_; }
    void setMetadataId(const std::string& id) { metadata_id_ = id; }

    const std::string& filename() const { return file_name_; }

    ChunksLayout chunks_layout() const { return chunks_layout_; }
    uint32_t chunk_size() const;

    // writes buf to the file with CRC32 appended and return the number of
    // bytes writen to the file
    uint64_t append(const uint8_t* buf, const uint64_t size);

    uint64_t emit_chunk(const ts_t start_ts, const ts_t end_ts,
                        const std::vector<uint8_t>& chunk_buf);

    /** Finish file with a proper metadata object, and header */
    void close();

    ~Writer();

    // copy/move = delete everything
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    Writer(Writer&&) = delete;
    Writer& operator=(Writer&&) = delete;

   private:
    // helper to construct the Metadata OSF Block at the end of writing
    std::vector<uint8_t> make_metadata() const;

    std::string file_name_;

    uint32_t header_size_{0};
    int64_t pos_{-1};
    bool started_{false};
    bool finished_{false};

    std::vector<ouster::osf::gen::ChunkOffset> chunks_{};
    ts_t start_ts_{ts_t::max()};
    ts_t end_ts_{ts_t::min()};
    uint64_t next_chunk_offset_{0};

    std::string metadata_id_{};

    ChunksLayout chunks_layout_{ChunksLayout::LAYOUT_STANDARD};

    MetadataStore meta_store_{};

    // ChunksWriter is reponsible for chunking strategy
    std::shared_ptr<ChunksWriter> chunks_writer_{nullptr};
};

class ChunkBuilder {
   public:
    ChunkBuilder(){};
    void saveMessage(const uint32_t stream_id, const ts_t ts,
                     const std::vector<uint8_t>& msg_buf);
    void reset();
    std::vector<uint8_t> finish();
    uint32_t size() const;
    uint32_t messages_count() const;
    ts_t start_ts() const { return start_ts_; }
    ts_t end_ts() const { return end_ts_; }

   private:
    void update_start_end(const ts_t ts);

    bool finished_{false};

    flatbuffers::FlatBufferBuilder fbb_{0x7fff};
    ts_t start_ts_{ts_t::max()};
    ts_t end_ts_{ts_t::min()};
    std::vector<flatbuffers::Offset<gen::StampedMessage>> messages_{};
};

}  // namespace osf
}  // namespace ouster