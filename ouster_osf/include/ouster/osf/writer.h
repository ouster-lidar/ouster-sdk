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
    /**
     * Save a message to a specified stream.
     *
     * @param[in] stream_id The stream id to associate with the message.
     * @param[in] ts The timestamp for the messages.
     * @param[in] msg_buf A vector of message buffers to record.
     */
    virtual void saveMessage(const uint32_t stream_id, const ts_t ts,
                             const std::vector<uint8_t>& buf) = 0;

    /**
     * Finish the process of saving messages and write out the stream stats.
     */
    virtual void finish() = 0;

    /**
     * Get the chunksize
     *
     * @return the chunk size
     */
    virtual uint32_t chunk_size() const = 0;

    /**
     * Default deconstructor.
     */
    virtual ~ChunksWriter() = default;
};

/**
 * %OSF Writer provides the base universal interface to store the collection
 * of metadata entries, streams and corresponding objects.
 *
 * Examples:
 *  @ref writer_test.cpp, writer_custom_test.cpp
 */
class Writer {
   public:
    /**
     * @param[in] file_name The filename of the output OSF file.
     *
     * @note Will abort on a size mismatch.
     * @todo Figure out a better way of handling this rather than aborting.
     */
    explicit Writer(const std::string& file_name);

    /**
     * @copydoc Writer(const std::string& file_name)
     * @param[in] metadata_id The flatbuffer metadata id label to use.
     * @param[in] chunk_size The chunk size to use for the OSF file,
     *                       this argument is optional.
     */
    Writer(const std::string& file_name, const std::string& metadata_id,
           uint32_t chunk_size = 0);

    /**
     * Add metadata to the OSF file.
     *
     * @tparam MetaType The type of metadata to add.
     * @tparam MetaParams The type of meta parameters to add.
     *
     * @param[in] params The parameters to add.
     */
    template <typename MetaType, typename... MetaParams>
    uint32_t addMetadata(MetaParams&&... params) {
        MetaType entry(std::forward<MetaParams>(params)...);
        return meta_store_.add(entry);
    }

    /**
     * Adds a MetadataEntry to the OSF file.
     *
     * @param[in] entry The metadata entry to add to the OSF file.
     */
    uint32_t addMetadata(MetadataEntry&& entry);

    /**
     * @copydoc addMetadata(MetadataEntry&& entry)
     */
    uint32_t addMetadata(MetadataEntry& entry);

    /**
     * Get and return a metadata entry.
     * @param[in] metadata_id The id of the metadata to get and return.
     * @return The correct MetadataEntry.
     */
    std::shared_ptr<MetadataEntry> getMetadata(
        const uint32_t metadata_id) const;

    /**
     * @copydoc getMetadata(const uint32_t metadata_id)
     *
     * @tparam MetadataEntryClass The type of metadata to get and return.
     */
    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> getMetadata(
        const uint32_t metadata_id) const {
        return meta_store_.get<MetadataEntryClass>(metadata_id);
    }

    /**
     * Creating streams by passing itself as first argument of the ctor and
     * following the all other parameters.
     *
     * @tparam Stream The specified stream object type.
     * @tparam StreamParams The specified stream parameter types.
     *
     * @param[in] params The parameters to use when creating a stream.
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
     *
     * @note Will abort on non existent stream id.
     * @todo Figure out a better way of handling this rather than aborting.
     *
     * @param[in] stream_id The stream to save the message to.
     * @param[in] ts The timestamp to use for the message.
     * @param[in] buf The message to save in the form of a byte vector.
     */
    void saveMessage(const uint32_t stream_id, const ts_t ts,
                     const std::vector<uint8_t>& buf);

    /**
     * Returns the metadata store. This is used for getting the entire
     * set of flatbuffer metadata entries.
     *
     * @return The flatbuffer metadata entries.
     */
    const MetadataStore& meta_store() const;

    /**
     * Returns the metadata id label.
     *
     * @return The metadata id label.
     */
    const std::string& metadata_id() const;

    /**
     * Sets the metadata id label.
     *
     * @param[in] The metadata id label to set.
     */
    void setMetadataId(const std::string& id);

    /**
     * Return the filename for the OSF file.
     *
     * @return The filename for the OSF file.
     */
    const std::string& filename() const;

    /**
     * Get the specific chunks layout of the OSF file.
     *
     * @relates ChunksLayout
     *
     * @return The chunks layout of the OSF file.
     */
    ChunksLayout chunks_layout() const;

    /**
     * Get the chunk size used for the OSF file.
     *
     * @return The chunk size for the OSF file.
     */
    uint32_t chunk_size() const;

    /**
     * Writes buf to the file with CRC32 appended and return the number of
     * bytes writen to the file
     *
     * @note Will abort on bad file position.
     * @note Will abort on a closed writer object.
     * @todo Figure out a better way of handling this rather than aborting.
     *
     * @param[in] buf The buffer to append.
     * @param[in] size The size of the buffer to append.
     * @return The number of bytes writen to the OSF file.
     */
    uint64_t append(const uint8_t* buf, const uint64_t size);

    /**
     * Save a specified chunk to the OSF file.
     *
     * @note Will abort on a size mismatch.
     * @todo Figure out a better way of handling this rather than aborting.
     *
     * @param[in] start_ts The lowest timestamp in the chunk.
     * @param[in] end_ts The highest timestamp in the chunk.
     * @param[in] chunk_buf The byte vector representation of the chunk.
     * @return The result offset in the OSF file.
     */
    uint64_t emit_chunk(const ts_t start_ts, const ts_t end_ts,
                        const std::vector<uint8_t>& chunk_buf);

    /**
     * Finish file with a proper metadata object, and header.
     *
     * @note Will abort on a size mismatch.
     * @todo Figure out a better way of handling this rather than aborting.
     */
    void close();

    /**
     * @relates close
     */
    ~Writer();

    /**
     * Disallow copying and moving.
     */
    Writer(const Writer&) = delete;

    /**
     * Disallow copying and moving.
     */
    Writer& operator=(const Writer&) = delete;

    /**
     * Disallow copying and moving.
     */
    Writer(Writer&&) = delete;

    /**
     * Disallow copying and moving.
     */
    Writer& operator=(Writer&&) = delete;

   private:
    /**
     * Helper to construct the Metadata OSF Block at the end of writing.
     * This function takes the metadata entries from the metadata store
     * and generates a raw flatbuffer blob for writing to file.
     *
     * @return The completed raw flatbuffer byte vector for
     *         the metadata section.
     */
    std::vector<uint8_t> make_metadata() const;

    /**
     * Internal filename of the OSF file.
     */
    std::string file_name_;

    /**
     * The size of the flatbuffer header blob.
     */
    uint32_t header_size_{0};

    /**
     * The internal file offset.
     */
    int64_t pos_{-1};

    /**
     * Internal status flag for whether we have started writing or not.
     */
    bool started_{false};

    /**
     * Internal status flag for whether the file has been closed or not.
     *
     * @relates close
     */
    bool finished_{false};

    /**
     * The internal vector of chunks.
     */
    std::vector<ouster::osf::gen::ChunkOffset> chunks_{};

    /**
     * The lowest timestamp in the OSF file.
     */
    ts_t start_ts_{ts_t::max()};

    /**
     * The highest timestamp in the OSF file.
     */
    ts_t end_ts_{ts_t::min()};

    /**
     * Cache of the chunk offset.
     */
    uint64_t next_chunk_offset_{0};

    /**
     * The metadata id label.
     */
    std::string metadata_id_{};

    /**
     * The internal chunk layout of the OSF file.
     */
    ChunksLayout chunks_layout_{ChunksLayout::LAYOUT_STANDARD};

    /**
     * The store of metadata entries.
     */
    MetadataStore meta_store_{};

    /**
     * ChunksWriter is reponsible for chunking strategy.
     */
    std::shared_ptr<ChunksWriter> chunks_writer_{nullptr};
};

/**
 * Encapsulate chunk seriualization operations.
 */
class ChunkBuilder {
   public:
    ChunkBuilder(){};

    /**
     * Save messages to the serialized chunks.
     *
     * @note Will abort on a size mismatch.
     * @todo Figure out a better way of handling this rather than aborting.
     *
     * @param[in] stream_id The stream to save the message to.
     * @param[in] ts The timestamp to use for the message.
     * @param[in] msg_buf The message to save in the form of a byte vector.
     */
    void saveMessage(const uint32_t stream_id, const ts_t ts,
                     const std::vector<uint8_t>& msg_buf);

    /**
     * Completely wipe all data and start the chunk anew.
     */
    void reset();

    /**
     * Finish out the serialization of the chunk and return the raw
     * flatbuffer output.
     *
     * @return The serialized chunk in a raw flatbuffer byte vector.
     */
    std::vector<uint8_t> finish();

    /**
     * Returns the flatbufferbuilder size.
     *
     * @return The flatbufferbuilder size.
     */
    uint32_t size() const;

    /**
     * Returns the number of messages saved so far.
     *
     * @return The number of messages saved so far.
     */
    uint32_t messages_count() const;

    /**
     * The lowest timestamp in the chunk.
     */
    ts_t start_ts() const;

    /**
     * The highest timestamp in the chunk.
     */
    ts_t end_ts() const;

   private:
    /**
     * Internal method for updating the corret start and end
     * timestamps.
     *
     * @param[in] ts The timestamp to check against for start and end.
     */
    void update_start_end(const ts_t ts);

    /**
     * Internal status flag for whether the builder is finished or not.
     */
    bool finished_{false};

    /**
     * Internal FlatBufferBuilder object used for the serialization.
     */
    flatbuffers::FlatBufferBuilder fbb_{0x7fff};

    /**
     * The lowest timestamp in the chunk.
     */
    ts_t start_ts_{ts_t::max()};

    /**
     * The highest timestamp in the chunk.
     */
    ts_t end_ts_{ts_t::min()};

    /**
     * Internal store of messages to be contained within the chunk
     */
    std::vector<flatbuffers::Offset<gen::StampedMessage>> messages_{};
};

}  // namespace osf
}  // namespace ouster
