/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file writer.h
 * @brief OSF file Writer
 */
#pragma once

#include <string>

#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace osf {

class LidarScanStream;

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
     * @param[in] buf A vector of message buffers to record.
     */
    virtual void save_message(const uint32_t stream_id, const ts_t ts,
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
 */
class Writer {
    friend class StreamingLayoutCW;

   public:
    /**
     * @throws std::runtime_error Exception on file writing issues.
     *
     * @param[in] file_name The filename of the output OSF file.
     * @param[in] chunk_size The chunk size in bytes to use for the OSF file.
     *     This argument is optional, and if not provided the default value of
     *     2MB is used. If the current chunk being written exceeds
     *     the chunk_size, a new chunk will be started on the next call to
     *     save. This allows an application to tune the number of messages (e.g.
     *     lidar scans) per chunk, which affects the granularity of the message
     *     index stored in the StreamingInfo in the file metadata. A smaller
     *     chunk_size means more messages are indexed and a larger number of
     *     index entries. A more granular index allows for more precise
     *     seeking at the slight expense of a larger file.
     */
    Writer(const std::string& file_name, uint32_t chunk_size = 0);

    /**
     * @param[in] filename The filename to output to.
     * @param[in] info The sensor info to use for a single stream OSF file.
     * @param[in] chunk_size The chunksize to use for the OSF file, this
     *                       parameter is optional.
     * @param[in] field_types The fields from scans to actually save into the
     *                        OSF. If not provided uses the fields from the
     *                        first saved lidar scan for each stream. This
     *                        parameter is optional.
     */
    Writer(const std::string& filename, const ouster::sensor::sensor_info& info,
           const LidarScanFieldTypes& field_types = LidarScanFieldTypes(),
           uint32_t chunk_size = 0);

    /**
     * @param[in] filename The filename to output to.
     * @param[in] info The sensor info vector to use for a multi stream OSF
     *                 file.
     * @param[in] chunk_size The chunksize to use for the OSF file, this
     *                       parameter is optional.
     * @param[in] field_types The fields from scans to actually save into the
     *                        OSF. If not provided uses the fields from the
     *                        first saved lidar scan for each stream. This
     *                        parameter is optional.
     */
    Writer(const std::string& filename,
           const std::vector<ouster::sensor::sensor_info>& info,
           const LidarScanFieldTypes& field_types = LidarScanFieldTypes(),
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
    uint32_t add_metadata(MetaParams&&... params) {
        MetaType entry(std::forward<MetaParams>(params)...);
        return meta_store_.add(entry);
    }

    /**
     * Adds a MetadataEntry to the OSF file.
     *
     * @param[in] entry The metadata entry to add to the OSF file.
     */
    uint32_t add_metadata(MetadataEntry&& entry);

    /**
     * @copydoc add_metadata(MetadataEntry&& entry)
     */
    uint32_t add_metadata(MetadataEntry& entry);

    /**
     * @defgroup OSFGetMetadataGroup Get specified metadata.
     * Get and return a metadata entry.
     *
     * @param[in] metadata_id The id of the metadata to get and return.
     * @return The correct MetadataEntry.
     */

    /**
     * @copydoc OSFGetMetadataGroup
     */
    std::shared_ptr<MetadataEntry> get_metadata(
        const uint32_t metadata_id) const;

    /**
     * @copydoc OSFGetMetadataGroup
     *
     * @tparam MetadataEntryClass The type of metadata to get and return.
     */
    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> get_metadata(
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
    Stream create_stream(StreamParams&&... params) {
        return Stream(*this, std::forward<StreamParams>(params)...);
    }

    /**
     * %Writer accepts messages in the form of bytes buffers with linked meta_id
     * and timestamp.
     * @todo [pb]: It should be hidden into private/protected, but I don't see
     * yet how to do it and give an access to every derived Stream objects.
     *
     * @throws std::logic_error Exception on non existent stream id.
     *
     * @param[in] stream_id The stream to save the message to.
     * @param[in] ts The timestamp to use for the message.
     * @param[in] buf The message to save in the form of a byte vector.
     */
    void save_message(const uint32_t stream_id, const ts_t ts,
                      const std::vector<uint8_t>& buf);

    /**
     * Adds info about a sensor to the OSF and returns the stream index to
     * to write scans to it's stream.
     *
     * @param[in] info The info of the sensor to add to the file.
     * @param[in] field_types The fields from scans to actually save into the
     *                        OSF. If not provided uses the fields from the
     *                        first saved lidar scan for this sensor. This
     *                        parameter is optional.
     *
     * @return The stream index for the newly added sensor.
     */
    uint32_t add_sensor(
        const ouster::sensor::sensor_info& info,
        const LidarScanFieldTypes& field_types = LidarScanFieldTypes());

    /**
     * Save a single scan to the specified stream_index in an OSF file.
     * The concept of the stream_index is related to the sensor_info vector.
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     LidarScan scan = RANDOM_SCAN_HERE;

     // To save the LidarScan of scan to the first sensor, you would do the
     // following
     output.save(0, scan);

     // To save the LidarScan of scan to the second sensor, you would do the
     // following
     output.save(1, scan);

     // To save the LidarScan of scan to the third sensor, you would do the
     // following
     output.save(2, scan);
     @endcode
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The index of the corrosponding sensor_info to
     *                         use.
     * @param[in] scan The scan to save.
     */
    void save(uint32_t stream_index, const LidarScan& scan);

    /**
     * Save a single scan to the specified stream_index in an OSF file indexed
     * with the provided timestamp.
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The index of the corrosponding sensor_info to
     *                         use.
     * @param[in] scan The scan to save.
     * @param[in] timestamp Timestamp to index this scan with.
     */
    void save(uint32_t stream_index, const LidarScan& scan,
              const ouster::osf::ts_t timestamp);

    /**
     * Save multiple scans in an OSF file.
     * The concept of the stream_index is related to the sensor_info vector.
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     LidarScan sensor1_scan = RANDOM_SCAN_HERE;
     LidarScan sensor2_scan = RANDOM_SCAN_HERE;
     LidarScan sensor3_scan = RANDOM_SCAN_HERE;

     // To save the scans matched appropriately to their sensors, you would do
     // the following
     output.save({sensor1_scan, sensor2_scan, sensor3_scan});
     @endcode
     *
     *
     * @throws std::logic_error Will throw exception on writer being closed
     *
     * @param[in] scans The vector of scans to save.
     */
    void save(const std::vector<LidarScan>& scans);

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
     * @param[in] id The metadata id label to set.
     */
    void set_metadata_id(const std::string& id);

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
     * Return the sensor info vector.
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     // The following will be true
     output.sensor_info() == std::vector<sensor_info>{info1, info2, info3};
     @endcode
     *
     * @return The sensor info vector.
     */
    const std::vector<ouster::sensor::sensor_info>& sensor_info() const;

    /**
     * Get the specified sensor info
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     // The following will be true
     output.sensor_info(0) == info1;
     output.sensor_info(1) == info2;
     output.sensor_info(2) == info3;
     @endcode
     *
     * @param[in] stream_index The sensor info to return.
     * @return The correct sensor info.
     */
    const ouster::sensor::sensor_info sensor_info(int stream_index) const;

    /**
     * Get the number of sensor_info objects.
     *
     * @return The sensor_info count.
     */
    uint32_t sensor_info_count() const;

    /**
     * Finish file with a proper metadata object, and header.
     */
    void close();

    /**
     * Returns if the writer is closed or not.
     *
     * @return If the writer is closed or not.
     */
    inline bool is_closed() const { return finished_; }

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
     * Internal method used to save a scan to a specified stream_index
     * specified stream. This method is here so that we can bypass
     * is_closed checking for speed sake. The calling functions will
     * do the check for us.
     *
     * @param[in] stream_index The stream to save to.
     * @param[in] scan The scan to save.
     * @param[in] time Timestamp to use to index scan.
     */
    void _save(uint32_t stream_index, const LidarScan& scan, const ts_t time);

    /**
     * Writes buf to the file with CRC32 appended and return the number of
     * bytes writen to the file
     *
     * @throws std::logic_error Exception on bad file position.
     * @throws std::logic_error Exception on a closed writer object.
     *
     * @param[in] buf The buffer to append.
     * @param[in] size The size of the buffer to append.
     * @return The number of bytes writen to the OSF file.
     */
    uint64_t append(const uint8_t* buf, const uint64_t size);

    /**
     * Save a specified chunk to the OSF file.
     *
     * @throws std::logic_error Exception on a size mismatch
     *
     * @param[in] start_ts The lowest timestamp in the chunk.
     * @param[in] end_ts The highest timestamp in the chunk.
     * @param[in] chunk_buf The byte vector representation of the chunk.
     * @return The result offset in the OSF file.
     */
    uint64_t emit_chunk(const ts_t start_ts, const ts_t end_ts,
                        const std::vector<uint8_t>& chunk_buf);

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

    /**
     * Internal store of field types to serialize for lidar scans
     */
    std::vector<LidarScanFieldTypes> field_types_;

    /**
     * Internal stream index to metadata map.
     */
    std::map<uint32_t, uint32_t> lidar_meta_id_;

    /**
     * Internal stream index to LidarScanStream map.
     */
    std::map<uint32_t, std::unique_ptr<ouster::osf::LidarScanStream>>
        lidar_streams_;

    /**
     * The internal sensor_info store ordered by stream_index.
     */
    std::vector<ouster::sensor::sensor_info> sensor_info_;
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
     * @throws std::logic_error Exception on a size mismatch
     *
     * @param[in] stream_id The stream to save the message to.
     * @param[in] ts The timestamp to use for the message.
     * @param[in] msg_buf The message to save in the form of a byte vector.
     */
    void save_message(const uint32_t stream_id, const ts_t ts,
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
