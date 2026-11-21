/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file writer.h
 * @brief OSF file Writer
 */
#pragma once

#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/frame_set_source.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/osf_encoder.h"

using ouster::sdk::core::FrameSetSourceMetadataSet;

namespace ouster {
namespace sdk {
namespace osf {

class LidarFrameStream;
class CameraFrameStream;
class CollationStream;
class SensorInfoStream;
class FrameSetSourceMetadataStream;

/**
 * Error thrown by ChunksWriter on receiving a message with decreasing timestamp
 */
class OUSTER_API_CLASS OsfDropFrameError : public std::logic_error {
   public:
    /**
     * Constructor
     *
     * @param[in] what error message
     */
    OUSTER_API_FUNCTION
    OsfDropFrameError(const std::string& what) : std::logic_error(what) {}
};

/**
 * Chunks writing strategy that decides when and how exactly write chunks
 * to a file. See RFC 0018 for Standard and Streaming Layout description.
 */
class OUSTER_API_CLASS ChunksWriter {
   public:
    /**
     * Save a message to a specified stream.
     *
     * @param[in] stream_id The stream id to associate with the message.
     * @param[in] receive_ts The receive timestamp for the messages.
     * @param[in] sensor_ts The sensor timestamp for the messages.
     * @param[in] buf A vector of message buffers to record.
     * @param[in] type Message type string of the message being saved.
     */
    OUSTER_API_FUNCTION
    virtual void save_message(const uint32_t stream_id, const ts_t receive_ts, const ts_t sensor_ts,
                              const std::vector<uint8_t>& buf, const std::string& type) = 0;

    /**
     * Finish the process of saving messages and write out the stream stats.
     */
    OUSTER_API_FUNCTION
    virtual void finish() = 0;

    /**
     * Flush the latest chunk on this stream to disk.
     * @param[in] stream_id id of the stream to flush
     */
    OUSTER_API_FUNCTION
    virtual void flush(uint32_t stream_id) = 0;

    /**
     * Get the chunksize
     *
     * @return the chunk size
     */
    OUSTER_API_FUNCTION
    virtual uint32_t chunk_size() const = 0;

    /**
     * Default destructor.
     */
    OUSTER_API_FUNCTION
    virtual ~ChunksWriter() = default;
};

namespace impl {
class WriterImpl;
}  // namespace impl
/**
 * @class Writer
 * @brief OSF Writer provides the base universal interface to store the
 * collection of metadata entries, streams, and corresponding objects. Detailed
 * description of the class follows here.
 *
 */
class OUSTER_API_CLASS Writer {
    friend class StreamingLayoutCW;

   public:
    /**
     * @throws std::runtime_error Exception on file writing issues.
     *
     * @param[in] file_name The file name of the output OSF file.
     * @param[in] chunk_size The chunk size in bytes to use for the OSF file.
     *     This argument is optional, and if not provided the default value of
     *     2MB is used. If the current chunk being written exceeds
     *     the chunk_size, a new chunk will be started on the next call to
     *     save. This allows an application to tune the number of messages (e.g.
     *     lidar frames) per chunk, which affects the granularity of the message
     *     index stored in the StreamingInfo in the file metadata. A smaller
     *     chunk_size means more messages are indexed and a larger number of
     *     index entries. A more granular index allows for more precise
     *     seeking at the slight expense of a larger file.
     */
    OUSTER_API_FUNCTION
    Writer(std::string file_name, uint32_t chunk_size = 0);

    /**
     * @param[in] filename The file name to output to.
     * @param[in] info The sensor info to use for a single stream OSF file.
     * @param[in] fields_to_write The fields from frames to actually save into
     *                            the OSF. If not provided uses the fields from
     *                            the first saved lidar frame for this sensor.
     *                            This parameter is optional.
     * @param[in] chunk_size The chunksize to use for the OSF file, this
     *                       parameter is optional.
     * @param[in] encoder An optional Encoder instance for configuring how the
     *                            Writer should encode the OSF.
     */
    OUSTER_API_FUNCTION
    Writer(const std::string& filename, const ouster::sdk::core::LidarInfo& info,
           const std::vector<std::string>& fields_to_write = std::vector<std::string>(),
           uint32_t chunk_size = 0, std::shared_ptr<Encoder> encoder = nullptr);

    /**
     * @param[in] filename The file name to output to.
     * @param[in] info The sensor info vector to use for a multi stream OSF
     *                 file.
     * @param[in] chunk_size The chunksize to use for the OSF file, this
     *                       parameter is optional.
     * @param[in] fields_to_write The fields from frames to actually save into
     *                            the OSF. If not provided uses the fields from
     *                            the first saved lidar frame for this sensor.
     *                            This parameter is optional.
     * @param[in] encoder An optional Encoder instance for configuring how the
     *                            Writer should encode the OSF.
     */
    OUSTER_API_FUNCTION
    Writer(const std::string& filename, const std::vector<ouster::sdk::core::LidarInfo>& info,
           const std::vector<std::string>& fields_to_write = std::vector<std::string>(),
           uint32_t chunk_size = 0, std::shared_ptr<Encoder> encoder = nullptr);

    /**
     * @param[in] filename The file name to output to.
     * @param[in] info The sensor info vector to use for a multi stream OSF
     *                 file. Each entry may be a LidarInfo or CameraInfo; the
     *                 corresponding lidar or camera stream is created in the
     *                 order the infos are provided.
     * @param[in] fields_to_write The fields from frames to actually save into
     *                            the OSF. If not provided uses the fields from
     *                            the first saved lidar frame for this sensor.
     *                            This parameter is optional.
     * @param[in] chunk_size The chunksize to use for the OSF file, this
     *                       parameter is optional.
     * @param[in] encoder An optional Encoder instance for configuring how the
     *                            Writer should encode the OSF.
     */
    OUSTER_API_FUNCTION
    Writer(const std::string& filename,
           const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& info,
           const std::vector<std::string>& fields_to_write = std::vector<std::string>(),
           uint32_t chunk_size = 0, std::shared_ptr<Encoder> encoder = nullptr);

    /**
     * Add metadata to the OSF file.
     *
     * @tparam MetaType The type of metadata to add.
     * @tparam MetaParams The type of meta parameters to add.
     *
     * @param[in] params The parameters to add.
     *
     * @return The corresponding lidar id of the metadata entry.
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
     *
     * @return The corresponding lidar id of the metadata entry
     */
    OUSTER_API_FUNCTION
    uint32_t add_metadata(MetadataEntry&& entry);

    /**
     * @copydoc add_metadata(MetadataEntry&& entry)
     */
    OUSTER_API_FUNCTION
    uint32_t add_metadata(MetadataEntry& entry);

    /**
     * Save a set of frame set source metadata to the OSF file.
     *
     * @param[in] frame_set_source_metadata_set The set of metadata entries to
     * save.
     */
    OUSTER_API_FUNCTION
    void save(const FrameSetSourceMetadataSet& frame_set_source_metadata_set);

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
    OUSTER_API_FUNCTION
    std::shared_ptr<MetadataEntry> get_metadata(const uint32_t metadata_id) const;

    /**
     * @copydoc OSFGetMetadataGroup
     *
     * @tparam MetadataEntryClass The type of metadata to get and return.
     */
    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> get_metadata(uint32_t metadata_id) const {
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
     *
     * @return Stream object created.
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
     * @param[in] receive_ts The receive timestamp to use for the message.
     * @param[in] sensor_ts The sensor timestamp to use for the message.
     * @param[in] buf The message to save in the form of a byte vector.
     * @param[in] type Message type string of the message being saved.
     */
    OUSTER_API_FUNCTION
    void save_message(const uint32_t stream_id, const ts_t receive_ts, const ts_t sensor_ts,
                      const std::vector<uint8_t>& buf, const std::string& type);

    /**
     * Adds info about a sensor to the OSF and returns the stream index to
     * write frames to its stream.
     *
     * @param[in] info The info of the sensor to add to the file.
     * @param[in] fields_to_write The fields from frames to actually save into
     *                            the OSF. If not provided uses the fields from
     *                            the first saved lidar frame for this sensor.
     *                            This parameter is optional.
     *
     * @return The unified stream index for the newly added sensor (its position
     *         in registration order; see save()).
     */
    OUSTER_API_FUNCTION
    uint32_t add_sensor(
        const ouster::sdk::core::LidarInfo& info,
        const std::vector<std::string>& fields_to_write = std::vector<std::string>());

    /**
     * Adds a camera to the OSF and returns the unified stream index used to
     * address it in subsequent save() calls. The CameraInfo is written to the
     * OSF footer via a CameraSensor metadata entry and the created
     * CameraFrameStream references that entry by id.
     *
     * @param[in] info The CameraInfo describing the camera.
     * @param[in] fields_to_write The fields from camera frames to actually save
     *                            into the OSF. If not provided, all fields on
     *                            each saved camera frame are written. This
     *                            parameter is optional.
     * @return The unified stream index for the newly added camera (its position
     *         in registration order; see save()).
     */
    OUSTER_API_FUNCTION
    uint32_t add_sensor(
        const ouster::sdk::core::CameraInfo& info,
        const std::vector<std::string>& fields_to_write = std::vector<std::string>());

    /**
     * Save a single frame to the specified stream_index in an OSF
     * file.
     *
     * The frame may be a LidarFrame or a CameraFrame; the correct stream type
     * is selected based on the concrete frame type. The stream_index is the
     * unified stream index: the sensor's position in registration order (the
     * order sensors were passed to the constructor and/or added via
     * add_sensor()), which is also the order exposed by
     * FrameSetSource::sensor_info() on read. Lidar and camera streams share this
     * single index space, so a save() to slot i writes to whichever sensor
     * (lidar or camera) was registered at position i; the frame's concrete type
     * must match that slot's kind.
     *
     * Consider the following:
     @code{.cpp}
     LidarInfo lidar1;   // registered at index 0
     CameraInfo camera;  // registered at index 1
     LidarInfo lidar2;   // registered at index 2

     Writer output = Writer(filename, {lidar1, camera, lidar2});

     // Save a LidarFrame to lidar1 (index 0):
     output.save(0, lidar_frame);

     // Save a CameraFrame to camera (index 1):
     output.save(1, camera_frame);

     // Save a LidarFrame to lidar2 (index 2):
     output.save(2, lidar_frame);
     @endcode
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     * @throws std::invalid_argument Will throw exception on an unsupported
     *                               frame type, or if the frame's kind does not
     *                               match the sensor registered at stream_index.
     *
     * @param[in] stream_index The unified stream index (registration order) of
     *                         the sensor to write this frame to.
     * @param[in] frame The frame to save.
     */
    OUSTER_API_FUNCTION
    void save(uint32_t stream_index, const ouster::sdk::core::Frame& frame);

    /**
     * Save a single frame with the specified timestamp to the
     * specified stream_index in an OSF file.
     *
     * The frame may be a LidarFrame or a CameraFrame; the correct stream type
     * is selected based on the concrete frame type. See the other save()
     * overload for the meaning of stream_index (unified registration order).
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     * @throws std::invalid_argument Will throw exception on an unsupported
     *                               frame type, or if the frame's kind does not
     *                               match the sensor registered at stream_index.
     *
     * @param[in] stream_index The unified stream index (registration order) of
     *                         the sensor to write this frame to.
     * @param[in] frame The frame to save.
     * @param[in] timestamp Receive timestamp to index this frame with.
     */
    OUSTER_API_FUNCTION
    void save(uint32_t stream_index, const ouster::sdk::core::Frame& frame,
              ouster::sdk::osf::ts_t timestamp);

    /**
     * Save collation of frames to the OSF file.
     *
     * The concept of the stream_index is related to the LidarInfo vector.
     * Consider the following:
     @code{.cpp}
     LidarInfo info1; // The first sensor in this OSF file
     LidarInfo info2; // The second sensor in this OSF file
     LidarInfo info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     LidarFrame sensor1_frame = RANDOM_FRAME_HERE;
     LidarFrame sensor2_frame = RANDOM_FRAME_HERE;
     LidarFrame sensor3_frame = RANDOM_FRAME_HERE;

     // To save the frames matched appropriately to their sensors, you would do
     // the following.
     FrameSet collation({sensor1_frame, sensor2_frame, sensor3_frame});

     // Collation also optionally allows saving custom fields directly to it
     // which are saved directly to osf
     collation.add_field("my_field", fd_array<double>(100, 200));

     output.save(collation);
     @endcode
     *
     * @throws std::logic_error Will throw exception on writer being closed
     * @throws std::logic_error Will throw exception on collation size not
     *                          matching the number of sensor infos
     * @throws OsfDropFrameError Will throw exception on missing timestamps
     *
     * @param[in] collation The FrameSet to save
     */
    OUSTER_API_FUNCTION
    void save(const ouster::sdk::core::FrameSet& collation);

    /**
     * Returns the metadata store. This is used for getting the entire
     * set of flatbuffer metadata entries.
     *
     * @return The flatbuffer metadata entries.
     */
    OUSTER_API_FUNCTION
    const MetadataStore& meta_store() const;

    /**
     * Returns the metadata id label.
     *
     * @return The metadata id label.
     */
    OUSTER_API_FUNCTION
    const std::string& metadata_id() const;

    /**
     * Sets the metadata id label.
     *
     * @param[in] id The metadata id label to set.
     */
    OUSTER_API_FUNCTION
    void set_metadata_id(const std::string& id);

    /**
     * Return the filename for the OSF file.
     *
     * @return The file name for the OSF file.
     */
    OUSTER_API_FUNCTION
    const std::string& filename() const;

    /**
     * Get the specific chunks layout of the OSF file.
     *
     * @relates ChunksLayout
     *
     * @return The chunks layout of the OSF file.
     */
    OUSTER_API_FUNCTION
    ChunksLayout chunks_layout() const;

    /**
     * Get the chunk size used for the OSF file.
     *
     * @return The chunk size for the OSF file.
     */
    OUSTER_API_FUNCTION
    uint32_t chunk_size() const;

    /**
     * Finish file with a proper metadata object, and header.
     * This method blocks until all remaining tasks generated by save() have
     * been finalized.
     * @param[in] fsync If true, force all writes on this file to disk.
     */
    OUSTER_API_FUNCTION
    void close(bool fsync = false);

    /**
     * Returns if the writer is closed or not.
     *
     * @return If the writer is closed or not.
     */
    OUSTER_API_FUNCTION
    inline bool is_closed() const {
        return finished_;
    }

    /**
     * Returns the Encoder object used by this Writer.
     *
     * @return the Encoder.
     */
    OUSTER_API_FUNCTION
    Encoder& encoder() const {
        return *encoder_;
    }

    /**
     * @relates close
     */
    OUSTER_API_FUNCTION
    ~Writer();

    /**
     * Disallow copying and moving.
     */
    OUSTER_API_FUNCTION
    Writer(const Writer&) = delete;

    /**
     * Disallow copying and moving.
     */
    OUSTER_API_FUNCTION
    Writer& operator=(const Writer&) = delete;

    /**
     * Disallow copying and moving.
     */
    OUSTER_API_FUNCTION
    Writer(Writer&&) = delete;

    /**
     * Disallow copying and moving.
     */
    OUSTER_API_FUNCTION
    Writer& operator=(Writer&&) = delete;

   protected:
    std::unique_ptr<impl::WriterImpl> impl_;

   private:
    /**
     * A runnable used to handle writes in the thread 'save_thread_'.
     */
    void save_thread_method();
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
     * Internal method used to save a frame to a specified stream_index
     * specified stream. This method is here so that we can bypass
     * is_closed checking for speed sake. The calling functions will
     * do the check for us.
     *
     * @param[in] stream_index The stream to save to.
     * @param[in] frame The frame to save.
     * @param[in] time Timestamp to use to index frame.
     */
    void save_internal(uint32_t stream_index, const ouster::sdk::core::LidarFrame& frame,
                       const ts_t time);

    void save_camera_internal(uint32_t camera_stream_index,
                              const ouster::sdk::core::CameraFrame& frame, ts_t time);

    /**
     * Duplicate a sensor's metadata into the data section, so that it can be
     * recovered from a file whose metadata footer never reached disk (e.g. the
     * writing process died mid-recording). Written once per stream, on the
     * first frame saved to it, for both lidars and cameras.
     *
     * @param[in] sensor_info The LidarInfo/CameraInfo to duplicate.
     * @param[in] sensor_meta_id Id of the LidarSensor/CameraSensor footer entry
     *                           holding this metadata.
     * @param[in] frame_stream_id Id of the frame stream the metadata describes.
     * @param[in] time Timestamp to index the message by, if it is the first.
     */
    void save_sensor_info_message(std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info,
                                  uint32_t sensor_meta_id, uint32_t frame_stream_id, ts_t time);

    /**
     * Save a frame addressed by its unified (registration-order) stream index.
     * Translates the unified index to the concrete per-type stream, validates
     * that the frame kind matches the registered slot, and dispatches to
     * save_internal()/save_camera_internal(). Shared by the public save()
     * overloads.
     *
     * @param[in] stream_index Unified stream index (into stream_order_).
     * @param[in] frame The frame to save.
     * @param[in] time Timestamp to use to index the frame.
     */
    void save_frame_to_slot(uint32_t stream_index, const ouster::sdk::core::Frame& frame,
                            ts_t time);

    void ensure_lidar_stream(uint32_t stream_index, const ouster::sdk::core::LidarFrame& frame,
                             const ts_t time);

    void register_frame_field_types(uint32_t stream_index,
                                    const ouster::sdk::core::LidarFrame& frame);

    void validate_expected_fields(uint32_t stream_index,
                                  const ouster::sdk::core::LidarFrame& frame);

    /**
     * Writes buf to the file with CRC32 appended and return the number of
     * bytes written to the file
     *
     * @throws std::logic_error Exception on bad file position.
     * @throws std::logic_error Exception on a closed writer object.
     *
     * @param[in] buf The buffer to append.
     * @param[in] size The size of the buffer to append.
     * @return The number of bytes written to the OSF file.
     */
    uint64_t append(const uint8_t* buf, uint64_t size);

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
    uint64_t emit_chunk(ts_t start_ts, ts_t end_ts, const std::vector<uint8_t>& chunk_buf);

    /**
     * Internal filename of the OSF file.
     */
    std::string filename_;

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
     * The lowest timestamp in the OSF file.
     */
    ts_t start_ts_{ts_t::max()};

    /**
     * The highest timestamp in the OSF file.
     */
    ts_t end_ts_{ts_t::min()};

    /**
     * The timestamp we save all LidarInfo messages at.
     * Determined by time of first saved frame.
     */
    ts_t info_ts_{ts_t::min()};

    /**
     * Cache of the chunk offset.
     */
    uint64_t next_chunk_offset_{0};

    /**
     * The metadata id label.
     */
    std::string metadata_id_{};

    /**
     * The store of metadata entries.
     */
    MetadataStore meta_store_{};

    /**
     * ChunksWriter is responsible for chunking strategy.
     */
    std::shared_ptr<ChunksWriter> chunks_writer_{nullptr};

    /**
     * Internal store of field types to serialize for lidar frames
     */
    std::vector<ouster::sdk::core::LidarFrameFieldTypes> field_types_;

    /**
     * Internal store of what fields the user wants to save from each frame.
     */
    std::vector<std::vector<std::string>> desired_fields_;

    /**
     * Internal stream index to metadata map.
     */
    std::map<uint32_t, uint32_t> lidar_meta_id_;

    /**
     * Internal stream index to LidarFrameStream map.
     */
    std::map<uint32_t, std::unique_ptr<ouster::sdk::osf::LidarFrameStream>> lidar_streams_;

    std::unique_ptr<ouster::sdk::osf::SensorInfoStream> sensor_info_stream_;
    std::unique_ptr<ouster::sdk::osf::FrameSetSourceMetadataStream>
        frame_set_source_metadata_stream_;

    /**
     * Internal CollationStream.
     */
    std::unique_ptr<ouster::sdk::osf::CollationStream> collation_stream_;

    /**
     * Camera stream index to CameraFrameStream metadata id map.
     */
    std::map<uint32_t, uint32_t> camera_meta_id_;

    /**
     * Camera stream index to CameraFrameStream map.
     */
    std::map<uint32_t, std::unique_ptr<ouster::sdk::osf::CameraFrameStream>> camera_streams_;

    /**
     * Camera stream indices whose CameraInfo has already been duplicated into
     * the data section for recovery. Cameras register their stream up front in
     * add_sensor(), so unlike lidars the stream's own presence can't be used to
     * tell whether that has happened yet.
     */
    std::set<uint32_t> camera_info_saved_;

    /**
     * Describes a single registered stream slot in registration order.
     */
    struct StreamSlot {
        bool is_camera;  ///< true for a camera stream, false for a lidar stream
        uint32_t index;  ///< index into the per-type stream map (lidar or camera)
    };

    /**
     * Registration order of sensor streams (as passed to the constructor or
     * added via add_sensor()), lidar and camera possibly
     * interleaved. save(FrameSet) uses this to map each position in a collation
     * to its registered stream, so the collation's frame order only needs to
     * match the order sensors were registered - not "all lidar then all camera".
     */
    std::vector<StreamSlot> stream_order_;

    // TODO[tws] make private, access from LidarFrameStream via "friend class"
    std::shared_ptr<Encoder> encoder_;
};

class ChunkBuilderImpl;

/**
 * Encapsulate chunk serialization operations.
 */
class OUSTER_API_CLASS ChunkBuilder {
   public:
    OUSTER_API_FUNCTION
    ChunkBuilder();

    OUSTER_API_FUNCTION
    ~ChunkBuilder();

    /**
     * Save messages to the serialized chunks.
     *
     * @throws std::logic_error Exception on a size mismatch
     *
     * @param[in] stream_id The stream to save the message to.
     * @param[in] receive_ts The receive timestamp to use for the message.
     * @param[in] sensor_ts The sensor timestamp to use for the message.
     * @param[in] msg_buf The message to save in the form of a byte vector.
     * @param[in] type Message type string of the message being saved.
     */
    OUSTER_API_FUNCTION
    void save_message(const uint32_t stream_id, const ts_t receive_ts, const ts_t sensor_ts,
                      const std::vector<uint8_t>& msg_buf, const std::string& type);

    /**
     * Completely wipe all data and start the chunk anew.
     */
    OUSTER_API_FUNCTION
    void reset();

    /**
     * Finish out the serialization of the chunk and return the raw
     * flatbuffer output.
     *
     * @return The serialized chunk in a raw flatbuffer byte vector.
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> finish();

    /**
     * Returns the flatbufferbuilder size.
     *
     * @return The flatbufferbuilder size.
     */
    OUSTER_API_FUNCTION
    uint32_t size() const;

    /**
     * Returns the number of messages saved so far.
     *
     * @return The number of messages saved so far.
     */
    OUSTER_API_FUNCTION
    uint32_t messages_count() const;

    /**
     * The lowest timestamp in the chunk.
     *
     * @return The lowest timestamp in the chunk.
     */
    OUSTER_API_FUNCTION
    ts_t start_ts() const;

    /**
     * The highest timestamp in the chunk.
     *
     * @return The highest timestamp in the chunk.
     */
    OUSTER_API_FUNCTION
    ts_t end_ts() const;

   private:
    std::shared_ptr<ChunkBuilderImpl> impl_;

    /**
     * Internal method for updating the correct start and end
     * timestamps.
     *
     * @param[in] timestamp The timestamp to check against for start and end.
     */
    void update_start_end(const ts_t timestamp);

    /**
     * Internal status flag for whether the builder is finished or not.
     */
    bool finished_{false};

    std::string type_;

    /**
     * The lowest timestamp in the chunk.
     */
    ts_t start_ts_{ts_t::max()};

    /**
     * The highest timestamp in the chunk.
     */
    ts_t end_ts_{ts_t::min()};
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
