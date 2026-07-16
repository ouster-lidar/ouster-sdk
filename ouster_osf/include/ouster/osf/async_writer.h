/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/core/impl/threadsafe_queue.h"
#include "ouster/osf/osf_encoder.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * %OSF AsyncWriter wraps osf::Writer so that saving occurs in a background
 * thread. Calls to save() return a std::future<void> instead of void to enable
 * propagating exceptions from the save thread.
 */
class OUSTER_API_CLASS AsyncWriter {
   public:
    /**
     * @param[in] filename The filename to output to.
     * @param[in] info The sensor info vector to use for a multi stream OSF
     *                 file.
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
    AsyncWriter(const std::string& filename, const std::vector<ouster::sdk::core::SensorInfo>& info,
                const std::vector<std::string>& fields_to_write = std::vector<std::string>(),
                uint32_t chunk_size = 0, std::shared_ptr<Encoder> encoder = nullptr);

    /**
     * Closes the writer and finalizes any pending writes.
     */
    OUSTER_API_FUNCTION
    ~AsyncWriter();

    /**
     * Save a single frame to the specified stream_index in an OSF
     * file.
     *
     * The concept of the stream_index is related to the sensor_info vector.
     * Consider the following:
     @code{.cpp}
     SensorInfo info1; // The first sensor in this OSF file
     SensorInfo info2; // The second sensor in this OSF file
     SensorInfo info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     LidarFrame frame = RANDOM_FRAME_HERE;

     // To save the LidarFrame to the first sensor, you would do the
     // following
     output.save(0, frame);

     // To save the LidarFrame to the second sensor, you would do the
     // following
     output.save(1, frame);

     // To save the LidarFrame to the third sensor, you would do the
     // following
     output.save(2, frame);
     @endcode
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The index of the corrosponding SensorInfo to
     *                         use.
     * @param[in] frame The frame to save.
     * @return a future, which can propagate exceptions that may have occurred
     * in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(uint32_t stream_index, const ouster::sdk::core::LidarFrame& frame);

    /**
     * Save a single frame with the specified timestamp to the
     * specified stream_index in an OSF file.
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The index of the corrosponding SensorInfo to
     *                         use.
     * @param[in] frame The frame to save.
     * @param[in] timestamp Receive timestamp to index this frame with.
     * @return a future, which can propagate exceptions that may have occurred
     * in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(uint32_t stream_index, const ouster::sdk::core::LidarFrame& frame,
                           ouster::sdk::osf::ts_t timestamp);

    /**
     * Save multiple frames to the OSF file.
     *
     * The concept of the stream_index is related to the SensorInfo vector.
     * Consider the following:
     @code{.cpp}
     SensorInfo info1; // The first sensor in this OSF file
     SensorInfo info2; // The second sensor in this OSF file
     SensorInfo info3; // The third sensor in this OSF file

     Writer output = Writer(filename, {info1, info2, info3});

     std::shared_ptr<LidarFrame> sensor1_frame = RANDOM_FRAME_HERE;
     std::shared_ptr<LidarFrame> sensor2_frame = RANDOM_FRAME_HERE;
     std::shared_ptr<LidarFrame> sensor3_frame = RANDOM_FRAME_HERE;

     FrameSet frame_set{sensor1_frame, sensor2_frame, sensor3_frame};

     // To save the frames matched appropriately to their sensors, you would do
     // the following
     output.save(frame_set);
     @endcode
     *
     *
     * @throws std::logic_error Will throw exception on writer being closed
     * @throws OsfDropFrameError Will throw exception on missing timestamps
     *
     * @param[in] frames FrameSet to save.
     * @return a future, which can propagate exceptions that may have
     *         occurred in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(const ouster::sdk::core::FrameSet& frames);

    /**
     * Synchronously save a set of frame set source metadata to the OSF file.
     *
     * @param[in] frame_set_source_metadata_set The set of metadata entries to
     * save.
     */
    OUSTER_API_FUNCTION
    void save(const FrameSetSourceMetadataSet& frame_set_source_metadata_set);

    /**
     * Finish file with a proper metadata object, and header.
     * This method blocks until all remaining tasks generated by save() have
     * been finalized.
     * @param[in] fsync If true, force all writes on this file to disk.
     */
    OUSTER_API_FUNCTION
    void close(bool fsync = false);

   private:
    /**
     * Encapsulates everything that's needed to encode and save the provided
     * lidar frame into the OSF.
     *
     */
    struct OUSTER_API_IGNORE LidarFrameMessage {
        int stream_index;
        ouster::sdk::osf::ts_t timestamp;
        ouster::sdk::core::FrameSet frame_set;
        std::promise<void> promise;
        bool saving_uncollated;

        // TODO: can we get out of these copies?

        // Note - this constructor deliberately copies the LidarFrame because it
        // could be modified in a different thread.
        OUSTER_API_IGNORE
        LidarFrameMessage(int stream_index, const ouster::sdk::osf::ts_t& timestamp,
                          const ouster::sdk::core::LidarFrame& lidar_frame,
                          std::promise<void>& promise)
            : stream_index(stream_index),
              timestamp(timestamp),
              frame_set({std::make_shared<ouster::sdk::core::LidarFrame>(lidar_frame)}),
              promise(std::move(promise)),
              saving_uncollated(true) {}

        // Note - this constructor deliberately copies the FrameSet because
        // it could be modified in a different thread.
        OUSTER_API_IGNORE
        LidarFrameMessage(const ouster::sdk::core::FrameSet& frame_set, std::promise<void>& promise)
            : stream_index(-1),
              timestamp(),
              frame_set(frame_set.clone()),
              promise(std::move(promise)),
              saving_uncollated(false) {}

        LidarFrameMessage(const LidarFrameMessage&) = delete;
        OUSTER_API_IGNORE
        LidarFrameMessage(LidarFrameMessage&&) = default;
    };

    Writer writer_;
    /**
     * Internal job queue, used to keep the save function from blocking the
     * calling thread for the duration of encoding.
     */
    ThreadsafeQueue<LidarFrameMessage> save_queue_;
    std::thread save_thread_;
    std::mutex stream_mutex_;
    std::unordered_map<uint32_t, ouster::sdk::osf::ts_t> last_timestamp_;

    /**
     * A runnable used to handle writes in the thread 'save_thread_'.
     */
    void save_thread_method();

    /**
     * Exception propagated from the save thread.
     */
    std::exception_ptr save_exception_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
