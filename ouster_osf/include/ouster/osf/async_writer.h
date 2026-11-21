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
    AsyncWriter(const std::string& filename,
                const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& info,
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
     * The stream_index is the sensor's position in registration order, the
     * same unified index Writer::save() uses and the same order returned by
     * FrameSetSource::sensor_info() on read. Lidar and camera streams share
     * this one index space. Consider the following:
     @code{.cpp}
     LidarInfo lidar1;   // registered at index 0
     CameraInfo camera;  // registered at index 1
     LidarInfo lidar2;   // registered at index 2

     AsyncWriter output = AsyncWriter(filename, {lidar1, camera, lidar2});

     // Save a LidarFrame to lidar1 (index 0):
     output.save(0, lidar_frame);

     // Save a CameraFrame to camera (index 1):
     output.save(1, camera_frame);

     // Save a LidarFrame to lidar2 (index 2):
     output.save(2, lidar_frame);
     @endcode
     *
     * The frame may be a LidarFrame or a CameraFrame; the correct stream type
     * is selected based on the concrete frame type.
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The unified stream index (registration order) of
     *                         the sensor to write this frame to.
     * @param[in] frame The frame to save.
     * @return a future, which can propagate exceptions that may have occurred
     * in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(uint32_t stream_index, const ouster::sdk::core::Frame& frame);

    /**
     * Save a single frame with the specified timestamp to the
     * specified stream_index in an OSF file.
     *
     * The frame may be a LidarFrame or a CameraFrame; the correct stream type
     * is selected based on the concrete frame type.
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The unified stream index (registration order) of
     *                         the sensor to write this frame to.
     * @param[in] frame The frame to save.
     * @param[in] timestamp Receive timestamp to index this frame with.
     * @return a future, which can propagate exceptions that may have occurred
     * in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(uint32_t stream_index, const ouster::sdk::core::Frame& frame,
                           ouster::sdk::osf::ts_t timestamp);

    /**
     * Save multiple frames to the OSF file.
     *
     * A FrameSet's slots are in the same registration order as the sensors,
     * so slot i holds the frame for the sensor registered at index i. Lidar
     * and camera sensors share that one ordering. Consider the following:
     @code{.cpp}
     LidarInfo lidar1;   // registered at index 0
     CameraInfo camera;  // registered at index 1
     LidarInfo lidar2;   // registered at index 2

     AsyncWriter output = AsyncWriter(filename, {lidar1, camera, lidar2});

     std::shared_ptr<LidarFrame> lidar1_frame = RANDOM_FRAME_HERE;
     std::shared_ptr<CameraFrame> camera_frame = RANDOM_FRAME_HERE;
     std::shared_ptr<LidarFrame> lidar2_frame = RANDOM_FRAME_HERE;

     FrameSet frame_set{lidar1_frame, camera_frame, lidar2_frame};

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

        // Note - this constructor deliberately copies the frame (via clone,
        // preserving its concrete LidarFrame/CameraFrame type) because it could
        // be modified in a different thread.
        OUSTER_API_IGNORE
        LidarFrameMessage(int stream_index, const ouster::sdk::osf::ts_t& timestamp,
                          const ouster::sdk::core::Frame& frame, std::promise<void>& promise)
            : stream_index(stream_index),
              timestamp(timestamp),
              frame_set({frame.clone()}),
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
    // Unlike Writer, AsyncWriter requires all sensors to be known at construction.
    // Cache the number of sensors so we can validate stream_index in save() calls.
    const size_t num_sensors_;
    /**
     * Internal job queue, used to keep the save function from blocking the
     * calling thread for the duration of encoding.
     */
    ThreadsafeQueue<LidarFrameMessage> save_queue_;
    std::thread save_thread_;
    std::mutex stream_mutex_;
    /**
     * Serializes the producer side of the writer (the save() overloads and
     * close()). It guards last_timestamp_ and the closed check/enqueue
     * sequence so that concurrent callers do not race on this state.
     *
     * Note: this mutex must NOT be held by the save thread, otherwise close()
     * (which acquires it and then joins the save thread) could deadlock.
     */
    std::mutex save_mutex_;
    // Keyed by a composite of frame type and stream index, since lidar and
    // camera stream indices are independent 0-based spaces that would otherwise
    // collide. See make_timestamp_key() in the implementation.
    std::unordered_map<uint64_t, ouster::sdk::osf::ts_t> last_timestamp_;

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
