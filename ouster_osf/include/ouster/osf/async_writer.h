/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ouster/impl/threadsafe_queue.h"
#include "ouster/osf/osf_encoder.h"
#include "ouster/osf/writer.h"

namespace ouster {
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
     * @param[in] fields_to_write The fields from scans to actually save into
     *                            the OSF. If not provided uses the fields from
     *                            the first saved lidar scan for this sensor.
     *                            This parameter is optional.
     * @param[in] chunk_size The chunksize to use for the OSF file, this
     *                       parameter is optional.
     * @param[in] encoder An optional Encoder instance for configuring how the
     *                            Writer should encode the OSF.
     */
    OUSTER_API_FUNCTION
    AsyncWriter(const std::string& filename,
                const std::vector<ouster::sensor::sensor_info>& info,
                const std::vector<std::string>& fields_to_write =
                    std::vector<std::string>(),
                uint32_t chunk_size = 0,
                std::shared_ptr<Encoder> encoder = nullptr);

    /**
     * Closes the writer and finalizes any pending writes.
     */
    OUSTER_API_FUNCTION
    ~AsyncWriter();

    /**
     * Save a single scan to the specified stream_index in an OSF
     * file.
     *
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
     * @return a future, which can propagate exceptions that may have occurred
     * in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(uint32_t stream_index, const LidarScan& scan);

    /**
     * Save a single scan with the specified timestamp to the
     * specified stream_index in an OSF file.
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The index of the corrosponding sensor_info to
     *                         use.
     * @param[in] scan The scan to save.
     * @param[in] timestamp Receive timestamp to index this scan with.
     * @return a future, which can propagate exceptions that may have occurred
     * in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::future<void> save(uint32_t stream_index, const LidarScan& scan,
                           ouster::osf::ts_t timestamp);

    /**
     * Save multiple scans to the OSF file.
     *
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
     * @return a vector of futures, which can propagate exceptions that may have
     * occurred in the background from the save thread.
     */
    OUSTER_API_FUNCTION
    std::vector<std::future<void>> save(const std::vector<LidarScan>& scans);

    /**
     * Finish file with a proper metadata object, and header.
     * This method blocks until all remaining tasks generated by save() have
     * been finalized.
     */
    OUSTER_API_FUNCTION
    void close();

   private:
    /**
     * Encapsulates everything that's needed to encode and save the provided
     * lidar scan into the OSF.
     *
     */
    struct OUSTER_API_IGNORE LidarScanMessage {
        int stream_index_;
        ouster::osf::ts_t timestamp_;
        ouster::LidarScan lidar_scan_;
        std::promise<void> promise_;

        // Note - this constructor deliberately copies the LidarScan because it
        // could be modified in a different thread.
        OUSTER_API_IGNORE
        LidarScanMessage(int stream_index, const ouster::osf::ts_t& timestamp,
                         const ouster::LidarScan& lidar_scan,
                         std::promise<void>& promise)
            : stream_index_(stream_index),
              timestamp_(timestamp),
              lidar_scan_(lidar_scan),
              promise_(std::move(promise)) {}
        LidarScanMessage(const LidarScanMessage&) = delete;
        OUSTER_API_IGNORE
        LidarScanMessage(LidarScanMessage&&) = default;
    };

    Writer writer_;
    /**
     * Internal job queue, used to keep the save function from blocking the
     * calling thread for the duration of encoding.
     */
    ThreadsafeQueue<LidarScanMessage> save_queue_;
    std::thread save_thread_;
    std::mutex stream_mutex_;

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
}  // namespace ouster
