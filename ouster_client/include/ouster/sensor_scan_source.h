/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Provides a simple API to get scans from sensors.
 *
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "ouster/sensor_client.h"

namespace ouster {
namespace sensor {

/// Provides a simple API for configuring sensors and retreiving LidarScans from
/// them
class SensorScanSource {
   public:
    /// Construct a SensorScanSource to connect to the listed sensors
    SensorScanSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        double config_timeout =
            45,  ///< [in] timeout in seconds for configuring sensors
        unsigned int queue_size = 2,  ///< [in] maximum number of scans to queue
        bool soft_id_check =
            false  ///< [in] if true, allow accepting packets with mismatched
                   ///< sensor serial numbers and init_ids
    );

    /// Construct a SensorScanSource to connect to the listed sensors
    /// If infos are provided, they are used instead of configuring the sensors
    /// and retrieving the sensor info from them.
    SensorScanSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<sensor_info>&
            infos,  ///< [in] metadata for each sensor, if present used instead
                    ///< of configuring each sensor
        double config_timeout = 45,   ///< [in] timeout for sensor config
        unsigned int queue_size = 2,  ///< [in] maximum number of scans to queue
        bool soft_id_check =
            false  ///< [in] if true, allow accepting packets with mismatched
                   ///< sensor serial numbers and init_ids
    );

    /// Construct a SensorScanSource to connect to the listed sensors
    /// If infos are provided, they are used instead of configuring the sensors
    /// and retrieving the sensor info from them.
    SensorScanSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<sensor_info>&
            infos,  ///< [in] metadata for each sensor, if present used instead
                    ///< of configuring each sensor
        const std::vector<LidarScanFieldTypes>&
            fields,  ///< [in] fields to batch into LidarScans for each lidar.
                     ///< If empty default fields for that profile are used.
        double config_timeout = 45,   ///< [in] timeout for sensor config
        unsigned int queue_size = 2,  ///< [in] maximum number of scans to queue
        bool soft_id_check =
            false  ///< [in] if true, allow accepting packets with mismatched
                   ///< sensor serial numbers and init_ids
    );

    /// Destruct the SensorScanSource
    ~SensorScanSource();

    /// Get the sensor_infos for each connected sensor
    /// @return the sensor_infos for each connected sensor
    inline const std::vector<sensor_info>& get_sensor_info() {
        return client_.get_sensor_info();
    }

    /// Flush any buffered scans.
    void flush();

    /// Get the number of scans that were dropped due to buffer overflow.
    /// @return the number of dropped scans
    inline uint64_t dropped_scans() {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        return dropped_scans_;
    }

    /// Get the number of packets that had an id verification error
    /// @return the number of errors
    inline uint64_t id_error_count() { return id_error_count_; }

    /// Retrieves a scan from the queue or waits up to timeout_sec until one is
    /// available.
    /// Important: may return a nullptr if the underlying condition var
    /// experiences a spurious wakeup.
    /// @return the resulting lidar scan with the idx of the producing sensor
    ///         if no result, the returned scan will be nullptr
    std::pair<int, std::unique_ptr<LidarScan>> get_scan(
        double timeout_sec = 0.0  /// [in] timeout for retrieving a scan
    );

    /// Shut down the scan source, closing any sockets and threads
    void close();

   private:
    SensorClient client_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::deque<std::pair<int, std::unique_ptr<LidarScan>>> buffer_;
    uint64_t dropped_scans_ = 0;
    std::vector<LidarScanFieldTypes> fields_;
    bool run_thread_;
    std::thread batcher_thread_;
    std::atomic<uint64_t> id_error_count_;
};
}  // namespace sensor
}  // namespace ouster
