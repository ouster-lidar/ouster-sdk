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

#include "nonstd/optional.hpp"
#include "ouster/open_source.h"
#include "ouster/scan_source.h"
#include "ouster/sensor_packet_source.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sensor {

/// Options for theSensorScanSource
struct SensorScanSourceOptions : private ScanSourceOptions {
    using ScanSourceOptions::config_timeout;
    using ScanSourceOptions::do_not_reinitialize;
    using ScanSourceOptions::extrinsics;
    using ScanSourceOptions::extrinsics_file;
    using ScanSourceOptions::field_names;
    using ScanSourceOptions::imu_port;
    using ScanSourceOptions::lidar_port;
    using ScanSourceOptions::no_auto_udp_dest;
    using ScanSourceOptions::queue_size;
    using ScanSourceOptions::raw_fields;
    using ScanSourceOptions::raw_headers;
    using ScanSourceOptions::sensor_config;
    using ScanSourceOptions::sensor_info;
    using ScanSourceOptions::soft_id_check;
    using ScanSourceOptions::timeout;

    using ScanSourceOptions::check;

    SensorScanSourceOptions(const ScanSourceOptions& o)
        : ScanSourceOptions(o) {}

    SensorScanSourceOptions() {}
};

/// Provides a simple API for configuring sensors and retreiving LidarScans from
/// them
class OUSTER_API_CLASS SensorScanSource
    : public core::ScanSource,
      ouster::impl::ScanSourceBuilderMulti<ouster::core::IoType::SENSOR,
                                           SensorScanSource> {
    friend class SensorScanIteratorImpl;

    // registers this with the open_source factory
    inline void dummy() { (void)registered_; }

   public:
    /// Construct the SensorScanSource
    SensorScanSource(
        const std::string& source,       ///< [in] sensor hostname to connect to
        SensorScanSourceOptions options  ///< [in] scan source options
    );

    /// Construct the SensorScanSource
    SensorScanSource(
        const std::vector<std::string>&
            source,  ///< [in] sensor hostnames to connect to
        SensorScanSourceOptions options  ///< [in] scan source options
    );

    /// construct the SensorScanSource
    SensorScanSource(
        const std::string& source,  ///< [in] sensor hostname to connect to
        const std::function<void(SensorScanSourceOptions&)>& options = {}
        ///< [in] scan source options
    );

    /// construct the SensorScanSource
    SensorScanSource(
        const std::vector<std::string>&
            source,  ///< [in] sensor hostnames to connect
        const std::function<void(SensorScanSourceOptions&)>& options = {}
        ///< [in] scan source options
    );

    /// Construct a SensorScanSource to connect to the listed sensors
    OUSTER_API_FUNCTION
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
    OUSTER_API_FUNCTION
    SensorScanSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<ouster::sensor::sensor_info>&
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
    OUSTER_API_FUNCTION
    SensorScanSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<ouster::sensor::sensor_info>&
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

    OUSTER_API_FUNCTION
    ~SensorScanSource();

    /// Flush any buffered scans.
    OUSTER_API_FUNCTION
    void flush();

    /// Get the number of scans that were dropped due to buffer overflow.
    /// @return the number of dropped scans
    OUSTER_API_FUNCTION
    inline uint64_t dropped_scans() {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        return dropped_scans_;
    }

    /// Get the number of packets that had an id verification error
    /// @return the number of errors
    OUSTER_API_FUNCTION
    inline uint64_t id_error_count() { return id_error_count_; }

    /// Retrieves a scan from the queue or waits up to timeout_sec until one is
    /// available.
    /// Important: may return a nullptr if the underlying condition var
    /// experiences a spurious wakeup.
    /// @return the resulting lidar scan with the idx of the producing sensor
    ///         if no result, the returned scan will be nullptr
    OUSTER_API_FUNCTION
    std::pair<int, std::unique_ptr<LidarScan>> get_scan(
        double timeout_sec = 0.0  /// [in] timeout for retrieving a scan
    );

    OUSTER_API_FUNCTION
    bool is_live() const override { return true; }

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override {
        return client_.sensor_info();
    }

    OUSTER_API_FUNCTION
    core::ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    core::ScanIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    ScanSource* move() override {
        throw std::runtime_error("Moving SensorScanSource not supported.");
    }

   protected:
    void close() override;

   private:
    SensorPacketSource client_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::deque<std::pair<int, std::unique_ptr<LidarScan>>> buffer_;
    uint64_t dropped_scans_ = 0;
    std::vector<LidarScanFieldTypes> fields_;
    bool run_thread_;
    std::thread batcher_thread_;
    std::atomic<uint64_t> id_error_count_;
    double timeout_ = -1.0;
    int64_t timeout_ns_ = -1;
    std::vector<int64_t> last_receive_times_;

    void start_thread(unsigned int queue_size, bool soft_id_check);
};

}  // namespace sensor
}  // namespace ouster
