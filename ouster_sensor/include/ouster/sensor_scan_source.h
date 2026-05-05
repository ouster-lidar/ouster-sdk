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
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ouster/open_source.h"
#include "ouster/scan_source.h"
#include "ouster/sensor_packet_source.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace sensor {

/// Options for theSensorScanSource
struct OUSTER_API_CLASS SensorScanSourceOptions : private ScanSourceOptions {
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
    using ScanSourceOptions::reuse_ports;
    using ScanSourceOptions::sensor_config;
    using ScanSourceOptions::sensor_info;
    using ScanSourceOptions::soft_id_check;
    using ScanSourceOptions::timeout;

    using ScanSourceOptions::check;

    /**
     * Construct SensorScanSourceOptions from a ScanSourceOptions object.
     *
     * @param[in] o The ScanSourceOptions object to copy from.
     */
    OUSTER_API_FUNCTION
    SensorScanSourceOptions(const ScanSourceOptions& o)
        : ScanSourceOptions(o) {}

    OUSTER_API_FUNCTION
    SensorScanSourceOptions() = default;
};

/// Provides a simple API for configuring sensors and retreiving LidarScans from
/// them
class OUSTER_API_CLASS SensorScanSource
    : public ouster::sdk::core::ScanSource,
      ouster::sdk::impl::ScanSourceBuilder<ouster::sdk::core::IoType::SENSOR,
                                           SensorScanSource> {
    friend class SensorScanIteratorImpl;

    // registers this with the open_source factory
    inline void dummy() { (void)REGISTERED; }

   public:
    /// Construct the SensorScanSource
    OUSTER_API_FUNCTION
    SensorScanSource(
        const std::string& source,       ///< [in] sensor hostname to connect to
        SensorScanSourceOptions options  ///< [in] scan source options
    );

    /// Construct the SensorScanSource
    OUSTER_API_FUNCTION
    SensorScanSource(
        const std::vector<std::string>&
            source,  ///< [in] sensor hostnames to connect to
        SensorScanSourceOptions options  ///< [in] scan source options
    );

    /// construct the SensorScanSource
    OUSTER_API_FUNCTION
    SensorScanSource(
        const std::string& source,  ///< [in] sensor hostname to connect to
        const std::function<void(SensorScanSourceOptions&)>& options = {}
        ///< [in] scan source options
    );

    /// construct the SensorScanSource
    OUSTER_API_FUNCTION
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
        const std::vector<ouster::sdk::core::SensorInfo>&
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
        const std::vector<ouster::sdk::core::SensorInfo>&
            infos,  ///< [in] metadata for each sensor, if present used instead
                    ///< of configuring each sensor
        const std::vector<ouster::sdk::core::LidarScanFieldTypes>&
            fields,  ///< [in] fields to batch into LidarScans for each lidar.
                     ///< If empty default fields for that profile are used.
        double config_timeout = 45,   ///< [in] timeout for sensor config
        unsigned int queue_size = 2,  ///< [in] maximum number of scans to queue
        bool soft_id_check =
            false  ///< [in] if true, allow accepting packets with mismatched
                   ///< sensor serial numbers and init_ids
    );

    OUSTER_API_FUNCTION
    ~SensorScanSource() override;

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
    std::pair<int, std::unique_ptr<ouster::sdk::core::LidarScan>> get_scan(
        double timeout_sec = 0.0  /// [in] timeout for retrieving a scan
    );

    OUSTER_API_FUNCTION
    bool is_live() const override { return true; }

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
    sensor_info() const override {
        return client_.sensor_info();
    }

    OUSTER_API_FUNCTION
    ouster::sdk::core::ScanIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::sdk::core::ScanIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<ScanSource> move() override {
        throw std::runtime_error("Moving SensorScanSource not supported.");
    }

    /**
     * open_source compatible factory.
     *
     * @relates ouster::open_source
     *
     * @param[in] sources source filenames
     * @param[in] options source options
     * @param[in] collate whether to collate the source or not
     * @param[in] sensor_idx access specific sensor index in the osf
     * @return unique_ptr of ScanSource type
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<ouster::sdk::core::ScanSource> create(
        const std::vector<std::string>& sources,
        const ScanSourceOptions& options, bool collate, int sensor_idx = -1);

   protected:
    void close() override;

   private:
    SensorPacketSource client_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::deque<std::pair<int, std::unique_ptr<ouster::sdk::core::LidarScan>>>
        buffer_;
    uint64_t dropped_scans_ = 0;
    std::vector<ouster::sdk::core::LidarScanFieldTypes> fields_;
    bool run_thread_;
    std::thread batcher_thread_;
    std::atomic<uint64_t> id_error_count_;
    double timeout_ = -1.0;
    int64_t timeout_ns_ = -1;
    std::vector<int64_t> last_receive_times_;

    void start_thread(unsigned int queue_size, bool soft_id_check);
};

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster
