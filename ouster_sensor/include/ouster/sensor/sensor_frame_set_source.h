/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Provides a simple API to get frames from sensors.
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

#include "ouster/core/frame_set_source.h"
#include "ouster/core/open_source.h"
#include "ouster/core/visibility.h"
#include "ouster/sensor/sensor_packet_source.h"

namespace ouster {
namespace sdk {
namespace sensor {

/// Options for the SensorFrameSetSource
struct OUSTER_API_CLASS SensorFrameSetSourceOptions : private FrameSetSourceOptions {
    using FrameSetSourceOptions::config_timeout;
    using FrameSetSourceOptions::do_not_reinitialize;
    using FrameSetSourceOptions::extrinsics;
    using FrameSetSourceOptions::extrinsics_file;
    using FrameSetSourceOptions::field_names;
    using FrameSetSourceOptions::imu_port;
    using FrameSetSourceOptions::lidar_port;
    using FrameSetSourceOptions::no_auto_udp_dest;
    using FrameSetSourceOptions::queue_size;
    using FrameSetSourceOptions::raw_fields;
    using FrameSetSourceOptions::raw_headers;
    using FrameSetSourceOptions::rcvbuf_size;
    using FrameSetSourceOptions::reuse_ports;
    using FrameSetSourceOptions::sensor_config;
    using FrameSetSourceOptions::sensor_info;
    using FrameSetSourceOptions::soft_id_check;
    using FrameSetSourceOptions::timeout;

    using FrameSetSourceOptions::check;

    /**
     * Construct SensorFrameSetSourceOptions from a FrameSetSourceOptions
     * object.
     *
     * @param[in] o The FrameSetSourceOptions object to copy from.
     */
    OUSTER_API_FUNCTION
    SensorFrameSetSourceOptions(const FrameSetSourceOptions& o) : FrameSetSourceOptions(o) {}

    OUSTER_API_FUNCTION
    SensorFrameSetSourceOptions() = default;
};

/// Provides a simple API for configuring sensors and retreiving LidarFrames
/// from them
class OUSTER_API_CLASS SensorFrameSetSource
    : public ouster::sdk::core::FrameSetSource,
      ouster::sdk::impl::FrameSetSourceBuilder<ouster::sdk::core::IoType::SENSOR,
                                               SensorFrameSetSource> {
    friend class SensorFrameSetIteratorImpl;

    // registers this with the open_source factory
    inline void dummy() {
        (void)REGISTERED;
    }

   public:
    /// Construct the SensorFrameSetSource
    OUSTER_API_FUNCTION
    SensorFrameSetSource(const std::string& source,  ///< [in] sensor hostname to connect to
                         SensorFrameSetSourceOptions options  ///< [in] frame set source options
    );

    /// Construct the SensorFrameSetSource
    OUSTER_API_FUNCTION
    SensorFrameSetSource(
        const std::vector<std::string>& source,  ///< [in] sensor hostnames to connect to
        SensorFrameSetSourceOptions options      ///< [in] frame set source options
    );

    /// construct the SensorFrameSetSource
    OUSTER_API_FUNCTION
    SensorFrameSetSource(const std::string& source,  ///< [in] sensor hostname to connect to
                         const std::function<void(SensorFrameSetSourceOptions&)>& options = {}
                         ///< [in] frame set source options
    );

    /// construct the SensorFrameSetSource
    OUSTER_API_FUNCTION
    SensorFrameSetSource(
        const std::vector<std::string>& source,  ///< [in] sensor hostnames to connect
        const std::function<void(SensorFrameSetSourceOptions&)>& options = {}
        ///< [in] frame set source options
    );

    /// Construct a SensorFrameSetSource to connect to the listed sensors
    OUSTER_API_FUNCTION
    SensorFrameSetSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        double config_timeout = 45,          ///< [in] timeout in seconds for configuring sensors
        unsigned int queue_size = 2,         ///< [in] maximum number of frames to queue
        bool soft_id_check = false  ///< [in] if true, allow accepting packets with mismatched
                                    ///< sensor serial numbers and init_ids
    );

    /// Construct a SensorFrameSetSource to connect to the listed sensors
    /// If infos are provided, they are used instead of configuring the sensors
    /// and retrieving the sensor info from them.
    OUSTER_API_FUNCTION
    SensorFrameSetSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<ouster::sdk::core::SensorInfo>&
            infos,                    ///< [in] metadata for each sensor, if present used instead
                                      ///< of configuring each sensor
        double config_timeout = 45,   ///< [in] timeout for sensor config
        unsigned int queue_size = 2,  ///< [in] maximum number of frames to queue
        bool soft_id_check = false    ///< [in] if true, allow accepting packets with mismatched
                                      ///< sensor serial numbers and init_ids
    );

    /// Construct a SensorFrameSetSource to connect to the listed sensors
    /// If infos are provided, they are used instead of configuring the sensors
    /// and retrieving the sensor info from them.
    OUSTER_API_FUNCTION
    SensorFrameSetSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<ouster::sdk::core::SensorInfo>&
            infos,  ///< [in] metadata for each sensor, if present used instead
                    ///< of configuring each sensor
        const std::vector<ouster::sdk::core::LidarFrameFieldTypes>&
            fields,                   ///< [in] fields to batch into LidarFrames for each lidar.
                                      ///< If empty default fields for that profile are used.
        double config_timeout = 45,   ///< [in] timeout for sensor config
        unsigned int queue_size = 2,  ///< [in] maximum number of frames to queue
        bool soft_id_check = false    ///< [in] if true, allow accepting packets with mismatched
                                      ///< sensor serial numbers and init_ids
    );

    OUSTER_API_FUNCTION
    ~SensorFrameSetSource() override;

    /// Flush any buffered frames.
    OUSTER_API_FUNCTION
    void flush();

    /// Get the number of frames that were dropped due to buffer overflow.
    /// @return the number of dropped frames
    OUSTER_API_FUNCTION
    inline uint64_t dropped_frames() {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        return dropped_frames_;
    }

    /// @deprecated Use dropped_frames() instead
    /// @copydoc dropped_frames()
    OUSTER_DEPRECATED_MSG("dropped_frames()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    inline uint64_t dropped_scans() {
        return dropped_frames();
    }

    /// Get the number of packets that had an id verification error
    /// @return the number of errors
    OUSTER_API_FUNCTION
    inline uint64_t id_error_count() {
        return id_error_count_;
    }

    /// Retrieves a frame from the queue or waits up to timeout_sec until one is
    /// available.
    /// Important: may return a nullptr if the underlying condition var
    /// experiences a spurious wakeup.
    /// @return the resulting lidar frame with the idx of the producing sensor
    ///         if no result, the returned frame will be nullptr
    OUSTER_API_FUNCTION
    std::pair<int, std::unique_ptr<ouster::sdk::core::LidarFrame>> get_frame(
        double timeout_sec = 0.0  /// [in] timeout for retrieving a frame
    );

    /// @deprecated Use get_frame() instead
    /// @copydoc get_frame()
    OUSTER_DEPRECATED_MSG("get_frame()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    std::pair<int, std::unique_ptr<ouster::sdk::core::LidarFrame>> get_scan(
        double timeout_sec = 0.0);

    OUSTER_API_FUNCTION
    bool is_live() const override {
        return true;
    }

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_info()
        const override {
        return client_.sensor_info();
    }

    OUSTER_API_FUNCTION
    ouster::sdk::core::FrameSetIterator begin() const override;

    OUSTER_API_FUNCTION
    ouster::sdk::core::FrameSetIterator begin(int sensor_index) const override;

    OUSTER_API_FUNCTION
    size_t size_hint() const override;

    OUSTER_API_FUNCTION
    std::unique_ptr<FrameSetSource> move() override {
        throw std::runtime_error("Moving SensorFrameSetSource not supported.");
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
     * @return unique_ptr of FrameSetSource type
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<ouster::sdk::core::FrameSetSource> create(
        const std::vector<std::string>& sources, const FrameSetSourceOptions& options, bool collate,
        int sensor_idx = -1);

    /// Get the sockets used in the client. Useful for changing options.
    /// @return list of socket handles
    OUSTER_API_FUNCTION
    const std::vector<SOCKET>& sockets() const;

   protected:
    void close() override;

   private:
    SensorPacketSource client_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::deque<std::pair<int, std::unique_ptr<ouster::sdk::core::LidarFrame>>> buffer_;
    uint64_t dropped_frames_ = 0;
    std::vector<ouster::sdk::core::LidarFrameFieldTypes> fields_;
    bool run_thread_;
    std::thread batcher_thread_;
    std::atomic<uint64_t> id_error_count_;
    double timeout_ = -1.0;
    int64_t timeout_ns_ = -1;
    std::vector<int64_t> last_receive_times_;
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> orig_sensor_info_;

    void start_thread(unsigned int queue_size, bool soft_id_check);
};

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace sensor {

OUSTER_DEPRECATED_TYPE(SensorScanSource, SensorFrameSetSource, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(SensorScanSourceOptions, SensorFrameSetSourceOptions,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster
