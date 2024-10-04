/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Provides a simple interface to configure sensors and receive packets
 * from them.
 *
 */

#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include "ouster/client.h"
#include "ouster/impl/netcompat.h"
#include "ouster/impl/ring_buffer.h"
#include "ouster/lidar_scan.h"
#include "ouster/sensor_http.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {

/// Struct that describes a result from retriving a packet from SensorClient
struct ClientEvent {
    /// Types of events that can occur
    enum EventType {
        Error,  ///< An error occurred in the SensorClient, it may no longer
                /// function.
        Exit,   ///< The client has been closed and will not return any more
                /// packets.
        PollTimeout,  ///< get_packet has timed out waiting for an event/packet
        ImuPacket,    ///< An IMU packet from a sensor
        LidarPacket   ///< A Lidar packet from a sensor
    };

    int source;  ///< negative if not applicable to a source, like PollTimeout
                 ///< or Error
    EventType type;  ///< The type of event that occurred.
};

/// Class that indicates a sensor and its desired configuration
class Sensor {
   public:
    /// Construct a sensor descriptor with the given hostname and desired config
    Sensor(const std::string& hostname,  ///< [in] sensor hostname
           const sensor_config& config   ///< [in] desired sensor configuration
    );

    /// Queries the sensor metadata.
    /// @return the parsed sensor_info object containing the metadata.
    sensor_info fetch_metadata(
        int timeout = 10  ///< [in] timeout for the request in seconds
    ) const;

    /// Get a SensorHttp client for this sensor.
    /// @return the SensorHttp client
    std::shared_ptr<ouster::sensor::util::SensorHttp> http_client() const;

    /// Get the desired config of this sensor.
    /// @return the desired config
    inline const sensor_config& desired_config() const { return config_; }

    /// Get the hostname of this sensor.
    /// @return the sensor hostname
    inline const std::string& hostname() const { return hostname_; }

   private:
    mutable std::shared_ptr<ouster::sensor::util::SensorHttp> http_client_;
    std::string hostname_;

    sensor_config config_;
};

/// An interface to configure and retrieve packets from one or multiple lidars
class SensorClient {
   public:
    /// Build a sensor client to retrieve packets for the provided sensors.
    /// Configures the sensors if necessary according to their desired configs.
    SensorClient(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        double config_timeout_sec = 45,      ///< [in] timeout for sensor config
        double buffer_time_sec =
            0  ///< [in] time in seconds to buffer packets for. If zero no
               ///< buffering is performed outside of the OS.
    );

    /// Build a sensor client to retrieve packets for the provided sensors.
    /// If provided, uses the provided metadata for each sensor rather
    /// configuring and retrieving them from each sensor.
    SensorClient(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<sensor_info>&
            infos,  ///< [in] metadata for each sensor, if present used instead
                    ///< of configuring each sensor
        double config_timeout_sec = 45,  ///< [in] timeout for sensor config
        double buffer_time_sec = 0  ///< [in] time in seconds to buffer packets
                                    ///< for. If zero no buffering is performed
                                    ///< outside of the OS.
    );

    /// Destruct the sensor client
    ~SensorClient();

    /// Retrieve a packet from the sensor with a given timeout.
    /// timeout_sec of 0 = return instantly, timeout_sec < 0 = wait forever
    /// Important: may return a timeout event if the underlying condition var
    /// experiences a spurious wakeup.
    /// @return a ClientEvent representing the result of the call
    ClientEvent get_packet(
        LidarPacket& lp,    ///< [out] output LidarPacket if received
        ImuPacket& ip,      ///< [out] output ImuPacket if received
        double timeout_sec  ///< [in] timeout in seconds to wait for a packet
    );

    /// Get the sensor_infos for each connected sensor
    /// @return the sensor_infos for each connected sensor
    inline const std::vector<sensor_info>& get_sensor_info() {
        return sensor_info_;
    }

    /// Get the number of packets dropped due to buffer overflow.
    /// @return the number of dropped packets
    uint64_t dropped_packets();

    /// Flush the internal packet buffer (if enabled)
    void flush();

    /// Shut down the client, closing any sockets and threads
    void close();

    /// Get the number of packets in the internal buffer.
    /// @return the number of packets in the internal buffer
    size_t buffer_size();

   private:
    struct BufferEvent {
        ClientEvent event;
        uint64_t timestamp;
        std::vector<uint8_t> data;
    };

    std::vector<sensor_info> sensor_info_;
    std::vector<SOCKET> sockets_;
    std::vector<packet_format> formats_;

    bool do_buffer_ = false;
    uint64_t dropped_packets_ = 0;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::thread buffer_thread_;
    std::deque<BufferEvent> buffer_;

    struct Addr {
        uint32_t ipv4;
        uint8_t ipv6[16];
        uint8_t ipv6_4[16];
    };
    std::vector<Addr> addresses_;

    ClientEvent get_packet_internal(std::vector<uint8_t>& data, uint64_t& ts,
                                    double timeout_sec);

    /// Start a background thread to do buffering if requested
    void start_buffer_thread(double buffer_time  ///< [in] time in seconds
    );
};
}  // namespace sensor
}  // namespace ouster
