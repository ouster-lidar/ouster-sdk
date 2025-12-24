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
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ouster/client.h"
#include "ouster/impl/netcompat.h"
#include "ouster/impl/ring_buffer.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/packet.h"
#include "ouster/packet_source.h"
#include "ouster/sensor_http.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace sensor {

/**
 * @brief Base class for client-related exceptions.
 */
class OUSTER_API_CLASS ClientError : public std::runtime_error {
   public:
    /**
     * Constructs a ClientError with the given error message.
     * @param[in] msg A description of the client error.
     */
    OUSTER_API_FUNCTION
    ClientError(const std::string& msg) : runtime_error(msg) {}
};

/**
 * @brief Exception for buffer overflow errors in the client.
 *
 * Thrown when a buffer exceeds its capacity or an overflow condition
 * is detected during client data handling.
 * @param[in] msg The error message indicating buffer overflow.
 */
class OUSTER_API_CLASS ClientOverflow : public ClientError {
   public:
    /**
     * Constructs a ClientOverflow with the given error message.
     * @param[in] msg The error message indicating buffer overflow.
     */
    OUSTER_API_FUNCTION
    ClientOverflow(const std::string& msg) : ClientError(msg) {}
};

/**
 * @brief Exception for timeout errors in the client.
 *
 * Thrown when a client operation exceeds the expected time limit.
 * @param[in] msg The error message indicating a timeout occurred.
 */
class OUSTER_API_CLASS ClientTimeout : public ClientError {
   public:
    /**
     * Constructs a ClientTimeout with the given error message.
     * @param[in] msg The error message indicating a timeout occurred.
     */
    OUSTER_API_FUNCTION
    ClientTimeout(const std::string& msg) : ClientError(msg) {}
};

/// Struct that describes a result from retriving a packet from SensorClient
class SensorPacketSource;
/**
 * @brief Represents an event from the SensorPacketSource.
 *
 * Contains the result of polling the sensor client, such as received
 * packets, timeouts, and errors.
 */
class OUSTER_API_CLASS ClientEvent {
    friend class SensorPacketSource;

   public:
    OUSTER_API_FUNCTION ClientEvent();

    /// Types of events that can occur
    enum EventType {
        ERR,           ///< An error occurred in the SensorClient, it may no
                       ///< longer
                       /// function.
        EXIT,          ///< The client has been closed and will not return any
                       ///< more
                       /// packets.
        POLL_TIMEOUT,  ///< get_packet has timed out waiting for an
                       ///< event/packet
        PACKET,        ///< An packet from a sensor
        Error = ERR,   ///< Deprecated alias for ERR
        Exit = EXIT,   ///< Deprecated alias for EXIT
        PollTimeout = POLL_TIMEOUT,  ///< Deprecated alias for POLL_TIMEOUT
        Packet = PACKET              ///< Deprecated alias for PACKET
    };
    // Fix when OUSTER_DEPRECATED_LAST_SUPPORTED_0_16 is removed

    int source;  ///< negative if not applicable to a source, like PollTimeout
                 ///< or Error
    EventType type;  ///< The type of event that occurred.

    /**
     * Access the packet associated with this event.
     * @return Reference to the packet associated with the event.
     */
    OUSTER_API_FUNCTION inline ouster::sdk::core::Packet& packet() {
        return *packet_;
    }

   private:
    ouster::sdk::core::Packet* packet_;
    ClientEvent(ouster::sdk::core::Packet* packet, int src, EventType tpe)
        : source{src}, type{tpe}, packet_{packet} {}
};

/// Class that indicates a sensor and its desired configuration
class OUSTER_API_CLASS Sensor {
   public:
    /// Construct a sensor descriptor with the given hostname and desired config
    OUSTER_API_FUNCTION
    Sensor(const std::string& hostname,  ///< [in] sensor hostname
           const ouster::sdk::core::SensorConfig&
               config  ///< [in] desired sensor configuration
    );

    /// Queries the sensor metadata.
    /// @return the parsed SensorInfo object containing the metadata.
    OUSTER_API_FUNCTION
    ouster::sdk::core::SensorInfo fetch_metadata(
        int timeout = 10  ///< [in] timeout for the request in seconds
    ) const;

    /// Get a SensorHttp client for this sensor.
    /// @return the SensorHttp client
    OUSTER_API_FUNCTION
    std::shared_ptr<ouster::sdk::sensor::SensorHttp> http_client() const;

    /// Get the desired config of this sensor.
    /// @return the desired config
    OUSTER_API_FUNCTION
    inline const ouster::sdk::core::SensorConfig& desired_config() const {
        return config_;
    }

    /// Get the hostname of this sensor.
    /// @return the sensor hostname
    OUSTER_API_FUNCTION
    inline const std::string& hostname() const { return hostname_; }

   private:
    mutable std::shared_ptr<ouster::sdk::sensor::SensorHttp> http_client_;
    std::string hostname_;

    ouster::sdk::core::SensorConfig config_;
};

/// SensorScanSource specific configuration options
struct OUSTER_API_CLASS SensorPacketSourceOptions
    : private PacketSourceOptions {
    using PacketSourceOptions::buffer_time_sec;
    using PacketSourceOptions::config_timeout;
    using PacketSourceOptions::do_not_reinitialize;
    using PacketSourceOptions::extrinsics;
    using PacketSourceOptions::extrinsics_file;
    using PacketSourceOptions::imu_port;
    using PacketSourceOptions::lidar_port;
    using PacketSourceOptions::no_auto_udp_dest;
    using PacketSourceOptions::reuse_ports;
    using PacketSourceOptions::sensor_config;
    using PacketSourceOptions::sensor_info;
    using PacketSourceOptions::timeout;

    using PacketSourceOptions::check;

    /**
     * Constructs SensorPacketSourceOptions
     *
     * @param[in] o The base options to initialize from.
     */
    OUSTER_API_FUNCTION
    SensorPacketSourceOptions(const PacketSourceOptions& o)
        : PacketSourceOptions(o) {}

    OUSTER_API_FUNCTION
    SensorPacketSourceOptions() = default;
};

class SensorPacketIteratorImpl;
class SensorScanSource;
/// An interface to configure and retrieve packets from one or multiple lidars
class OUSTER_API_CLASS SensorPacketSource
    : public ouster::sdk::core::PacketSource,
      ouster::sdk::impl::PacketSourceBuilderMulti<
          ouster::sdk::core::IoType::SENSOR, SensorPacketSource> {
   public:
    friend class SensorPacketIteratorImpl;
    friend class SensorScanSource;

    /// Construct a sensor packet source for a single sensor
    OUSTER_API_FUNCTION
    SensorPacketSource(
        const std::string& sensor,         ///< [in] sensor hostname
        SensorPacketSourceOptions options  ///< [in] source options
    );

    /// Construct a sensor packet source for a multiple sensors
    OUSTER_API_FUNCTION
    SensorPacketSource(
        const std::vector<std::string>& sources,  ///< [in] sensor hostnames
        SensorPacketSourceOptions options         ///< [in] source options
    );

    /// Construct a sensor packet source for a single sensor
    OUSTER_API_FUNCTION
    SensorPacketSource(
        const std::string& source,  ///< [in] sensor hostname
        const std::function<void(SensorPacketSourceOptions&)>& options = {}
        ///< [in] source options
    );

    /// Construct a sensor packet source for multiple sensors
    OUSTER_API_FUNCTION
    SensorPacketSource(
        const std::vector<std::string>& source,  ///< [in] sensor hostnames
        const std::function<void(SensorPacketSourceOptions&)>& options = {}
        ///< [in] source options
    );

    // todo deprecate
    /// Build a sensor client to retrieve packets for the provided sensors.
    /// Configures the sensors if necessary according to their desired configs.
    OUSTER_API_FUNCTION
    SensorPacketSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        double config_timeout_sec = 45,      ///< [in] timeout for sensor config
        double buffer_time_sec =
            0  ///< [in] time in seconds to buffer packets for. If zero no
               ///< buffering is performed outside of the OS.
    );

    // todo deprecate
    /// Build a sensor client to retrieve packets for the provided sensors.
    /// If provided, uses the provided metadata for each sensor rather
    /// configuring and retrieving them from each sensor.
    OUSTER_API_FUNCTION
    SensorPacketSource(
        const std::vector<Sensor>& sensors,  ///< [in] sensors to connect to
        const std::vector<ouster::sdk::core::SensorInfo>&
            infos,  ///< [in] metadata for each sensor, if present used instead
                    ///< of configuring each sensor
        double config_timeout_sec = 45,  ///< [in] timeout for sensor config
        double buffer_time_sec = 0,  ///< [in] time in seconds to buffer packets
                                     ///< for. If zero no buffering is performed
                                     ///< outside of the OS.
        bool reuse_ports = false  ///< [in] If true bind ports with port reuse
                                  ///< enabled
    );

    /// Destruct the sensor client
    OUSTER_API_FUNCTION
    ~SensorPacketSource() override;

    /// Retrieve a packet from the sensor with a given timeout.
    /// timeout_sec of 0 = return immediately, timeout_sec < 0 = wait forever
    /// Important: may return a timeout event if the underlying condition var
    /// experiences a spurious wakeup.
    /// @return a ClientEvent representing the result of the call
    OUSTER_API_FUNCTION
    ClientEvent get_packet(
        double timeout_sec  ///< [in] timeout in seconds to wait for a packet
    );

    /// Get the number of packets dropped due to buffer overflow.
    /// @return the number of dropped packets
    OUSTER_API_FUNCTION
    uint64_t dropped_packets();

    /// Flush the internal packet buffer (if enabled)
    OUSTER_API_FUNCTION
    void flush();

    /// Get the number of packets in the internal buffer.
    /// @return the number of packets in the internal buffer
    OUSTER_API_FUNCTION
    size_t buffer_size();

    // PacketSource implementation
    OUSTER_API_FUNCTION
    core::PacketIterator begin() const override;

    OUSTER_API_FUNCTION
    bool is_live() const override;

    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
    sensor_info() const override;

   protected:
    /// Shut down the client, closing any sockets and threads
    OUSTER_API_FUNCTION
    void close() override;

   private:
    struct OUSTER_API_IGNORE InternalEvent {
        int source;
        ouster::sdk::core::PacketType packet_type;
        ClientEvent::EventType event_type;
    };

    struct OUSTER_API_IGNORE BufferEvent {
        InternalEvent event;
        uint64_t timestamp;
        std::vector<uint8_t> data;
    };

    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> sensor_info_;
    std::vector<SOCKET> sockets_;
    std::vector<std::shared_ptr<ouster::sdk::core::PacketFormat>> formats_;

    bool do_buffer_ = false;
    uint64_t dropped_packets_ = 0;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::thread buffer_thread_;
    std::deque<BufferEvent> buffer_;
    double iterator_timeout_ = 1.0;

    std::vector<uint8_t> staging_buffer_;
    ouster::sdk::core::ImuPacket imu_packet_;
    ouster::sdk::core::LidarPacket lidar_packet_;
    ouster::sdk::core::ZonePacket zone_packet_;
    struct OUSTER_API_IGNORE Addr6 {
        uint8_t address[16];
        int sensor_index;
    };
    struct OUSTER_API_IGNORE Addr4 {
        uint32_t address;
        int sensor_index;
    };
    std::vector<Addr6> addresses6_;
    std::vector<Addr4> addresses4_;

    InternalEvent get_packet_internal(std::vector<uint8_t>& data, uint64_t& ts,
                                      double timeout_sec);

    /// Start a background thread to do buffering if requested
    void start_buffer_thread(double buffer_time  ///< [in] time in seconds
    );
};
}  // namespace sensor
}  // namespace sdk
}  // namespace ouster
