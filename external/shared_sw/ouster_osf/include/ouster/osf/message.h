#pragma once

#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"

#include "ouster/osf/file_info.h"
#include "ouster/osf/types.h"

namespace ouster {
namespace OSF {

// User land type Message type, duplicates values from osfChunk::Message
// generated
enum class MessageType {
    NONE = 0,
    LIDAR_SCAN = 1,
    TRAJECTORY = 2,
    GPS_WAYPOINT = 3,
    IMU = 4,
    MESSAGE_EXTENSION = 5,
    MIN = NONE,
    MAX = MESSAGE_EXTENSION
};

std::string to_string(const MessageType message_type);

// Thin interface class that holds the pointer to the message
// and reconstructs underlying data to the corresponding typed class: LidarScan,
// OSF::Gps, OSF::Imu, OSF::Trajectory
class MessageRef {
   public:
    using ts_t = OSF::ts_t;

    // The only way to create the MessageRef is to point to the corresponding
    // byte buffer of the message in OSF file.
    // @param file_info data used in types reconstruction
    explicit MessageRef(const uint8_t* buf, const FileInfo& file_info)
        : buf_(buf), file_info_(file_info) {}

    // Message generator identifier the sensor (1,2,3 ..), gps (253) or car
    // (254)
    uint8_t id() const;

    // Timestamp of the message
    ts_t ts() const;

    // Type of the stored data
    MessageType type() const;

    // Reconstruct the underlying data to the class (copies data)
    std::unique_ptr<LidarScan> as_lidar_scan() const;
    std::unique_ptr<Gps> as_gps() const;
    std::unique_ptr<Imu> as_imu() const;
    std::unique_ptr<Trajectory> as_trajectory() const;

    // Pointer to the underlying data
    const uint8_t* buf() const { return buf_; }

    // FileInfo with metadata about sensors, record session length, etc
    const FileInfo& file_info() const { return file_info_; }

   private:
    const uint8_t* buf_;
    const FileInfo& file_info_;
};

}  // namespace OSF
}  // namespace ouster