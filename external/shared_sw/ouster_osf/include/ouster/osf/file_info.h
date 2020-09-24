#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <vector>

#include "ouster/osf/common.h"
#include "ouster/osf/file.h"
#include "ouster/osf/types.h"

namespace ouster {
namespace OSF {

class FileInfo {
   public:
    using ts_t = OSF::ts_t;
    // sensor.id to sensor
    using sensors_map = OSF::sensors_map;
    // (ts OR frame_index) to chunk_offset, see comments for 'cmap_' member
    using frames_map_t = std::map<uint64_t, size_t>;

    explicit FileInfo(const OsfFile& osf_file);

    // Session id
    std::string id() const;

    // TODO[pb]: Remove? Once we have frames() support in OSF::Reader and start
    // processing framed OSF we probably want to hide this internal detail to
    // smth like isFramed() accessor only.
    OSF_SESSION_MODE mode() const;

    // OSF files currently store messages in two modes:
    //   OSF_STREAM - messages stored in first in first stored
    //   OSF_FRAMED - messages grouped by lidar_scan timestamps into chunks and
    //     all messages within such chunk correspond to one frame (physically
    //     this is data related to one LIDAR sweep)
    //   NOTE: framed reading not yet supported - TBD
    bool isFramed() const;

    // LidarScan data encoding method. Currently there two modes in discussion:
    //   OSF_32 - 1 PNG - (RGBA, 32bit per pixel) lossy storage mode.
    //     (supported)
    //   OSF_56 - 3 PNG - (RGB 24bit, Gray - 16bit, Gray - 16bit) -
    //     lossless storage. TBD
    OSF_FRAME_MODE lidar_frame_mode() const;

    // Quantization param used in OSF_32 frame encoding
    uint8_t range_multiplier() const;

    // Session start timestamp (nanoseconds)
    ts_t start_ts() const;

    // Session end timestamp
    ts_t end_ts() const;

    // Sensors metadata from session
    const sensors_map& sensors() const;

    // Keys for chunked/framed requests for Reader.messages(frame_key)
    std::vector<uint64_t> frames_keys() const;

    // Get the number of chunks/frames.
    size_t frames_count() const;

    // Map of chunk/frames with offsets (internal)
    const frames_map_t& frames_map() const;

    // Direct access to the session data storage (use with care)
    inline const uint8_t* buf() const { return buf_; }

    // Convenience method to view the string representations
    std::string to_string() const;

    // OSF schema version
    OSF_VERSION version() const { return version_; }

   private:
    sensors_map read_sensors();

    OSF_VERSION version_;

    const uint8_t* buf_;

    // Cached reconstructed sensor map
    sensors_map smap_;

    // This map can be on of:
    //   1 - in OSF_STREAM mode it maps timestamp   to chunk_offset
    //   2 - in OSF_FRAMED mode it maps frame_index to chunk_offset
    frames_map_t cmap_;
};

}  // namespace OSF
}  // namespace ouster