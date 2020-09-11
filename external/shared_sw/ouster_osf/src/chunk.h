#include <map>
#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "ouster/osf/common.h"
#include "ouster/osf/util.h"
#include "ouster/osf/version.h"

#include "osfChunk_generated.h"

namespace ouster {
class LidarScan;
namespace OSF {

class OSFChunk {
   public:
    size_t max_chunk_size;
    flatbuffers::FlatBufferBuilder builder;
    std::vector<flatbuffers::Offset<StampedMessage>> message_stream;
    int range_multiplier;
    uint64_t start_ts;
    uint64_t stop_ts;
    std::string session_id;
    OSF_FRAME_MODE frame_mode;

    explicit OSFChunk(
        const int chunk_size,
        const OSF_FRAME_MODE frameMode = OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_32,
        const int range_multiplier = RANGE_MULTIPLIER_DEFAULT);
    void logLidarScan(const uint8_t id, const std::vector<int>& px_offset,
                      LidarScan& lidar_scan, uint64_t ts);
    void logTrajectory(const uint8_t id, const osf_poses& poses, uint64_t ts);
    void logImu(const uint8_t id, const std::array<double, 3>& angular_vel,
                const std::array<double, 3>& linear_accel,
                const std::array<uint64_t, 3>& ts_v, const uint64_t ts);
    void logGps(const uint8_t id, const Gps& gps, uint64_t ts);
    void logMessage(const StampedMessage* msg);
    void logScanMessage(const StampedMessage* msg);
    void logTrajectoryMessage(const StampedMessage* msg);
    void logGpsMessage(const StampedMessage* msg);
    void logImuMessage(const StampedMessage* msg);

    void setFrameMode(OSF_FRAME_MODE fMode);
    OSF_FRAME_MODE getFrameMode();

    void restartChunk();

    void updateEnd(uint64_t ts);
    bool checkChunkSize(uint64_t ts);
    void checkFirstMessage(uint64_t ts);
    size_t getSize();
    size_t saveChunk(std::string filename);
};
}  // namespace OSF
}  // namespace ouster
