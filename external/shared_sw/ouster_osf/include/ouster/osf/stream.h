#include <map>
#include <memory>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/osf/util.h"
#include "ouster/osf/version.h"

constexpr int NUM_SENSORS_PER_VEHICLE = 3;

namespace ouster {
namespace OSF {

// The OSFStream class manages a recorded session. It is saving OSFChunks into a
// temp file until we decide to finish the session, then it merges the session
// header containing the ts<>offset map and the stream of chunks into a single
// file
struct OSFChunk;

class OSFStream {
   public:
    OSF::sensors_map sensor_map;
    std::map<uint64_t, uint64_t> ts_to_offset;
    std::unique_ptr<OSFChunk> chunk_builder;
    uint64_t current_offset;
    uint64_t ts_start;
    uint64_t ts_end;
    bool newChunk;
    const std::string dump_path;
    const std::string temp_path;
    size_t header_size;
    int range_multiplier;
    std::string session_id;

    OSFStream(
        const OSF::sensors_map& sensors,
        const std::string& dump_path,
        const OSF::OSF_FRAME_MODE lidar_frame_mode =
            OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_32,
        const int range_multiplier = ouster::OSF::RANGE_MULTIPLIER_DEFAULT);
    ~OSFStream();

    void logLidarScan(uint8_t id, LidarScan& lidar_scan, uint64_t ts);
    void logTrajectory(uint8_t id, const osf_poses& poses, uint64_t ts);
    void logGps(uint8_t id, const Gps& gps, uint64_t ts);
    void logImu(uint8_t id, const std::array<double, 3>& angular_vel,
                const std::array<double, 3>& linear_accel,
                const std::array<uint64_t, 3>& ts_v, uint64_t ts);

    void setSessionId(std::string sId);

    std::string getSessionId();

    // updateStartEnd keeps track of the start and the end of the recorded
    // session
    void updateStartEnd(uint64_t ts);

    // createSession is called to create the session header and aggregate it
    // with the stream of chunks from the temp file in a single file. Returns
    // the name of the file created
    std::string createSession(const std::string& file_name = "");

    // checkNewChunk checks if we are creating a new empty chunk and add the ts
    // to the map ts<>offset to be stored in the session header
    void checkNewChunk(uint64_t ts);
    // checkChunk simply checks the size of the current chunk, saves it if it
    // reached the max_size and creates a new one
    void checkChunk(uint64_t ts);
    // checkTempFileIsInit checks that the temp_file used to log the chunks has
    // been initialized
    void checkTempFileIsInit();
    // saveChunk saves the current chunk to a temp file, aggregating all the
    // chunks in it until we call createSession() to merge them with the session
    // header
    void saveChunk();

    // get current size of stream
    size_t getSize();

};
}  // namespace OSF
}  // namespace ouster
