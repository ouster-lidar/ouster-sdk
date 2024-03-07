/**
 * Copyright(c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include <map>
#include <memory>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/osf/writer.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

// @todo write documentation
class WriterV2 {
   public:
    WriterV2(const std::string& filename,
             const ouster::sensor::sensor_info& info, uint32_t chunk_size = 0);
    WriterV2(const std::string& filename,
             const std::vector<ouster::sensor::sensor_info>& info,
             uint32_t chunk_size = 0);

    void save(uint32_t stream_index, const LidarScan& scan);
    void save(const std::vector<LidarScan>& scans);

    const std::vector<ouster::sensor::sensor_info>& get_sensor_info() const;
    const ouster::sensor::sensor_info get_sensor_info(int stream_index) const;
    uint32_t sensor_info_count() const;

    const std::string& get_filename() const;

    uint32_t get_chunk_size() const;

    void close();
    bool is_closed() const;

   protected:
    const std::string filename;
    const std::vector<ouster::sensor::sensor_info> info;
    const uint32_t chunk_size;
    std::map<uint32_t, std::unique_ptr<LidarScanStream>> streams;
    std::map<uint32_t, uint32_t> meta_id;
    std::unique_ptr<Writer> writer;

private:
    void _save(uint32_t stream_index, const LidarScan& scan);
};

}  // namespace osf
}  // namespace ouster
