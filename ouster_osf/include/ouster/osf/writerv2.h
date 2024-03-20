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

/**
 * Simplified OSF writer class.
 */
class WriterV2 {
   public:
    /**
     * @param[in] filename The filename to output to.
     * @param[in] info The sensor info to use for a single stream OSF file.
     * @param chunk_size The chunksize to use for the OSF file, this parameter
     *                   is optional.
     */
    WriterV2(const std::string& filename,
             const ouster::sensor::sensor_info& info, uint32_t chunk_size = 0);

    /**
     * @param[in] filename The filename to output to.
     * @param[in] info The sensor info vector to use for a multi stream OSF
     *                 file.
     * @param chunk_size The chunksize to use for the OSF file, this parameter
     *                   is optional.
     */
    WriterV2(const std::string& filename,
             const std::vector<ouster::sensor::sensor_info>& info,
             uint32_t chunk_size = 0);

    /**
     * Save a single scan to the specified stream_index in an OSF file.
     * The concept of the stream_index is related to the sensor_info vector.
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     WriterV2 output = WriterV2(filename, {info1, info2, info3});

     LidarScan scan = RANDOM_SCAN_HERE;

     // To save the LidarScan of scan to the first sensor, you would do the
     // following
     output.save(0, scan);

     // To save the LidarScan of scan to the second sensor, you would do the
     // following
     output.save(1, scan);

     // To save the LidarScan of scan to the third sensor, you would do the
     // following
     output.save(2, scan);
     @endcode
     *
     * @throws std::logic_error Will throw exception on writer being closed.
     * @throws std::logic_error ///< Will throw exception on
     *                          ///< out of bound stream_index.
     *
     * @param[in] stream_index The index of the corrosponding sensor_info to
     *                         use.
     * @param[in] scan The scan to save.
     */
    void save(uint32_t stream_index, const LidarScan& scan);

    /**
     * Save multiple scans in an OSF file.
     * The concept of the stream_index is related to the sensor_info vector.
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     WriterV2 output = WriterV2(filename, {info1, info2, info3});

     LidarScan sensor1_scan = RANDOM_SCAN_HERE;
     LidarScan sensor2_scan = RANDOM_SCAN_HERE;
     LidarScan sensor3_scan = RANDOM_SCAN_HERE;

     // To save the scans matched appropriately to their sensors, you would do
     // the following
     output.save({sensor1_scan, sensor2_scan, sensor3_scan});
     @endcode
     *
     *
     * @throws std::logic_error Will throw exception on writer being closed
     *
     * @param[in] scans The vector of scans to save.
     */
    void save(const std::vector<LidarScan>& scans);

    /**
     * Return the sensor info vector.
     *
     * @return The sensor info vector.
     */
    const std::vector<ouster::sensor::sensor_info>& get_sensor_info() const;

    /**
     * Get the specified sensor info
     * Consider the following:
     @code{.cpp}
     sensor_info info1; // The first sensor in this OSF file
     sensor_info info2; // The second sensor in this OSF file
     sensor_info info3; // The third sensor in this OSF file

     WriterV2 output = WriterV2(filename, {info1, info2, info3});

     // The following will be true
     output.get_sensor_info(0) == info1;
     output.get_sensor_info(1) == info2;
     output.get_sensor_info(2) == info3;
     @endcode
     *
     * @param[in] sensor_info The sensor info to return.
     * @return The correct sensor info
     */
    const ouster::sensor::sensor_info get_sensor_info(int stream_index) const;

    /**
     * Get the number of sensor_info objects.
     *
     * @return The sensor_info count.
     */
    uint32_t sensor_info_count() const;

    /**
     * Get the OSF filename.
     *
     * @return The OSF filename.
     */
    const std::string& get_filename() const;

    /**
     * Get the OSF chunk size
     *
     * @return The OSF chunk size.
     */
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
