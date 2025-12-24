/**
 * Copyright (c) 2023, Ouster Inc.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/deprecation.h"
#include "ouster/os_pcap.h"
#include "ouster/pcap.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace pcap {

/**
 * @brief Represents a global index for a frame in a pcap file.
 *
 * This structure contains the file offset, sensor index, and timestamp
 * for a specific frame in the pcap file.
 */
struct OUSTER_API_CLASS GlobalIndex {
    uint64_t
        file_offset;  ///< Byte offset in the PCAP file where the frame starts.
    uint64_t sensor_index;  ///< Sensor index corresponding to the frame
    uint64_t timestamp;     ///< Timestamp associated with the packet.
};

/**
 * @brief Indexing structure for efficient lookup of frames in a PCAP file.
 *
 * Provides indices to map sensor-specific and global frame
 * numbers to their corresponding byte offsets in the PCAP file.
 * This allows fast random access to lidar frames for analysis or replay.
 */
class OUSTER_API_CLASS PcapIndex {
   public:
    using frame_index =
        std::vector<uint64_t>;  ///< Maps a frame number to a file offset

    std::vector<frame_index> frame_indices;  ///< frame index for each sensor

    std::vector<GlobalIndex>
        global_frame_indices;  ///< frame index for all sensors, contains the
                               ///< offset followed by the index of each sensor
    /**
     * Used to locate packets based on precise capture timestamps.
     */
    using timestamp_index = std::unordered_map<uint64_t, uint64_t>;

    // TODO: this isn't used for now but in the future might be used to
    //  solve the issue with the limited frame_id span (we could remove it)
    /**
     * Used for efficient lookup of packets based on timestamps.
     */
    std::vector<timestamp_index> frame_timestamp_indices;

    /**
     * Allows lookup of packet positions based on frame IDs.
     */
    using frame_id_index = std::unordered_map<int32_t, uint64_t>;

    // TODO[IMPORTANT]: this has an issue if the recorded pcap file to span
    // over 50 mins.
    /**
     * Used for efficient lookup of packets based on frame IDs.
     */
    std::vector<frame_id_index> frame_id_indices;

    /**
     * @brief Constructs a PcapIndex for the specified number of sensors.
     *
     * Initializes frame indices and timestamp-based indices for each sensor.
     *
     * @param[in] num_sensors The number of sensors captured in the PCAP.
     */
    OUSTER_API_FUNCTION
    PcapIndex(size_t num_sensors)
        : frame_indices(num_sensors),
          frame_timestamp_indices(num_sensors),
          frame_id_indices(num_sensors) {}

    /**
     * Simple method to clear the index.
     */
    OUSTER_API_FUNCTION
    void clear();

    /**
     * Returns the number of frames in the frame index for the given sensor
     * index.
     *
     * @param[in] sensor_index The position of the sensor for which to retrieve
     * the desired frame count.
     * @return The number of frames in the sensor's frame index.
     */
    OUSTER_API_FUNCTION
    size_t frame_count(size_t sensor_index) const;

    // TODO[UN]: in my opinion we are better off removing this method from this
    // class It is better if we keep this class as a simple POD object. Another
    // problem with this method specifically is that it creates a cyclic and
    // this is the reason why we are passing PcapReader instead of
    // IndexedPcapReader to avoid this cyclic relation. If it mounts to anything
    // this method should be part of the IndexedPcapReader
    /**
     * Seeks the given reader to the given frame number for the given sensor
     * index
     *
     * @param[in,out] reader The reader to use for seeking.
     * @param[in] sensor_index The position of the sensor for which to
     *                         seek for.
     * @param[in] frame_number The frame number to seek to.
     */
    OUSTER_API_FUNCTION
    void seek_to_frame(PcapReader& reader, size_t sensor_index,
                       unsigned int frame_number);
};

OUSTER_DIAGNOSTIC_PUSH
OUSTER_DIAGNOSTIC_IGNORE_UNUSED
/**
 * @brief Error type returned during index validation or seeking operations.
 */
enum class IdxErrorType {
    NONE,
    SIZE,
    ID,
    None = IdxErrorType::NONE,
    Size = IdxErrorType::SIZE,
    Id = IdxErrorType::ID
};
OUSTER_DIAGNOSTIC_POP

/**
 * @brief Constructs the PcapDuplicatePortException with a descriptive message.
 *
 * @param[in] msg Description of the duplicate port error encountered during
 * PCAP parsing.
 */
class OUSTER_API_CLASS PcapDuplicatePortException : public std::runtime_error {
   public:
    /**
     * @brief Exception thrown when duplicate sensor ports are detected during
     * PCAP indexing.
     *
     * This exception is raised when two or more sensors are found to be using
     * the same UDP port (for either lidar or IMU data), which would prevent
     * unambiguous packet-to-sensor mapping during index construction.
     *
     * @param[in] msg Descriptive error message indicating the conflicting port
     * and sensor serial.
     *
     * @throws PcapDuplicatePortException always
     */
    OUSTER_API_FUNCTION
    PcapDuplicatePortException(const std::string& msg) : runtime_error(msg) {}
};

/**
 * A PcapReader that allows seeking to the start of a lidar frame.
 *
 * The index must be computed by iterating through all packets and calling
 * `update_index_for_current_packet()` for each one.
 */
class OUSTER_API_CLASS IndexedPcapReader : public PcapReader {
   public:
    /**
     * @param[in] pcap_filename A file path of the pcap to read
     * @param[in] metadata_filenames A vector of sensor metadata filepaths
     */
    OUSTER_API_FUNCTION
    IndexedPcapReader(
        const std::string&
            pcap_filename,  ///<  [in] - A file path of the pcap to read
        const std::vector<std::string>&
            metadata_filenames  ///< [in] - A vector of sensor metadata file
                                ///< paths
    );

    /**
     * @param[in] pcap_filename A file path of the pcap to read
     * @param[in] sensor_infos A vector of sensor info structures for each
     * sensors
     */
    OUSTER_API_FUNCTION
    IndexedPcapReader(
        const std::string& pcap_filename,
        const std::vector<ouster::sdk::core::SensorInfo>& sensor_infos);

    /**
     * This method constructs the index. Call this method before requesting the
     * index information using get_index()
     */
    OUSTER_API_FUNCTION
    void build_index();

    /**
     * Get index for the underlying pcap
     *
     * @return returns a PcapIndex object
     */
    OUSTER_API_FUNCTION
    const PcapIndex& get_index() const;

    /**
     * Attempts to match the current packet to one of the sensor info objects
     * and returns the appropriate packet format if there is one
     *
     * @param[in] soft_id_check if id mismatches should be ignored
     *
     * @return An optional sensor index for the current packet
     */
    OUSTER_API_FUNCTION
    nonstd::optional<size_t> sensor_idx_for_current_packet(
        bool soft_id_check = false) const;

    /**
     * Attempts to match the current packet to one of the sensor info objects
     * and returns the appropriate packet format if there is one
     *
     * @param[in] soft_id_check if id mismatches should be ignored
     *
     * @return An optional packet format for the current packet
     */
    OUSTER_API_FUNCTION
    std::pair<IdxErrorType, nonstd::optional<size_t>>
    check_sensor_idx_for_current_packet(bool soft_id_check) const;

    /**
     * @return the current packet's frame_id
     * if the packet is associated with a sensor (and its corresponding packet
     * format)
     */
    OUSTER_API_FUNCTION
    nonstd::optional<uint16_t> current_frame_id() const;

    /**
     * Updates the frame index for the current packet
     *
     * @todo I recommend take this a private method,
     * the problem with exposing this method is that
     * it only yields right results if invoked sequentially
     * ; the results are dependent on the internal state.
     *
     * @return the progress of indexing as an int from [0, 100]
     */
    OUSTER_API_FUNCTION
    int update_index_for_current_packet();

    /**
     * Return true if the frame_id from the packet stream has rolled over,
     * hopefully avoiding spurious result that could occur from out of order or
     * dropped packets.
     *
     * @param[in] previous The previous frame id.
     * @param[in] current The current frame id.
     * @return true if the frame id has rolled over.
     */
    OUSTER_API_FUNCTION
    static bool frame_id_rolled_over(uint16_t previous, uint16_t current);

    /**
     * Get the SensorInfo for each sensor in this pcap.
     * @return the SensorInfo for each sensor in this pcap
     */
    OUSTER_API_FUNCTION
    const std::vector<ouster::sdk::core::SensorInfo>& sensor_info() const;

   protected:
    void init_();
    std::vector<ouster::sdk::core::SensorInfo>
        sensor_infos_;  ///< A vector of SensorInfo that correspond to the
                        ///< provided metadata files
    std::vector<ouster::sdk::core::PacketFormat> packet_formats_;
    PcapIndex index_;

    // TODO: remove, this should be a transient variable
    std::vector<nonstd::optional<uint16_t>>
        previous_frame_ids_;  ///< previous frame id for each sensor
    std::unordered_map<uint16_t, std::map<std::string, uint64_t>> port_map_;

    std::string filename_;
};

}  // namespace pcap
}  // namespace sdk
}  // namespace ouster
