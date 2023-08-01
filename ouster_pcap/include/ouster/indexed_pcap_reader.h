/**
 * Copyright (c) 2023, Ouster Inc.
 */

#pragma once

#include "ouster/os_pcap.h"
#include "ouster/pcap.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor_utils {

struct PcapIndex {
    using frame_index =
        std::vector<uint64_t>;  ///< Maps a frame number to a file offset
    std::vector<frame_index> frame_indices_;  ///< frame index for each sensor

    PcapIndex(size_t num_sensors) : frame_indices_(num_sensors) {}

    /**
     * Returns the number of frames in the frame index for the given sensor
     * index.
     *
     * @param sensor_index[in] The position of the sensor for which to retrieve
     * the desired frame count.
     * @return The number of frames in the sensor's frame index.
     */
    size_t frame_count(size_t sensor_index) const;

    /**
     * Seeks the given reader to the given frame number for the given sensor
     * index
     */
    void seek_to_frame(PcapReader& reader, size_t sensor_index,
                       unsigned int frame_number);
};

/**
 * A PcapReader that allows seeking to the start of a lidar frame.
 *
 * The index must be computed by iterating through all packets and calling
 * `update_index_for_current_packet()` for each one.
 */
struct IndexedPcapReader : public PcapReader {
    /**
     * @param pcap_filename[in] A file path of the pcap to read
     * @param metadata_filenames[in] A vector of sensor metadata file paths
     */
    IndexedPcapReader(const std::string& pcap_filename,
                      const std::vector<std::string>& metadata_filenames);

    const PcapIndex& get_index() const;

    /**
     * Attempts to match the current packet to one of the sensor info objects
     * and returns the appropriate packet format if there is one
     *
     * @return An optional packet format for the current packet
     */
    nonstd::optional<size_t> sensor_idx_for_current_packet() const;

    /**
     * @return the current packet's frame_id
     * if the packet is associated with a sensor (and its corresponding packet
     * format)
     */
    nonstd::optional<uint16_t> current_frame_id() const;

    /**
     * Updates the frame index for the current packet
     * @return the progress of indexing as an int from [0, 100]
     */
    int update_index_for_current_packet();

    /**
     * Return true if the frame_id from the packet stream has rolled over,
     * hopefully avoiding spurious result that could occur from out of order or
     * dropped packets.
     *
     * @return true if the frame id has rolled over.
     */
    static bool frame_id_rolled_over(uint16_t previous, uint16_t current);

    std::vector<ouster::sensor::sensor_info>
        sensor_infos_;  ///< A vector of sensor_info that correspond to the
                        ///< provided metadata files
    PcapIndex index_;
    std::vector<nonstd::optional<uint16_t>>
        previous_frame_ids_;  ///< previous frame id for each sensor
};

}  // namespace sensor_utils
}  // namespace ouster
