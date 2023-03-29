/**
 * Copyright (c) 2023, Ouster Inc.
 */

#pragma once

#include "ouster/pcap.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor_utils {

/**
 * A PcapReader that allows seeking to the start of a lidar frame.
 * To do this, the constructor calls `get_stream_info`, which in turn calls
 * IndexedPcapReader::update_index_for_current_packet for each packet.
 * This allows us to compute the index and obtain the stream_info while reading
 * the PCAP file only once.
 */
struct IndexedPcapReader : public PcapReader {
    using frame_index = std::vector<uint64_t>;                      ///< Maps a frame number to a file offset

    /**
     * @param pcap_filename[in] A file path of the pcap to read
     * @param metadata_filenames[in] A vector of sensor metadata file paths
     */
    IndexedPcapReader(
        const std::string& pcap_filename,
        const std::vector<std::string>& metadata_filenames,
        std::function<void(uint64_t, uint64_t, uint64_t)> progress_callback
    );

    /**
     * Attempts to match the current packet to one of the sensor info objects
     * and returns the appropriate packet format if there is one
     *
     * @return An optional packet format for the current packet
     */
    nonstd::optional<size_t> sensor_idx_for_current_packet() const;

    /**
     * @return the current packet's frame_id
     * if the packet is associated with a sensor (and its corresponding packet format)
     */
    nonstd::optional<uint16_t> current_frame_id() const;

    /**
     * Updates the frame index for the current packet
     *
     * Important: this method is only meant to be invoked from `get_stream_info`!
     */
    void update_index_for_current_packet();

    /**
     * @return The stream_info associated with this PcapReader
     */
    std::shared_ptr<stream_info> get_stream_info() const; // TODO move to parent class

    /**
     * Seeks to the given frame number for the given sensor index
     */
    void seek_to_frame(size_t sensor_index, unsigned int frame_number);

    /**
     * Returns the number of frames in the frame index for the given sensor index.
     *
     * @param sensor_index[in] The position of the sensor for which to retrieve the desired frame count.
     * @return The number of frames in the sensor's frame index.
     */
    size_t frame_count(size_t sensor_index) const;

    /**
     * Return true if the frame_id from the packet stream has rolled over,
     * hopefully avoiding spurious result that could occur from out of order or dropped packets.
     *
     * @return true if the frame id has rolled over.
     */
    static bool frame_id_rolled_over(uint16_t previous, uint16_t current);

    std::vector<ouster::sensor::sensor_info> sensor_infos_;         ///< A vector of sensor_info that correspond to the provided metadata files
    std::shared_ptr<stream_info> stream_info_;                      ///< TODO: move to parent class
    std::vector<frame_index> frame_indices_;                        ///< frame index for each sensor
    std::vector<nonstd::optional<uint16_t>> previous_frame_ids_;  ///< previous frame id for each sensor
};

} // namespace sensor_utils
} // namespace ouster
