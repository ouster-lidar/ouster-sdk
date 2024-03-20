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

    using timestamp_index = std::unordered_map<uint64_t, uint64_t>;

    // TODO: this isn't used for now but in the future might be used to
    //  solve the issue with the limited frame_id span (we could remove it)
    std::vector<timestamp_index> frame_timestamp_indices_;

    using frame_id_index = std::unordered_map<int32_t, uint64_t>;

    // TODO[IMPORTANT]: this has an issue if the recorded pcap file to span
    // over 50 mins.
    std::vector<frame_id_index> frame_id_indices_;

    PcapIndex(size_t num_sensors)
        : frame_indices_(num_sensors),
          frame_timestamp_indices_(num_sensors),
          frame_id_indices_(num_sensors) {}

    /**
     * Simple method to clear the index.
     */
    void clear();

    /**
     * Returns the number of frames in the frame index for the given sensor
     * index.
     *
     * @param[in] sensor_index The position of the sensor for which to retrieve
     * the desired frame count.
     * @return The number of frames in the sensor's frame index.
     */
    size_t frame_count(size_t sensor_index) const;

    /**
     * Seeks the given reader to the given frame number for the given sensor
     * index
     */
    // TODO[UN]: in my opinion we are better off removing this method from this
    // class It is better if we keep this class as a simple POD object. Another
    // problem with this method specifically is that it creates a cyclic and
    // this is the reason why we are passing PcapReader instead of
    // IndexedPcapReader to avoid this cyclic relation. If it mounts to anything
    // this method should be part of the IndexedPcapReader.
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
     * @param[in] pcap_filename A file path of the pcap to read
     * @param[in] metadata_filenames A vector of sensor metadata file paths
     */
    IndexedPcapReader(const std::string& pcap_filename,
                      const std::vector<std::string>& metadata_filenames);

    /**
     * @param[in] pcap_filename A file path of the pcap to read
     * @param[in] sensor_infos A vector of sensor info structures for each
     * sensors
     */
    IndexedPcapReader(
        const std::string& pcap_filename,
        const std::vector<ouster::sensor::sensor_info>& sensor_infos);

    /**
     * This method constructs the index. Call this method before requesting the
     * index information using get_index()
     */
    void build_index();

    /**
     * Get index for the underlying pcap
     *
     * @return returns a PcapIndex object
     */
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
    // TODO: I recommend take this a private method, the problem with exposing
    // this method is that it only yields right results if invoked sequentially
    // ; the results are dependent on the internal state.
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

    // TODO: remove, this should be a transient variable
    std::vector<nonstd::optional<uint16_t>>
        previous_frame_ids_;  ///< previous frame id for each sensor
};

}  // namespace sensor_utils
}  // namespace ouster
