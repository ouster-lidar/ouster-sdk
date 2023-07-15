#include "ouster/indexed_pcap_reader.h"

namespace ouster {
namespace sensor_utils {

IndexedPcapReader::IndexedPcapReader(
    const std::string& pcap_filename,
    const std::vector<std::string>& metadata_filenames)
    : PcapReader(pcap_filename),
      index_(metadata_filenames.size()),
      previous_frame_ids_(metadata_filenames.size()) {
    for (const std::string& metadata_filename : metadata_filenames) {
        sensor_infos_.push_back(
            ouster::sensor::metadata_from_json(metadata_filename));
    }
}

nonstd::optional<size_t> IndexedPcapReader::sensor_idx_for_current_packet()
    const {
    const auto& pkt_info = current_info();
    for (size_t i = 0; i < sensor_infos_.size(); i++) {
        if (pkt_info.dst_port == sensor_infos_[i].udp_port_lidar) {
            // TODO use the packet format and match serial number if it's
            // available this will allow us to have multiple sensors on the same
            // port
            return i;
        }
    }
    return nonstd::nullopt;
}

nonstd::optional<uint16_t> IndexedPcapReader::current_frame_id() const {
    if (nonstd::optional<size_t> sensor_idx = sensor_idx_for_current_packet()) {
        const ouster::sensor::packet_format& pf =
            ouster::sensor::packet_format(sensor_infos_[*sensor_idx]);
        return pf.frame_id(current_data());
    }
    return nonstd::nullopt;
}

bool IndexedPcapReader::frame_id_rolled_over(uint16_t previous,
                                             uint16_t current) {
    static_assert(sizeof(uint16_t) == 2, "expected frame_id to be two bytes");
    return previous > 0xff00 && current < 0x00ff;
}

int IndexedPcapReader::update_index_for_current_packet() {
    if (nonstd::optional<size_t> sensor_info_idx =
            sensor_idx_for_current_packet()) {
        if (nonstd::optional<uint16_t> frame_id = current_frame_id()) {
            if (!previous_frame_ids_[*sensor_info_idx] ||
                *previous_frame_ids_[*sensor_info_idx] <
                    *frame_id  // frame_id is greater than previous
                || frame_id_rolled_over(*previous_frame_ids_[*sensor_info_idx],
                                        *frame_id)) {
                index_.frame_indices_[*sensor_info_idx].push_back(
                    current_info().file_offset);
                previous_frame_ids_[*sensor_info_idx] = *frame_id;
            }
        }
    }
    return static_cast<int>(100 * static_cast<float>(current_offset()) /
                            file_size());
}

const PcapIndex& IndexedPcapReader::get_index() const { return index_; }

void PcapIndex::seek_to_frame(PcapReader& reader, size_t sensor_index,
                              unsigned int frame_number) {
    reader.seek(frame_indices_.at(sensor_index).at(frame_number));
}

size_t PcapIndex::frame_count(size_t sensor_index) const {
    return frame_indices_.at(sensor_index).size();
}

}  // namespace sensor_utils
}  // namespace ouster
