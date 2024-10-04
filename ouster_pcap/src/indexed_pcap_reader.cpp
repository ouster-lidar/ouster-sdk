#include "ouster/indexed_pcap_reader.h"

#include <iostream>
#include <stdexcept>

#include "ouster/types.h"

namespace ouster {
namespace sensor_utils {

IndexedPcapReader::IndexedPcapReader(
    const std::string& pcap_filename,
    const std::vector<std::string>& metadata_filenames)
    : PcapReader(pcap_filename),
      index_(metadata_filenames.size()),
      previous_frame_ids_(metadata_filenames.size()) {
    for (const std::string& metadata_filename : metadata_filenames) {
        auto temp_info = ouster::sensor::metadata_from_json(metadata_filename);
        sensor_infos_.push_back(temp_info);
    }
    init_();
}

IndexedPcapReader::IndexedPcapReader(
    const std::string& pcap_filename,
    const std::vector<ouster::sensor::sensor_info>& sensor_infos)
    : PcapReader(pcap_filename),
      sensor_infos_(sensor_infos),
      index_(sensor_infos.size()),
      previous_frame_ids_(sensor_infos.size()) {
    init_();
}

void IndexedPcapReader::init_() {
    uint64_t index = 0;
    for (auto it : sensor_infos_) {
        std::string sn_lidar = it.sn;
        std::string sn_imu = "LEGACY_IMU";
        if (it.config.udp_profile_lidar ==
            ouster::sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
            sn_lidar = "LEGACY_LIDAR";
        }
        packet_formats_.push_back(ouster::sensor::packet_format(it));

        if (port_map_[*it.config.udp_port_lidar].find(sn_lidar) !=
            port_map_[*it.config.udp_port_lidar].end()) {
            std::cout << "Duplicate lidar port/sn found for indexing pcap: " +
                             sn_lidar + ":" +
                             std::to_string(*it.config.udp_port_lidar)
                      << std::endl;
            throw std::runtime_error(
                "Duplicate lidar port/sn found for indexing pcap: " + sn_lidar +
                ":" + std::to_string(*it.config.udp_port_lidar));
        }
        port_map_[*it.config.udp_port_lidar][sn_lidar] = index;
        if (port_map_[*it.config.udp_port_imu].find(sn_imu) !=
            port_map_[*it.config.udp_port_imu].end()) {
            std::cout << "Duplicate imu port/sn found for indexing pcap: " +
                             sn_imu + ":" +
                             std::to_string(*it.config.udp_port_imu)
                      << std::endl;
            throw std::runtime_error(
                "Duplicate imu port/sn found for indexing pcap: " + sn_imu +
                ":" + std::to_string(*it.config.udp_port_imu));
        }
        port_map_[*it.config.udp_port_imu][sn_imu] = index;

        index++;
    }
}

nonstd::optional<size_t> IndexedPcapReader::sensor_idx_for_current_packet()
    const {
    const auto& pkt_info = current_info();
    auto temp_match = port_map_.find(pkt_info.dst_port);
    if (temp_match != port_map_.end()) {
        for (auto it : temp_match->second) {
            auto res = validate_packet(
                sensor_infos_[it.second], packet_formats_[it.second], data,
                pkt_info.payload_size,
                ouster::sensor::PacketValidationType::LIDAR);
            if (res == ouster::sensor::PacketValidationFailure::NONE) {
                return it.second;
            }
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
                *previous_frame_ids_[*sensor_info_idx] < *frame_id ||
                frame_id_rolled_over(*previous_frame_ids_[*sensor_info_idx],
                                     *frame_id)) {
                index_.frame_indices_[*sensor_info_idx].push_back(
                    current_info().file_offset);
                index_.frame_timestamp_indices_[*sensor_info_idx].insert(
                    {current_info().timestamp.count(),
                     current_info().file_offset});
                index_.frame_id_indices_[*sensor_info_idx].insert(
                    {*frame_id, current_info().file_offset});
                previous_frame_ids_[*sensor_info_idx] = *frame_id;
            }
        }
    }

    return static_cast<int>(100 * static_cast<float>(current_offset()) /
                            file_size());
}

void IndexedPcapReader::build_index() {
    index_.clear();
    reset();
    while (next_packet() != 0) update_index_for_current_packet();
    reset();
}

const PcapIndex& IndexedPcapReader::get_index() const { return index_; }

void PcapIndex::clear() {
    for (size_t i = 0; i < frame_indices_.size(); ++i) {
        frame_indices_[i].clear();
        frame_timestamp_indices_[i].clear();
        frame_id_indices_[i].clear();
    }
}

void PcapIndex::seek_to_frame(PcapReader& reader, size_t sensor_index,
                              unsigned int frame_number) {
    reader.seek(frame_indices_.at(sensor_index).at(frame_number));
}

size_t PcapIndex::frame_count(size_t sensor_index) const {
    return frame_indices_.at(sensor_index).size();
}

}  // namespace sensor_utils
}  // namespace ouster
