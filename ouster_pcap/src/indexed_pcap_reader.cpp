#include "ouster/indexed_pcap_reader.h"

#include <iostream>
#include <stdexcept>

#include "ouster/os_pcap.h"
#include "ouster/packet.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor_utils {

IndexedPcapReader::IndexedPcapReader(
    const std::string& pcap_filename,
    const std::vector<std::string>& metadata_filenames)
    : PcapReader(pcap_filename),
      index_(metadata_filenames.size()),
      previous_frame_ids_(metadata_filenames.size()),
      filename_(pcap_filename) {
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
      previous_frame_ids_(sensor_infos.size()),
      filename_(pcap_filename) {
    init_();
}

static std::vector<std::pair<int, int>> guess_ports(
    ouster::sensor_utils::stream_info& stream_info,
    const ouster::sensor::sensor_info& sensor_info) {
    ouster::sensor::packet_format pf(sensor_info);
    std::vector<std::pair<int, int>> guesses;
    for (const auto& guess : ouster::sensor_utils::guess_ports(
             stream_info, pf.lidar_packet_size, pf.imu_packet_size,
             sensor_info.config.udp_port_lidar.value_or(0),
             sensor_info.config.udp_port_imu.value_or(0))) {
        guesses.push_back({guess.lidar, guess.imu});
    }
    std::sort(guesses.begin(), guesses.end(), [](auto a, auto b) {
        // Prefer lidar port being present
        if ((a.first != 0) != (b.first != 0)) {
            return (a.first != 0) > (b.first != 0);
        }
        // Secondly prefer imu port being present
        if ((a.second != 0) != (b.second != 0)) {
            return (a.second != 0) > (b.second != 0);
        }
        // Finally sort by port numbers
        return a > b;  // Default comparison based on the entire vector
    });
    return guesses;
}

static ouster::sensor_utils::stream_info packet_info_stream(
    const std::string& path, int n_packets) {
    return *ouster::sensor_utils::get_stream_info(path, n_packets);
}

void IndexedPcapReader::init_() {
    // sample pcap and attempt to find UDP ports consistent with metadatas
    // NOTE[pb]: Needed for port guessing logic for old single sensor data.
    auto n_packets = 1000;
    auto stats = packet_info_stream(filename_, n_packets);

    uint64_t index = 0;
    for (auto& it : sensor_infos_) {
        std::string sn_lidar = std::to_string(it.sn);
        std::string sn_imu = "LEGACY_IMU";
        if (it.config.udp_profile_lidar ==
            ouster::sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
            sn_lidar = "LEGACY_LIDAR";
        }
        packet_formats_.push_back(ouster::sensor::packet_format(it));

        // guess ports
        auto guesses = guess_ports(stats, it);
        if (guesses.size() > 0) {
            auto lidar_guess = guesses[0].first;
            auto imu_guess = guesses[0].second;
            if (!it.config.udp_port_lidar) {
                it.config.udp_port_lidar = lidar_guess;
            }
            if (!it.config.udp_port_imu) {
                it.config.udp_port_imu = imu_guess;
            }
        }

        if (!it.config.udp_port_lidar) {
            // printf("WARNING: udp_port_lidar not known for sensor %s\n",
            // sn_lidar.c_str());
        } else if (port_map_[*it.config.udp_port_lidar].find(sn_lidar) !=
                   port_map_[*it.config.udp_port_lidar].end()) {
            std::cout << "Duplicate lidar port/sn found for indexing pcap: " +
                             sn_lidar + ":" +
                             std::to_string(*it.config.udp_port_lidar)
                      << std::endl;
            throw PcapDuplicatePortException(
                "Duplicate lidar port/sn found for indexing pcap: " + sn_lidar +
                ":" + std::to_string(*it.config.udp_port_lidar));
        } else {
            port_map_[*it.config.udp_port_lidar][sn_lidar] = index;
        }
        if (!it.config.udp_port_imu) {
            // printf("WARNING: udp_port_imu not known for sensor %s\n",
            // sn_lidar.c_str());
        } else if (port_map_[*it.config.udp_port_imu].find(sn_imu) !=
                   port_map_[*it.config.udp_port_imu].end()) {
            std::cout << "Duplicate imu port/sn found for indexing pcap: " +
                             sn_imu + ":" +
                             std::to_string(*it.config.udp_port_imu)
                      << std::endl;
            throw PcapDuplicatePortException(
                "Duplicate imu port/sn found for indexing pcap: " + sn_imu +
                ":" + std::to_string(*it.config.udp_port_imu));
        } else {
            port_map_[*it.config.udp_port_imu][sn_imu] = index;
        }

        index++;
    }
}

nonstd::optional<size_t> IndexedPcapReader::sensor_idx_for_current_packet(
    bool soft_id_check) const {
    return check_sensor_idx_for_current_packet(soft_id_check).second;
}

std::pair<IdxErrorType, nonstd::optional<size_t>>
IndexedPcapReader::check_sensor_idx_for_current_packet(
    bool soft_id_check) const {
    nonstd::optional<size_t> value;
    IdxErrorType error = IdxErrorType::None;
    const auto& pkt_info = current_info();
    auto temp_match = port_map_.find(pkt_info.dst_port);
    if (temp_match != port_map_.end()) {
        for (auto it : temp_match->second) {
            auto& pf = packet_formats_[it.second];
            auto type = ouster::sensor::PacketType::Lidar;
            if (pkt_info.payload_size == pf.imu_packet_size) {
                type = ouster::sensor::PacketType::Imu;
            }
            auto res = validate_packet(sensor_infos_[it.second], pf, data_,
                                       pkt_info.payload_size, type);
            if (res == ouster::sensor::PacketValidationFailure::NONE) {
                return {IdxErrorType::None, it.second};
            } else if (res == ouster::sensor::PacketValidationFailure::ID) {
                if (soft_id_check) {
                    if (value) {
                        throw std::runtime_error(
                            "Soft ID Checking Does NOT Work With Multiple "
                            "Sensors");
                    }
                    value = it.second;
                }
                error = IdxErrorType::Id;
            } else if (res ==
                       ouster::sensor::PacketValidationFailure::PACKET_SIZE) {
                // Dont let a size error override an id error
                if (error == IdxErrorType::None) {
                    error = IdxErrorType::Size;
                }
            }
        }
    }
    return {error, value};
}

nonstd::optional<uint16_t> IndexedPcapReader::current_frame_id() const {
    const auto& pkt_info = current_info();
    if (pkt_info.payload_size == 48) {
        return nonstd::nullopt;
    }
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
                index_.global_frame_indices_.push_back(
                    {current_info().file_offset, *sensor_info_idx,
                     static_cast<uint64_t>(current_info().timestamp.count())});
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

const std::vector<ouster::sensor::sensor_info>& IndexedPcapReader::sensor_info()
    const {
    return sensor_infos_;
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
