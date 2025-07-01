/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/pcap_packet_source.h"

#include "ouster/compat_ops.h"
#include "ouster/indexed_pcap_reader.h"

namespace ouster {
namespace pcap {

PcapPacketSourceOptions::PcapPacketSourceOptions(const PacketSourceOptions& o)
    : PacketSourceOptions(o) {}

PcapPacketSourceOptions::PcapPacketSourceOptions() {}

class PcapPacketIteratorImpl : public ouster::core::PacketIteratorImpl {
    PcapPacketSource* source_;
    std::pair<int, std::shared_ptr<ouster::sensor::Packet>> packet_;
    int64_t current_location_;

   public:
    PcapPacketIteratorImpl(PcapPacketSource* source, uint64_t scan_index)
        : source_(source) {
        current_location_ = source_->start_location_;
        if (!source_->reader_) {
            throw std::runtime_error("Cannot iterate over a closed source.");
        }
        if (scan_index > 0) {
            const auto& index = source_->reader_->get_index();
            if (scan_index >= index.global_frame_indices_.size()) {
                throw std::out_of_range(
                    "Indexed past the end of the scan source.");
            }
            current_location_ =
                index.global_frame_indices_[scan_index].file_offset;
        }
    }

    // return true if we hit the end, return false otherwise
    bool advance(size_t offset) override {
        if (!source_->reader_) {
            throw std::runtime_error("Cannot iterate over a closed source.");
        }
        // seek to where we last were
        auto& reader = *source_->reader_;
        reader.seek(current_location_);
        for (size_t i = 0; i < offset; i++) {
            auto psize = reader.next_packet();
            if (psize == 0) {
                // we hit the end
                return true;
            }

            const auto& info = reader.current_info();
            const auto len = reader.current_length();
            const auto data = reader.current_data();
            const auto res = reader.check_sensor_idx_for_current_packet(
                source_->soft_id_check_);
            auto& idx = res.second;

            // increment our errors
            if (res.first == ouster::sensor_utils::IdxErrorType::Size) {
                source_->size_error_count_++;
            } else if (res.first == ouster::sensor_utils::IdxErrorType::Id) {
                source_->id_error_count_++;
            }
            if (!idx) {
                // no idea who this is from, try again
                offset++;
                continue;
            }

            // lets just always assume sensor index 0 for now
            const auto& pf = source_->packet_formats_[idx.value()];
            if (len == pf->lidar_packet_size) {
                packet_.second = std::shared_ptr<ouster::sensor::Packet>(
                    new ouster::sensor::LidarPacket());
            } else if (len == pf->imu_packet_size) {
                packet_.second = std::shared_ptr<ouster::sensor::Packet>(
                    new ouster::sensor::ImuPacket());
            } else {
                packet_.second.reset();
            }

            // Finalize the packet if we got one
            if (packet_.second) {
                packet_.first = idx.value();
                packet_.second->host_timestamp = info.timestamp.count() * 1000;
                packet_.second->format = pf;
                packet_.second->buf.resize(len);
                memcpy(packet_.second->buf.data(), data, len);
                continue;
            }

            // unknown packet, try again
            offset++;
        }
        // save seek location
        current_location_ = reader.current_offset();
        return false;
    }

    std::pair<int, std::shared_ptr<ouster::sensor::Packet>>& value() override {
        return packet_;
    }
};

// Drop the extension from a filename (but leave the dot)
static std::string drop_extension(const std::string& a) {
    auto index = a.find_last_of('.');
    if (index != std::string::npos) {
        return a.substr(0, index + 1);
    }
    return a;
}

static std::string get_extension(const std::string& a) {
    auto index = a.find_last_of('.');
    if (index != std::string::npos) {
        return a.substr(index);
    }
    return "";
}

// Calculate the common non-extension prefix between two filenames
static size_t common_prefix_length(const std::string& a, const std::string& b) {
    std::string a_prefix = drop_extension(a);
    std::string b_prefix = drop_extension(b);
    size_t min_length = std::min(a_prefix.length(), b_prefix.length());
    for (size_t i = 0; i < min_length; i++) {
        if (a_prefix[i] != b_prefix[i]) {
            return i;
        }
    }
    return min_length;
}

#include <algorithm>
#include <string>
#include <vector>

static std::string get_directory(const std::string& str) {
    auto found = str.find_last_of("/\\");
    if (found == std::string::npos) {
        return ".";
    }
    return str.substr(0, found);
}

static std::string get_filename(const std::string& str) {
    auto found = str.find_last_of("/\\");
    if (found == std::string::npos) {
        return str;
    }
    return str.substr(found + 1);
}

std::vector<std::string> resolve_metadata_multi(const std::string& data_path) {
    // get all json files in this directory
    std::string dir = get_directory(data_path);
    std::string root_file = get_filename(data_path);
    std::vector<std::string> files = ouster::core::files_in_directory(dir);

    std::vector<std::pair<int, std::string>> scores;
    for (const auto& file : files) {
        // only grab .json files
        if (get_extension(file) != ".json") {
            continue;
        }

        auto score = common_prefix_length(root_file, file);
        scores.push_back({score, file});
    }

    std::sort(
        scores.begin(), scores.end(),
        [](const std::pair<int, std::string>& a,
           const std::pair<int, std::string>& b) { return a.first > b.first; });

    // finally grab all scans with the same score
    std::vector<std::string> found;
    auto best_score = scores.size() ? scores.front().first : 0;
    for (const auto& file : scores) {
        if (file.first == 0) {
            continue;
        }
        if (file.first != best_score) {
            break;
        }
        // todo this might not be the best method
        found.push_back(dir + "/" + file.second);
    }
    if (found.size() == 0) {
        throw std::runtime_error("Could not find matching metadata.");
    }
    return found;
}

static std::vector<ouster::sensor::sensor_info> find_metadata(
    const std::string& file, PcapPacketSourceOptions& options) {
    bool has_sensor_info = options.sensor_info.retrieve().size();
    bool has_meta = options.meta.retrieve().size();
    if (has_sensor_info && has_meta) {
        throw std::invalid_argument(
            "Cannot provide both sensor_info and meta to PcapScanSource.");
    }
    if (has_sensor_info) {
        return options.sensor_info.retrieve();
    }
    auto meta =
        has_meta ? options.meta.retrieve() : resolve_metadata_multi(file);
    std::vector<ouster::sensor::sensor_info> list;
    for (const auto& file : meta) {
        auto temp_info = ouster::sensor::metadata_from_json(file);
        list.push_back(temp_info);
    }
    return list;
}

PcapPacketSource::PcapPacketSource(
    const std::string& n,
    const std::function<void(PcapPacketSourceOptions&)>& options)
    : PcapPacketSource(n, ouster::impl::get_packet_options(options)) {}

PcapPacketSource::PcapPacketSource(
    const std::string& file,  ///< [in] sensor hostnames to connect to, for
                              ///< multiple comma separate
    PcapPacketSourceOptions
        options  ///< [in] common scan source options or null for default
    )
    : reader_(new ouster::sensor_utils::IndexedPcapReader(
          file, find_metadata(file, options))),
      index_(options.index.retrieve()) {
    if (index_) {
        reader_->build_index();
    }
    soft_id_check_ = options.soft_id_check.retrieve();
    start_location_ = reader_->current_offset();

    // Determine metadata
    for (const auto& info : reader_->sensor_info()) {
        sensor_info_.emplace_back(new ouster::sensor::sensor_info(info));
    }

    ouster::populate_extrinsics(options.extrinsics_file.retrieve(),
                                options.extrinsics.retrieve(), sensor_info_);

    for (const auto& info : sensor_info()) {
        packet_formats_.emplace_back(
            std::make_shared<ouster::sensor::packet_format>(*info));
    }

    options.check("PcapPacketSource");
}

bool PcapPacketSource::is_live() const { return false; }

uint64_t PcapPacketSource::id_error_count() const { return id_error_count_; }

uint64_t PcapPacketSource::size_error_count() const {
    return size_error_count_;
}

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
PcapPacketSource::sensor_info() const {
    return sensor_info_;
}

ouster::core::PacketIterator PcapPacketSource::begin() const {
    return ouster::core::PacketIterator(
        this, new PcapPacketIteratorImpl((PcapPacketSource*)this, 0));
}

ouster::core::PacketIterator PcapPacketSource::begin_scan(
    uint64_t scan_index) const {
    if (!index_) {
        throw std::runtime_error("not supported on unindexed scan sources");
    }
    return ouster::core::PacketIterator(
        this, new PcapPacketIteratorImpl((PcapPacketSource*)this, scan_index));
}

void PcapPacketSource::close() { reader_.reset(); }
}  // namespace pcap
}  // namespace ouster
