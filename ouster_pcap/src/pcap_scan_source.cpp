/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/pcap_scan_source.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ouster/impl/open_source_impl.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/packet_source.h"
#include "ouster/scan_source.h"
#include "ouster/types.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace pcap {

PcapScanSourceOptions::PcapScanSourceOptions(const ScanSourceOptions& opts)
    : ScanSourceOptions(opts) {}

PcapScanSourceOptions::PcapScanSourceOptions() = default;

class PcapScanIteratorImpl : public ScanIteratorImpl {
    const PcapScanSource* source_;
    PacketIterator packet_iter_;
    std::shared_ptr<LidarScan> scan_;
    int sensor_idx_;
    int64_t i_ = -1;

    std::vector<ScanBatcher> batchers_;
    std::vector<std::shared_ptr<LidarScan>> scans_;

   public:
    // Clarify
    explicit PcapScanIteratorImpl(const PcapScanSource* source,
                                  int sensor_idx = -1)
        : source_(source), sensor_idx_(sensor_idx < 0 ? -1 : sensor_idx) {
        packet_iter_ = source->packets_.begin();

        for (const auto& info : source_->sensor_info()) {
            batchers_.emplace_back(info);
        }

        scans_.resize(source_->sensor_info().size());
    }

    bool advance(size_t offset) override {
        // seek!
        if (offset > 1) {
            if (!source_->indexed_) {
                throw std::runtime_error(
                    "Not supported on non-indexed scan sources.");
            }

            if (i_ + static_cast<int64_t>(offset) >= length()) {
                throw std::out_of_range(
                    "Indexed past the end of the scan source.");
            }
            // seek to the start of this scan then allow the loop below to
            // extract it

            // need to reset batchers and scans
            int i = 0;
            for (auto& batcher : batchers_) {
                batcher.reset();
                scans_[i++].reset();
            }

            packet_iter_ = source_->packets_.begin_scan(i_ + offset);
            i_ += static_cast<int>(offset - 1);
            offset = 1;
        }
        for (size_t i = 0; i < offset; i++) {
            // read packets until we get a scan or run out
            while (true) {
                if (packet_iter_ == source_->packets_.end()) {
                    // if finished return any partial scans we have
                    // todo should these be in any particular order?
                    bool found = false;
                    for (size_t i = 0; i < scans_.size(); i++) {
                        if (scans_[i]) {
                            // ignore scans with no packets batched
                            if (batchers_[i].batched_packets() == 0) {
                                scans_[i].reset();
                                continue;
                            }
                            scan_.reset();
                            scan_.swap(scans_[i]);
                            found = true;
                            break;
                        }
                    }
                    if (found) {
                        i_++;
                        break;
                    }
                    return true;
                }

                // try and build a scan with this packet
                // make sure to copy this out as we increment right after
                auto packet = *packet_iter_;
                packet_iter_++;

                // todo handle other packet types in the future
                int index = packet.first;

                // skip packets from sensors we dont care about
                if (sensor_idx_ >= 0 && index != sensor_idx_) {
                    continue;
                }

                // allocate the scan if it hasnt already been
                if (!scans_[index]) {
                    scans_[index] = std::make_shared<LidarScan>(
                        source_->sensor_info()[index],
                        source_->field_types_[index]);
                }

                // finally batch
                if (batchers_[index](*packet.second, *scans_[index])) {
                    scan_.reset();
                    scan_.swap(scans_[index]);
                    i_++;
                    break;
                }
            }
        }
        return false;
    }

    int64_t length() override {
        source_->assert_indexed("length");
        if (sensor_idx_ >= 0) {
            return static_cast<int64_t>(source_->scans_num()[sensor_idx_]);
        }
        return static_cast<int64_t>(source_->size());
    }

    LidarScanSet value() override { return LidarScanSet{{scan_}}; }
};

ScanIterator PcapScanSource::begin() const {
    return ScanIterator(this, new PcapScanIteratorImpl(this));
}

ScanIterator PcapScanSource::begin(int sensor_index) const {
    if (sensor_index >= static_cast<int>(sensor_info().size())) {
        throw std::runtime_error("Invalid sensor index");
    }
    return ScanIterator(this, new PcapScanIteratorImpl(this, sensor_index));
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
PcapScanSource::individual_index() const {
    assert_indexed("individual_index");
    return index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& PcapScanSource::full_index()
    const {
    assert_indexed("full_index");
    return real_index_;
}

void PcapScanSource::assert_indexed(const char* function) const {
    if (!indexed_) {
        throw std::runtime_error(
            "Cannot perform '" + std::string(function) +
            "' on an unindexed source. Specify "
            "the index parameter as true when creating the source to "
            "produce an index.");
    }
}

/// open_source compatible constructor
PcapScanSource::PcapScanSource(
    const std::string& source,
    const std::function<void(PcapScanSourceOptions&)>& options)
    : PcapScanSource(source, ouster::sdk::impl::get_scan_options(options)) {}

/// open_source compatible constructor
PcapScanSource::PcapScanSource(const std::string& source,
                               PcapScanSourceOptions options)
    : packets_(source, PacketSourceOptions(
                           reinterpret_cast<ScanSourceOptions&>(options))),
      indexed_(options.index.retrieve()) {
    if (indexed_) {
        const auto& index = packets_.reader_->get_index();

        // count the scans
        num_scans_ = 0;
        for (size_t i = 0; i < sensor_info().size(); i++) {
            auto count = index.frame_count(i);
            scans_count_.push_back(count);
            num_scans_ += count;
        }

        // timestamp based index of all scans in the file for each sensor, each
        // pair is timestamp followed by global scan index build index
        index_.resize(sensor_info().size());
        for (const auto& idx : index.global_frame_indices) {
            real_index_.emplace_back(idx.timestamp, idx.sensor_index);
        }
    } else {
        // calculate estimated length if we are unindexed
        uint64_t size = packets_.reader_->file_size();

        // get average scan size plus some overhead
        const int pcap_pkt_header = 100;
        size_t scan_size = 0;
        for (const auto& sensor : sensor_info()) {
            PacketFormat packet_format(*sensor);
            auto pkt_size = packet_format.lidar_packet_size + pcap_pkt_header;
            scan_size += pkt_size * sensor->format.lidar_packets_per_frame();
            auto imu_pkt_size = packet_format.imu_packet_size + pcap_pkt_header;
            scan_size += imu_pkt_size * sensor->format.imu_packets_per_frame;
            if (sensor->format.zone_monitoring_enabled) {
                scan_size += packet_format.zone_packet_size + pcap_pkt_header;
            }
        }

        if (scan_size == 0) {
            throw std::runtime_error(
                "Unexpected scan data size of 0. SensorInfo may be corrupt.");
        }

        // divide size of all scans to get average scan size
        scan_size /= sensor_info().size();

        // number of scans is approximately file size divided by scan size
        size_hint_ = size / scan_size;
    }

    field_types_ = resolve_field_types(
        sensor_info(), options.raw_headers.retrieve(),
        options.raw_fields.retrieve(), options.field_names.retrieve());
    options.check("PcapScanSource");
}

const std::vector<std::shared_ptr<SensorInfo>>& PcapScanSource::sensor_info()
    const {
    return packets_.sensor_info();
}

size_t PcapScanSource::size() const {
    assert_indexed("size");

    return num_scans_;
}

size_t PcapScanSource::size_hint() const {
    if (indexed_) {
        return num_scans_;
    }

    return size_hint_;
}

bool PcapScanSource::is_indexed() const { return indexed_; }

uint64_t PcapScanSource::id_error_count() const {
    return packets_.id_error_count();
}

uint64_t PcapScanSource::size_error_count() const {
    return packets_.size_error_count();
}

const std::vector<size_t>& PcapScanSource::scans_num() const {
    assert_indexed("scans_num");

    return scans_count_;
}

void PcapScanSource::close() { packets_.close(); }

std::unique_ptr<ScanSource> PcapScanSource::move() {
    return std::make_unique<PcapScanSource>(std::move(*this));
}

std::unique_ptr<ScanSource> PcapScanSource::create(
    const std::vector<std::string>& sources, const ScanSourceOptions& options,
    bool collate, int sensor_idx) {
    if (sources.size() > 1) {
        throw std::invalid_argument(
            "PcapScanSource allows opening only one file at a time.");
    }

    std::unique_ptr<ScanSource> source =
        std::make_unique<PcapScanSource>(sources[0], options);
    if (sensor_idx >= 0) {
        source = std::make_unique<Singler>(std::move(source), sensor_idx);
    } else if (collate) {
        source = std::make_unique<Collator>(std::move(source));
    }

    return source;
};

}  // namespace pcap
}  // namespace sdk
}  // namespace ouster
