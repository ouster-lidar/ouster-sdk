/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/pcap_scan_source.h"

namespace ouster {
namespace pcap {

PcapScanSourceOptions::PcapScanSourceOptions(const ScanSourceOptions& o)
    : ScanSourceOptions(o) {}

PcapScanSourceOptions::PcapScanSourceOptions() {}

class PcapScanIteratorImpl : public ouster::core::ScanIteratorImpl {
    const PcapScanSource* source_;
    ouster::core::PacketIterator packet_iter_;
    std::vector<std::shared_ptr<LidarScan>> scan_;
    int sensor_idx_;
    int64_t i_ = -1;

    std::vector<ouster::ScanBatcher> batchers_;
    std::vector<std::shared_ptr<ouster::LidarScan>> scans_;

   public:
    PcapScanIteratorImpl(const PcapScanSource* source, int sensor_idx = -1) {
        source_ = source;
        packet_iter_ = source->packets_.begin();
        sensor_idx_ = sensor_idx < 0 ? -1 : sensor_idx;

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
            i_ += offset - 1;
            offset = 1;
        }
        for (size_t i = 0; i < offset; i++) {
            // read packets until we get a scan or run out
            while (true) {
                if (packet_iter_ == source_->packets_.end()) {
                    // if finished return any partial scans we have
                    // todo should these be in any particular order?
                    bool found = false;
                    for (auto& scan : scans_) {
                        if (scan) {
                            scan_ = {scan};
                            scan.reset();
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
                if (packet.second->type() ==
                    ouster::sensor::PacketType::Lidar) {
                    auto& lidar_packet =
                        static_cast<ouster::sensor::LidarPacket&>(
                            *packet.second);

                    int index = packet.first;

                    // skip packets from sensors we dont care about
                    if (sensor_idx_ >= 0 && index != sensor_idx_) {
                        continue;
                    }

                    // allocate the scan if it hasnt already been
                    if (!scans_[index]) {
                        scans_[index] = std::make_shared<ouster::LidarScan>(
                            source_->sensor_info()[index],
                            source_->field_types_[index]);
                    }

                    // finally batch
                    if (batchers_[index](lidar_packet, *scans_[index])) {
                        scan_ = {scans_[index]};
                        scans_[index].reset();
                        i_++;
                        break;
                    }
                }
            }
        }
        return false;
    }

    int64_t length() override {
        source_->assert_indexed("length");
        if (sensor_idx_ >= 0) {
            return source_->scans_num()[sensor_idx_];
        }
        return source_->size();
    }

    std::vector<std::shared_ptr<LidarScan>>& value() override { return scan_; }
};

ouster::core::ScanIterator PcapScanSource::begin() const {
    return ouster::core::ScanIterator(this, new PcapScanIteratorImpl(this));
}

ouster::core::ScanIterator PcapScanSource::begin(int sensor_index) const {
    if (sensor_index >= (int)sensor_info().size()) {
        throw std::runtime_error("Invalid sensor index");
    }
    return ouster::core::ScanIterator(
        this, new PcapScanIteratorImpl(this, sensor_index));
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
    const std::string& n,
    const std::function<void(PcapScanSourceOptions&)>& options)
    : PcapScanSource(n, ouster::impl::get_scan_options(options)) {}

/// open_source compatible constructor
PcapScanSource::PcapScanSource(const std::string& source,
                               PcapScanSourceOptions options)
    : packets_(source, PacketSourceOptions((ScanSourceOptions&)options)),
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
        size_t i = 0;
        for (const auto& idx : index.global_frame_indices_) {
            index_[idx.sensor_index].push_back({idx.timestamp, i++});
            real_index_.push_back({idx.timestamp, idx.sensor_index});
        }
    }

    field_types_ = ouster::resolve_field_types(
        sensor_info(), options.raw_headers.retrieve(),
        options.raw_fields.retrieve(), options.field_names.retrieve());
    options.check("PcapScanSource");
}

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
PcapScanSource::sensor_info() const {
    return packets_.sensor_info();
}

size_t PcapScanSource::size() const {
    assert_indexed("size");

    return num_scans_;
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

ouster::core::ScanSource* PcapScanSource::move() {
    return new PcapScanSource(std::move(*this));
}
}  // namespace pcap
}  // namespace ouster
