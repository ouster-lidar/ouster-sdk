#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "ouster/scan_source_utils.h"

namespace ouster {
namespace sdk {
namespace core {

class MultiScanSource;
class MultiScanIteratorImpl : public ScanIteratorImpl {
   public:
    explicit MultiScanIteratorImpl(const MultiScanSource* scan_source,
                                   int sensor_idx = -1) {
        source_ = scan_source;
        sensor_idx_ = sensor_idx < 0 ? -1 : sensor_idx;

        // todo try not to decode scans in the constructor
        for (size_t i = 0; i < scan_source->sources_.size(); i++) {
            iters_[i] = scan_source->sources_[i]->begin();
            if (iters_[i] != scan_source->sources_[i]->end()) {
                nexts_[i] =
                    std::make_unique<LidarScanSet>(std::move(*iters_[i]));
            } else {
                iters_.erase(i);
            }
        }
    }

    int64_t length() override {
        if (!source_->is_indexed()) {
            throw std::runtime_error(
                "Difference is not supported on non-indexed scan sources.");
        }
        if (sensor_idx_ >= 0) {
            return static_cast<int64_t>(source_->scans_num()[sensor_idx_]);
        }
        return static_cast<int64_t>(source_->size());
    }

    bool advance(size_t offset) override {
        std::vector<size_t> to_remove;
        for (size_t i = 0; i < offset; i++) {
            // todo support skipping with a singled sensor
            // Skip forward through the source by more than one if required
            if (offset > 1 && source_->is_indexed() && sensor_idx_ < 0) {
                // basically just loop through from current position to target,
                // finding how much we need to increment each source iterator
                // the fun for this is we need to know which scan source each
                // thing came from so will need _another_ index for this

                if (position_ + offset > static_cast<size_t>(length())) {
                    return true;
                }

                if (position_ < 0) {
                    position_ = 0;
                }

                // figure out how much we need to increment each iterator
                std::map<int, int> increment_amounts;
                for (size_t j = position_; j < position_ + offset; j++) {
                    auto source_index = source_->source_index_[j].source_index;
                    auto iter = increment_amounts.find(source_index);
                    if (iter == increment_amounts.end()) {
                        increment_amounts[source_index] = 1;
                    } else {
                        iter->second += 1;
                    }
                }

                // Get the source of the next scan
                int target =
                    source_->source_index_[position_ + offset].source_index;

                for (const auto& amt : increment_amounts) {
                    iters_[amt.first] += amt.second;
                    if (iters_[amt.first] ==
                        source_->sources_[amt.first]->end()) {
                        nexts_.erase(amt.first);
                        iters_.erase(amt.first);
                    } else {
                        nexts_[amt.first] = std::make_unique<LidarScanSet>(
                            std::move(*iters_[amt.first]));
                    }
                }

                // then just grab the value from the target iterator and update
                // their nexts
                scan_ = std::move(nexts_[target]);
                nexts_[target].reset();

                if (scan_->size() > 1) {
                    throw std::runtime_error(
                        "MultiScanSource can only be used with non-collated "
                        "sources.");
                }

                // update sensor info
                auto res =
                    source_->info_map_.find((*scan_)[0]->sensor_info.get());
                (*scan_)[0]->sensor_info = res->second.second;

                position_ += offset;
                return false;
            }

            // first fill in any missing slots
            for (auto& item : nexts_) {
                if (item.second) {
                    continue;
                }

                auto& iter = iters_[item.first];

                iter++;

                if (iter == source_->sources_[item.first]->end()) {
                    to_remove.push_back(item.first);
                    continue;
                }
                nexts_[item.first] =
                    std::make_unique<LidarScanSet>(std::move(*iter));
            }

            // remove any we exhausted
            for (const auto& item : to_remove) {
                nexts_.erase(item);
                iters_.erase(item);
            }
            to_remove.clear();

            // we are fully exhausted: end of file
            if (nexts_.size() == 0) {
                return true;
            }

            // todo use a priority queue to speed this up

            // finally grab the one with the earliest time
            int earliest = -1;
            uint64_t minimum_ts = std::numeric_limits<uint64_t>::max();
            for (const auto& item : nexts_) {
                // ideally wouldnt calculate all timestamps each time
                auto timestamp =
                    (*item.second)[0]->get_first_valid_packet_timestamp();
                if (timestamp < minimum_ts) {
                    earliest = item.first;
                    minimum_ts = timestamp;
                }
            }

            // "pop" off the scan
            scan_ = std::move(nexts_[earliest]);
            nexts_[earliest].reset();

            if (scan_->size() > 1) {
                throw std::runtime_error(
                    "MultiScanSource can only be used with non-collated "
                    "sources.");
            }

            // update to the correct sensor info instance
            auto res = source_->info_map_.find((*scan_)[0]->sensor_info.get());
            (*scan_)[0]->sensor_info = res->second.second;

            // skip scans from sensors we want to filter out
            if (sensor_idx_ >= 0 && res->second.first != sensor_idx_) {
                offset++;
                continue;
            }

            position_++;
        }
        return false;
    }

    LidarScanSet value() override { return *scan_; }

   private:
    const MultiScanSource* source_;
    std::unique_ptr<LidarScanSet> scan_;
    int sensor_idx_ = -1;

    int64_t position_ = -1;

    std::map<size_t, ScanIterator> iters_;
    std::map<size_t, std::unique_ptr<LidarScanSet>> nexts_;
};

MultiScanSource::MultiScanSource(
    std::vector<std::shared_ptr<ScanSource>> sources)
    : sources_(std::move(sources)) {
    // calculate everything!

    // okay, build a list of each unique sensor into
    indexed_ = true;
    size_ = 0;
    for (const auto& src : sources_) {
        for (const auto& info : src->sensor_info()) {
            std::shared_ptr<SensorInfo> found;
            int sensor_index = 0;
            for (const auto& si2 : sensor_info_) {
                if (*info == *si2) {
                    found = si2;
                    break;
                }
                sensor_index++;
            }

            if (!found) {
                sensor_info_.push_back(info);
                info_map_[info.get()] = {sensor_index, info};
            } else {
                info_map_[info.get()] = {sensor_index, found};
            }
        }

        if (!src->is_indexed()) {
            indexed_ = false;
        }
    }

    // if we are indexed, build our massive indices and calculate size and scans
    // num
    if (!indexed_) {
        size_hint_ = 0;
        for (const auto& src : sources_) {
            size_hint_ += src->size_hint();
        }
        return;
    }

    scans_num_.resize(sensor_info_.size(), 0);
    for (size_t src_index = 0; src_index < sources_.size(); src_index++) {
        const auto& src = sources_[src_index];
        size_ += src->size();

        std::vector<uint64_t> src_idx_to_dest_index;
        for (size_t i = 0; i < src->sensor_info().size(); i++) {
            const auto& si = src->sensor_info()[i];
            size_t sensor_index = info_map_[si.get()].first;
            scans_num_[sensor_index] += src->scans_num()[i];

            src_idx_to_dest_index.push_back(sensor_index);
        }

        const auto& f_idx = src->full_index();
        for (size_t i = 0; i < f_idx.size(); i++) {
            const auto& sample = f_idx[i];
            full_index_.push_back(
                {sample.first, src_idx_to_dest_index[sample.second]});

            IndexSample sample2;
            sample2.source_index = src_index;
            sample2.timestamp = sample.first;
            sample2.scan_index = i;
            source_index_.push_back(sample2);
        }
    }
    size_hint_ = size_;

    // finally sort the indices
    std::sort(full_index_.begin(), full_index_.end(),
              [](const std::pair<uint64_t, uint64_t>& a,
                 const std::pair<uint64_t, uint64_t>& b) {
                  return a.first < b.first;
              });

    std::sort(source_index_.begin(), source_index_.end(),
              [](const IndexSample& a, const IndexSample& b) {
                  return a.timestamp < b.timestamp;
              });

    // then build individual indices
    individual_index_.resize(sensor_info_.size());
    for (uint64_t i = 0; i < full_index_.size(); i++) {
        const auto& sample = full_index_[i];
        individual_index_[sample.second].push_back({sample.first, i});
    }
}

ScanIterator MultiScanSource::begin() const {
    return ScanIterator(this, new MultiScanIteratorImpl(this));
}

ScanIterator MultiScanSource::begin(int idx) const {
    return ScanIterator(this, new MultiScanIteratorImpl(this, idx));
}

ScanIterator MultiScanSource::end() const { return ScanIterator(this); }

const std::vector<std::shared_ptr<SensorInfo>>& MultiScanSource::sensor_info()
    const {
    return sensor_info_;
}

bool MultiScanSource::is_live() const { return false; }

bool MultiScanSource::is_indexed() const { return indexed_; }

size_t MultiScanSource::size() const { return size_; }

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
MultiScanSource::individual_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'individual_index' not supported on unindexed scan sources.");
    }
    return individual_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& MultiScanSource::full_index()
    const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'full_index' not supported on unindexed scan sources.");
    }
    return full_index_;
}

size_t MultiScanSource::size_hint() const { return size_hint_; }

const std::vector<size_t>& MultiScanSource::scans_num() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'scans_num' not supported on unindexed scan sources.");
    }
    return scans_num_;
}

std::unique_ptr<ScanSource> MultiScanSource::move() {
    return std::make_unique<MultiScanSource>(std::move(*this));
}

void MultiScanSource::close() { sources_.clear(); }

}  // namespace core
}  // namespace sdk
}  // namespace ouster
