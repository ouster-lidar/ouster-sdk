/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/scan_source_utils.h"

namespace ouster {
namespace core {

template <class T>
class CollatorManager {
    std::vector<T> list_;
    int64_t min_ts_ = -1;
    int64_t max_ts_ = -1;
    int64_t dt_ = 100000000;

   public:
    CollatorManager(int len, int64_t dt) : dt_(dt) { list_.resize(len); }

    // returns true if the list is complete
    bool collate(int index, T& item, int64_t ts, bool& consumed) {
        consumed = false;
        // if we collated for too long, return early
        if (min_ts_ < 0 || max_ts_ < 0 ||
            (ts >= min_ts_ + dt_ || ts < max_ts_ - dt_)) {
            min_ts_ = max_ts_ = ts;  // stash the scan for later
            // if we have any, yield
            bool any = false;
            for (const auto& item : list_) {
                if (item) {
                    any = true;
                    break;
                }
            }
            if (any) {
                return true;
            }
        }

        // break early if we get a second from a single sensor
        if (list_[index]) {
            min_ts_ = max_ts_ = ts;
            return true;
        }

        list_[index] = item;
        consumed = true;

        // break early whenever the list is full
        bool full = true;
        for (const auto& item : list_) {
            if (!item) {
                full = false;
                break;
            }
        }
        if (full) {
            min_ts_ = max_ts_ = ts;
        }
        return full;
    }

    std::vector<T>& list() { return list_; }

    void reset() {
        for (auto& i : list_) {
            i = {};
        }
    }
};

class DefaultCollatedScanIteratorImpl : public ScanIteratorImpl {
    const ScanSource* source_;
    ScanIterator iter_;
    std::map<const void*, int> ids_;
    int sensor_count_;
    int i_ = -1;
    int64_t length_ = 0;
    bool first_ = true;
    std::vector<int32_t> index_;

    CollatorManager<std::shared_ptr<LidarScan>> collator_;

   public:
    DefaultCollatedScanIteratorImpl(const ScanSource* source, uint64_t dt)
        : source_(source),
          sensor_count_(source->sensor_info().size()),
          i_(0),
          collator_(source->sensor_info().size(), dt) {
        // build a map of sensor_info to sensor idx
        int i = 0;
        for (auto& si : source->sensor_info()) {
            ids_[si.get()] = i++;
        }

        iter_ = source->begin();

        // calculate the length by collating the index
        if (source_->is_indexed()) {
            CollatorManager<int> collator(sensor_count_, dt);

            // while we collate also build an index of the first scan in each
            // collated set to use for random access
            length_ = 0;
            for (size_t i = 0; i < source_->full_index().size();) {
                int v = i + 1;
                auto& item = source->full_index()[i];
                bool consumed = false;
                if (collator.collate(item.second, v, item.first, consumed)) {
                    length_++;

                    auto min = std::numeric_limits<int>::max();
                    for (const auto& item : collator.list()) {
                        if (item > 0) {
                            min = std::min(min, item - 1);
                        }
                    }
                    index_.push_back(min);
                    collator.reset();
                }

                if (consumed) {
                    i++;
                }
            }
            // increment length one more time if we have anything left
            auto min = std::numeric_limits<int>::max();
            for (const auto& item : collator.list()) {
                if (item > 0) {
                    min = std::min(min, item - 1);
                }
            }
            if (min != std::numeric_limits<int>::max()) {
                length_++;
                index_.push_back(min);
            }
        }
    }

    bool advance(size_t offset) override {
        if (offset > 1 && source_->is_indexed()) {
            auto desired_index = i_ + offset - 1;

            // now seek our iterator that that position
            iter_ = source_->begin();
            iter_ += index_[desired_index];
            i_ = desired_index;
            offset = 1;
        }

        for (size_t i = 0; i < offset; i++) {
            // now while loop time!
            if (do_one()) {
                return true;
            }
        }
        return false;
    }

    int64_t length() override {
        if (!source_->is_indexed()) {
            throw std::runtime_error(
                "Difference is not supported on non-indexed scan sources.");
        }
        return length_;
    }

    bool do_one() {
        collator_.reset();

        // collate until we want to yield
        while (iter_ != source_->end()) {
            auto val = *iter_;
            if (val.size() != 1) {
                throw std::runtime_error(
                    "Attempted to collate multi-source. Or source returned no "
                    "scans.");
            }

            int64_t ts = val[0]->get_first_valid_packet_timestamp();

            auto index_iter = ids_.find(val[0]->sensor_info.get());
            if (index_iter == ids_.end()) {
                throw std::runtime_error(
                    "Source must use consistent sensor infos in scans and "
                    "sensor_info()");
            }
            bool consumed = false;
            if (collator_.collate(index_iter->second, val[0], ts, consumed)) {
                if (consumed) {
                    iter_++;
                }
                i_++;
                return false;
            }

            if (consumed) {
                iter_++;
            }
        }

        // return remaining scans if we have any
        bool any = false;
        for (const auto& item : collator_.list()) {
            if (item) {
                any = true;
                break;
            }
        }
        if (any) {
            i_++;
            return false;
        }

        return true;  // hit end of file
    }

    virtual std::vector<std::shared_ptr<LidarScan>>& value() override {
        return collator_.list();
    }
};

Collator::Collator(const ScanSource& source, uint64_t dt_ns)
    : source_(&source), dt_(dt_ns) {}

Collator::Collator(ouster::core::ScanSource&& copy, uint64_t dt_ns) {
    dt_ = dt_ns;
    parent_ = std::unique_ptr<ouster::core::ScanSource>(
        (ouster::core::ScanSource*)copy.move());
    source_ = parent_.get();
}

Collator::Collator(std::unique_ptr<ScanSource> source, uint64_t dt_ns)
    : parent_(std::move(source)) {
    dt_ = dt_ns;
    source_ = parent_.get();
}

ScanIterator Collator::begin() const {
    return ScanIterator(this,
                        new DefaultCollatedScanIteratorImpl(source_, dt_));
}

ScanIterator Collator::begin(int /*sensor_index*/) const {
    throw std::runtime_error(
        "Cannot get a single stream from an already collated source. Please do "
        "this on the uncollated source instead.");
}

ScanSource* Collator::move() { return new Collator(std::move(*this)); }

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
Collator::sensor_info() const {
    return source_->sensor_info();
}

bool Collator::is_live() const { return source_->is_live(); }

bool Collator::is_indexed() const { return source_->is_indexed(); }

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
Collator::individual_index() const {
    return source_->individual_index();
}

const std::vector<std::pair<uint64_t, uint64_t>>& Collator::full_index() const {
    return source_->full_index();
}

const std::vector<size_t>& Collator::scans_num() const {
    return source_->scans_num();
}

void Collator::close() { parent_.reset(); }

Collator collate(const ScanSource& source) { return Collator(source); }

Collator collate(ScanSource&& source) { return Collator(std::move(source)); }

/// Iterates over a single stream in the ScanSource
Singler::Singler(const ScanSource& source, size_t idx)
    : source_(&source), idx_(idx) {
    if (idx >= source.sensor_info().size()) {
        throw std::invalid_argument(
            "Sensor index must be less than the count of sensors.");
    }
    sensor_info_.push_back(source_->sensor_info()[idx]);
    if (source_->is_indexed()) {
        scans_num_.push_back(source_->scans_num()[idx]);
        build_index();
    }
    // test begin to see if we should fail out because already collated
    begin();
}

Singler::Singler(ouster::core::ScanSource&& source, size_t idx) {
    idx_ = idx;
    parent_ = std::unique_ptr<ouster::core::ScanSource>(
        (ouster::core::ScanSource*)source.move());
    source_ = parent_.get();
    if (idx >= source_->sensor_info().size()) {
        throw std::invalid_argument(
            "Sensor index must be less than the count of sensors.");
    }
    sensor_info_.push_back(source_->sensor_info()[idx]);
    if (source_->is_indexed()) {
        scans_num_.push_back(source_->scans_num()[idx]);
        build_index();
    }
    // test begin to see if we should fail out because already collated
    begin();
}

Singler::Singler(std::unique_ptr<ouster::core::ScanSource> source, size_t idx)
    : parent_(std::move(source)) {
    idx_ = idx;
    source_ = parent_.get();
    if (idx >= source_->sensor_info().size()) {
        throw std::invalid_argument(
            "Sensor index must be less than the count of sensors.");
    }
    sensor_info_.push_back(source_->sensor_info()[idx]);
    if (source_->is_indexed()) {
        scans_num_.push_back(source_->scans_num()[idx]);
        build_index();
    }
    // test begin to see if we should fail out because already collated
    begin();
}

void Singler::build_index() {
    individual_index_.emplace_back();
    auto& index = individual_index_[0];
    int i = 0;
    for (auto& item : source_->individual_index()[idx_]) {
        index.push_back({item.first, i++});
    }

    for (auto& item : source_->full_index()) {
        if (item.second == idx_) {
            full_index_.push_back({item.first, 0});
        }
    }
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
Singler::individual_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'individual_index' not supported on unindexed scan sources.");
    }
    return individual_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& Singler::full_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'full_index' not supported on unindexed scan sources.");
    }
    return full_index_;
}

ScanSource* Singler::move() { return new Singler(std::move(*this)); }

ScanIterator Singler::begin() const { return source_->begin(idx_); }

ScanIterator Singler::begin(int idx) const {
    if (idx != 0) {
        throw std::runtime_error(
            "Sensor index must be less than the count of sensors.");
    }
    return begin();
}

ScanIterator Singler::end() const { return source_->end(); }

bool Singler::is_live() const { return source_->is_live(); }

bool Singler::is_indexed() const { return source_->is_indexed(); }

const std::vector<size_t>& Singler::scans_num() const {
    if (scans_num_.size()) {
        return scans_num_;
    }
    throw std::runtime_error(
        "'scans_num' not supported on unindexed scan sources.");
}

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
Singler::sensor_info() const {
    return sensor_info_;
}

void Singler::close() { parent_.release(); }

template <class Parent, class Iterator, class Base, class Return>
class SlicerScanIteratorImpl : public Base {
    const Parent* source_;
    Iterator iter_;
    uint64_t i_ = 0;

    int start_;
    int end_;
    int step_;
    size_t length_;

    bool first_ = true;

   public:
    SlicerScanIteratorImpl(const Parent* source, int start, int end, int step)
        : source_(source), start_(start), end_(end), step_(step) {
        // incoming slice is pre-normalized
        length_ = ((end_ - 1) - start_) / step_ + 1;

        //  if slice is empty, dont bother skipping through the source
        if (start_ == end_) {
            first_ = false;  // skip doing any advance
            iter_ = source_->end();
            return;
        }

        iter_ = source->begin();
    }

    bool advance(size_t offset) override {
        // skip to the start if this is the first
        if (first_ && offset > 0) {
            iter_ += start_;
            first_ = false;
            offset -= 1;
        }
        // make sure our slice isnt empty
        if (iter_ == source_->end()) {
            return true;
        }

        // if we just go one over, it should be equal to end
        // if we go past that throw

        for (size_t i = 0; i < offset; i++) {
            // stop if we would stop past the end
            if ((i_ + 1) >= length_) {
                return true;
            }

            iter_ += step_;
            if (iter_ == source_->end()) {
                return true;
            }
            i_++;

            if (i_ == length_) {
                return true;
            }
        }
        return false;
    }

    int64_t length() override { return length_; }

    virtual Return& value() override { return *iter_; }
};

static void normalize_slice(const ScanSource* s, int& start, int& end,
                            int& step) {
    if (step <= 0) {
        throw std::invalid_argument("Step size must be > 0 for slice.");
    }
    // normalize start and end
    auto length = s->end() - s->begin();

    // clamp our length at the length, but don't throw if end is past the end
    if (end > length) {
        end = length;
    }
    if (start < 0) {
        start = length + start;
    }
    if (end < 0) {
        end = length + end;
    }

    if (start > end) {
        throw std::invalid_argument("End must be after start for slice.");
    }
}

Slicer::Slicer(const ScanSource& source, int start, int end, int step)
    : source_(&source), start_(start), end_(end), step_(step) {
    normalize_slice(source_, start_, end_, step_);
    build_index();
}

Slicer::Slicer(ScanSource&& source, int start, int end, int step)
    : start_(start), end_(end), step_(step) {
    parent_ = std::unique_ptr<ScanSource>(source.move());
    source_ = parent_.get();

    normalize_slice(source_, start_, end_, step_);
    build_index();
}

ScanIterator Slicer::begin() const {
    return ScanIterator(
        this,
        new SlicerScanIteratorImpl<ScanSource, ScanIterator, ScanIteratorImpl,
                                   std::vector<std::shared_ptr<LidarScan>>>(
            source_, start_, end_, step_));
}

ScanIterator Slicer::begin(int /*idx*/) const {
    // this is difficult to implement, not supported for now
    throw std::runtime_error("Not yet supported. Please single then slice.");
}

ScanSource* Slicer::move() { return new Slicer(std::move(*this)); }

void Slicer::build_index() {
    individual_index_.resize(source_->individual_index().size());

    scans_num_.resize(individual_index_.size());

    // now rebuild each
    int index = 0;
    auto& original = source_->full_index();
    for (int i = start_; i < end_; i++) {
        int delta = i - start_;
        if ((delta % step_) != 0) {
            continue;
        }

        auto& item = original[i];

        scans_num_[item.second]++;

        full_index_.push_back(item);
        individual_index_[item.second].push_back({item.first, index});
        index++;
    }
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
Slicer::individual_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'individual_index' not supported on unindexed scan sources.");
    }
    return individual_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& Slicer::full_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'full_index' not supported on unindexed scan sources.");
    }
    return full_index_;
}

const std::vector<size_t>& Slicer::scans_num() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'scans_num' not supported on unindexed scan sources.");
    }
    return scans_num_;
}

bool Slicer::is_live() const { return source_->is_live(); }

bool Slicer::is_indexed() const { return source_->is_indexed(); }

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
Slicer::sensor_info() const {
    return source_->sensor_info();
}

void Slicer::close() { parent_.reset(); }

AnyScanSource::AnyScanSource(std::unique_ptr<ScanSource> source)
    : source_(std::move(source)) {}

ScanIterator AnyScanSource::begin() const { return source_->begin(); }

ScanIterator AnyScanSource::begin(int idx) const { return source_->begin(idx); }

ScanIterator AnyScanSource::end() const { return source_->end(); }

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
AnyScanSource::sensor_info() const {
    return source_->sensor_info();
}

bool AnyScanSource::is_live() const { return source_->is_live(); }

bool AnyScanSource::is_indexed() const { return source_->is_indexed(); }

size_t AnyScanSource::size() const { return source_->size(); }

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
AnyScanSource::individual_index() const {
    return source_->individual_index();
}

const std::vector<std::pair<uint64_t, uint64_t>>& AnyScanSource::full_index()
    const {
    return source_->full_index();
}

const std::vector<size_t>& AnyScanSource::scans_num() const {
    return source_->scans_num();
}

ScanSource* AnyScanSource::move() {
    return new AnyScanSource(std::move(*this));
}

std::shared_ptr<ScanSource> AnyScanSource::child() const { return source_; }

void AnyScanSource::close() { source_.reset(); }

AnyPacketSource::AnyPacketSource(std::unique_ptr<PacketSource> source)
    : source_(std::move(source)) {}

PacketIterator AnyPacketSource::begin() const { return source_->begin(); }

PacketIterator AnyPacketSource::end() const { return source_->end(); }

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
AnyPacketSource::sensor_info() const {
    return source_->sensor_info();
}

bool AnyPacketSource::is_live() const { return source_->is_live(); }

std::shared_ptr<PacketSource> AnyPacketSource::child() const { return source_; }

void AnyPacketSource::close() { source_.reset(); }
}  // namespace core
}  // namespace ouster
