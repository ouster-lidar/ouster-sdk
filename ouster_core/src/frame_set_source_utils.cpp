/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/frame_set_source_utils.h"

#include <ouster/core/impl/logging.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

FrameSetSourceWrapper::FrameSetSourceWrapper(const FrameSetSource& source) : source_(&source) {}

// NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
FrameSetSourceWrapper::FrameSetSourceWrapper(FrameSetSource&& source)
    : parent_(source.move()), source_(parent_.get()) {}

FrameSetSourceWrapper::FrameSetSourceWrapper(std::unique_ptr<FrameSetSource> source)
    : parent_(std::move(source)), source_(parent_.get()) {}

void FrameSetSourceWrapper::close() {
    if (parent_) {
        parent_.reset();
    }
}

const std::vector<std::shared_ptr<SensorInfo>>& FrameSetSourceWrapper::sensor_info() const {
    return source_->sensor_info();
}

bool FrameSetSourceWrapper::is_live() const {
    return source_->is_live();
}

bool FrameSetSourceWrapper::is_indexed() const {
    return source_->is_indexed();
}

bool FrameSetSourceWrapper::is_collated() const {
    return source_->is_collated();
}

bool FrameSetSourceWrapper::contains_collations() const {
    return source_->contains_collations();
}

size_t FrameSetSourceWrapper::size_hint() const {
    return source_->size_hint();
}

const std::vector<std::pair<uint64_t, uint64_t>>& FrameSetSourceWrapper::full_index() const {
    return source_->full_index();
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
FrameSetSourceWrapper::individual_index() const {
    return source_->individual_index();
}

const std::vector<size_t>& FrameSetSourceWrapper::frames_num() const {
    return source_->frames_num();
}

bool FrameSetSourceWrapper::has_metadata(const std::string& key) const {
    auto it = metadata_.entries.find(key);
    return it != metadata_.entries.end() || source_->has_metadata(key);
}

std::set<std::string> FrameSetSourceWrapper::metadata_keys() const {
    std::set<std::string> names = metadata_.keys();
    std::set<std::string> union_names;

    auto source_names = source_->metadata_keys();
    std::set_union(names.begin(), names.end(), source_names.begin(), source_names.end(),
                   std::inserter(union_names, union_names.begin()));
    return union_names;
}

const FrameSetSourceMetadata& FrameSetSourceWrapper::metadata(const std::string& name) const {
    auto it = metadata_.entries.find(name);
    if (it != metadata_.entries.end()) {
        return it->second;
    }
    return source_->metadata(name);
}

namespace {
// check if the sensors are configured such that sync is possible
bool could_be_synchronized(const FrameSetSource& src) {
    if (src.sensor_info().size() <= 1) {
        return false;
    }
    double rate = 0;
    for (const auto& si : src.sensor_info()) {
        if (rate == 0) {
            rate = si->format.fps;
        } else if (si->format.fps != rate) {
            return false;
        }
        if (!si->config.phase_lock_enable.value_or(false)) {
            return false;
        }
        auto ts_mode = ouster::sdk::core::TimestampMode::TIME_FROM_INTERNAL_OSC;
        if (si->config.timestamp_mode.value_or(ts_mode) == ts_mode) {
            return false;
        }
    }

    return true;
}
}  // namespace

template <class T>
class CollatorManager {
    std::vector<T> list_;
    int64_t min_ts_ = -1;
    int64_t delta_t_ = 100000000;
    size_t count_ = 0;
    int min_idx_ = -1;
    std::vector<int64_t> latencies_;

   public:
    // delta t holds the collation period
    CollatorManager(size_t size, uint64_t delta_t, const std::vector<int64_t>& latencies)
        : delta_t_(static_cast<int64_t>(delta_t)), latencies_(latencies) {
        list_.resize(size);
    }

    // returns true if the set is ready to be consumed
    bool collate(int index, T& item, int64_t timestamp, bool& consumed) {
        consumed = false;
        // if we collated for too long, return a partial set early
        if (latencies_.empty()) {
            // legacy simple timeout based early exit
            if (min_ts_ < 0 || (timestamp >= min_ts_ + delta_t_)) {
                // if we have any, yield
                if (count_ > 0) {
                    return true;
                }
                min_ts_ = timestamp;
            }
        } else {
            // expected arrival time based early exit for synchronized sensors
            if (min_ts_ < 0) {
                // first frame in the set is used as the measuring point
                min_ts_ = timestamp;
                min_idx_ = index;
            } else {
                // using the first frame as a measuring point and the expected
                // relative arrival times of frames in a set, determine
                // which set this frame is closest to
                // e.g. if more than half a period late, it's closer to next
                int64_t set_origin_ts = min_ts_ - latencies_[min_idx_];
                int64_t expected_ts = set_origin_ts + latencies_[index];
                int64_t threshold_ts = expected_ts + (delta_t_ / 2);
                if (timestamp > threshold_ts) {
                    return true;
                }
            }
        }

        // break early if we get a second from a single sensor in a set
        if (list_[index]) {
            return true;
        }

        list_[index] = item;
        count_++;
        consumed = true;

        // return a set whenever the it is full
        return count_ >= list_.size();
    }

    std::vector<T>& list() {
        return list_;
    }

    // gets ready to collate the next set of frames after one is consumed
    void reset() {
        for (auto& i : list_) {
            i = {};
        }
        count_ = 0;
        min_ts_ = -1;
    }
};

class DefaultCollatedFrameSetIteratorImpl : public FrameSetIteratorImpl {
    const Collator* source_;
    FrameSetIterator iter_;
    std::map<const void*, int> ids_;
    int i_ = -1;
    bool need_advance_ = false;

    CollatorManager<std::shared_ptr<LidarFrame>> collator_;

   public:
    DefaultCollatedFrameSetIteratorImpl(const Collator* source, uint64_t delta_t,
                                        const std::vector<int64_t>& latencies)
        : source_(source), i_(0), collator_(source->sensor_info().size(), delta_t, latencies) {
        // build a map of sensor_info to sensor idx
        int i = 0;
        for (auto& si : source->sensor_info()) {
            ids_[si.get()] = i++;
        }

        iter_ = source_->source_->begin();
    }

    bool advance(size_t offset) override {
        if (offset > 1 && source_->is_indexed()) {
            auto desired_index = i_ + offset - 1;

            // now seek our iterator that that position
            iter_ = source_->source_->begin();
            iter_ += source_->index_[desired_index];
            i_ = static_cast<int>(desired_index);
            offset = 1;
            need_advance_ = false;
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
                "Difference is not supported on non-indexed frame set "
                "sources.");
        }
        return source_->length_;
    }

    bool do_one() {
        collator_.reset();

        // advance the iterator if we yielded last operation
        if (need_advance_) {
            need_advance_ = false;
            iter_++;
        }

        // collate until we want to yield
        while (iter_ != source_->source_->end()) {
            auto val = *iter_;
            if (val.size() != 1) {
                throw std::runtime_error(
                    "Attempted to collate multi-source. Or source returned no "
                    "frames.");
            }

            int64_t timestamp = 0;
            try {
                timestamp = static_cast<int64_t>(val[0]->get_max_valid_packet_timestamp());
            } catch (const std::runtime_error& /*e*/) {
                // leave timestamp at zero
            }

            auto index_iter = ids_.find(val[0]->sensor_info.get());
            if (index_iter == ids_.end()) {
                throw std::runtime_error(
                    "Source must use consistent sensor infos in frames and "
                    "sensor_info()");
            }
            bool consumed = false;
            if (collator_.collate(index_iter->second, val[0], timestamp, consumed)) {
                // dont increment iter here or we wait until the next frame
                // comes
                if (consumed) {
                    need_advance_ = true;
                }
                i_++;
                return false;
            }

            if (consumed) {
                iter_++;
            }
        }

        // return remaining frames if we have any
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

    FrameSet value() override {
        return FrameSet{collator_.list()};
    }
};

Collator::Collator(const FrameSetSource& source, uint64_t dt_ns)
    : FrameSetSourceWrapper(source), dt_(dt_ns) {
    build_index();
}

Collator::Collator(FrameSetSource&& source, uint64_t dt_ns)
    : FrameSetSourceWrapper(std::move(source)), dt_(dt_ns) {
    build_index();
}

Collator::Collator(std::unique_ptr<FrameSetSource> source, uint64_t dt_ns)
    : FrameSetSourceWrapper(std::move(source)), dt_(dt_ns) {
    build_index();
}

void Collator::build_index() {
    // first determine parameters for collation
    if (dt_ == 0 && could_be_synchronized(*source_)) {
        // if synchronized use latency based determination
        ouster::sdk::core::logger().warn(
            "Assuming sensors are synchronized for collation purposes. If this "
            "is not the case please override this behavior or disable phase "
            "lock on sensors.");

        // calculate latencies between starts of frames
        std::vector<double> latencies;
        double period = 0.0;
        for (const auto& sensor : sensor_info()) {
            const auto& config = sensor->config;
            double rate = sensor->format.fps;
            if (rate == 0) {
                throw std::runtime_error("SensorInfo.format had an invalid FPS of 0.");
            }
            period = 1.0 / rate;

            // Determine the true "phase offset" of the frame
            // e.g. when does the frame end compared to the others
            // This is affected by both the phase offset setting and others
            unsigned int end_imu_zm_phase = 0;
            unsigned int end_azi_phase = 360000;

            // The phase offset always starts with the phase_lock_offset
            unsigned int phase_offset = config.phase_lock_offset.value_or(0);

            // If new IMU is enabled we always get an IMU packet and EoF
            if (config.udp_profile_imu.value_or(ouster::sdk::core::UDPProfileIMU::LEGACY) !=
                ouster::sdk::core::UDPProfileIMU::LEGACY) {
                // we always get an IMU packet at EoF
                end_imu_zm_phase = 360000;
            }

            // if ZM packets are enabled, then it's always the last packet
            if (config.udp_port_zm.value_or(0) != 0 && !config.udp_dest_zm.value_or("").empty()) {
                end_imu_zm_phase = 360000;
            }

            // If azimuth doesnt wrap around the frame boundary it can affect
            // the time of the first and last packet in a frame
            // If it wraps around the end of frame you still get a lidar packet
            // at start and end of frame
            auto aziw = config.azimuth_window.value_or(std::make_pair(0, 360000));
            if (aziw.first < aziw.second) {
                end_azi_phase = aziw.second;
            }

            // finally pick the phase we are synching off of and offset
            phase_offset += std::max(end_imu_zm_phase, end_azi_phase);

            // wrap the phase offset so its <= 360 degrees
            phase_offset = phase_offset % 360000;

            // convert latency to seconds
            double latency = period * static_cast<double>(phase_offset) / 360000.0;
            latencies.push_back(latency);
        }

        // try and offset latencies to try and minimize total
        std::vector<double> latencies_offset;
        for (auto lat : latencies) {
            if (lat < period * 0.5) {
                lat += period;
            }
            lat -= period;
            latencies_offset.push_back(lat);
        }

        // swap to this if it has a smaller distance between min and max
        auto minmax = std::minmax_element(latencies.begin(), latencies.end());
        auto diff = *minmax.second - *minmax.first;
        auto minmax2 = std::minmax_element(latencies_offset.begin(), latencies_offset.end());
        auto diff2 = *minmax2.second - *minmax2.first;
        if (diff2 < diff) {
            latencies = latencies_offset;
        }

        // finally convert to uint64 and shift so latencies start at 0
        auto min_val = *std::min_element(latencies.begin(), latencies.end());
        for (auto lat : latencies) {
            latencies_.push_back(static_cast<int64_t>((lat - min_val) * 1e9));
        }
        dt_ = static_cast<uint64_t>(period * 1e9);
    } else if (dt_ == 0) {
        // default if not synchronized and not set
        dt_ = static_cast<uint64_t>(1e9 * 0.21);
    }

    // calculate the length and a jump table for seeks by collating the index
    if (source_->is_indexed()) {
        CollatorManager<int> collator(sensor_info().size(), dt_, latencies_);

        // while we collate also build an index of the first frame in each
        // collated set to use for random access
        length_ = 0;
        for (size_t i = 0; i < source_->full_index().size();) {
            int value = static_cast<int>(i) + 1;
            auto& item = source_->full_index()[i];
            bool consumed = false;
            if (collator.collate(static_cast<int>(item.second), value,
                                 static_cast<int64_t>(item.first), consumed)) {
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

FrameSetIterator Collator::begin() const {
    return FrameSetIterator(this, new DefaultCollatedFrameSetIteratorImpl(this, dt_, latencies_));
}

FrameSetIterator Collator::begin(int /*sensor_index*/) const {
    throw std::runtime_error(
        "Cannot get a single stream from an already collated source. Please do "
        "this on the uncollated source instead.");
}

std::unique_ptr<FrameSetSource> Collator::move() {
    return std::make_unique<Collator>(std::move(*this));
}

bool Collator::is_collated() const {
    return true;
}

size_t Collator::size_hint() const {
    if (source_->is_indexed()) {
        return size();
    }
    return source_->size_hint() / source_->sensor_info().size();
}

int64_t Collator::collation_period() const {
    return static_cast<int64_t>(dt_);
}

std::vector<int64_t> Collator::collation_latencies() const {
    return latencies_;
}

Collator collate(const FrameSetSource& source) {
    return Collator(source);
}

Collator collate(FrameSetSource&& source) {
    return Collator(std::move(source));
}

/// Iterates over a single stream in the FrameSetSource
Singler::Singler(const FrameSetSource& source, size_t idx)
    : FrameSetSourceWrapper(source), idx_(idx) {
    if (idx >= source.sensor_info().size()) {
        throw std::invalid_argument("Sensor index must be less than the count of sensors.");
    }
    sensor_info_.push_back(source_->sensor_info()[idx]);
    if (source_->is_indexed()) {
        frames_num_.push_back(source_->frames_num()[idx]);
        build_index();
    }
    // test begin to see if we should fail out because already collated
    begin();
}

Singler::Singler(FrameSetSource&& source, size_t idx)
    : FrameSetSourceWrapper(std::move(source)), idx_(idx) {
    if (idx >= source_->sensor_info().size()) {
        throw std::invalid_argument("Sensor index must be less than the count of sensors.");
    }
    sensor_info_.push_back(source_->sensor_info()[idx]);
    if (source_->is_indexed()) {
        frames_num_.push_back(source_->frames_num()[idx]);
        build_index();
    }
    // test begin to see if we should fail out because already collated
    begin();
}

Singler::Singler(std::unique_ptr<FrameSetSource> source, size_t idx)
    : FrameSetSourceWrapper(std::move(source)), idx_(idx) {
    if (idx >= source_->sensor_info().size()) {
        throw std::invalid_argument("Sensor index must be less than the count of sensors.");
    }
    sensor_info_.push_back(source_->sensor_info()[idx]);
    if (source_->is_indexed()) {
        frames_num_.push_back(source_->frames_num()[idx]);
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
        index.emplace_back(item.first, i++);
    }

    for (auto& item : source_->full_index()) {
        if (item.second == idx_) {
            full_index_.emplace_back(item.first, 0);
        }
    }
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& Singler::individual_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'individual_index' not supported on unindexed frame set sources.");
    }
    return individual_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& Singler::full_index() const {
    if (!is_indexed()) {
        throw std::runtime_error("'full_index' not supported on unindexed frame set sources.");
    }
    return full_index_;
}

std::unique_ptr<FrameSetSource> Singler::move() {
    return std::make_unique<Singler>(std::move(*this));
}

FrameSetIterator Singler::begin() const {
    return source_->begin(static_cast<int>(idx_));
}

FrameSetIterator Singler::begin(int idx) const {
    if (idx != 0) {
        throw std::runtime_error("Sensor index must be less than the count of sensors.");
    }
    return begin();
}

FrameSetIterator Singler::end() const {
    return source_->end();
}

bool Singler::is_collated() const {
    return false;
}

size_t Singler::size_hint() const {
    if (source_->is_indexed()) {
        return source_->size();
    }
    return source_->size_hint() / source_->sensor_info().size();
}

const std::vector<size_t>& Singler::frames_num() const {
    if (!frames_num_.empty()) {
        return frames_num_;
    }
    throw std::runtime_error("'frames_num' not supported on unindexed frame set sources.");
}

const std::vector<std::shared_ptr<SensorInfo>>& Singler::sensor_info() const {
    return sensor_info_;
}

template <class Parent, class Iterator, class Base, class Return>
class SlicerFrameSetIteratorImpl : public Base {
    const Parent* source_;
    Iterator iter_;
    uint64_t i_ = 0;

    int start_;
    int end_;
    int step_;
    size_t length_;

    bool first_ = true;

   public:
    SlicerFrameSetIteratorImpl(const Parent* source, int start, int end, int step)
        : source_(source), start_(start), end_(end), step_(step) {
        // incoming slice is pre-normalized
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
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

    int64_t length() override {
        return length_;
    }

    Return value() override {
        return *iter_;
    }
};

namespace {

void normalize_slice(const FrameSetSource* source, int& start, int& end, int& step) {
    if (step <= 0) {
        throw std::invalid_argument("Step size must be > 0 for slice.");
    }
    // normalize start and end
    auto length = source->end() - source->begin();

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

}  // namespace

Slicer::Slicer(const FrameSetSource& source, int start, int end, int step)
    : FrameSetSourceWrapper(source), start_(start), end_(end), step_(step) {
    normalize_slice(source_, start_, end_, step_);
    build_index();
}

Slicer::Slicer(FrameSetSource&& source, int start, int end, int step)
    : FrameSetSourceWrapper(std::move(source)), start_(start), end_(end), step_(step) {
    normalize_slice(source_, start_, end_, step_);
    build_index();
}

FrameSetIterator Slicer::begin() const {
    return FrameSetIterator(
        this, new SlicerFrameSetIteratorImpl<FrameSetSource, FrameSetIterator, FrameSetIteratorImpl,
                                             FrameSet>(source_, start_, end_, step_));
}

FrameSetIterator Slicer::begin(int /*idx*/) const {
    // this is difficult to implement, not supported for now
    throw std::runtime_error("Not yet supported. Please single then slice.");
}

std::unique_ptr<FrameSetSource> Slicer::move() {
    return std::make_unique<Slicer>(std::move(*this));
}

void Slicer::build_index() {
    individual_index_.resize(source_->individual_index().size());

    frames_num_.resize(individual_index_.size());

    // now rebuild each
    int index = 0;
    auto& original = source_->full_index();
    for (int i = start_; i < end_; i++) {
        int delta = i - start_;
        if ((delta % step_) != 0) {
            continue;
        }

        auto& item = original[i];

        frames_num_[item.second]++;

        full_index_.push_back(item);
        individual_index_[item.second].emplace_back(item.first, index);
        index++;
    }
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& Slicer::individual_index() const {
    if (!is_indexed()) {
        throw std::runtime_error(
            "'individual_index' not supported on unindexed frame set sources.");
    }
    return individual_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& Slicer::full_index() const {
    if (!is_indexed()) {
        throw std::runtime_error("'full_index' not supported on unindexed frame set sources.");
    }
    return full_index_;
}

const std::vector<size_t>& Slicer::frames_num() const {
    if (!is_indexed()) {
        throw std::runtime_error("'frames_num' not supported on unindexed frame set sources.");
    }
    return frames_num_;
}

size_t Slicer::size_hint() const {
    return size();
}

AnyFrameSetSource::AnyFrameSetSource(std::unique_ptr<FrameSetSource> source)
    : FrameSetSourceWrapper(std::move(source)) {}

FrameSetIterator AnyFrameSetSource::begin() const {
    return parent_->begin();
}

FrameSetIterator AnyFrameSetSource::begin(int idx) const {
    return source_->begin(idx);
}

FrameSetIterator AnyFrameSetSource::end() const {
    return parent_->end();
}

std::unique_ptr<FrameSetSource> AnyFrameSetSource::move() {
    return std::make_unique<AnyFrameSetSource>(std::move(*this));
}

std::shared_ptr<FrameSetSource> AnyFrameSetSource::child() const {
    return parent_;
}

void AnyFrameSetSource::close() {
    parent_.reset();
}

AnyPacketSource::AnyPacketSource(std::unique_ptr<PacketSource> source)
    : source_(std::move(source)) {}

PacketIterator AnyPacketSource::begin() const {
    return source_->begin();
}

PacketIterator AnyPacketSource::end() const {
    return source_->end();
}

const std::vector<std::shared_ptr<SensorInfo>>& AnyPacketSource::sensor_info() const {
    return source_->sensor_info();
}

bool AnyPacketSource::is_live() const {
    return source_->is_live();
}

std::shared_ptr<PacketSource> AnyPacketSource::child() const {
    return source_;
}

void AnyPacketSource::close() {
    source_.reset();
}
}  // namespace core
}  // namespace sdk
}  // namespace ouster
