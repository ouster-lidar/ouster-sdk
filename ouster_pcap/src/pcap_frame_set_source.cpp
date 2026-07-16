/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/pcap/pcap_frame_set_source.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/frame_set_source.h"
#include "ouster/core/impl/open_source_impl.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/packet_source.h"
#include "ouster/core/types.h"

using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace pcap {

PcapFrameSetSourceOptions::PcapFrameSetSourceOptions(const FrameSetSourceOptions& opts)
    : FrameSetSourceOptions(opts) {}

PcapFrameSetSourceOptions::PcapFrameSetSourceOptions() = default;

class PcapFrameSetIteratorImpl : public FrameSetIteratorImpl {
    const PcapFrameSetSource* source_;
    PacketIterator packet_iter_;
    std::shared_ptr<LidarFrame> frame_;
    int sensor_idx_;
    int64_t i_ = -1;

    std::vector<FrameBatcher> batchers_;
    std::vector<std::shared_ptr<LidarFrame>> frames_;

   public:
    // Clarify
    explicit PcapFrameSetIteratorImpl(const PcapFrameSetSource* source, int sensor_idx = -1)
        : source_(source) {
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        sensor_idx_ = sensor_idx < 0 ? -1 : sensor_idx;
        packet_iter_ = source->packets_.begin();

        for (const auto& info : source_->internal_sensor_info_) {
            batchers_.emplace_back(*info);
        }

        frames_.resize(source_->sensor_info().size());
    }

    bool advance(size_t offset) override {
        // seek!
        if (offset > 1) {
            if (!source_->indexed_) {
                throw std::runtime_error("Not supported on non-indexed frame set sources.");
            }

            if (i_ + static_cast<int64_t>(offset) >= length()) {
                throw std::out_of_range("Indexed past the end of the frame set source.");
            }
            // seek to the start of this frame then allow the loop below to
            // extract it

            // need to reset batchers and frames
            int i = 0;
            for (auto& batcher : batchers_) {
                batcher.reset();
                frames_[i++].reset();
            }

            packet_iter_ = source_->packets_.begin_frame(i_ + offset);
            i_ += static_cast<int>(offset - 1);
            offset = 1;
        }
        for (size_t i = 0; i < offset; i++) {
            // read packets until we get a frame or run out
            while (true) {
                if (packet_iter_ == source_->packets_.end()) {
                    // if finished return any partial frames we have
                    // todo should these be in any particular order?
                    bool found = false;
                    for (size_t i = 0; i < frames_.size(); i++) {
                        if (frames_[i]) {
                            // ignore frames with no packets batched
                            if (batchers_[i].batched_packets() == 0) {
                                frames_[i].reset();
                                continue;
                            }
                            frame_.reset();
                            frame_.swap(frames_[i]);
                            // override the sensor info from the batcher
                            frame_->sensor_info = source_->sensor_info()[i];
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

                // try and build a frame with this packet
                // make sure to copy this out as we increment right after
                auto packet = *packet_iter_;
                packet_iter_++;

                // todo handle other packet types in the future
                int index = packet.first;

                // skip packets from sensors we dont care about
                if (sensor_idx_ >= 0 && index != sensor_idx_) {
                    continue;
                }

                // allocate the frame if it hasnt already been
                if (!frames_[index]) {
                    frames_[index] = std::make_shared<LidarFrame>(
                        source_->internal_sensor_info_[index], source_->field_types_[index]);
                }

                // finally batch
                if (batchers_[index].batch(*packet.second, *frames_[index])) {
                    frame_.reset();
                    frame_.swap(frames_[index]);
                    // override the sensor info from the batcher
                    frame_->sensor_info = source_->sensor_info()[index];
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
            return static_cast<int64_t>(source_->frames_num()[sensor_idx_]);
        }
        return static_cast<int64_t>(source_->size());
    }

    FrameSet value() override {
        return FrameSet{{frame_}};
    }
};

FrameSetIterator PcapFrameSetSource::begin() const {
    return FrameSetIterator(this, new PcapFrameSetIteratorImpl(this));
}

FrameSetIterator PcapFrameSetSource::begin(int sensor_index) const {
    if (sensor_index >= static_cast<int>(sensor_info().size())) {
        throw std::runtime_error("Invalid sensor index");
    }
    return FrameSetIterator(this, new PcapFrameSetIteratorImpl(this, sensor_index));
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
PcapFrameSetSource::individual_index() const {
    assert_indexed("individual_index");
    return index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& PcapFrameSetSource::full_index() const {
    assert_indexed("full_index");
    return real_index_;
}

void PcapFrameSetSource::assert_indexed(const char* function) const {
    if (!indexed_) {
        throw std::runtime_error("Cannot perform '" + std::string(function) +
                                 "' on an unindexed source. Specify "
                                 "the index parameter as true when creating the source to "
                                 "produce an index.");
    }
}

/// open_source compatible constructor
PcapFrameSetSource::PcapFrameSetSource(
    const std::string& source, const std::function<void(PcapFrameSetSourceOptions&)>& options)
    : PcapFrameSetSource(source, ouster::sdk::impl::get_frame_set_source_options(options)) {}

/// open_source compatible constructor
PcapFrameSetSource::PcapFrameSetSource(const std::string& source, PcapFrameSetSourceOptions options)
    : packets_(source, PacketSourceOptions(reinterpret_cast<FrameSetSourceOptions&>(options))),
      indexed_(options.index.retrieve()) {
    if (indexed_) {
        const auto& index = packets_.reader_->get_index();

        // count the frames
        num_frames_ = 0;
        for (size_t i = 0; i < sensor_info().size(); i++) {
            auto count = index.frame_count(i);
            frames_count_.push_back(count);
            num_frames_ += count;
        }

        // timestamp based index of all frames in the file for each sensor, each
        // pair is timestamp followed by global frame index build index
        index_.resize(sensor_info().size());
        size_t i = 0;
        for (const auto& idx : index.global_frame_indices) {
            index_[idx.sensor_index].emplace_back(idx.timestamp, i++);
            real_index_.emplace_back(idx.timestamp, idx.sensor_index);
        }
    } else {
        // calculate estimated length if we are unindexed
        uint64_t size = packets_.reader_->file_size();

        // get average frame size plus some overhead
        const int pcap_pkt_header = 100;
        size_t frame_size = 0;
        for (const auto& sensor : sensor_info()) {
            PacketFormat packet_format(*sensor);
            auto pkt_size = packet_format.lidar_packet_size + pcap_pkt_header;
            frame_size += pkt_size * sensor->format.lidar_packets_per_frame();
            auto imu_pkt_size = packet_format.imu_packet_size + pcap_pkt_header;
            frame_size += imu_pkt_size * sensor->format.imu_packets_per_frame;
            if (sensor->format.zone_monitoring_enabled) {
                frame_size += packet_format.zone_packet_size + pcap_pkt_header;
            }
        }

        if (frame_size == 0) {
            throw std::runtime_error("Unexpected frame data size of 0. SensorInfo may be corrupt.");
        }

        // divide size of all frames to get average frame size
        frame_size /= sensor_info().size();

        // number of frames is approximately file size divided by frame size
        size_hint_ = size / frame_size;
    }

    internal_sensor_info_.reserve(sensor_info().size());
    for (auto& info : sensor_info()) {
        internal_sensor_info_.push_back(std::make_shared<SensorInfo>(*info));
    }

    field_types_ =
        resolve_field_types(sensor_info(), options.raw_headers.retrieve(),
                            options.raw_fields.retrieve(), options.field_names.retrieve());
    options.check("PcapFrameSetSource");
}

const std::vector<std::shared_ptr<SensorInfo>>& PcapFrameSetSource::sensor_info() const {
    return packets_.sensor_info();
}

size_t PcapFrameSetSource::size() const {
    assert_indexed("size");

    return num_frames_;
}

size_t PcapFrameSetSource::size_hint() const {
    if (indexed_) {
        return num_frames_;
    }

    return size_hint_;
}

bool PcapFrameSetSource::is_indexed() const {
    return indexed_;
}

uint64_t PcapFrameSetSource::id_error_count() const {
    return packets_.id_error_count();
}

uint64_t PcapFrameSetSource::size_error_count() const {
    return packets_.size_error_count();
}

const std::vector<size_t>& PcapFrameSetSource::frames_num() const {
    assert_indexed("frames_num");

    return frames_count_;
}

void PcapFrameSetSource::close() {
    packets_.close();
}

std::unique_ptr<FrameSetSource> PcapFrameSetSource::move() {
    return std::make_unique<PcapFrameSetSource>(std::move(*this));
}

std::unique_ptr<FrameSetSource> PcapFrameSetSource::create(const std::vector<std::string>& sources,
                                                           const FrameSetSourceOptions& options,
                                                           bool collate, int sensor_idx) {
    if (sources.size() > 1) {
        throw std::invalid_argument("PcapFrameSetSource allows opening only one file at a time.");
    }

    std::unique_ptr<FrameSetSource> source =
        std::make_unique<PcapFrameSetSource>(sources[0], options);
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
