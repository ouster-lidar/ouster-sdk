/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/sensor/sensor_frame_set_source.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/impl/logging.h"
#include "ouster/core/open_source.h"

using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace sensor {

class SensorFrameSetSource;
class SensorFrameSetIteratorImpl : public ouster::sdk::core::FrameSetIteratorImpl {
    SensorFrameSetSource* source_;
    std::shared_ptr<LidarFrame> frame_;
    int sensor_idx_ = -1;

   public:
    SensorFrameSetIteratorImpl(SensorFrameSetSource* source, int sensor_idx = -1)
        : source_(source) {
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        sensor_idx_ = sensor_idx < 0 ? -1 : sensor_idx;
    }

    bool advance(size_t offset) override {
        // default to a get_frame timeout of 1 if timeout is disabled
        auto get_timeout = source_->timeout_;
        if (get_timeout <= 0) {
            get_timeout = 1.0;
        }
        for (size_t i = 0; i < offset; i++) {
            auto result = source_->get_frame(get_timeout);

            // check for timeouts if enabled
            if (source_->timeout_ > 0) {
                int64_t now = 0;
                if (result.second) {
                    now = static_cast<int64_t>(result.second->get_last_valid_packet_timestamp());
                } else {
                    auto now_sec = std::chrono::system_clock::now();
                    now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now_sec.time_since_epoch())
                              .count();
                }

                if (result.second) {
                    source_->last_receive_times_[result.first] = now;
                }

                int i = 0;
                for (const auto time : source_->last_receive_times_) {
                    if (now - time > source_->timeout_ns_) {
                        const auto& metadata = source_->sensor_info()[i];
                        throw ouster::sdk::sensor::ClientTimeout(
                            "No valid frames received within " + std::to_string(source_->timeout_) +
                            " from sensor " + std::to_string(metadata->sn) +
                            " using udp destination '" + metadata->config.udp_dest.value_or("") +
                            "' on port " +
                            std::to_string(metadata->config.udp_port_lidar.value_or(0)));
                    }
                    i++;
                }
            }

            if (!result.second) {
                offset++;
                continue;
            }

            // skip frames we want to filter out
            if (sensor_idx_ >= 0 && result.first != sensor_idx_) {
                offset++;
                continue;
            }

            frame_.reset(result.second.release());
        }
        return false;
    }

    FrameSet value() override {
        return FrameSet{{frame_}};
    }
};

SensorFrameSetSource::SensorFrameSetSource(
    const std::string& source, const std::function<void(SensorFrameSetSourceOptions&)>& options)
    : SensorFrameSetSource(source, ouster::sdk::impl::get_frame_set_source_options(options)) {}

SensorFrameSetSource::SensorFrameSetSource(
    const std::vector<std::string>& source,
    const std::function<void(SensorFrameSetSourceOptions&)>& options)
    : SensorFrameSetSource(source, ouster::sdk::impl::get_frame_set_source_options(options)) {}

SensorFrameSetSource::SensorFrameSetSource(const std::string& source,
                                           SensorFrameSetSourceOptions options)
    : SensorFrameSetSource(std::vector<std::string>({source}), std::move(options)) {}

SensorFrameSetSource::SensorFrameSetSource(const std::vector<std::string>& source,
                                           SensorFrameSetSourceOptions options)
    : client_(source, SensorPacketSourceOptions(
                          PacketSourceOptions(reinterpret_cast<FrameSetSourceOptions&>(options)))),
      id_error_count_{0},
      timeout_(options.timeout.retrieve()),
      timeout_ns_(static_cast<int64_t>(options.timeout.retrieve() * 1e9)) {
    fields_ = resolve_field_types(sensor_info(), options.raw_headers.retrieve(),
                                  options.raw_fields.retrieve(), options.field_names.retrieve());

    start_thread(options.queue_size.retrieve(), options.soft_id_check.retrieve());

    options.check("SensorFrameSetSource");
}

SensorFrameSetSource::SensorFrameSetSource(const std::vector<Sensor>& sensors,
                                           double config_timeout, unsigned int queue_size,
                                           bool soft_id_check)
    : SensorFrameSetSource(sensors, {}, {}, config_timeout, queue_size, soft_id_check) {}

SensorFrameSetSource::SensorFrameSetSource(const std::vector<Sensor>& sensors,
                                           const std::vector<SensorInfo>& infos,
                                           double config_timeout, unsigned int queue_size,
                                           bool soft_id_check)
    : SensorFrameSetSource(sensors, infos, {}, config_timeout, queue_size, soft_id_check) {}

SensorFrameSetSource::SensorFrameSetSource(const std::vector<Sensor>& sensors,
                                           const std::vector<SensorInfo>& infos,
                                           const std::vector<LidarFrameFieldTypes>& fields,
                                           double config_timeout, unsigned int queue_size,
                                           bool soft_id_check)
    : client_(sensors, infos, config_timeout), id_error_count_{0} {
    if (queue_size == 0) {
        throw std::invalid_argument("The queue_size cannot be less than 1.");
    }

    if ((!infos.empty()) && infos.size() != sensors.size()) {
        throw std::invalid_argument(
            "If sensor_infos are provided, must provide one for each sensor.");
    }

    if ((!fields.empty()) && fields.size() != sensors.size()) {
        throw std::invalid_argument("If fields are provided, must provide one for each sensor.");
    }

    fields_ = fields;
    if (fields_.empty()) {
        for (const auto& meta : client_.sensor_info()) {
            fields_.push_back(get_field_types(*meta));
        }
    }

    start_thread(queue_size, soft_id_check);
}

void SensorFrameSetSource::start_thread(unsigned int queue_size, bool soft_id_check) {
    auto now_sec = std::chrono::system_clock::now();
    int64_t now =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now_sec.time_since_epoch()).count();
    last_receive_times_.resize(sensor_info().size(), now);
    run_thread_ = true;
    for (const auto& info : sensor_info()) {
        orig_sensor_info_.push_back(std::make_shared<SensorInfo>(*info));
    }
    batcher_thread_ = std::thread([this, queue_size, soft_id_check]() {
        std::vector<std::unique_ptr<LidarFrame>> frames;
        std::vector<FrameBatcher> batchers;
        for (size_t i = 0; i < orig_sensor_info_.size(); i++) {
            const auto& info = orig_sensor_info_[i];
            batchers.emplace_back(info);
            frames.push_back(std::make_unique<LidarFrame>(info, fields_[i]));
        }
        while (run_thread_) {
            auto packet_event = client_.get_packet(0.05);
            if (packet_event.type == ClientEvent::PACKET) {
                const auto& info = orig_sensor_info_[packet_event.source];
                const auto& packet = packet_event.packet();

                auto result = packet.validate(*info);
                if (result == PacketValidationFailure::ID) {
                    id_error_count_++;
                    if (!soft_id_check) {
                        logger().warn(
                            "Metadata init_id/sn does not match: expected "
                            "by metadata - {}/{}, but got from packet "
                            "buffer - {}/{}",
                            info->init_id, info->sn, packet.init_id(), packet.prod_sn());
                        continue;
                    }
                }

                // Add the packet to the batch
                if (batchers[packet_event.source].batch(packet, *frames[packet_event.source])) {
                    {
                        std::unique_lock<std::mutex> lock(buffer_mutex_);
                        frames[packet_event.source]->sensor_info =
                            sensor_info()[packet_event.source];
                        buffer_.emplace_back(packet_event.source,
                                             std::move(frames[packet_event.source]));
                        while (buffer_.size() > queue_size) {
                            buffer_.pop_front();
                            dropped_frames_++;
                        }
                        buffer_cv_.notify_one();
                    }
                    frames[packet_event.source] =
                        std::make_unique<LidarFrame>(info, fields_[packet_event.source]);
                }
            }
        }
    });
}

SensorFrameSetSource::~SensorFrameSetSource() {
    close();
}

std::pair<int, std::unique_ptr<LidarFrame>> SensorFrameSetSource::get_frame(double timeout_sec) {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    // if theres anything in the queue, just pop it and leave
    if (!buffer_.empty()) {
        auto result = std::move(buffer_.front());
        buffer_.pop_front();
        return result;
    }

    // otherwise we have to wait
    auto duration = std::chrono::duration<double>(timeout_sec);
    buffer_cv_.wait_for(lock, duration, [this] { return !buffer_.empty() || !run_thread_; });
    // check for timeout or for spurious wakeup of "wait_for"
    // by checking whether the buffer is empty
    if (buffer_.empty()) {
        return {0, std::unique_ptr<LidarFrame>(nullptr)};
    }

    // return the result
    auto result = std::move(buffer_.front());
    buffer_.pop_front();
    return result;
}

OUSTER_DIAGNOSTIC_PUSH
OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED
std::pair<int, std::unique_ptr<LidarFrame>> SensorFrameSetSource::get_scan(double timeout_sec) {
    return get_frame(timeout_sec);
}
OUSTER_DIAGNOSTIC_POP

void SensorFrameSetSource::flush() {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
}

size_t SensorFrameSetSource::size_hint() const {
    return 0;
}

void SensorFrameSetSource::close() {
    run_thread_ = false;
    buffer_cv_.notify_all();
    if (batcher_thread_.joinable()) {
        batcher_thread_.join();
    }
    client_.close();
}

core::FrameSetIterator SensorFrameSetSource::begin() const {
    return core::FrameSetIterator(
        this, new SensorFrameSetIteratorImpl(const_cast<SensorFrameSetSource*>(this)));
}

core::FrameSetIterator SensorFrameSetSource::begin(int sensor_index) const {
    if (sensor_index >= static_cast<int>(sensor_info().size())) {
        throw std::runtime_error("Invalid index");
    }
    return core::FrameSetIterator(this, new SensorFrameSetIteratorImpl(
                                            const_cast<SensorFrameSetSource*>(this), sensor_index));
}

std::unique_ptr<core::FrameSetSource> SensorFrameSetSource::create(
    const std::vector<std::string>& sources, const FrameSetSourceOptions& options, bool collate,
    int sensor_idx) {
    std::unique_ptr<core::FrameSetSource> source =
        std::make_unique<SensorFrameSetSource>(sources, options);
    if (sensor_idx >= 0) {
        source = std::make_unique<Singler>(std::move(source), sensor_idx);
    } else if (collate) {
        source = std::make_unique<Collator>(std::move(source));
    }

    return source;
};

const std::vector<SOCKET>& SensorFrameSetSource::sockets() const {
    return client_.sockets();
}

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster
