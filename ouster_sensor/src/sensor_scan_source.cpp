/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/sensor_scan_source.h"

#include <chrono>

#include "ouster/impl/logging.h"
#include "ouster/open_source.h"

using ouster::sensor::impl::Logger;

namespace ouster {
namespace sensor {

class SensorScanSource;
class SensorScanIteratorImpl : public ouster::core::ScanIteratorImpl {
    SensorScanSource* source_;
    std::vector<std::shared_ptr<LidarScan>> scan_;
    int sensor_idx_ = -1;

   public:
    SensorScanIteratorImpl(SensorScanSource* ss, int sensor_idx = -1) {
        source_ = ss;
        sensor_idx_ = sensor_idx < 0 ? -1 : sensor_idx;
    }

    bool advance(size_t offset) override {
        // default to a get_scan timeout of 1 if timeout is disabled
        auto get_timeout = source_->timeout_;
        if (get_timeout <= 0) {
            get_timeout = 1.0;
        }
        for (size_t i = 0; i < offset; i++) {
            auto scan = source_->get_scan(get_timeout);

            // check for timeouts if enabled
            if (source_->timeout_ > 0) {
                int64_t now;
                if (scan.second) {
                    now = scan.second->get_last_valid_packet_timestamp();
                } else {
                    auto now_sec = std::chrono::system_clock::now();
                    now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now_sec.time_since_epoch())
                              .count();
                }

                if (scan.second) {
                    source_->last_receive_times_[scan.first] = now;
                }

                int i = 0;
                for (const auto t : source_->last_receive_times_) {
                    if (now - t > source_->timeout_ns_) {
                        const auto& metadata = source_->sensor_info()[i];
                        throw ouster::sensor::ClientTimeout(
                            "No valid scans received within " +
                            std::to_string(source_->timeout_) +
                            " from sensor " + std::to_string(metadata->sn) +
                            " using udp destination '" +
                            metadata->config.udp_dest.value_or("") +
                            "' on port " +
                            std::to_string(
                                metadata->config.udp_port_lidar.value_or(0)));
                    }
                    i++;
                }
            }

            if (!scan.second) {
                offset++;
                continue;
            }

            // skip scans we want to filter out
            if (sensor_idx_ >= 0 && scan.first != sensor_idx_) {
                offset++;
                continue;
            }

            scan_ = {std::shared_ptr<LidarScan>(scan.second.release())};
        }
        return false;
    }

    std::vector<std::shared_ptr<LidarScan>>& value() override { return scan_; }
};

SensorScanSource::SensorScanSource(
    const std::string& source,
    const std::function<void(SensorScanSourceOptions&)>& options)
    : SensorScanSource(source, ouster::impl::get_scan_options(options)) {}

SensorScanSource::SensorScanSource(
    const std::vector<std::string>& source,
    const std::function<void(SensorScanSourceOptions&)>& options)
    : SensorScanSource(source, ouster::impl::get_scan_options(options)) {}

SensorScanSource::SensorScanSource(const std::string& source,
                                   SensorScanSourceOptions options)
    : SensorScanSource(std::vector<std::string>({source}), options) {}

SensorScanSource::SensorScanSource(const std::vector<std::string>& source,
                                   SensorScanSourceOptions options)
    : client_(source, SensorPacketSourceOptions(
                          PacketSourceOptions((ScanSourceOptions&)options))),
      timeout_(options.timeout.retrieve()),
      timeout_ns_(options.timeout.retrieve() * 1e9) {
    id_error_count_ = 0;

    fields_ = ouster::resolve_field_types(
        sensor_info(), options.raw_headers.retrieve(),
        options.raw_fields.retrieve(), options.field_names.retrieve());

    start_thread(options.queue_size.retrieve(), false);

    options.check("SensorScanSource");
}

SensorScanSource::SensorScanSource(const std::vector<Sensor>& sensors,
                                   double config_timeout,
                                   unsigned int queue_size, bool soft_id_check)
    : SensorScanSource(sensors, {}, {}, config_timeout, queue_size,
                       soft_id_check) {}

SensorScanSource::SensorScanSource(
    const std::vector<Sensor>& sensors,
    const std::vector<ouster::sensor::sensor_info>& infos,
    double config_timeout, unsigned int queue_size, bool soft_id_check)
    : SensorScanSource(sensors, infos, {}, config_timeout, queue_size,
                       soft_id_check) {}

SensorScanSource::SensorScanSource(
    const std::vector<Sensor>& sensors,
    const std::vector<ouster::sensor::sensor_info>& infos,
    const std::vector<LidarScanFieldTypes>& fields, double config_timeout,
    unsigned int queue_size, bool soft_id_check)
    : client_(sensors, infos, config_timeout) {
    id_error_count_ = 0;
    if (queue_size == 0) {
        throw std::invalid_argument("The queue_size cannot be less than 1.");
    }

    if (infos.size() && infos.size() != sensors.size()) {
        throw std::invalid_argument(
            "If sensor_infos are provided, must provide one for each sensor.");
    }

    if (fields.size() && fields.size() != sensors.size()) {
        throw std::invalid_argument(
            "If fields are provided, must provide one for each sensor.");
    }

    fields_ = fields;
    if (fields_.size() == 0) {
        for (const auto& meta : client_.sensor_info()) {
            fields_.push_back(get_field_types(meta->format.udp_profile_lidar));
        }
    }

    start_thread(queue_size, soft_id_check);
}

void SensorScanSource::start_thread(unsigned int queue_size,
                                    bool soft_id_check) {
    auto now_sec = std::chrono::system_clock::now();
    int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      now_sec.time_since_epoch())
                      .count();
    last_receive_times_.resize(sensor_info().size(), now);
    run_thread_ = true;
    batcher_thread_ = std::thread([this, queue_size, soft_id_check]() {
        std::vector<std::unique_ptr<LidarScan>> scans;
        std::vector<ScanBatcher> batchers;
        auto infos = sensor_info();
        for (size_t i = 0; i < infos.size(); i++) {
            const auto& info = infos[i];
            batchers.push_back(ScanBatcher(info));
            size_t w = info->format.columns_per_frame;
            size_t h = info->format.pixels_per_column;
            scans.push_back(std::make_unique<LidarScan>(
                w, h, fields_[i].begin(), fields_[i].end(),
                info->format.columns_per_packet));
        }
        while (run_thread_) {
            auto p = client_.get_packet(0.05);
            if (p.type == ClientEvent::Packet &&
                p.packet().type() == PacketType::Lidar) {
                const auto& info = infos[p.source];
                const auto& lp = static_cast<LidarPacket&>(p.packet());
                auto result = lp.validate(*info);
                if (result == PacketValidationFailure::ID) {
                    id_error_count_++;
                    if (!soft_id_check) {
                        logger().warn(
                            "Metadata init_id/sn does not match: expected by "
                            "metadata - {}/{}, but got from packet buffer - "
                            "{}/{}",
                            info->init_id, info->sn, lp.init_id(),
                            lp.prod_sn());
                        continue;
                    }
                }

                // Add the packet to the batch
                if (batchers[p.source](lp, *scans[p.source])) {
                    {
                        std::unique_lock<std::mutex> lock(buffer_mutex_);
                        buffer_.push_back(
                            {p.source, std::move(scans[p.source])});
                        while (buffer_.size() > queue_size) {
                            buffer_.pop_front();
                            dropped_scans_++;
                        }
                        buffer_cv_.notify_one();
                    }
                    size_t w = info->format.columns_per_frame;
                    size_t h = info->format.pixels_per_column;
                    scans[p.source] = std::make_unique<LidarScan>(
                        w, h, fields_[p.source].begin(),
                        fields_[p.source].end(),
                        info->format.columns_per_packet);
                }
            }
        }
    });
}

SensorScanSource::~SensorScanSource() { close(); }

std::pair<int, std::unique_ptr<LidarScan>> SensorScanSource::get_scan(
    double timeout_sec) {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    // if theres anything in the queue, just pop it and leave
    if (buffer_.size()) {
        auto result = std::move(buffer_.front());
        buffer_.pop_front();
        return result;
    }

    // otherwise we have to wait
    auto duration = std::chrono::duration<double>(timeout_sec);
    buffer_cv_.wait_for(lock, duration,
                        [this] { return !buffer_.empty() || !run_thread_; });
    // check for timeout or for spurious wakeup of "wait_for"
    // by checking whether the buffer is empty
    if (buffer_.empty()) {
        return {0, std::unique_ptr<LidarScan>(nullptr)};
    }

    // return the result
    auto result = std::move(buffer_.front());
    buffer_.pop_front();
    return result;
}

void SensorScanSource::flush() {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
}

void SensorScanSource::close() {
    run_thread_ = false;
    buffer_cv_.notify_all();
    if (batcher_thread_.joinable()) {
        batcher_thread_.join();
    }
    client_.close();
}

core::ScanIterator SensorScanSource::begin() const {
    return core::ScanIterator(
        this, new SensorScanIteratorImpl((SensorScanSource*)this));
}

core::ScanIterator SensorScanSource::begin(int sensor_index) const {
    if (sensor_index >= (int)sensor_info().size()) {
        throw std::runtime_error("Invalid index");
    }
    return core::ScanIterator(this, new SensorScanIteratorImpl(
                                        (SensorScanSource*)this, sensor_index));
}

}  // namespace sensor
}  // namespace ouster
