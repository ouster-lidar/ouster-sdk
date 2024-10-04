/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/sensor_scan_source.h"

#include "ouster/impl/logging.h"

using ouster::sensor::impl::Logger;

namespace ouster {
namespace sensor {

SensorScanSource::SensorScanSource(const std::vector<Sensor>& sensors,
                                   double config_timeout,
                                   unsigned int queue_size, bool soft_id_check)
    : SensorScanSource(sensors, {}, {}, config_timeout, queue_size,
                       soft_id_check) {}

SensorScanSource::SensorScanSource(const std::vector<Sensor>& sensors,
                                   const std::vector<sensor_info>& infos,
                                   double config_timeout,
                                   unsigned int queue_size, bool soft_id_check)
    : SensorScanSource(sensors, infos, {}, config_timeout, queue_size,
                       soft_id_check) {}

SensorScanSource::SensorScanSource(
    const std::vector<Sensor>& sensors, const std::vector<sensor_info>& infos,
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
        for (const auto& meta : client_.get_sensor_info()) {
            fields_.push_back(get_field_types(meta.format.udp_profile_lidar));
        }
    }

    run_thread_ = true;
    batcher_thread_ = std::thread([this, queue_size, soft_id_check]() {
        LidarPacket lp;
        ImuPacket ip;
        std::vector<std::unique_ptr<LidarScan>> scans;
        std::vector<ScanBatcher> batchers;
        std::vector<packet_format> pfs;
        auto infos = get_sensor_info();
        for (size_t i = 0; i < infos.size(); i++) {
            const auto& info = infos[i];
            batchers.push_back(ScanBatcher(info));
            size_t w = info.format.columns_per_frame;
            size_t h = info.format.pixels_per_column;
            scans.push_back(std::make_unique<LidarScan>(
                w, h, fields_[i].begin(), fields_[i].end(),
                info.format.columns_per_packet));
            pfs.push_back(packet_format(info));
        }
        while (run_thread_) {
            auto p = client_.get_packet(lp, ip, 0.05);
            if (p.type == ClientEvent::LidarPacket) {
                const auto& pf = pfs[p.source];
                const auto& info = infos[p.source];
                auto result = lp.validate(info, pf);
                if (result == PacketValidationFailure::ID) {
                    id_error_count_++;
                    if (!soft_id_check) {
                        auto init_id = pf.init_id(lp.buf.data());
                        auto prod_sn = pf.prod_sn(lp.buf.data());
                        logger().warn(
                            "Metadata init_id/sn does not match: expected by "
                            "metadata - {}/{}, but got from packet buffer - "
                            "{}/{}",
                            info.init_id, info.sn, init_id, prod_sn);
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
                    size_t w = info.format.columns_per_frame;
                    size_t h = info.format.pixels_per_column;
                    scans[p.source] = std::make_unique<LidarScan>(
                        w, h, fields_[p.source].begin(),
                        fields_[p.source].end(),
                        info.format.columns_per_packet);
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

}  // namespace sensor
}  // namespace ouster
