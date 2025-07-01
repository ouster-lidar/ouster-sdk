#include "ouster/osf/async_writer.h"

#include "ouster/impl/logging.h"
#include "ouster/osf/stream_lidar_scan.h"

using ouster::sensor::logger;

namespace ouster {
namespace osf {

AsyncWriter::AsyncWriter(const std::string& filename,
                         const std::vector<ouster::sensor::sensor_info>& info,
                         const std::vector<std::string>& fields_to_write,
                         uint32_t chunk_size, std::shared_ptr<Encoder> encoder)
    : writer_(filename, info, fields_to_write, chunk_size, encoder),
      save_queue_{10} {
    save_thread_ = std::thread([this] { save_thread_method(); });
}

void AsyncWriter::save_thread_method() {
    while (true) {
        nonstd::optional<LidarScanMessage> msg = save_queue_.pop();
        if (msg == nonstd::nullopt) {
            break;
        }
        auto& msg_value = msg.value();
        std::lock_guard<std::mutex> lock(stream_mutex_);
        try {
            writer_._save(msg_value.stream_index_, msg_value.lidar_scan_,
                          msg_value.timestamp_);
            msg_value.promise_.set_value();
        } catch (const std::exception& ex) {
            logger().error("Exception when saving LidarScan as OSF: {}",
                           ex.what());
            try {
                msg_value.promise_.set_exception(std::current_exception());
            } catch (...) {
                logger().error(
                    "An exception occurred during std::promise set_exception.");
            }
        }
    }
}

std::future<void> AsyncWriter::save(uint32_t stream_index,
                                    const LidarScan& scan) {
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    std::promise<void> promise;
    std::future<void> result = promise.get_future();
    ts_t time = ts_t(scan.get_first_valid_packet_timestamp());
    save_queue_.push(LidarScanMessage(stream_index, time, scan, promise));
    return result;
}

std::future<void> AsyncWriter::save(uint32_t stream_index,
                                    const LidarScan& scan,
                                    const ouster::osf::ts_t timestamp) {
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    std::promise<void> promise;
    std::future<void> result = promise.get_future();
    save_queue_.push(LidarScanMessage(stream_index, timestamp, scan, promise));
    return result;
}

std::vector<std::future<void>> AsyncWriter::save(
    const std::vector<LidarScan>& scans) {
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    if (scans.size() != writer_.lidar_meta_id_.size()) {
        throw std::logic_error(
            "ERROR: Scans passed in to writer "
            "does not match number of sensor infos");
    } else {
        std::vector<std::future<void>> results;
        for (uint32_t i = 0; i < scans.size(); i++) {
            ts_t time = ts_t(scans[i].get_first_valid_packet_timestamp());
            std::promise<void> promise;
            std::future<void> result = promise.get_future();
            save_queue_.push(LidarScanMessage(i, time, scans[i], promise));
            results.push_back(std::move(result));
        }
        return results;
    }
}

void AsyncWriter::close() {
    save_queue_.shutdown();
    if (save_thread_.joinable()) {
        save_thread_.join();
    }
    writer_.close();
}

AsyncWriter::~AsyncWriter() { close(); }

}  // namespace osf
}  // namespace ouster
