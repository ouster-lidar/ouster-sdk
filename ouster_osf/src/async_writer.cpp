#include "ouster/osf/async_writer.h"

#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/osf/osf_encoder.h"

using ouster::sdk::core::logger;
using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace osf {

AsyncWriter::AsyncWriter(const std::string& filename, const std::vector<SensorInfo>& info,
                         const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
                         std::shared_ptr<Encoder> encoder)
    : writer_(filename, info, fields_to_write, chunk_size, std::move(encoder)), save_queue_{10} {
    save_thread_ = std::thread([this] { save_thread_method(); });
}

void AsyncWriter::save_thread_method() {
    while (true) {
        nonstd::optional<LidarFrameMessage> msg = save_queue_.pop();
        if (msg == nonstd::nullopt) {
            break;
        }
        auto& msg_value = msg.value();
        std::lock_guard<std::mutex> lock(stream_mutex_);
        try {
            if (msg_value.saving_uncollated) {
                writer_.save(msg_value.stream_index, *msg_value.frame_set[0], msg_value.timestamp);
            } else {
                writer_.save(msg_value.frame_set);
            }
            msg_value.promise.set_value();
        } catch (const std::exception& ex) {
            logger().error("Exception when saving LidarFrame as OSF: {}", ex.what());
            try {
                msg_value.promise.set_exception(std::current_exception());
            } catch (...) {
                logger().error("An exception occurred during std::promise set_exception.");
            }
        }
    }
}

std::future<void> AsyncWriter::save(uint32_t stream_index, const LidarFrame& frame) {
    ts_t time;
    try {
        time = ts_t(frame.get_max_valid_packet_timestamp());
    } catch (const std::runtime_error& /*e*/) {
        time = ts_t(0);
    }
    return save(stream_index, frame, time);
}

std::future<void> AsyncWriter::save(uint32_t stream_index, const LidarFrame& frame,
                                    const ouster::sdk::osf::ts_t timestamp) {
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }

    // make sure timestamp didnt go backwards
    auto& last_timestamp = last_timestamp_[stream_index];
    if (timestamp < last_timestamp) {
        throw OsfDropFrameError(
            "ERROR: Can't write with a decreasing timestamp: " + std::to_string(timestamp.count()) +
            " for stream_index: " + std::to_string(stream_index) +
            " ( previously recorded timestamp: " + std::to_string(last_timestamp.count()) + ")");
    }
    last_timestamp = timestamp;
    std::promise<void> promise;
    std::future<void> result = promise.get_future();
    save_queue_.push(LidarFrameMessage(static_cast<int>(stream_index), timestamp, frame, promise));
    return result;
}

std::future<void> AsyncWriter::save(const FrameSet& frames) {
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    if (frames.size() != writer_.lidar_meta_id_.size()) {
        throw std::logic_error(
            "ERROR: Frames passed in to writer "
            "does not match number of sensor infos");
    }

    // make sure all frames have timestamps
    for (size_t i = 0; i < frames.size(); i++) {
        const auto& frame = frames[i];
        if (!frame) {
            continue;
        }
        ts_t timestamp{0};
        try {
            timestamp = ts_t(frame->get_max_valid_packet_timestamp());
        } catch (const std::runtime_error& /*e*/) {
        }

        if (timestamp.count() == 0) {
            throw OsfDropFrameError(
                "Tried saving collation with frames having no valid "
                "timestamps");
        }

        auto& last_timestamp = last_timestamp_[i];
        if (timestamp < last_timestamp) {
            throw OsfDropFrameError(
                "ERROR: Can't write with a decreasing timestamp: " +
                std::to_string(timestamp.count()) + " for stream_index: " + std::to_string(i) +
                " ( previously recorded timestamp: " + std::to_string(last_timestamp.count()) +
                ")");
        }
        last_timestamp = timestamp;
    }
    std::promise<void> promise;
    std::future<void> result = promise.get_future();
    save_queue_.push(LidarFrameMessage(frames, promise));
    return result;
}

void AsyncWriter::save(
    const ouster::sdk::core::FrameSetSourceMetadataSet& frame_set_source_metadata_set) {
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    std::lock_guard<std::mutex> lock(stream_mutex_);
    writer_.save(frame_set_source_metadata_set);
}

void AsyncWriter::close(bool fsync) {
    save_queue_.shutdown();
    if (save_thread_.joinable()) {
        save_thread_.join();
    }

    writer_.close(fsync);
}

AsyncWriter::~AsyncWriter() {
    close();
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
