#include "ouster/osf/async_writer.h"

#include <algorithm>
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

#include "ouster/core/camera_frame.h"
#include "ouster/core/frame.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/types.h"
#include "ouster/osf/osf_encoder.h"

using ouster::sdk::core::logger;
using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace osf {

AsyncWriter::AsyncWriter(const std::string& filename,
                         const std::vector<std::shared_ptr<SensorInfo>>& info,
                         const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
                         std::shared_ptr<Encoder> encoder)
    : writer_(filename, info, fields_to_write, chunk_size, std::move(encoder)),
      num_sensors_(info.size()),
      save_queue_{10} {
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

namespace {
// Derive an indexing timestamp from a frame when the caller does not provide
// one. LidarFrames use their max valid packet timestamp; other frames (e.g.
// CameraFrame) have no packets, so the max receive_timestamp entry (populated
// for per-frame timing) is used.
ts_t derive_frame_timestamp(const Frame& frame) {
    if (const auto* lidar = dynamic_cast<const LidarFrame*>(&frame)) {
        try {
            return ts_t(lidar->get_max_valid_receive_timestamp());
        } catch (const std::runtime_error& /*e*/) {
            return ts_t(0);
        }
    }
    const auto packet_ts = frame.receive_timestamp();
    uint64_t max_ts = 0;
    for (const auto& packet_time : packet_ts) {
        max_ts = std::max(max_ts, packet_time);
    }
    return ts_t(max_ts);
}

// Lidar and camera stream indices are independent 0-based spaces, so encode the
// frame type in the high bits to keep their last-timestamp tracking separate.
uint64_t make_timestamp_key(uint32_t stream_index, bool is_camera) {
    return (static_cast<uint64_t>(is_camera) << 32) | stream_index;
}
}  // namespace

std::future<void> AsyncWriter::save(uint32_t stream_index, const Frame& frame) {
    return save(stream_index, frame, derive_frame_timestamp(frame));
}

std::future<void> AsyncWriter::save(uint32_t stream_index, const Frame& frame,
                                    const ouster::sdk::osf::ts_t timestamp) {
    std::lock_guard<std::mutex> lock(save_mutex_);
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }

    // make sure timestamp didn't go backwards
    const bool is_camera = (dynamic_cast<const CameraFrame*>(&frame) != nullptr);
    auto& last_timestamp = last_timestamp_[make_timestamp_key(stream_index, is_camera)];
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
    std::lock_guard<std::mutex> lock(save_mutex_);
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    // The sensor count is fixed at construction (AsyncWriter has no
    // add_sensor()), so it can be validated directly against the FrameSet size.
    if (frames.size() != num_sensors_) {
        throw std::logic_error(
            "ERROR: Frames passed in to writer "
            "does not match number of sensor infos");
    }

    // Make sure all lidar frames have valid, non-decreasing timestamps. Camera
    // frames are skipped by the dynamic_pointer_cast below.
    for (size_t i = 0; i < frames.size(); i++) {
        const auto frame = std::dynamic_pointer_cast<LidarFrame>(frames[i]);
        if (!frame) {
            continue;
        }
        ts_t timestamp{0};
        try {
            timestamp = ts_t(frame->get_max_valid_receive_timestamp());
        } catch (const std::runtime_error& /*e*/) {
        }

        if (timestamp.count() == 0) {
            throw OsfDropFrameError(
                "Tried saving collation with frames having no valid "
                "timestamps");
        }

        auto& last_timestamp =
            last_timestamp_[make_timestamp_key(static_cast<uint32_t>(i), /*is_camera=*/false)];
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
    std::lock_guard<std::mutex> producer_lock(save_mutex_);
    if (writer_.is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    std::lock_guard<std::mutex> lock(stream_mutex_);
    writer_.save(frame_set_source_metadata_set);
}

void AsyncWriter::close(bool fsync) {
    // Serialize with the save() overloads so that no frame can be enqueued
    // after the queue has been shut down (which would leave its future
    // unfulfilled). This mutex is never held by the save thread, so joining it
    // below cannot deadlock.
    std::lock_guard<std::mutex> lock(save_mutex_);
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
