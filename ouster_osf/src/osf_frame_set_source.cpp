/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/osf_frame_set_source.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <functional>
#include <ios>
#include <iterator>
#include <map>
#include <memory>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/frame_set_source.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/types.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/meta_camera_sensor.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata_stream.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_camera_frame.h"
#include "ouster/osf/stream_lidar_frame.h"
#include "ouster/osf/writer.h"

using ouster::sdk::core::logger;
using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace osf {

OsfFrameSetSourceOptions::OsfFrameSetSourceOptions(const FrameSetSourceOptions& options)
    : FrameSetSourceOptions(options) {}

OsfFrameSetSourceOptions::OsfFrameSetSourceOptions() = default;

namespace {

/// Sentinel frame-stream id for a registered sensor that produced no frames.
constexpr uint32_t INVALID_STREAM_ID = 0xFFFFFFFFu;

/// Decode a single message into the concrete Frame type implied by its
/// SensorInfo (LidarInfo -> LidarFrame, CameraInfo -> CameraFrame) and attach
/// that info. Lidar and camera frames are handled identically by the callers;
/// only this decode step differs. Returns nullptr on decode failure.
std::shared_ptr<ouster::sdk::core::Frame> decode_frame(
    const std::shared_ptr<ouster::sdk::core::SensorInfo>& info, const MessageRef& msg,
    const nonstd::optional<std::vector<std::string>>& desired_fields) {
    if (auto camera_info = std::dynamic_pointer_cast<ouster::sdk::core::CameraInfo>(info)) {
        auto camera_frame = msg.decode_msg<ouster::sdk::osf::CameraFrameStream>(desired_fields);
        if (!camera_frame) {
            return nullptr;
        }
        camera_frame->set_sensor_info(std::move(camera_info));
        return std::shared_ptr<ouster::sdk::core::Frame>(camera_frame.release());
    }
    auto frame = msg.decode_msg<ouster::sdk::osf::LidarFrameStream>(desired_fields);
    if (!frame) {
        return nullptr;
    }
    // The info came in as the base type and is only known to be non-camera
    // here, so this is one of the few genuinely runtime-typed attachments.
    ouster::sdk::core::impl::assign_sensor_info(*frame, info);
    return std::shared_ptr<ouster::sdk::core::Frame>(frame.release());
}

}  // namespace

class OsfFrameSetIteratorImpl : public ouster::sdk::core::FrameSetIteratorImpl {
    OsfFrameSetSource* source_;
    ouster::sdk::osf::MessagesStreamingIter iter_, end_iter_;
    ouster::sdk::osf::SequentialMessagesStreamingIter siter_, end_siter_;
    size_t i_ = 0;
    int64_t sensor_idx_;
    bool first_ = true;

    std::vector<uint32_t> stream_ids_;

   public:
    explicit OsfFrameSetIteratorImpl(OsfFrameSetSource* source, int sensor_idx = -1)
        : source_{source}, sensor_idx_{sensor_idx} {
        if (sensor_idx >= 0) {
            // sensor_idx is a unified sensor index into stream_ids_ (the same
            // order exposed by sensor_info()); it can select a lidar or camera.
            const uint32_t stream_id = source_->stream_ids_[sensor_idx];
            if (stream_id == INVALID_STREAM_ID) {
                return;  // registered sensor with no frames
            }
            stream_ids_ = {stream_id};
        } else {
            for (const uint32_t stream_id : source_->stream_ids_) {
                if (stream_id != INVALID_STREAM_ID) {
                    stream_ids_.push_back(stream_id);
                }
            }
        }
        if (stream_ids_.empty()) {
            return;
        } else if (stream_ids_.size() == 1) {
            auto range = source_->reader_->messages_by_index(stream_ids_[0], 0,
                                                             std::numeric_limits<uint32_t>::max());
            siter_ = range.begin();
            end_siter_ = range.end();
        } else {
            auto range = source_->reader_->messages(stream_ids_);
            iter_ = range.begin();
            end_iter_ = range.end();
        }
    }

    bool advance(size_t offset) override {
        if (!source_->reader_) {
            throw std::runtime_error("Invalid operation on closed frame set source.");
        }
        // we already are at the first frame, so don't advance the first time
        if (first_) {
            first_ = false;
            offset--;

            if (iter_ == end_iter_ && siter_ == end_siter_) {
                return true;
            }
        }
        if (offset > 1) {
            if (!source_->indexed_) {
                throw std::runtime_error("Not supported on non-indexed frame set sources.");
            }
            // seek
            i_ += offset;

            if (stream_ids_.size() == 1) {
                // seek within a sensor
                auto range = source_->reader_->messages_by_index(
                    stream_ids_[0], i_, std::numeric_limits<uint32_t>::max());
                siter_ = range.begin();
                end_siter_ = range.end();
                if (siter_ == end_siter_) {
                    throw std::out_of_range("Tried accessing frame beyond index range");
                }
            } else {
                // seek globally
                if (i_ >= source_->index_.size()) {
                    throw std::out_of_range("Tried accessing frame beyond index range");
                }
                auto start = ouster::sdk::osf::ts_t(source_->index_[i_].first);
                auto range =
                    source_->reader_->messages(stream_ids_, start, source_->reader_->end_ts());
                iter_ = range.begin();
                end_iter_ = range.end();
            }
        } else if (offset == 1) {
            // just advance
            if (stream_ids_.size() == 1) {
                ++siter_;
            } else {
                ++iter_;
            }
            if (iter_ == end_iter_ && siter_ == end_siter_) {
                return true;  // we hit the end
            } else {
                i_++;
            }
        }
        return false;
    }

    int64_t length() override {
        if (sensor_idx_ >= 0) {
            return static_cast<int64_t>(source_->frames_num()[sensor_idx_]);
        }
        return static_cast<int64_t>(source_->size());
    }

    // The message under whichever iterator is active: a single stream is
    // walked by message index, several streams are merged by timestamp.
    ouster::sdk::osf::MessageRef current_message() const {
        return stream_ids_.size() == 1 ? *siter_ : *iter_;
    }

    FrameSet value() override {
        const auto message = current_message();
        auto info_it = source_->info_by_stream_.find(message.id());
        std::shared_ptr<ouster::sdk::core::SensorInfo> info =
            info_it != source_->info_by_stream_.end() ? info_it->second : nullptr;

        // Lidar and camera frames are handled identically: decode the single
        // message into its concrete frame type and yield a 1-frame FrameSet.
        // Downstream collation (if requested) positions frames by sensor.
        auto frame = decode_frame(info, message, source_->desired_fields_);
        if (!frame) {
            return FrameSet{};
        }
        FrameSet out{};
        out.frames().emplace_back(std::move(frame));
        return out;
    }
};

class OsfCollationIteratorImpl : public ouster::sdk::core::FrameSetIteratorImpl {
    OsfFrameSetSource* source_;
    ouster::sdk::osf::SequentialMessagesStreamingIter iter_, end_iter_;  // collation iters
    ResolveFrameFn resolve_frame_;
    uint32_t stream_id_;
    size_t i_ = 0;
    bool first_ = true;

   public:
    explicit OsfCollationIteratorImpl(OsfFrameSetSource* frame_set_source)
        : source_(frame_set_source) {
        stream_id_ = source_->reader_->meta_store().get<CollationStreamMeta>()->id();

        auto range = source_->reader_->messages_by_index(stream_id_, 0,
                                                         std::numeric_limits<uint32_t>::max());

        iter_ = range.begin();
        end_iter_ = range.end();

        // Unified, index-based resolver: given a unified sensor index and a
        // message index, look up that sensor's frame stream and decode a
        // LidarFrame or CameraFrame identically. Legacy lidar-only files stored
        // the per-type lidar index in sensor_id, which equals the unified index,
        // so they resolve through the same path.
        resolve_frame_ = [this](FrameUniqueId frame_uid) -> std::shared_ptr<core::Frame> {
            const uint32_t sensor_idx = frame_uid.first;
            if (sensor_idx >= source_->stream_ids_.size()) {
                return nullptr;
            }
            const uint32_t stream_id = source_->stream_ids_[sensor_idx];
            if (stream_id == INVALID_STREAM_ID) {
                return nullptr;
            }
            auto msg_range = source_->reader_->messages_by_index(
                stream_id, frame_uid.second, std::numeric_limits<uint32_t>::max());
            auto msg_it = msg_range.begin();
            if (msg_it == msg_range.end()) {
                throw std::out_of_range("Tried accessing frame beyond index range");
            }

            auto info_it = source_->info_by_stream_.find(stream_id);
            std::shared_ptr<core::SensorInfo> info =
                info_it != source_->info_by_stream_.end() ? info_it->second : nullptr;
            return decode_frame(info, *msg_it, source_->desired_fields_);
        };
    }

    bool advance(size_t offset) override {
        if (!source_->reader_) {
            throw std::runtime_error("Invalid operation on closed frame set source.");
        }

        if (first_) {
            first_ = false;
            offset--;

            if (iter_ == end_iter_) {
                return true;
            }
        }
        if (offset > 1) {
            // seek new range
            if (i_ + offset > (source_->collation_index_.size() - 1)) {
                throw std::out_of_range("Tried accessing collation beyond index range");
            }
            i_ += offset;
            auto range = source_->reader_->messages_by_index(stream_id_, i_,
                                                             std::numeric_limits<uint32_t>::max());
            iter_ = range.begin();
            end_iter_ = range.end();
        } else if (offset == 1) {
            // just advance
            if (++iter_ == end_iter_) {
                return true;  // we hit the end
            } else {
                i_++;
            }
        }
        return false;
    }

    int64_t length() override {
        return static_cast<int64_t>(source_->size());
    }

    FrameSet value() override {
        // collations are shallow copy, so this is fine
        return *iter_->decode_msg<CollationStream>(resolve_frame_);
    }
};

namespace {

void progressbar(double progress, double total, const char* prefix, const char* suffix) {
    if (total == 0) {
        throw std::runtime_error("Progress cannot be displayed for a total of 0 items.");
    }

    progress = progress > total ? total : progress;
    int percent = static_cast<int>(round(100.0 * progress / total));
    int filled_length = static_cast<int>(round(percent * 20.0 / 100.0));
    std::string bar;
    for (int i = 0; i < filled_length; i++) {
        bar += '#';
    }
    for (int i = 0; i < 20 - filled_length; i++) {
        bar += '-';
    }
    logger().info("{} {} {}%% {}\r", prefix, bar.c_str(), percent, suffix);
}

}  // namespace

OsfFrameSetSource::OsfFrameSetSource(const std::string& file,
                                     const std::function<void(OsfFrameSetSourceOptions&)>& options)
    : OsfFrameSetSource(file, ouster::sdk::impl::get_frame_set_source_options(options), true) {}

OsfFrameSetSource::OsfFrameSetSource(const std::string& file, OsfFrameSetSourceOptions options)
    : OsfFrameSetSource(file, std::move(options), true) {}

OsfFrameSetSource::OsfFrameSetSource(const std::string& file, OsfFrameSetSourceOptions options,
                                     bool read_collations)
    : OsfFrameSetSource(make_file(file), options, read_collations) {}

OsfFrameSetSource::OsfFrameSetSource(std::unique_ptr<OsfFile> osf_file,
                                     OsfFrameSetSourceOptions options, bool read_collations)
    : reader_{std::make_unique<Reader>(std::move(osf_file), options.error_handler.retrieve())},
      desired_fields_{options.field_names.retrieve()},
      indexed_{reader_->has_message_idx() && reader_->has_timestamp_idx()} {
    // index if not indexed
    bool want_indexed = options.index.retrieve();

    // Build the unified footer-id -> SensorInfo map from both sensor metadata
    // types. Lidars are stored as LidarSensor entries and cameras as CameraSensor
    // entries; both expose sensor_info() as the polymorphic base. Keying by
    // footer id in a single (ordered) map preserves registration order across
    // the two types, which is the canonical sensor/collation slot order.
    std::map<uint32_t, std::shared_ptr<ouster::sdk::core::SensorInfo>> sensors;
    for (const auto& item : reader_->meta_store().find<ouster::sdk::osf::LidarSensor>()) {
        sensors[item.first] = item.second->sensor_info();
    }
    for (const auto& item : reader_->meta_store().find<ouster::sdk::osf::CameraSensor>()) {
        sensors[item.first] = item.second->sensor_info();
    }

    // if not newer OSF and > 1 sensor we should be considered unindexed since
    // frames are stored in the incorrect order
    if (sensors.size() > 1 && reader_->version() < ouster::sdk::core::Version(2, 2, 0)) {
        auto handler = options.error_handler.retrieve();
        if (handler) {
            handler(Severity::OUSTER_WARNING,
                    "Legacy OSF contains out-of-order data. Playback order and "
                    "collations may be incorrect. Resave the file without "
                    "saving collations to fix.");
        }
        indexed_ = false;
    }

    if (!indexed_ && want_indexed) {
        // index in place
        logger().info("OSF file not indexed. Indexing in place...");

        // TODO: figure out how to get the current chunk_size
        std::string tmp_folder;
        if (!make_tmp_dir(tmp_folder)) {
            throw std::runtime_error("Couldn't obtain tmp file name");
        }

        // get filename
        std::string path = reader_->file().path();
        size_t last_slash = path.find_last_of("/\\");
        std::string tmp_filename =
            last_slash == std::string::npos ? path : path.substr(last_slash + 1);
        tmp_filename = tmp_folder + "/" + tmp_filename;
        logger().debug("temporary file name: {}\n", tmp_filename);
        int chunk_size = 0;
        logger().disable_auto_newline();
        try {
            logger().info("\n");
            progressbar(0, 1, "", "indexed");
            ouster::sdk::osf::Writer writer(tmp_filename, chunk_size);
            writer.set_metadata_id(reader_->metadata_id());
            for (const auto& item : reader_->meta_store().entries()) {
                if (dynamic_cast<ouster::sdk::osf::StreamingInfo*>(item.second.get()) != nullptr) {
                    // StreamingInfo is always generated by Writer automatically
                    // in default STREAMING chunks layout, so we don't copy the
                    // original
                    continue;
                }
                writer.add_metadata(*item.second);
            }
            // convert
            if (!reader_->has_stream_info()) {
                writer.close();
                throw std::runtime_error("Standard Message Layout No Longer Supported");
            }
            size_t msgs_count = 0;
            auto streaming_info = reader_->meta_store().get<ouster::sdk::osf::StreamingInfo>();
            if (streaming_info) {
                for (const auto& kv : streaming_info->stream_stats()) {
                    msgs_count += kv.second.message_count;
                }
            } else {
                logger().info("counting messages...");
                msgs_count = std::distance(reader_->messages().begin(), reader_->messages().end());
            }
            // Guard against empty files so progressbar never divides by zero;
            // the loop body won't execute in that case anyway.
            size_t progress_total = msgs_count > 0 ? msgs_count : 1;
            size_t idx = 0;
            for (const auto& message : reader_->messages()) {
                ouster::sdk::osf::ts_t sensor_ts;
                auto frame = message.decode_msg<ouster::sdk::osf::LidarFrameStream>();
                auto msg_ts = message.ts();
                if (frame) {
                    try {
                        sensor_ts = ouster::sdk::osf::ts_t(
                            frame->timestamp()[frame->get_last_valid_column()]);
                    } catch (const std::runtime_error& /*e*/) {
                        /**
                         * TODO: this is incorrect as we have real usecases now
                         * where e.g. only zone packets are available
                         *
                         * we need `get_first|last_valid_timestamp` method in
                         * lidar ts domain to do correct re-indexing
                         */
                        sensor_ts = ouster::sdk::osf::ts_t(0);
                    }

                    try {
                        msg_ts = ouster::sdk::osf::ts_t(frame->get_max_valid_receive_timestamp());
                    } catch (const std::runtime_error& /*e*/) {
                        // leave msg_ts where it is if frame has no valid
                        // packets
                    }
                }
                writer.save_message(message.id(), msg_ts, sensor_ts, message.buffer(), "");
                progressbar(static_cast<double>(idx++), static_cast<double>(progress_total), "",
                            "indexed");
            }
            logger().info("\n");
            logger().enable_auto_newline();
            logger().info("finished building index");
            writer.close();
        } catch (const std::exception&) {
            // attempt restoring the original pattern state
            logger().enable_auto_newline();
            throw;
        }

        reader_.reset();

        // now copy
        {
            std::ifstream src(tmp_filename, std::ios::binary);
            std::ofstream dst(path, std::ios::binary);

            dst << src.rdbuf();
        }

        // reopen
        reader_.reset();
        reader_ = std::make_unique<osf::Reader>(path, options.error_handler.retrieve());
        indexed_ = true;
    }

    // Retrieve FrameSetSourceMetadata
    auto frame_set_source_meta =
        reader_->meta_store().get<ouster::sdk::osf::FrameSetSourceMetadataStreamMeta>();
    std::unordered_map<std::string, FrameSetSourceMetadata> metadata_tmp;
    if (frame_set_source_meta) {
        auto id = frame_set_source_meta->id();
        for (const auto& message : reader_->messages({id})) {
            auto meta = message.decode_msg<ouster::sdk::osf::FrameSetSourceMetadataStream>();
            if (meta) {
                for (const auto& kv : meta->entries) {
                    // equivalent to insert_or_assign, but works in C++11
                    metadata_tmp.emplace(kv.first, kv.second).first->second = kv.second;
                }
            }
        }
    }
    for (const auto& kv : metadata_tmp) {
        add_metadata(kv.first, kv.second);
    }

    // Build the unified sensor list. Lidar and camera sensors are treated
    // identically once collected above: lidars come from LidarSensor footer
    // entries and cameras from CameraSensor footer entries, each referenced by a
    // frame stream via sensor_meta_id(). Footer entries are assigned ids in
    // registration order and are merged into a single id-ordered map, so
    // iterating them yields the canonical sensor (and collation slot) order.
    // Each sensor's index in this list is its unified sensor index.
    //
    // Map each footer id to the frame stream (lidar or camera) that references
    // it.
    std::map<uint32_t, uint32_t> footer_to_stream;
    for (const auto& item : reader_->meta_store().find<ouster::sdk::osf::LidarFrameStreamMeta>()) {
        footer_to_stream[item.second->sensor_meta_id()] = item.first;
    }
    for (const auto& item : reader_->meta_store().find<ouster::sdk::osf::CameraFrameStreamMeta>()) {
        footer_to_stream[item.second->sensor_meta_id()] = item.first;
    }

    // footer id -> unified sensor index, so Extrinsics (keyed by footer id) can
    // be applied below.
    std::map<uint32_t, size_t> footer_to_index;
    for (const auto& item : sensors) {
        const uint32_t footer_id = item.first;
        // Copy the SensorInfo (LidarInfo or CameraInfo) so later extrinsics
        // edits don't alias the footer metadata entry.
        std::shared_ptr<ouster::sdk::core::SensorInfo> info = item.second->clone();

        auto stream_it = footer_to_stream.find(footer_id);
        const uint32_t stream_id =
            stream_it != footer_to_stream.end() ? stream_it->second : INVALID_STREAM_ID;

        footer_to_index[footer_id] = all_info_.size();
        all_info_.push_back(info);
        stream_ids_.push_back(stream_id);
        if (stream_id != INVALID_STREAM_ID) {
            info_by_stream_[stream_id] = info;
        }
    }

    // load extrinsics (Extrinsics metadata reference their sensor by footer id).
    auto extrinsics = reader_->meta_store().find<ouster::sdk::osf::Extrinsics>();
    for (const auto& item : extrinsics) {
        auto result = footer_to_index.find(item.second->ref_meta_id());
        if (result != footer_to_index.end()) {
            all_info_[result->second]->sensor_to_body = item.second->extrinsics();
        }
    }

    // build frames index, indexed uniformly by unified sensor index.
    frame_count_ = 0;
    frames_num_.resize(all_info_.size(), 0);
    // stream id -> unified sensor index
    std::map<uint32_t, size_t> stream_to_index;
    for (size_t idx = 0; idx < stream_ids_.size(); ++idx) {
        if (stream_ids_[idx] != INVALID_STREAM_ID) {
            stream_to_index[stream_ids_[idx]] = idx;
        }
    }
    auto info = reader_->meta_store().find<ouster::sdk::osf::StreamingInfo>();
    for (const auto& item : info) {
        for (const auto& stats : item.second->stream_stats()) {
            auto sensor_it = stream_to_index.find(stats.first);
            if (sensor_it == stream_to_index.end()) {
                continue;  // not a lidar/camera frame stream (e.g. collation)
            }
            const size_t sensor_index = sensor_it->second;
            frame_count_ += stats.second.message_count;
            for (const auto& timestamp : stats.second.receive_timestamps) {
                index_.emplace_back(timestamp, static_cast<uint32_t>(sensor_index));
            }
            frames_num_[sensor_index] = stats.second.message_count;
        }
    }

    auto collation_meta = reader_->meta_store().get<CollationStreamMeta>();
    had_collations_ = static_cast<bool>(collation_meta);
    // if collations are available, count them up
    if (read_collations && collation_meta) {
        auto collation_stream_id = collation_meta->id();

        for (const auto& item : info) {
            for (const auto& stats : item.second->stream_stats()) {
                if (stats.first != collation_stream_id) {
                    continue;
                }
                for (const auto& timestamp : stats.second.receive_timestamps) {
                    collation_index_.push_back(timestamp);
                }
            }
        }

        std::sort(collation_index_.begin(), collation_index_.end());
    }

    // sort the index
    std::sort(index_.begin(), index_.end(),
              [](const std::pair<uint64_t, uint32_t>& a, const std::pair<uint64_t, uint32_t>& b) {
                  return a.first < b.first;
              });

    // build the full index
    real_index_.resize(all_info_.size());
    int i = 0;
    for (const auto& item : index_) {
        real_index_[item.second].emplace_back(item.first, i++);
    }

    // Load extrinsics for both lidar sensors and cameras (matched by serial
    // number from the file, or positionally from the extrinsics list). Use
    // all_info_ so a positional extrinsics[i] maps to the i-th sensor in
    // registration/collation order - i.e. the same order exposed by
    // sensor_info() and used for FrameSet frame slots. Building a separate
    // lidar-then-camera vector here would mis-apply positional extrinsics for
    // mixed files where a camera is registered before a lidar.
    populate_extrinsics(options.extrinsics_file.retrieve(), options.extrinsics.retrieve(),
                        all_info_);
    options.check("OsfFrameSetSource");
}

ouster::sdk::core::FrameSetIterator OsfFrameSetSource::begin() const {
    if (!reader_) {
        throw std::runtime_error("Invalid operation on closed frame set source.");
    }
    if (is_collated()) {
        return ouster::sdk::core::FrameSetIterator(
            this, new OsfCollationIteratorImpl(const_cast<OsfFrameSetSource*>(this)));
    } else {
        return ouster::sdk::core::FrameSetIterator(
            this, new OsfFrameSetIteratorImpl(const_cast<OsfFrameSetSource*>(this)));
    }
}

ouster::sdk::core::FrameSetIterator OsfFrameSetSource::begin(int sensor_index) const {
    // sensor_index addresses the unified sensor list (lidar and camera sensors
    // in registration order) exposed via sensor_info(), so cameras are
    // selectable too.
    if (sensor_index < 0 || static_cast<size_t>(sensor_index) >= stream_ids_.size()) {
        throw std::runtime_error("Invalid index");
    }
    if (!reader_) {
        throw std::runtime_error("Invalid operation on closed frame set source.");
    }
    // TODO: should we throw here if collated?
    return ouster::sdk::core::FrameSetIterator(
        this, new OsfFrameSetIteratorImpl(const_cast<OsfFrameSetSource*>(this), sensor_index));
}

void OsfFrameSetSource::close() {
    reader_.reset();
}

size_t OsfFrameSetSource::size() const {
    if (!indexed_) {
        throw std::runtime_error("not indexed");
    }
    if (is_collated()) {
        return collation_index_.size();
    } else {
        return frame_count_;
    }
}

size_t OsfFrameSetSource::size_hint() const {
    if (indexed_) {
        return size();
    }
    // todo maybe make a better guess if we are unindexed
    // however this is a _very_ rare case
    return 0;
}

bool OsfFrameSetSource::is_indexed() const {
    return indexed_;
}

const std::vector<size_t>& OsfFrameSetSource::frames_num() const {
    (void)REGISTERED;
    if (!indexed_) {
        throw std::runtime_error("not indexed");
    }
    return frames_num_;
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>& OsfFrameSetSource::individual_index()
    const {
    if (!indexed_) {
        throw std::runtime_error(
            "'individual_index' not supported on unindexed frame set sources.");
    }
    return real_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& OsfFrameSetSource::full_index() const {
    if (!indexed_) {
        throw std::runtime_error("'full_index' not supported on unindexed frame set sources.");
    }
    return index_;
}

const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& OsfFrameSetSource::sensor_info()
    const {
    return all_info_;
}

std::unique_ptr<ouster::sdk::core::FrameSetSource> OsfFrameSetSource::move() {
    return std::make_unique<OsfFrameSetSource>(std::move(*this));
}

bool OsfFrameSetSource::is_collated() const {
    return !collation_index_.empty();
}

bool OsfFrameSetSource::contains_collations() const {
    return had_collations_;
};

std::unique_ptr<core::FrameSetSource> OsfFrameSetSource::create(
    const std::vector<std::string>& sources, const FrameSetSourceOptions& options, bool collate,
    int sensor_idx) {
    if (sources.size() > 1) {
        throw std::invalid_argument("OsfFrameSetSource allows opening only one file at a time.");
    }

    // calls a private constructor here
    OsfFrameSetSource* osf_source = new OsfFrameSetSource(sources[0], options, collate);
    std::unique_ptr<core::FrameSetSource> source{osf_source};

    if (sensor_idx >= 0) {
        source = std::make_unique<Singler>(std::move(source), sensor_idx);
    } else if (collate && !osf_source->is_collated()) {
        // only need collator if source wasn't collated at write time
        source = std::make_unique<Collator>(std::move(source));
    }

    return source;
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
