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
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/core/frame_set_source.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/types.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata_stream.h"
#include "ouster/osf/reader.h"
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

class OsfFrameSetIteratorImpl : public ouster::sdk::core::FrameSetIteratorImpl {
    OsfFrameSetSource* source_;
    ouster::sdk::osf::MessagesStreamingIter iter_, end_iter_;
    size_t i_ = 0;
    int64_t sensor_idx_;
    bool first_ = true;

    std::vector<uint32_t> stream_ids_;

   public:
    explicit OsfFrameSetIteratorImpl(OsfFrameSetSource* source, int sensor_idx = -1)
        : source_{source}, sensor_idx_{sensor_idx} {
        if (sensor_idx >= 0) {
            std::vector<uint32_t> ids;
            ids.push_back(source_->sensor_ids_[sensor_idx]);
            stream_ids_ = ids;
            auto range = source_->reader_->messages(ids);
            iter_ = range.begin();
            end_iter_ = range.end();
        } else {
            stream_ids_ = source_->valid_ids_;
            if (stream_ids_.empty()) {
                return;
            }
            auto range = source_->reader_->messages(source_->valid_ids_);
            iter_ = range.begin();
            end_iter_ = range.end();
        }
    }

    bool advance(size_t offset) override {
        if (!source_->reader_) {
            throw std::runtime_error("Invalid operation on closed frame set source.");
        }
        // we already are at the first frame, so dont advance the first time
        if (first_) {
            first_ = false;
            offset--;

            if (iter_ == end_iter_) {
                return true;
            }
        }
        if (offset > 1) {
            if (!source_->indexed_) {
                throw std::runtime_error("Not supported on non-indexed frame set sources.");
            }
            // seek
            i_ += offset;

            if (sensor_idx_ >= 0) {
                // seek within a sensor
                // TODO: lookup should be index based as timestamps can overlap
                auto start_ts = source_->reader_->ts_by_message_idx(stream_ids_[0], i_);
                if (!start_ts) {
                    throw std::out_of_range("Tried accessing frame beyond index range");
                }
                auto range = source_->reader_->messages(stream_ids_, start_ts.value(),
                                                        source_->reader_->end_ts());
                iter_ = range.begin();
                end_iter_ = range.end();
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
            if (++iter_ == end_iter_) {
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

    FrameSet value() override {
        // todo should cache or something
        auto frame =
            iter_->decode_msg<ouster::sdk::osf::LidarFrameStream>(source_->desired_fields_);
        // map back to the sensor info using the id of the lidar frame stream
        if (frame) {
            frame->sensor_info = source_->sensor_info_.at(
                iter_->id());  // this id is the id of the lidar frame stream
            FrameSet out{};
            out.frames().emplace_back(frame.release());
            return out;
        }
        return FrameSet{};
    }
};

class OsfCollationIteratorImpl : public ouster::sdk::core::FrameSetIteratorImpl {
    OsfFrameSetSource* source_;
    ouster::sdk::osf::MessagesStreamingIter iter_,
        end_iter_;  // collation iters
    ResolveFrameFn resolve_frame_;
    uint32_t stream_id_;
    size_t i_ = 0;
    bool first_ = true;

   public:
    explicit OsfCollationIteratorImpl(OsfFrameSetSource* frame_set_source)
        : source_(frame_set_source) {
        stream_id_ = source_->reader_->meta_store().get<CollationStreamMeta>()->id();

        auto range = frame_set_source->reader_->messages({stream_id_});

        iter_ = range.begin();
        end_iter_ = range.end();

        resolve_frame_ = [this](FrameUniqueId frame_uid) -> std::shared_ptr<LidarFrame> {
            if (frame_uid == INVALID_FRAME_UID) {
                return nullptr;
            }

            auto lidar_stream_id = source_->valid_ids_[frame_uid.first];
            // TODO: lookup should be index based as timestamps can overlap
            auto start_ts = source_->reader_->ts_by_message_idx(lidar_stream_id, frame_uid.second);
            if (!start_ts) {
                throw std::out_of_range("Tried accessing frame beyond index range");
            }
            auto range = source_->reader_->messages({lidar_stream_id}, start_ts.value(),
                                                    source_->reader_->end_ts());
            auto msg_it = range.begin();

            std::unique_ptr<LidarFrame> frame =
                msg_it->decode_msg<LidarFrameStream>(source_->desired_fields_);
            if (frame) {
                frame->sensor_info = source_->sensor_info_[msg_it->id()];

                return std::shared_ptr<LidarFrame>(frame.release());
            }
            return nullptr;
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
            // TODO: lookup should be index based as timestamps can overlap
            if (i_ + offset > (source_->collation_index_.size() - 1)) {
                throw std::out_of_range("Tried accessing collation beyond index range");
            }
            i_ += offset;
            auto start_ts = ouster::sdk::osf::ts_t(source_->collation_index_[i_]);
            auto range =
                source_->reader_->messages({stream_id_}, start_ts, source_->reader_->end_ts());
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
    : reader_{std::make_unique<Reader>(file, options.error_handler.retrieve())},
      desired_fields_{options.field_names.retrieve()},
      indexed_{reader_->has_message_idx() && reader_->has_timestamp_idx()} {
    // index if not indexed
    bool want_indexed = options.index.retrieve();

    auto sensors = reader_->meta_store().find<ouster::sdk::osf::LidarSensor>();

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
        size_t last_slash = file.find_last_of("/\\");
        std::string tmp_filename =
            last_slash == std::string::npos ? file : file.substr(last_slash + 1);
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
                        msg_ts = ouster::sdk::osf::ts_t(frame->get_max_valid_packet_timestamp());
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
            std::ofstream dst(file, std::ios::binary);

            dst << src.rdbuf();
        }

        // reopen
        reader_.reset();
        reader_ = std::make_unique<osf::Reader>(file, options.error_handler.retrieve());
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

    // Retrieve SensorInfos
    // Maps LidarSensor id to LidarFrameStream id
    std::map<int, int> match;
    std::map<int, std::shared_ptr<SensorInfo>> sensor_info_by_id;

    auto streams = reader_->meta_store().find<ouster::sdk::osf::LidarFrameStreamMeta>();
    for (const auto& item : streams) {
        match[static_cast<int>(item.second->sensor_meta_id())] = static_cast<int>(item.first);
        valid_ids_.push_back(item.first);
    }

    for (const auto& item : sensors) {
        // we need to map the lidar frame stream back to the appropriate sensor
        // info
        // printf("Got sensor %i index %i\n", item.first, index++);
        // item.first contains the id of the relevant LidarFrameStream
        auto info = std::make_shared<ouster::sdk::core::SensorInfo>(item.second->info());
        sensor_info_by_id[static_cast<int>(item.first)] = info;
        sensor_info_[match[static_cast<int>(item.first)]] = info;
        sensor_infos_.push_back(info);
        sensor_ids_[static_cast<int>(sensor_infos_.size() - 1)] =
            match[static_cast<int>(item.first)];
    }

    // load extrinsics
    auto extrinsics = reader_->meta_store().find<ouster::sdk::osf::Extrinsics>();
    for (const auto& item : extrinsics) {
        auto result = match.find(static_cast<int>(item.second->ref_meta_id()));
        if (result != match.end()) {
            sensor_info_[result->second]->sensor_to_body = item.second->extrinsics();
        }
    }

    // build frames index
    frame_count_ = 0;
    frames_num_.resize(sensor_infos_.size(), 0);
    auto info = reader_->meta_store().find<ouster::sdk::osf::StreamingInfo>();
    for (const auto& item : info) {
        for (const auto& stats : item.second->stream_stats()) {
            if (sensor_info_.find(stats.first) == sensor_info_.end()) {
                continue;  // ignore non-lidar streams
            }

            int sensor_index = -1;
            for (const auto& id : sensor_ids_) {
                if (id.second == stats.first) {
                    sensor_index = id.first;
                    break;
                }
            }
            if (sensor_index < 0) {
                throw std::runtime_error("not able to find sensor");
            }

            // now get the info
            frame_count_ += stats.second.message_count;
            for (const auto& timestamp : stats.second.receive_timestamps) {
                index_.emplace_back(timestamp, sensor_index);
            }
            // TODO: is this right?
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
    real_index_.resize(sensors.size());
    int i = 0;
    for (const auto& item : index_) {
        real_index_[item.second].emplace_back(item.first, i++);
    }

    // load extrinsics
    populate_extrinsics(options.extrinsics_file.retrieve(), options.extrinsics.retrieve(),
                        sensor_infos_);
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
    if (sensor_index >= static_cast<int>(sensor_ids_.size())) {
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
    return sensor_infos_;
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
