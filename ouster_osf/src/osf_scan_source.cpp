/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/osf_scan_source.h"

#include "compat_ops.h"
#include "ouster/impl/logging.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/writer.h"

using ouster::sensor::logger;

namespace ouster {
namespace osf {

OsfScanSourceOptions::OsfScanSourceOptions(const ScanSourceOptions& o)
    : ScanSourceOptions(o) {}

OsfScanSourceOptions::OsfScanSourceOptions() {}

class OsfScanSource;
class OsfScanIteratorImpl : public ouster::core::ScanIteratorImpl {
    OsfScanSource* source_;
    ouster::osf::MessagesStreamingIter iter_, end_iter_;
    int i_ = 0;
    int64_t sensor_idx_;
    bool first_ = true;

    std::vector<std::shared_ptr<LidarScan>> value_;
    std::vector<uint32_t> stream_ids_;

   public:
    OsfScanIteratorImpl(OsfScanSource* ss, int sensor_idx = -1) {
        source_ = ss;
        if (sensor_idx >= 0) {
            std::vector<uint32_t> ids;
            ids.push_back(ss->sensor_ids_[sensor_idx]);
            stream_ids_ = ids;
            auto range = ss->reader_->messages(ids);
            iter_ = range.begin();
            end_iter_ = range.end();
            sensor_idx_ = sensor_idx;
        } else {
            stream_ids_ = ss->valid_ids_;
            auto range = ss->reader_->messages(ss->valid_ids_);
            iter_ = range.begin();
            end_iter_ = range.end();
            sensor_idx_ = -1;
        }
    }

    bool advance(size_t offset) override {
        if (!source_->reader_) {
            throw std::runtime_error(
                "Invalid operation on closed scan source.");
        }
        // we already are at the first scan, so dont advance the first time
        if (first_) {
            first_ = false;
            offset--;

            if (iter_ == end_iter_) {
                return true;
            }
        }
        if (offset > 1) {
            if (!source_->indexed_) {
                throw std::runtime_error(
                    "Not supported on non-indexed scan sources.");
            }
            // seek
            i_ += offset;
            if (sensor_idx_ >= 0) {
                // seek within a sensor
                auto start =
                    source_->reader_->ts_by_message_idx(stream_ids_[0], i_)
                        .value();
                auto range = source_->reader_->messages(
                    stream_ids_, start, source_->reader_->end_ts());
                iter_ = range.begin();
                end_iter_ = range.end();
            } else {
                // seek globally
                auto start = ouster::osf::ts_t(source_->index_[i_].first);
                auto range = source_->reader_->messages(
                    stream_ids_, start, source_->reader_->end_ts());
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
            return source_->scans_num()[sensor_idx_];
        }
        return source_->size();
    }

    std::vector<std::shared_ptr<LidarScan>>& value() override {
        // todo should cache or something
        auto scan = iter_->decode_msg<ouster::osf::LidarScanStream>(
            source_->desired_fields_);
        // map back to the sensor info using the id of the lidar scan stream
        scan->sensor_info =
            source_->sensor_info_[iter_->id()];  // this id is the id of the
                                                 // lidarscanstream

        value_ = {std::shared_ptr<LidarScan>(scan.release())};
        return value_;
    }
};

OsfScanSource::OsfScanSource(
    const std::string& n,
    const std::function<void(OsfScanSourceOptions&)>& options)
    : OsfScanSource(n, ouster::impl::get_scan_options(options)) {}

static void progressbar(double progress, double total, const char* prefix,
                        const char* suffix) {
    if (total == 0) {
        throw std::runtime_error(
            "Progress cannot be displayed for a total of 0 items.");
    }

    progress = progress > total ? total : progress;
    int percent = round(100 * progress / total);
    int filled_length = round(percent * 20 / 100);
    std::string bar;
    for (int i = 0; i < filled_length; i++) {
        bar += '#';
    }
    for (int i = 0; i < 20 - filled_length; i++) {
        bar += '-';
    }
    logger().info("{} {} {}%% {}\r", prefix, bar.c_str(), percent, suffix);
}

OsfScanSource::OsfScanSource(
    const std::string& file,  ///< [in] OSF file to open
    OsfScanSourceOptions
        options  ///< [in] common scan source options or null for default
) {
    reader_ = std::make_unique<ouster::osf::Reader>(
        file, options.error_handler.retrieve());

    desired_fields_ = options.field_names.retrieve();

    // index if not indexed
    indexed_ = reader_->has_message_idx() && reader_->has_timestamp_idx();
    bool want_indexed = options.index.retrieve();

    if (!indexed_ & want_indexed) {
        // index in place
        logger().info("OSF file not indexed. Indexing in place...");

        // TODO: figure out how to get the current chunk_size
        std::string tmp_folder;
        if (!make_tmp_dir(tmp_folder)) {
            throw std::runtime_error("Couldn't obtain tmp file name");
        }

        // get filename
        size_t lastSlash = file.find_last_of("/\\");
        std::string tmp_filename =
            lastSlash == std::string::npos ? file : file.substr(lastSlash + 1);
        tmp_filename = tmp_folder + "/" + tmp_filename;
        logger().debug("temporary file name: {}\n", tmp_filename);
        int chunk_size = 0;
        logger().disable_auto_newline();
        try {
            progressbar(0, 1, "", "indexed");
            ouster::osf::Writer writer(tmp_filename, chunk_size);
            writer.set_metadata_id(reader_->metadata_id());
            for (const auto& item : reader_->meta_store().entries()) {
                if (dynamic_cast<ouster::osf::StreamingInfo*>(
                        item.second.get())) {
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
                throw std::runtime_error(
                    "Standard Message Layout No Longer Supported");
            }
            size_t msgs_count = 0;
            for (const auto& msg : reader_->messages()) {
                msgs_count++;
            }
            size_t idx = 0;
            for (auto& msg : reader_->messages()) {
                ouster::osf::ts_t sensor_ts;
                auto scan = msg.decode_msg<ouster::osf::LidarScanStream>();
                if (scan) {
                    sensor_ts = ouster::osf::ts_t(
                        scan->get_first_valid_column_timestamp());
                }
                writer.save_message(msg.id(), msg.ts(), sensor_ts,
                                    msg.buffer());
                progressbar(idx++, msgs_count, "", "indexed");
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
        reader_ = std::make_unique<ouster::osf::Reader>(file);
        indexed_ = true;
    }

    // Retrieve sensor_infos
    // Maps LidarSensor id to LidarScanStream id
    std::map<int, int> match;

    auto streams =
        reader_->meta_store().find<ouster::osf::LidarScanStreamMeta>();
    for (const auto& item : streams) {
        // printf("Got stream %i with meta id %i\n", item.first,
        // item.second->sensor_meta_id());
        match[item.second->sensor_meta_id()] = item.first;
        valid_ids_.push_back(item.first);
    }

    auto sensors = reader_->meta_store().find<ouster::osf::LidarSensor>();
    for (const auto& item : sensors) {
        // we need to map the lidar scan stream back to the appropriate sensor
        // info
        // printf("Got sensor %i index %i\n", item.first, index++);
        // item.first contains the id of the relevant LidarScanStream
        auto info =
            std::make_shared<ouster::sensor::sensor_info>(item.second->info());
        sensor_info_[match[item.first]] = info;
        sensor_infos_.push_back(info);
        sensor_ids_[sensor_infos_.size() - 1] = match[item.first];
    }

    // load extrinsics
    auto extrinsics = reader_->meta_store().find<ouster::osf::Extrinsics>();
    for (const auto& item : extrinsics) {
        auto result = match.find(item.second->ref_meta_id());
        if (result != match.end()) {
            sensor_info_[result->second]->extrinsic = item.second->extrinsics();
        }
    }

    // build our index
    scan_count_ = 0;
    scans_num_.resize(sensor_infos_.size(), 0);
    auto info = reader_->meta_store().find<ouster::osf::StreamingInfo>();
    int info_index = 0;
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
                throw std::runtime_error("not able to find sensor\n");
            }

            // now get the info
            scan_count_ += stats.second.message_count;
            for (const auto& ts : stats.second.receive_timestamps) {
                index_.push_back({ts, sensor_index});
            }
            scans_num_[sensor_index] = stats.second.message_count;
        }
        info_index++;
    }

    // sort the index
    std::sort(index_.begin(), index_.end(),
              [](const std::pair<uint64_t, uint32_t>& a,
                 const std::pair<uint64_t, uint32_t>& b) {
                  return a.first < b.first;
              });

    // build the full index
    real_index_.resize(sensors.size());
    int i = 0;
    for (const auto& item : index_) {
        real_index_[item.second].emplace_back(item.first, i++);
    }

    // load extrinsics
    ouster::populate_extrinsics(options.extrinsics_file.retrieve(),
                                options.extrinsics.retrieve(), sensor_infos_);
    options.check("OsfScanSource");
}

ouster::core::ScanIterator OsfScanSource::begin() const {
    if (!reader_) {
        throw std::runtime_error("Invalid operation on closed scan source.");
    }
    return ouster::core::ScanIterator(
        this, new OsfScanIteratorImpl((OsfScanSource*)this));
}

ouster::core::ScanIterator OsfScanSource::begin(int sensor_index) const {
    if (sensor_index >= (int)sensor_ids_.size()) {
        throw std::runtime_error("Invalid index");
    }
    if (!reader_) {
        throw std::runtime_error("Invalid operation on closed scan source.");
    }
    return ouster::core::ScanIterator(
        this, new OsfScanIteratorImpl((OsfScanSource*)this, sensor_index));
}

void OsfScanSource::close() { reader_.reset(); }

size_t OsfScanSource::size() const {
    if (!indexed_) {
        throw std::runtime_error("not indexed");
    }
    return scan_count_;
}

bool OsfScanSource::is_indexed() const { return indexed_; }

const std::vector<size_t>& OsfScanSource::scans_num() const {
    (void)registered_;
    if (!indexed_) {
        throw std::runtime_error("not indexed");
    }
    return scans_num_;
}

const std::vector<std::vector<std::pair<uint64_t, uint64_t>>>&
OsfScanSource::individual_index() const {
    if (!indexed_) {
        std::runtime_error(
            "'individual_index' not supported on unindexed scan sources.");
    }
    return real_index_;
}

const std::vector<std::pair<uint64_t, uint64_t>>& OsfScanSource::full_index()
    const {
    if (!indexed_) {
        std::runtime_error(
            "'full_index' not supported on unindexed scan sources.");
    }
    return index_;
}

const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
OsfScanSource::sensor_info() const {
    return sensor_infos_;
}

ouster::core::ScanSource* OsfScanSource::move() {
    return new OsfScanSource(std::move(*this));
}

}  // namespace osf
}  // namespace ouster
