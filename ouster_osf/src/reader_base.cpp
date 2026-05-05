/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/reader_base.h"

#include <cstdint>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/impl/logging.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/file.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/sensor_info_stream.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace osf {

ReaderBase::ReaderBase(std::unique_ptr<OsfFile> osf_file,
                       const error_handler_t& error_handler)
    : file_(std::move(osf_file)), error_handler_{error_handler} {
    version_ = file_->version();

    if (version_.major > OsfFile::CURRENT_VERSION.major) {
        std::stringstream stream;
        stream << "The OSF file was created with schema version "
               << version_.simple_version_string()
               << " but this reader supports up to major version "
               << OsfFile::CURRENT_VERSION.major << ". "
               << "Major version differences may indicate breaking changes. "
               << "The file will not be read to prevent possible "
               << "misinterpretation or data corruption.";
        // fatal error, since a major revision indicates structural or other
        // changes that would prevent us from reading any data
        error_handler_(ouster::sdk::core::Severity::OUSTER_ERROR, stream.str());
    } else if (version_.major == OsfFile::CURRENT_VERSION.major &&
               version_.minor > OsfFile::CURRENT_VERSION.minor) {
        std::stringstream stream;
        stream << "The OSF file was created with schema version "
               << version_.simple_version_string()
               << ", but this reader only supports up to "
               << OsfFile::CURRENT_VERSION.major << "."
               << OsfFile::CURRENT_VERSION.minor << ". "
               << "Continuing to read using best-effort compatibility mode. "
               << "Some fields introduced in newer versions may be ignored or "
                  "unrecognized. "
               << "For full feature support, consider updating to the latest "
                  "version of the OusterSDK.";
        error_handler_(ouster::sdk::core::Severity::OUSTER_WARNING,
                       stream.str());
    }

    chunks_base_offset_ = file_->chunks_offset().offset();

    read_metadata();
}

bool ReaderBase::has_message_idx() const { return chunks_.has_message_idx(); };

bool ReaderBase::has_timestamp_idx() const {
    // just check metadata for any elements in the stats timestamp array
    for (auto& item : meta_store().find<StreamingInfo>()) {
        // return false if anything has the wrong number of timestamps for
        // number of messages
        for (auto& stats : item.second->stream_stats()) {
            if (stats.second.message_count > 0 &&
                stats.second.receive_timestamps.size() !=
                    stats.second.message_count) {
                return false;
            }
        }
        return true;
    }
    return false;
}

std::string ReaderBase::metadata_id() const {
    if (!file_->get_metadata_chunk().has_value()) {
        return std::string{};
    }
    if (auto metadata = impl::get_osf_metadata_from_buf(
            file_->get_metadata_chunk().data())) {
        if (metadata->id() != nullptr) {
            return metadata->id()->str();
        }
    }
    return std::string{};
}

ts_t ReaderBase::start_ts() const { return start_ts_; }

ts_t ReaderBase::end_ts() const { return end_ts_; }

const MetadataStore& ReaderBase::meta_store() const { return meta_store_; }

bool ReaderBase::has_stream_info() const { return has_streaming_info_; }

ouster::sdk::core::Version ReaderBase::version() const { return version_; }

void ReaderBase::read_metadata() {
    auto chunk_end = file_->chunks_offset().size();
    std::vector<ChunkState> chunk_states;
    if (!file_->get_metadata_chunk().has_value()) {
        error_handler_(Severity::OUSTER_WARNING,
                       "Osf: File has missing or invalid metadata. Some data "
                       "may be missing.");
        // need to read whole file to get this

        // lets start by reading each chunk and printing their types
        auto current_offset = file_->chunks_offset().offset();

        auto chunk_start = current_offset;

        chunk_end = file_->size() - chunk_start;

        osf::StreamingInfo info;

        uint64_t all_start_ts = std::numeric_limits<uint64_t>::max();
        uint64_t all_end_ts = 0;

        const int chunk_header_footer_size = 8;

        // first iterate over all chunks
        while (current_offset < file_->size()) {
            auto size_buf =
                file_->read(OsfOffset(current_offset, sizeof(uint32_t)));

            uint32_t size = *reinterpret_cast<const uint32_t*>(size_buf.data());

            // make sure we dont run past the end of the file
            if (current_offset + size > file_->size()) {
                error_handler_(Severity::OUSTER_WARNING,
                               "Osf: Incomplete chunk found at end of file. "
                               "Discarding chunk.");
                break;
            }

            // first read the size
            auto buf = file_->read(
                OsfOffset(current_offset, size + chunk_header_footer_size));

            auto chunk =
                ouster::sdk::osf::impl::gen::GetSizePrefixedChunk(buf.data());

            if (chunk->type() == nullptr) {
                throw std::runtime_error(
                    "Osf: File format too old to recover without metadata.");
            }

            // now recover metadata for each
            bool is_sensor_info =
                strcmp("ouster/v1/os_sensor/SensorInfoStreamMeta",
                       chunk->type()->c_str()) == 0;

            bool is_collation =
                strcmp("ouster/v1/os_sensor/CollationStreamMeta",
                       chunk->type()->c_str()) == 0;

            if (chunk->messages()->size() == 0) {
                throw std::runtime_error("Unexpected chunk with 0 messages.");
            }

            // to do this iterate over messages in the chunk
            uint64_t chunk_start_ts = std::numeric_limits<uint64_t>::max();
            uint64_t stream_id = 0;
            uint64_t chunk_end_ts = 0;
            auto messages = chunk->messages();
            for (size_t i = 0; i < messages->size(); i++) {
                auto msg = messages->Get(i);

                stream_id = msg->id();

                auto timestamp = msg->ts();
                chunk_end_ts = std::max(timestamp, chunk_end_ts);
                chunk_start_ts = std::min(timestamp, chunk_start_ts);

                // if its a sensor info chunk decode it and add it to the
                // metadata
                if (is_sensor_info) {
                    // printf("decoding sensor info\n");
                    std::vector<uint8_t> data;
                    data.resize(msg->buffer()->size());
                    memcpy(data.data(), msg->buffer()->data(), data.size());
                    auto msg = SensorInfoStream::from_buffer(data);
                    uint32_t lidar_sensor_id = msg->lidar_sensor_id;
                    LidarSensor meta3(msg->sensor_info);
                    meta3.set_id(lidar_sensor_id);
                    meta_store_.add(meta3);

                    // add the lidar scan stream meta
                    LidarScanStreamMeta meta(lidar_sensor_id);
                    meta.set_id(msg->scan_stream_id);
                    meta_store_.add(meta);

                    // add the meta for this stream if it doesnt already exist
                    if (!meta_store_.get(stream_id)) {
                        SensorInfoStreamMeta meta2;
                        meta2.set_id(stream_id);
                        meta_store_.add(meta2);
                    }
                }

                // update stats for this chunk
                auto& stats = info.stream_stats()[stream_id];
                stats.stream_id = stream_id;
                stats.message_count++;
                stats.receive_timestamps.push_back(timestamp);
                stats.sensor_timestamps.push_back(msg->ts_sensor());
            }

            // todo handle chunks with 0 messages

            // Add collation meta if this was a collation and was not yet added
            if (is_collation && !meta_store_.get(stream_id)) {
                CollationStreamMeta meta;
                meta.set_id(stream_id);
                meta_store_.add(meta);
            }

            // update whole file start and end timestamps
            all_end_ts = std::max(all_end_ts, chunk_end_ts);
            all_start_ts = std::min(all_start_ts, chunk_start_ts);

            // add it to the list of chunks
            chunk_states.emplace_back(current_offset - chunk_start,
                                      ts_t{chunk_start_ts}, ts_t{chunk_end_ts});

            ChunkInfo chunk_info{};
            chunk_info.offset = current_offset - chunk_start;
            chunk_info.stream_id = stream_id;
            chunk_info.message_count = chunk->messages()->size();
            info.chunks_info()[chunk_info.offset] = chunk_info;

            current_offset = current_offset + size + chunk_header_footer_size;
        }

        // update stats start and end timestamps
        for (auto& stat : info.stream_stats()) {
            if (!stat.second.receive_timestamps.empty()) {
                stat.second.start_ts = ts_t{stat.second.receive_timestamps[0]};
                stat.second.end_ts =
                    ts_t{stat.second.receive_timestamps.back()};
            }
        }

        start_ts_ = ts_t{all_start_ts};
        end_ts_ = ts_t{all_end_ts};

        meta_store_.add(info);
    } else {
        auto metadata = ouster::sdk::osf::impl::gen::GetSizePrefixedMetadata(
            file_->get_metadata_chunk().data());
        auto entries = metadata->entries();
        for (uint32_t i = 0; i < entries->size(); ++i) {
            auto entry = entries->Get(i);
            OsfBuffer entry_buf;
            uint64_t offset = reinterpret_cast<const uint8_t*>(entry) -
                              file_->get_metadata_chunk().data();
            entry_buf.load_data(file_->get_metadata_chunk(), offset,
                                file_->get_metadata_chunk().size() - offset);
            MetadataEntryRef meta_ref(entry_buf);
            // Option 1: Late reconstruction
            // meta_store_.add(meta_ref);

            // Option 2: Early reconstruction (with dynamic_pointer_cast later)

            auto meta_obj = meta_ref.as_type();
            if (meta_obj) {
                // Successfull reconstruction of the metadata here.
                meta_store_.add(*meta_obj);
            } else {
                // Can't reconstruct, adding the MetadataEntryRef proxy object
                // i.e. late reconstruction path
                meta_store_.add(meta_ref);
            }
        }

        if (auto metadata = impl::get_osf_metadata_from_buf(
                file_->get_metadata_chunk().data())) {
            start_ts_ = ts_t{metadata->start_ts()};
            end_ts_ = ts_t{metadata->end_ts()};
        }

        // TODO[cb&tws] - consider refactoring to construct ChunksPile from the
        // metadata directly, which would effectively encapsulate all the logic
        // below

        // Get chunks states
        if ((metadata->chunks() != nullptr) && metadata->chunks()->size() > 0) {
            for (uint32_t i = 0; i < metadata->chunks()->size(); ++i) {
                auto chunk_offset = metadata->chunks()->Get(i);
                chunk_states.emplace_back(chunk_offset->offset(),
                                          ts_t{chunk_offset->start_ts()},
                                          ts_t{chunk_offset->end_ts()});
            }
        }
    }
    chunks_ = ChunksPile(chunk_states, chunk_end);

    // Check that it has StreamingInfo and thus a valid StreamingLayout OSF
    // see RFC0018 for details
    auto streaming_info = meta_store_.get<osf::StreamingInfo>();
    if (!streaming_info) {
        has_streaming_info_ = false;
        return;
    }

    if (streaming_info->chunks_info().size() != chunks_.size()) {
        throw std::logic_error(
            "ERROR: StreamingInfo chunks info should equal chunks in the "
            "Reader");
    }

    for (const auto& sci : streaming_info->chunks_info()) {
        chunks_.add_info(sci.first, sci.second.stream_id,
                         sci.second.message_count);
    }

    has_streaming_info_ = true;

    chunks_.link_stream_chunks();
}

bool ReaderBase::verify_chunk(OsfOffset offset, OsfBuffer& buf) {
    if (!buf.has_value()) {
        return false;
    }
    auto chunk_state = chunks_.get(offset.offset());
    if (chunk_state == nullptr) {
        return false;
    }
    if (chunk_state->status == ChunkValidity::UNKNOWN) {
        try {
            chunk_state->status = osf::impl::check_osf_chunk_buf(buf)
                                      ? ChunkValidity::VALID
                                      : ChunkValidity::INVALID;
            if (chunk_state->status != ChunkValidity::VALID) {
                error_handler_(Severity::OUSTER_WARNING,
                               "Invalid chunk at file offset " +
                                   std::to_string(offset.offset()));
            }
        } catch (const std::runtime_error& e) {
            error_handler_(Severity::OUSTER_WARNING, e.what());
            chunk_state->status = ChunkValidity::INVALID;
        }
    }
    return (chunk_state->status == ChunkValidity::VALID);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
