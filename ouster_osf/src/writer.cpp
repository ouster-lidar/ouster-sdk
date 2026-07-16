/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/writer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/impl/logging.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/impl/sensor_info_stream.h"
#include "ouster/osf/layout_streaming.h"
#include "ouster/osf/metadata_stream.h"
#include "ouster/osf/stream_lidar_frame.h"
#include "ouster/osf/zpng_lidarframe_encoder.h"

using namespace ouster::sdk::core;
constexpr size_t MAX_CHUNK_SIZE = 500UL * 1024 * 1024;

namespace ouster {
namespace sdk {
namespace osf {

Writer::Writer(std::string file_name, uint32_t chunk_size)
    : impl_(std::make_unique<impl::WriterImpl>()),
      filename_(std::move(file_name)),
      metadata_id_{"ouster_sdk"},
      encoder_{std::make_shared<Encoder>(std::make_shared<ZPngLidarFrameEncoder>(
          ouster::sdk::osf::DEFAULT_ZPNG_OSF_COMPRESSION_LEVEL))} {
    // chunks STREAMING_LAYOUT
    chunks_writer_ = std::make_shared<StreamingLayoutCW>(*this, chunk_size);

    // or chunks STANDARD_LAYOUT (left for now to show the mechanisms of
    // switching layout strategies) chunks_writer_ =
    // std::make_shared<StandardLayoutCW>(*this, chunk_size);

    // TODO[pb]: Check if file exists, add flag overwrite/not overwrite, etc

    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
    header_size_ = impl::start_osf_file(filename_);

    if (header_size_ > 0) {
        pos_ = static_cast<int>(header_size_);
    } else {
        throw std::runtime_error("ERROR: Can't write to file :(");
    }
}

Writer::Writer(const std::string& filename, const SensorInfo& info,
               const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
               std::shared_ptr<Encoder> encoder)
    : Writer(filename, std::vector<SensorInfo>{info}, fields_to_write, chunk_size,
             std::move(encoder)) {}

Writer::Writer(const std::string& filename, const std::vector<SensorInfo>& info,
               const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
               std::shared_ptr<Encoder> encoder)
    : Writer(filename, chunk_size) {
    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
    sensor_info_ = info;
    for (uint32_t i = 0; i < info.size(); i++) {
        lidar_meta_id_[i] = add_metadata(ouster::sdk::osf::LidarSensor(info[i]));
        field_types_.emplace_back();
        desired_fields_.push_back(fields_to_write);
    }
    if (encoder) {
        // set encoder if one is specified
        encoder_ = std::move(encoder);
    }
}

const std::vector<SensorInfo>& Writer::sensor_info() const {
    return sensor_info_;
}

uint32_t Writer::add_sensor(const SensorInfo& info,
                            const std::vector<std::string>& fields_to_write) {
    lidar_meta_id_[lidar_meta_id_.size()] = add_metadata(ouster::sdk::osf::LidarSensor(info));
    field_types_.emplace_back();
    desired_fields_.push_back(fields_to_write);
    sensor_info_.push_back(info);
    return lidar_meta_id_.size() - 1;
}

void Writer::save_internal(uint32_t stream_index, const LidarFrame& frame, const ts_t time) {
    if (stream_index >= lidar_meta_id_.size()) {
        throw std::logic_error("ERROR: Bad Stream ID");
    }
    // this one isn't actually used anywhere later on, maybe clean up later
    ts_t timestamp;
    try {
        timestamp = ts_t(frame.timestamp()[frame.get_last_valid_column()]);
    } catch (const std::runtime_error& e) {
        // no valid columns case
        timestamp = ts_t(0);
    }

    auto item = lidar_streams_.find(stream_index);
    if (item == lidar_streams_.end()) {
        // build list of field types from provided or the first frame
        std::vector<ouster::sdk::core::FieldType> field_types;
        if (desired_fields_[stream_index].empty()) {
            field_types = frame.field_types();
        } else {
            for (const auto& desired : desired_fields_[stream_index]) {
                if (!frame.has_field(desired)) {
                    continue;
                }

                field_types.push_back(frame.field_type(desired));
            }
        }
        field_types_[stream_index] = field_types;

        lidar_streams_[stream_index] = std::make_unique<ouster::sdk::osf::LidarFrameStream>(
            *this, lidar_meta_id_[stream_index], field_types);

        // save meta
        if (!sensor_info_stream_) {
            sensor_info_stream_ = std::make_unique<ouster::sdk::osf::SensorInfoStream>(this);
        }

        ouster::sdk::osf::SensorInfoMessage msg;
        msg.sensor_info = sensor_info_[stream_index];
        msg.frame_stream_id = lidar_streams_[stream_index]->meta().id();
        msg.lidar_sensor_id = lidar_meta_id_[stream_index];
        if (info_ts_ == ts_t::min()) {
            info_ts_ = time;
        }
        sensor_info_stream_->save(msg, info_ts_);
        // todo dont actually need to flush for every single sensor just need to
        // flush after them all
        chunks_writer_->flush(sensor_info_stream_->meta().id());
    }

    // register any not yet registered field types so we can check if they
    // change later
    for (const auto& sft : frame.fields()) {
        bool found = false;
        for (const auto& dft : field_types_[stream_index]) {
            if (sft.first == dft.name) {
                found = true;
                break;
            }
        }
        if (!found) {
            auto field_type = frame.field_type(sft.first);
            bool is_desired = desired_fields_[stream_index].empty();
            for (const auto& desired : desired_fields_[stream_index]) {
                if (desired == field_type.name) {
                    is_desired = true;
                    break;
                }
            }
            // only allow desired and frame fields to be registered later
            if (is_desired && field_type.field_class == FieldClass::FRAME_FIELD) {
                field_types_[stream_index].push_back(field_type);
            } else if (is_desired) {
                // non-frame field was added after starting, thats an error
                throw std::invalid_argument(
                    "Field '" + field_type.name +
                    "' added after recording started. Only FrameFields can be "
                    "added/removed over time.");
            }
        }
    }

    // enforce that this frame meets our expected field types and that
    // dimensions didnt change when required to be the same
    for (const auto& field_type : field_types_[stream_index]) {
        const auto& field = frame.fields().find(field_type.name);
        // error if any non-frame field is missing
        if (field == frame.fields().end()) {
            if (field_type.field_class != FieldClass::FRAME_FIELD) {
                throw std::invalid_argument("Required field '" + field_type.name +
                                            "' is missing from frame.");
            } else {
                continue;
            }
        }

        if (field_type.element_type != field->second.tag()) {
            throw std::invalid_argument(
                "Field '" + field_type.name + "' has changed from '" +
                ouster::sdk::core::to_string(field_type.element_type) + "' to '" +
                ouster::sdk::core::to_string(field->second.tag()) +
                "'. Field types cannot change between saved frames from the "
                "same sensor.");
        }

        if (field_type.field_class != field->second.field_class()) {
            throw std::invalid_argument(
                "Field '" + field_type.name + "' has changed from '" +
                to_string(field_type.field_class) + "' to '" +
                to_string(field->second.field_class()) +
                "'. Field class cannot change between saved frames from the "
                "same sensor.");
        }

        // Dimensions should not change for pixel fields between frames
        if (field_type.field_class == FieldClass::PIXEL_FIELD) {
            if (field_type != frame.field_type(field_type.name)) {
                throw std::invalid_argument("Field '" + field_type.name +
                                            "' dimensions have changed. Pixel field dimensions "
                                            "cannot change for between saved frames from the same "
                                            "sensor.");
            }
        }
    }

    lidar_streams_[stream_index]->save(time, timestamp, frame, field_types_[stream_index]);
}

void Writer::save(uint32_t stream_index, const LidarFrame& frame) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }

    ts_t time;
    try {
        time = ts_t(frame.get_max_valid_packet_timestamp());
    } catch (const std::runtime_error& /*e*/) {
        time = ts_t(0);
    }
    save_internal(stream_index, frame, time);
}

void Writer::save(uint32_t stream_index, const LidarFrame& frame, const ts_t timestamp) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    save_internal(stream_index, frame, timestamp);
}

void Writer::save(const FrameSet& collation) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    if (collation.size() != lidar_meta_id_.size()) {
        throw std::logic_error(
            "ERROR: Frames passed in to writer "
            "does not match number of sensor infos");
    }
    if (!collation_stream_) {
        collation_stream_ = std::make_unique<CollationStream>(*this);
    }

    std::vector<FrameUniqueId> frame_uids;
    frame_uids.reserve(collation.size());

    std::vector<uint64_t> timestamps;
    timestamps.reserve(collation.size());

    uint64_t min_ts = std::numeric_limits<uint64_t>::max();

    // first pass, get data and checks sorted before saving
    // NOLINTNEXTLINE
    for (uint32_t i = 0; i < collation.size(); i++) {
        if (!collation[i]) {
            timestamps.push_back(0);
            continue;
        }

        uint64_t timestamp = 0;

        try {
            timestamp = collation[i]->get_max_valid_packet_timestamp();
        } catch (const std::runtime_error& /*e*/) {
        }

        if (timestamp == 0) {
            /**
             * Collation lookup relies on non-repeating timestamps.
             *
             * Technically ts==0 is a valid case, but realistically the only
             * case where timestamps would be repeating frame after frame is if
             * they were missing altogether.
             *
             * TODO: remove if/when message lookup logic no longer relies on ts
             */
            throw OsfDropFrameError(
                "Tried saving collation with frames having no valid "
                "timestamps");
        }

        min_ts = std::min(timestamp, min_ts);
        timestamps.push_back(timestamp);
    }

    for (uint32_t i = 0; i < collation.size(); i++) {
        if (!collation[i]) {
            frame_uids.push_back(INVALID_FRAME_UID);
            continue;
        }

        save_internal(i, *collation[i], ts_t{timestamps[i]});
        try {
            // get message index from stats
            StreamingLayoutCW* stream_cw = dynamic_cast<StreamingLayoutCW*>(chunks_writer_.get());
            const StreamStats& stats = stream_cw->get_stats(lidar_streams_[i]->meta().id());
            uint64_t msg_count = stats.message_count;
            frame_uids.emplace_back(i, msg_count - 1);
        } catch (const std::bad_cast&) {
            throw std::runtime_error(
                "Could not get streaming stats when saving collation. "
                "Are we using the streaming layout?");
        }
    }

    collation_stream_->save(ts_t{min_ts}, ts_t{}, collation, frame_uids);
}

uint32_t Writer::add_metadata(MetadataEntry&& entry) {
    return add_metadata(entry);
}

uint32_t Writer::add_metadata(MetadataEntry& entry) {
    return meta_store_.add(entry);
}

std::shared_ptr<MetadataEntry> Writer::get_metadata(const uint32_t metadata_id) const {
    return meta_store_.get(metadata_id);
}

uint64_t Writer::append(const uint8_t* buf, const uint64_t size) {
    if (pos_ < 0) {
        throw std::logic_error("ERROR: Writer is not ready (not started?)");
    }
    if (finished_) {
        throw std::logic_error("ERROR: Hmm, Writer is finished.");
    }
    if (size == 0) {
        logger().info("Writer::append has nothing to append");
        return 0;
    }
    uint64_t saved_bytes = impl::buffer_to_file(buf, size, filename_, true);
    pos_ += static_cast<int>(saved_bytes);
    return saved_bytes;
}

// > > > ===================== Chunk Emiter operations ======================

void Writer::save_message(const uint32_t stream_id, const ts_t receive_ts, const ts_t sensor_ts,
                          const std::vector<uint8_t>& msg_buf, const std::string& type) {
    if (!meta_store_.get(stream_id)) {
        std::stringstream string_stream;
        string_stream << "ERROR: Attempt to save the non existent stream: id = " << stream_id
                      << std::endl;

        throw std::logic_error(string_stream.str());

        return;
    }

    chunks_writer_->save_message(stream_id, receive_ts, sensor_ts, msg_buf, type);
}

const MetadataStore& Writer::meta_store() const {
    return meta_store_;
}

const std::string& Writer::metadata_id() const {
    return metadata_id_;
}

void Writer::set_metadata_id(const std::string& metadata_id) {
    metadata_id_ = metadata_id;
}

const std::string& Writer::filename() const {
    return filename_;
}

ChunksLayout Writer::chunks_layout() const {
    return impl_->chunks_layout;
}

uint64_t Writer::emit_chunk(const ts_t chunk_start_ts, const ts_t chunk_end_ts,
                            const std::vector<uint8_t>& chunk_buf) {
    uint64_t offset = pos_ - header_size_;
    uint64_t saved_bytes = append(chunk_buf.data(), chunk_buf.size());
    if ((saved_bytes != 0u) && saved_bytes == chunk_buf.size() + CRC_BYTES_SIZE) {
        impl_->chunks.emplace_back(chunk_start_ts.count(), chunk_end_ts.count(), offset);
        if (start_ts_ > chunk_start_ts) {
            start_ts_ = chunk_start_ts;
        }
        if (end_ts_ < chunk_end_ts) {
            end_ts_ = chunk_end_ts;
        }
        started_ = true;
        return offset;
    }
    std::stringstream string_stream;
    string_stream << "ERROR: Can't save to file. saved_bytes = " << saved_bytes << std::endl;
    throw std::logic_error(string_stream.str());
}

// < < < ================== Chunk Emiter operations ======================

std::vector<uint8_t> Writer::make_metadata() const {
    auto metadata_fbb = flatbuffers::FlatBufferBuilder(32768);

    std::vector<flatbuffers::Offset<ouster::sdk::osf::impl::gen::MetadataEntry>> entries =
        impl::make_entries(meta_store_, metadata_fbb);

    auto metadata = ouster::sdk::osf::impl::gen::CreateMetadataDirect(
        metadata_fbb, metadata_id_.c_str(), !impl_->chunks.empty() ? start_ts_.count() : 0,
        !impl_->chunks.empty() ? end_ts_.count() : 0, &impl_->chunks, &entries);

    metadata_fbb.FinishSizePrefixed(metadata, ouster::sdk::osf::impl::gen::MetadataIdentifier());

    const uint8_t* buf = metadata_fbb.GetBufferPointer();
    uint32_t size = metadata_fbb.GetSize();

    // Construct the std::vector<uint8_t> from the start/end pointers.
    return {buf, buf + size};
}

void Writer::close(bool fsync) {
    if (is_closed()) {
        return;
    }

    // Finish all chunks in flight
    chunks_writer_->finish();

    // Encode chunks, metadata entries and other fields into final buffer
    auto metadata_buf = make_metadata();

    uint64_t metadata_offset = pos_;
    uint64_t metadata_saved_size = append(metadata_buf.data(), metadata_buf.size());
    if ((metadata_saved_size != 0u) &&
        metadata_saved_size == metadata_buf.size() + CRC_BYTES_SIZE) {
        if (impl::finish_osf_file(filename_, metadata_offset, metadata_saved_size) ==
            header_size_) {
            finished_ = true;
        } else {
            logger().error(
                "ERROR: Can't finish OSF file!"
                "Recorded header of "
                "different sizes ...");
        }
    } else {
        logger().error(
            "ERROR: Oh, why we are here and "
            "didn't finish correctly?");
    }

    if (fsync) {
        if (!file_flush(filename_)) {
            logger().error("ERROR: Failed to flush file to disk.");
        }
    }
}

uint32_t Writer::chunk_size() const {
    return chunks_writer_->chunk_size();
}

Writer::~Writer() {
    close();
}

class ChunkBuilderImpl {
   public:
    /**
     * Internal FlatBufferBuilder object used for the serialization.
     */
    flatbuffers::FlatBufferBuilder fbb{0x7fff};

    /**
     * Internal store of messages to be contained within the chunk
     */
    std::vector<flatbuffers::Offset<impl::gen::StampedMessage>> messages{};
};

ChunkBuilder::ChunkBuilder() : impl_(std::make_shared<ChunkBuilderImpl>()) {}

ChunkBuilder::~ChunkBuilder() = default;

void ChunkBuilder::save_message(const uint32_t stream_id, const ts_t receive_ts,
                                const ts_t sensor_ts, const std::vector<uint8_t>& msg_buf,
                                const std::string& type) {
    if (finished_) {
        logger().error(
            "ERROR: ChunkBuilder is finished "
            "and can't accept new messages!");
        return;
    }

    if (impl_->fbb.GetSize() + msg_buf.size() > MAX_CHUNK_SIZE) {
        throw std::logic_error(
            "ERROR: reached max possible"
            " chunk size MAX_SIZE");
    }

    update_start_end(receive_ts);
    type_ = type;

    // wrap the buffer into StampedMessage
    auto stamped_msg = impl::gen::CreateStampedMessageDirect(
        impl_->fbb, receive_ts.count(), stream_id, &msg_buf, sensor_ts.count());
    impl_->messages.push_back(stamped_msg);
}

void ChunkBuilder::reset() {
    start_ts_ = ts_t::max();
    end_ts_ = ts_t::min();
    impl_->fbb.Clear();
    impl_->messages.clear();
    type_.clear();
    finished_ = false;
}

uint32_t ChunkBuilder::size() const {
    return impl_->fbb.GetSize();
}

uint32_t ChunkBuilder::messages_count() const {
    return static_cast<uint32_t>(impl_->messages.size());
}

ts_t ChunkBuilder::start_ts() const {
    return start_ts_;
}

ts_t ChunkBuilder::end_ts() const {
    return end_ts_;
}

void ChunkBuilder::update_start_end(const ts_t timestamp) {
    if (start_ts_ > timestamp) {
        start_ts_ = timestamp;
    }
    if (end_ts_ < timestamp) {
        end_ts_ = timestamp;
    }
}

std::vector<uint8_t> ChunkBuilder::finish() {
    if (impl_->messages.empty()) {
        finished_ = true;
        return {};
    }

    if (!finished_) {
        auto chunk = impl::gen::CreateChunkDirect(impl_->fbb, &impl_->messages,
                                                  !type_.empty() ? type_.c_str() : nullptr);
        impl_->fbb.FinishSizePrefixed(chunk, impl::gen::ChunkIdentifier());
        finished_ = true;
    }

    const uint8_t* buf = impl_->fbb.GetBufferPointer();
    uint32_t size = impl_->fbb.GetSize();

    return {buf, buf + size};
}

void Writer::save(const FrameSetSourceMetadataSet& frame_set_source_metadata_set) {
    // TODO[tws] always construct?
    if (!frame_set_source_metadata_stream_) {
        frame_set_source_metadata_stream_ =
            std::make_unique<ouster::sdk::osf::FrameSetSourceMetadataStream>(*this);
    }
    frame_set_source_metadata_stream_->save(frame_set_source_metadata_set, ts_t{});
    chunks_writer_->flush(frame_set_source_metadata_stream_->meta().id());
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
