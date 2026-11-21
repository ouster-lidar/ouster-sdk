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
#include <typeinfo>
#include <utility>
#include <vector>

#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/impl/sensor_info_stream.h"
#include "ouster/osf/layout_streaming.h"
#include "ouster/osf/meta_camera_sensor.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata_stream.h"
#include "ouster/osf/osf_encoder.h"
#include "ouster/osf/stream_camera_frame.h"
#include "ouster/osf/stream_lidar_frame.h"
#include "ouster/osf/zpng_frame_encoder.h"

// Auto-generated flatbuffer headers.
#include "metadata_generated.h"

using namespace ouster::sdk::core;
constexpr size_t MAX_CHUNK_SIZE = 500UL * 1024 * 1024;

namespace ouster {
namespace sdk {
namespace osf {

Writer::Writer(std::string file_name, uint32_t chunk_size)
    : impl_(std::make_unique<impl::WriterImpl>()),
      filename_(std::move(file_name)),
      metadata_id_{"ouster_sdk"},
      encoder_{std::make_shared<Encoder>(std::make_shared<ZPngFrameEncoder>(
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

Writer::Writer(const std::string& filename, const LidarInfo& info,
               const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
               std::shared_ptr<Encoder> encoder)
    : Writer(filename, std::vector<LidarInfo>{info}, fields_to_write, chunk_size,
             std::move(encoder)) {}

Writer::Writer(const std::string& filename, const std::vector<LidarInfo>& info,
               const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
               std::shared_ptr<Encoder> encoder)
    : Writer(filename, chunk_size) {
    for (uint32_t i = 0; i < info.size(); i++) {
        lidar_meta_id_[i] = add_metadata(ouster::sdk::osf::LidarSensor(info[i]));
        field_types_.emplace_back();
        desired_fields_.push_back(fields_to_write);
        stream_order_.push_back(StreamSlot{false, i});
    }
    if (encoder) {
        // set encoder if one is specified
        encoder_ = std::move(encoder);
    }
}

Writer::Writer(const std::string& filename, const std::vector<std::shared_ptr<SensorInfo>>& info,
               const std::vector<std::string>& fields_to_write, uint32_t chunk_size,
               std::shared_ptr<Encoder> encoder)
    : Writer(filename, chunk_size) {
    if (encoder) {
        // set encoder before registering sensors so any immediate writes use the
        // requested encoder.
        encoder_ = std::move(encoder);
    }
    for (const auto& si : info) {
        if (auto lidar = std::dynamic_pointer_cast<LidarInfo>(si)) {
            add_sensor(*lidar, fields_to_write);
        } else if (auto camera = std::dynamic_pointer_cast<CameraInfo>(si)) {
            add_sensor(*camera, fields_to_write);
        } else {
            throw std::runtime_error(
                "ERROR: Writer received a SensorInfo that is neither a "
                "LidarInfo nor a CameraInfo");
        }
    }
}

uint32_t Writer::add_sensor(const LidarInfo& info,
                            const std::vector<std::string>& fields_to_write) {
    lidar_meta_id_[lidar_meta_id_.size()] = add_metadata(ouster::sdk::osf::LidarSensor(info));
    field_types_.emplace_back();
    desired_fields_.push_back(fields_to_write);
    const auto lidar_index = static_cast<uint32_t>(lidar_meta_id_.size() - 1);
    stream_order_.push_back(StreamSlot{false, lidar_index});
    // Return the unified (registration-order) stream index, which is what
    // save() expects. For a lidar-only writer this equals the lidar index.
    return static_cast<uint32_t>(stream_order_.size() - 1);
}

uint32_t Writer::add_sensor(const ouster::sdk::core::CameraInfo& info,
                            const std::vector<std::string>& fields_to_write) {
    // Cameras store their CameraInfo in a dedicated CameraSensor footer entry
    // (a distinct OSF type from LidarSensor) and the CameraFrameStream references
    // it by id. Using a separate type means SDK versions that predate camera
    // support skip it as an unknown metadata entry instead of trying (and
    // failing) to validate it as a lidar, so they can still read the lidar
    // streams from a mixed file.
    auto sensor_meta_id = add_metadata(ouster::sdk::osf::CameraSensor(info));

    auto camera_stream_index = static_cast<uint32_t>(camera_meta_id_.size());
    // fields_to_write restricts which fields are written from each camera frame
    // (empty = all fields), mirroring the lidar add_sensor() overload.
    auto stream = std::make_unique<ouster::sdk::osf::CameraFrameStream>(*this, sensor_meta_id,
                                                                        fields_to_write);
    auto stream_meta_id = stream->meta().id();

    camera_meta_id_[camera_stream_index] = stream_meta_id;
    camera_streams_[camera_stream_index] = std::move(stream);
    stream_order_.push_back(StreamSlot{true, camera_stream_index});

    // Return the unified (registration-order) stream index, which is what
    // save() expects, so cameras are addressed consistently with lidars even
    // when the two are interleaved.
    return static_cast<uint32_t>(stream_order_.size() - 1);
}

void Writer::save_camera_internal(uint32_t camera_stream_index,
                                  const ouster::sdk::core::CameraFrame& frame, const ts_t time) {
    auto it = camera_streams_.find(camera_stream_index);
    if (it == camera_streams_.end()) {
        throw std::logic_error("ERROR: Bad camera stream index");
    }

    // add_sensor() already wrote this stream's CameraInfo to a CameraSensor
    // footer entry, but the footer only reaches disk on finalize. Duplicate it
    // into the data section on the first frame, exactly as the lidar path does,
    // so the metadata is recoverable from a file that was never finalized.
    if (camera_info_saved_.find(camera_stream_index) == camera_info_saved_.end()) {
        save_sensor_info_message(
            get_metadata<ouster::sdk::osf::CameraSensor>(it->second->meta().sensor_meta_id())
                ->sensor_info(),
            it->second->meta().sensor_meta_id(), it->second->meta().id(), time);
        camera_info_saved_.insert(camera_stream_index);
    }

    it->second->save(time, ts_t{frame.timestamp()[0]}, frame);
}

void Writer::save_sensor_info_message(std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info,
                                      uint32_t sensor_meta_id, uint32_t frame_stream_id,
                                      const ts_t time) {
    if (!sensor_info_stream_) {
        sensor_info_stream_ = std::make_unique<ouster::sdk::osf::SensorInfoStream>(this);
    }

    ouster::sdk::osf::SensorInfoMessage msg;
    msg.sensor_info = std::move(sensor_info);
    msg.sensor_meta_id = sensor_meta_id;
    msg.frame_stream_id = frame_stream_id;
    if (info_ts_ == ts_t::min()) {
        info_ts_ = time;
    }
    sensor_info_stream_->save(msg, info_ts_);
    // todo don't actually need to flush for every single sensor just need to
    // flush after them all
    chunks_writer_->flush(sensor_info_stream_->meta().id());
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
        // Read this lidar stream's LidarInfo back from the LidarSensor footer
        // entry that was written for it.
        save_sensor_info_message(
            get_metadata<ouster::sdk::osf::LidarSensor>(lidar_meta_id_[stream_index])
                ->sensor_info(),
            lidar_meta_id_[stream_index], lidar_streams_[stream_index]->meta().id(), time);
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
                // non-frame field was added after starting, that's an error
                throw std::invalid_argument(
                    "Field '" + field_type.name +
                    "' added after recording started. Only FrameFields can be "
                    "added/removed over time.");
            }
        }
    }

    // enforce that this frame meets our expected field types and that
    // dimensions didn't change when required to be the same
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

void Writer::save_frame_to_slot(uint32_t stream_index, const ouster::sdk::core::Frame& frame,
                                const ts_t time) {
    // stream_index is the unified (registration-order) index into stream_order_
    // - the same order as sensor_info() on read - so lidar and camera streams
    // are addressed identically. Translate it to the concrete per-type stream
    // and verify the frame kind matches the registered slot.
    if (stream_index >= stream_order_.size()) {
        throw std::logic_error("ERROR: Bad Stream ID");
    }
    const auto& slot = stream_order_[stream_index];

    if (const auto* lidar = dynamic_cast<const LidarFrame*>(&frame)) {
        if (slot.is_camera) {
            throw std::invalid_argument(
                "ERROR: LidarFrame saved to a stream index registered for a camera");
        }
        save_internal(slot.index, *lidar, time);
    } else if (const auto* camera = dynamic_cast<const CameraFrame*>(&frame)) {
        if (!slot.is_camera) {
            throw std::invalid_argument(
                "ERROR: CameraFrame saved to a stream index registered for a lidar");
        }
        save_camera_internal(slot.index, *camera, time);
    } else {
        throw std::invalid_argument("ERROR: Unsupported frame type passed to Writer::save");
    }
}

void Writer::save(uint32_t stream_index, const ouster::sdk::core::Frame& frame) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }

    // Derive the timestamp uniformly through the Frame interface so lidar and
    // camera frames are handled identically; only the stream write differs.
    ts_t time{0};
    try {
        time = ts_t(frame.get_max_valid_receive_timestamp());
    } catch (const std::runtime_error& /*e*/) {
        time = ts_t(0);
    }

    save_frame_to_slot(stream_index, frame, time);
}

void Writer::save(uint32_t stream_index, const ouster::sdk::core::Frame& frame,
                  const ts_t timestamp) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }

    save_frame_to_slot(stream_index, frame, timestamp);
}

// Collation validation and dispatch is inherently sequential; splitting
// obscures it.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Writer::save(const FrameSet& collation) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    // The collation's frame order must match the order sensors were registered
    // (via the constructor or add_sensor()); lidar and camera
    // streams may be interleaved. stream_order_ maps each collation position to
    // its registered stream.
    if (collation.size() != stream_order_.size()) {
        throw std::logic_error(
            "ERROR: Frames passed in to writer "
            "does not match number of registered lidar + camera streams");
    }
    if (!collation_stream_) {
        collation_stream_ = std::make_unique<CollationStream>(*this);
    }

    const auto& frames = collation.frames();

    // Per-slot message index, positionally aligned to the collation frames.
    // Slot k corresponds to the k-th registered sensor (its unified sensor
    // index), and this value is the message index within that sensor's frame
    // stream. INVALID_COLLATION_FRAME_INDEX marks a null slot. Written to the
    // file as index-based scan_ids{sensor_id=k, scan_idx=frame_indices[k]}.
    std::vector<uint64_t> frame_indices(stream_order_.size(), INVALID_COLLATION_FRAME_INDEX);

    // First pass: derive every frame's timestamp uniformly through the Frame
    // interface, independent of whether it originated from a lidar or a camera.
    uint64_t min_ts = std::numeric_limits<uint64_t>::max();
    std::vector<uint64_t> slot_timestamps(stream_order_.size(), 0);
    for (size_t slot = 0; slot < stream_order_.size(); ++slot) {
        const auto& frame = frames[slot];
        if (!frame) {
            continue;
        }

        uint64_t timestamp = 0;
        try {
            timestamp = frame->get_max_valid_receive_timestamp();
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

        slot_timestamps[slot] = timestamp;
        min_ts = std::min(timestamp, min_ts);
    }

    // Second pass: save each frame to its registered stream using its own
    // timestamp. Only the concrete stream write differs between lidar and
    // camera; everything else is type-independent.
    for (size_t slot = 0; slot < stream_order_.size(); ++slot) {
        const auto& stream = stream_order_[slot];
        const auto& frame = frames[slot];

        if (!stream.is_camera) {
            auto lidar = std::dynamic_pointer_cast<LidarFrame>(frame);
            if (!lidar) {
                if (frame) {
                    throw std::invalid_argument(
                        "ERROR: expected a LidarFrame at collation position " +
                        std::to_string(slot) + " for the registered lidar stream");
                }
                continue;  // null slot -> INVALID_COLLATION_FRAME_INDEX
            }
            save_internal(stream.index, *lidar, ts_t{slot_timestamps[slot]});
            try {
                StreamingLayoutCW* stream_cw =
                    dynamic_cast<StreamingLayoutCW*>(chunks_writer_.get());
                const StreamStats& stats =
                    stream_cw->get_stats(lidar_streams_[stream.index]->meta().id());
                frame_indices[slot] = stats.message_count - 1;
            } catch (const std::bad_cast&) {
                throw std::runtime_error(
                    "Could not get streaming stats when saving collation. "
                    "Are we using the streaming layout?");
            }
        } else {
            auto camera = std::dynamic_pointer_cast<CameraFrame>(frame);
            if (!camera) {
                if (frame) {
                    throw std::invalid_argument(
                        "ERROR: expected a CameraFrame at collation position " +
                        std::to_string(slot) + " for the registered camera stream");
                }
                continue;  // null slot -> INVALID_COLLATION_FRAME_INDEX
            }
            save_camera_internal(stream.index, *camera, ts_t{slot_timestamps[slot]});
            try {
                StreamingLayoutCW* stream_cw =
                    dynamic_cast<StreamingLayoutCW*>(chunks_writer_.get());
                const StreamStats& stats =
                    stream_cw->get_stats(camera_streams_[stream.index]->meta().id());
                frame_indices[slot] = stats.message_count - 1;
            } catch (const std::bad_cast&) {
                throw std::runtime_error(
                    "Could not get streaming stats when saving collation camera "
                    "frames. Are we using the streaming layout?");
            }
        }
    }

    // Build the legacy, lidar-only collation list for backward compatibility
    // with SDK versions that predate camera support. Such readers build their
    // sensor table from lidar frame streams only (in per-lidar index order) and
    // resolve collation entries against it, so a unified list that references a
    // camera slot would index past the end of that table and crash. Emitting a
    // lidar-only list lets them reconstruct a valid lidar FrameSet and silently
    // ignore the cameras.
    //
    // A lidar slot's stream.index is exactly its per-lidar index (lidars are
    // registered with sequential indices, and older readers order their lidar
    // streams by metadata id == registration order), so it doubles as the
    // legacy sensor_id. Camera slots are omitted entirely; null lidar slots use
    // the LEGACY_INVALID_SCAN_ID sentinel those readers understand.
    std::vector<FrameUniqueId> legacy_scan_ids;
    for (size_t slot = 0; slot < stream_order_.size(); ++slot) {
        const auto& stream = stream_order_[slot];
        if (stream.is_camera) {
            continue;
        }
        if (frame_indices[slot] == INVALID_COLLATION_FRAME_INDEX) {
            legacy_scan_ids.push_back(LEGACY_INVALID_SCAN_ID);
        } else {
            legacy_scan_ids.emplace_back(stream.index, frame_indices[slot]);
        }
    }

    // min_ts is always valid here: any non-null frame contributes a non-zero
    // timestamp (zero-timestamp frames trigger OsfDropFrameError above), so a
    // camera-only collation no longer records a bogus UINT64_MAX receive time.
    const ts_t collation_ts =
        (min_ts != std::numeric_limits<uint64_t>::max()) ? ts_t{min_ts} : ts_t{0};
    collation_stream_->save(collation_ts, ts_t{}, collation, frame_indices, legacy_scan_ids);
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
