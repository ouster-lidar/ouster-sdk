/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/writer.h"

#include <sstream>

#include "fb_utils.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/layout_streaming.h"
#include "ouster/osf/stream_lidar_scan.h"

constexpr size_t MAX_CHUNK_SIZE = 500 * 1024 * 1024;

namespace ouster {
namespace osf {

Writer::Writer(const std::string& filename, uint32_t chunk_size)
    : file_name_(filename),
      metadata_id_{"ouster_sdk"},
      chunks_layout_{ChunksLayout::LAYOUT_STREAMING} {
    // chunks STREAMING_LAYOUT
    chunks_writer_ = std::make_shared<StreamingLayoutCW>(*this, chunk_size);

    // or chunks STANDARD_LAYOUT (left for now to show the mechanisms of
    // switching layout strategies) chunks_writer_ =
    // std::make_shared<StandardLayoutCW>(*this, chunk_size);

    // TODO[pb]: Check if file exists, add flag overwrite/not overwrite, etc

    header_size_ = start_osf_file(file_name_);

    if (header_size_ > 0) {
        pos_ = static_cast<int>(header_size_);
    } else {
        throw std::runtime_error("ERROR: Can't write to file :(");
    }
}

Writer::Writer(const std::string& filename,
               const ouster::sensor::sensor_info& info,
               const LidarScanFieldTypes& field_types, uint32_t chunk_size)
    : Writer(filename, std::vector<ouster::sensor::sensor_info>{info},
             field_types, chunk_size) {}

Writer::Writer(const std::string& filename,
               const std::vector<ouster::sensor::sensor_info>& info,
               const LidarScanFieldTypes& field_types, uint32_t chunk_size)
    : Writer(filename, chunk_size) {
    sensor_info_ = info;
    for (uint32_t i = 0; i < info.size(); i++) {
        lidar_meta_id_[i] = add_metadata(ouster::osf::LidarSensor(info[i]));
        field_types_.push_back(field_types);
    }
}

const std::vector<ouster::sensor::sensor_info>& Writer::sensor_info() const {
    return sensor_info_;
}

const ouster::sensor::sensor_info Writer::sensor_info(int stream_index) const {
    return sensor_info_[stream_index];
}

uint32_t Writer::sensor_info_count() const { return sensor_info_.size(); }

uint32_t Writer::add_sensor(const ouster::sensor::sensor_info& info,
                            const LidarScanFieldTypes& field_types) {
    lidar_meta_id_[lidar_meta_id_.size()] =
        add_metadata(ouster::osf::LidarSensor(info));
    field_types_.push_back(field_types);
    sensor_info_.push_back(info);
    return lidar_meta_id_.size() - 1;
}

void Writer::_save(uint32_t stream_index, const LidarScan& scan,
                   const ts_t time) {
    if (stream_index < lidar_meta_id_.size()) {
        auto item = lidar_streams_.find(stream_index);
        if (item == lidar_streams_.end()) {
            const auto& fields = field_types_[stream_index].size()
                                     ? field_types_[stream_index]
                                     : get_field_types(scan);
            lidar_streams_[stream_index] =
                std::make_unique<ouster::osf::LidarScanStream>(
                    LidarScanStream::Token(), *this,
                    lidar_meta_id_[stream_index], fields);
        }
        lidar_streams_[stream_index]->save(time, scan);
    } else {
        throw std::logic_error("ERROR: Bad Stream ID");
    }
}

void Writer::save(uint32_t stream_index, const LidarScan& scan) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    ts_t time = ts_t(scan.get_first_valid_packet_timestamp());
    _save(stream_index, scan, time);
}

void Writer::save(uint32_t stream_index, const LidarScan& scan, const ts_t ts) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    _save(stream_index, scan, ts);
}

void Writer::save(const std::vector<LidarScan>& scans) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    if (scans.size() != lidar_meta_id_.size()) {
        throw std::logic_error(
            "ERROR: Scans passed in to writer "
            "does not match number of sensor infos");
    } else {
        for (uint32_t i = 0; i < scans.size(); i++) {
            ts_t time = ts_t(scans[i].get_first_valid_packet_timestamp());
            _save(i, scans[i], time);
        }
    }
}

uint32_t Writer::add_metadata(MetadataEntry&& entry) {
    return add_metadata(entry);
}

uint32_t Writer::add_metadata(MetadataEntry& entry) {
    return meta_store_.add(entry);
}

std::shared_ptr<MetadataEntry> Writer::get_metadata(
    const uint32_t metadata_id) const {
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
        std::cout << "nothing to append!!!\n";
        return 0;
    }
    uint64_t saved_bytes = buffer_to_file(buf, size, file_name_, true);
    pos_ += static_cast<int>(saved_bytes);
    return saved_bytes;
}

// > > > ===================== Chunk Emiter operations ======================

void Writer::save_message(const uint32_t stream_id, const ts_t ts,
                          const std::vector<uint8_t>& msg_buf) {
    if (!meta_store_.get(stream_id)) {
        std::stringstream ss;
        ss << "ERROR: Attempt to save the non existent stream: id = "
           << stream_id << std::endl;

        throw std::logic_error(ss.str());

        return;
    }

    chunks_writer_->save_message(stream_id, ts, msg_buf);
}

const MetadataStore& Writer::meta_store() const { return meta_store_; }

const std::string& Writer::metadata_id() const { return metadata_id_; }

void Writer::set_metadata_id(const std::string& id) { metadata_id_ = id; }

const std::string& Writer::filename() const { return file_name_; }

ChunksLayout Writer::chunks_layout() const { return chunks_layout_; }

uint64_t Writer::emit_chunk(const ts_t chunk_start_ts, const ts_t chunk_end_ts,
                            const std::vector<uint8_t>& chunk_buf) {
    uint64_t saved_bytes = append(chunk_buf.data(), chunk_buf.size());
    uint64_t res_chunk_offset{0};
    if (saved_bytes && saved_bytes == chunk_buf.size() + CRC_BYTES_SIZE) {
        chunks_.emplace_back(chunk_start_ts.count(), chunk_end_ts.count(),
                             next_chunk_offset_);
        res_chunk_offset = next_chunk_offset_;
        if (start_ts_ > chunk_start_ts) start_ts_ = chunk_start_ts;
        if (end_ts_ < chunk_end_ts) end_ts_ = chunk_end_ts;
        next_chunk_offset_ += saved_bytes;
        started_ = true;
    } else {
        std::stringstream ss;
        ss << "ERROR: Can't save to file. saved_bytes = " << saved_bytes
           << std::endl;
        throw std::logic_error(ss.str());
    }
    return res_chunk_offset;
}

// < < < ================== Chunk Emiter operations ======================

std::vector<uint8_t> Writer::make_metadata() const {
    auto metadata_fbb = flatbuffers::FlatBufferBuilder(32768);

    std::vector<flatbuffers::Offset<ouster::osf::gen::MetadataEntry>> entries =
        meta_store_.make_entries(metadata_fbb);

    auto metadata = ouster::osf::gen::CreateMetadataDirect(
        metadata_fbb, metadata_id_.c_str(),
        !chunks_.empty() ? start_ts_.count() : 0,
        !chunks_.empty() ? end_ts_.count() : 0, &chunks_, &entries);

    metadata_fbb.FinishSizePrefixed(metadata,
                                    ouster::osf::gen::MetadataIdentifier());

    const uint8_t* buf = metadata_fbb.GetBufferPointer();
    uint32_t size = metadata_fbb.GetSize();

    // Construct the std::vector<uint8_t> from the start/end pointers.
    return {buf, buf + size};
}

void Writer::close() {
    if (finished_) {
        // already did everything
        return;
    }

    // Finish all chunks in flight
    chunks_writer_->finish();

    // Encode chunks, metadata entries and other fields into final buffer
    auto metadata_buf = make_metadata();

    uint64_t metadata_offset = pos_;
    uint64_t metadata_saved_size =
        append(metadata_buf.data(), metadata_buf.size());
    if (metadata_saved_size &&
        metadata_saved_size == metadata_buf.size() + CRC_BYTES_SIZE) {
        if (finish_osf_file(file_name_, metadata_offset, metadata_saved_size) ==
            header_size_) {
            finished_ = true;
        } else {
            std::cerr << "ERROR: Can't finish OSF file!"
                         "Recorded header of "
                         "different sizes ..."
                      << std::endl;
        }
    } else {
        std::cerr << "ERROR: Oh, why we are here and "
                     "didn't finish correctly?"
                  << std::endl;
    }
}

uint32_t Writer::chunk_size() const { return chunks_writer_->chunk_size(); }

Writer::~Writer() { close(); }

// ================================================================

void ChunkBuilder::save_message(const uint32_t stream_id, const ts_t ts,
                                const std::vector<uint8_t>& msg_buf) {
    if (finished_) {
        std::cerr
            << "ERROR: ChunkBuilder is finished and can't accept new messages!"
            << std::endl;
        return;
    }

    if (fbb_.GetSize() + msg_buf.size() > MAX_CHUNK_SIZE) {
        throw std::logic_error(
            "ERROR: reached max possible"
            " chunk size MAX_SIZE");
    }

    update_start_end(ts);

    // wrap the buffer into StampedMessage
    auto stamped_msg =
        gen::CreateStampedMessageDirect(fbb_, ts.count(), stream_id, &msg_buf);
    messages_.push_back(stamped_msg);
}

void ChunkBuilder::reset() {
    start_ts_ = ts_t::max();
    end_ts_ = ts_t::min();
    fbb_.Clear();
    messages_.clear();
    finished_ = false;
}

uint32_t ChunkBuilder::size() const { return fbb_.GetSize(); }

uint32_t ChunkBuilder::messages_count() const {
    return static_cast<uint32_t>(messages_.size());
}

ts_t ChunkBuilder::start_ts() const { return start_ts_; }

ts_t ChunkBuilder::end_ts() const { return end_ts_; }

void ChunkBuilder::update_start_end(const ts_t ts) {
    if (start_ts_ > ts) start_ts_ = ts;
    if (end_ts_ < ts) end_ts_ = ts;
}

std::vector<uint8_t> ChunkBuilder::finish() {
    if (messages_.empty()) {
        finished_ = true;
        return {};
    }

    if (!finished_) {
        auto chunk = gen::CreateChunkDirect(fbb_, &messages_);
        fbb_.FinishSizePrefixed(chunk, gen::ChunkIdentifier());
        finished_ = true;
    }

    const uint8_t* buf = fbb_.GetBufferPointer();
    uint32_t size = fbb_.GetSize();

    return {buf, buf + size};
}

// ================================================================

}  // namespace osf
}  // namespace ouster
