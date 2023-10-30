/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/reader.h"

#include <memory>
#include <vector>

#include "fb_utils.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/file.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

namespace {

inline const ouster::osf::v2::Chunk* get_chunk_from_buf(const uint8_t* buf) {
    return ouster::osf::v2::GetSizePrefixedChunk(buf);
}

}  // namespace

// =======================================================
// =========== ChunksPile ================================
// =======================================================

void ChunksPile::add(uint64_t offset, ts_t start_ts, ts_t end_ts) {
    ChunkState cs{};
    cs.offset = offset;
    cs.next_offset = std::numeric_limits<uint64_t>::max();
    cs.start_ts = start_ts;
    cs.end_ts = end_ts;
    cs.status = ChunkValidity::UNKNOWN;
    pile_[offset] = cs;
}

void ChunksPile::add_info(uint64_t offset, uint32_t stream_id,
                          uint32_t message_count) {
    auto chunk_state = get(offset);
    if (chunk_state == nullptr) {
        // allowing adding info on chunks that already present with
        // a corresponding chunk states
        return;
    }
    ChunkInfoNode ci{};
    ci.offset = chunk_state->offset;
    ci.next_offset = std::numeric_limits<uint64_t>::max();
    ci.stream_id = stream_id;
    ci.message_count = message_count;
    pile_info_[offset] = ci;
}

ChunkState* ChunksPile::get(uint64_t offset) {
    auto cit = pile_.find(offset);
    if (cit == pile_.end()) {
        return nullptr;
    }
    return &cit->second;
}

ChunkInfoNode* ChunksPile::get_info(uint64_t offset) {
    auto cit = pile_info_.find(offset);
    if (cit == pile_info_.end()) {
        return nullptr;
    }
    return &cit->second;
}

ChunkInfoNode* ChunksPile::get_info_by_message_idx(uint32_t stream_id,
                                                   uint32_t message_idx) {
    if (!has_message_idx()) return nullptr;

    auto schunks = stream_chunks_.find(stream_id);
    if (schunks == stream_chunks_.end()) return nullptr;

    // out of bounds
    auto lci = get_info(schunks->second->back());
    if (message_idx >= lci->message_start_idx + lci->message_count)
        return nullptr;

    auto lb = std::lower_bound(
        schunks->second->begin(), schunks->second->end(), message_idx,
        [&](uint64_t a, uint32_t m_idx) {
            const auto* ci = get_info(a);
            return ci->message_start_idx + ci->message_count - 1 < m_idx;
        });

    return get_info(*lb);
}

ChunkState* ChunksPile::get_by_lower_bound_ts(uint32_t stream_id,
                                              const ts_t ts) {
    auto schunks = stream_chunks_.find(stream_id);
    if (schunks == stream_chunks_.end()) return nullptr;
    auto lb_offset = std::lower_bound(
        schunks->second->begin(), schunks->second->end(), ts,
        [&](uint64_t a, const ts_t t) { return get(a)->end_ts < t; });
    if (lb_offset == schunks->second->end()) return nullptr;
    return get(*lb_offset);
}

ChunkState* ChunksPile::next(uint64_t offset) {
    auto chunk = get(offset);
    if (!chunk) return nullptr;
    return get(chunk->next_offset);
}

ChunkState* ChunksPile::next_by_stream(uint64_t offset) {
    auto chunk_info = get_info(offset);
    if (!chunk_info) return nullptr;
    return get(chunk_info->next_offset);
}

ChunkState* ChunksPile::first() { return get(0); }

ChunksPile::ChunkStateIter ChunksPile::begin() { return pile_.begin(); }

ChunksPile::ChunkStateIter ChunksPile::end() { return pile_.end(); }

size_t ChunksPile::size() const { return pile_.size(); }

bool ChunksPile::has_info() const {
    return !pile_info_.empty() && pile_info_.size() == pile_.size();
}

bool ChunksPile::has_message_idx() const {
    // rely on the fact that message_count in the ChunkInfo, if present
    // during Writing/Chunk building, can't be 0 (by construction in
    // ChunkBuilder and StreamingLayoutCW)
    // In other words we can't have Chunks with 0 messages written to OSF
    // file
    return has_info() && pile_info_.begin()->second.message_count > 0;
}

void ChunksPile::link_stream_chunks() {
    // This function does a couple of things:
    // 1. Fills the stream_chunks_ map with offsets of chunks per stream id
    // 2, Links ChunkInfoNode from pile_info_ to the linked list by offsets
    //    so the traverse laterally along the same stream_id is easy
    // 3. Fills message_start_idx in ChunksInfoNode by counting previous chunks
    //    message_counts per stream
    // Thus the resulting lateral links between ChunkInfoNode allows traversing
    // between chunks per stream_id and quick search to the chunk by message_idx

    stream_chunks_.clear();

    if (has_info()) {
        // Do the next_offset links by streams
        auto curr_chunk = first();
        while (curr_chunk != nullptr) {
            auto ci = get_info(curr_chunk->offset);
            if (ci == nullptr) {
                throw std::logic_error("ERROR: Have a missing chunk info");
            }
            if (stream_chunks_.count(ci->stream_id)) {
                // verifying ts of prev and current chunks on non-decreasing
                // invariant.
                auto prev_chunk_offset = stream_chunks_[ci->stream_id]->back();
                auto prev_cs = get(prev_chunk_offset);
                if (prev_cs->end_ts > curr_chunk->start_ts) {
                    throw std::logic_error(
                        "ERROR: Can't have decreasing by timestamp chunks "
                        "order in StreamingLayout");
                }
                // get prev chunk info and update next_offset
                auto prev_ci = get_info(prev_chunk_offset);
                prev_ci->next_offset = curr_chunk->offset;
                ci->message_start_idx =
                    prev_ci->message_start_idx + prev_ci->message_count;
                stream_chunks_[ci->stream_id]->push_back(curr_chunk->offset);
            } else {
                stream_chunks_.insert(
                    {ci->stream_id, std::make_shared<std::vector<uint64_t>>(
                                        1, curr_chunk->offset)});
            }
            curr_chunk = get(curr_chunk->next_offset);
        }
    }
}

std::string to_string(const ChunkState& chunk_state) {
    std::stringstream ss;
    ss << "{offset = " << chunk_state.offset
       << ", next_offset = " << chunk_state.next_offset
       << ", start_ts = " << chunk_state.start_ts.count()
       << ", end_ts = " << chunk_state.end_ts.count()
       << ", status = " << (int)chunk_state.status << "}";
    return ss.str();
}

std::string to_string(const ChunkInfoNode& chunk_info) {
    std::stringstream ss;
    ss << "{offset = " << chunk_info.offset
       << ", next_offset = " << chunk_info.next_offset
       << ", stream_id = " << chunk_info.stream_id
       << ", message_count = " << chunk_info.message_count
       << ", message_start_idx = " << chunk_info.message_start_idx << "}";
    return ss.str();
}

// ==========================================================
// ========= Reader::ChunksIter =============================
// ==========================================================

ChunksIter::ChunksIter() : current_addr_(0), end_addr_(0), reader_(nullptr) {}

ChunksIter::ChunksIter(const ChunksIter& other)
    : current_addr_(other.current_addr_),
      end_addr_(other.end_addr_),
      reader_(other.reader_) {}

ChunksIter::ChunksIter(const uint64_t begin_addr, const uint64_t end_addr,
                       Reader* reader)
    : current_addr_(begin_addr), end_addr_(end_addr), reader_(reader) {
    if (current_addr_ != end_addr_ && !is_cleared()) next();
}

const ChunkRef ChunksIter::operator*() const {
    if (current_addr_ == end_addr_) {
        throw std::logic_error("ERROR: Can't dereference end iterator.");
    }
    return ChunkRef(current_addr_, reader_);
}

const std::unique_ptr<ChunkRef> ChunksIter::operator->() const {
    return std::make_unique<ChunkRef>(current_addr_, reader_);
}

ChunksIter& ChunksIter::operator++() {
    this->next();
    return *this;
}

ChunksIter ChunksIter::operator++(int) {
    auto res = *this;
    this->next();
    return res;
}

void ChunksIter::next() {
    if (current_addr_ == end_addr_) return;
    next_any();
    while (current_addr_ != end_addr_ && !is_cleared()) next_any();
}

void ChunksIter::next_any() {
    if (current_addr_ == end_addr_) return;
    auto next_chunk = reader_->chunks_.next(current_addr_);
    if (next_chunk) {
        current_addr_ = next_chunk->offset;
    } else {
        current_addr_ = end_addr_;
    }
}

bool ChunksIter::is_cleared() {
    if (current_addr_ == end_addr_) return false;
    return reader_->verify_chunk(current_addr_);
}

bool ChunksIter::operator==(const ChunksIter& other) const {
    return (current_addr_ == other.current_addr_ &&
            end_addr_ == other.end_addr_ && reader_ == other.reader_);
}

bool ChunksIter::operator!=(const ChunksIter& other) const {
    return !this->operator==(other);
}

std::string ChunksIter::to_string() const {
    std::stringstream ss;
    ss << "ChunksIter: [ca = " << current_addr_ << ", ea = " << end_addr_
       << "]";
    return ss.str();
}

// =======================================================
// ========= Reader::ChunksRange =========================
// =======================================================

ChunksRange::ChunksRange(const uint64_t begin_addr, const uint64_t end_addr,
                         Reader* reader)
    : begin_addr_(begin_addr), end_addr_(end_addr), reader_(reader) {}

ChunksIter ChunksRange::begin() const {
    return ChunksIter(begin_addr_, end_addr_, reader_);
}

ChunksIter ChunksRange::end() const {
    return ChunksIter(end_addr_, end_addr_, reader_);
}

std::string ChunksRange::to_string() const {
    std::stringstream ss;
    ss << "ChunksRange: [ba = " << begin_addr_ << ", ea = " << end_addr_ << "]";
    return ss.str();
}

// ==========================================================
// ========= Reader::MessagesStandardIter ===================
// ==========================================================

MessagesStandardIter::MessagesStandardIter()
    : current_chunk_it_{}, end_chunk_it_{}, msg_idx_{0} {}

MessagesStandardIter::MessagesStandardIter(const MessagesStandardIter& other)
    : current_chunk_it_(other.current_chunk_it_),
      end_chunk_it_(other.end_chunk_it_),
      msg_idx_(other.msg_idx_) {}

MessagesStandardIter::MessagesStandardIter(const ChunksIter begin_it,
                                           const ChunksIter end_it,
                                           const size_t msg_idx)
    : current_chunk_it_{begin_it}, end_chunk_it_{end_it}, msg_idx_{msg_idx} {
    if (current_chunk_it_ != end_chunk_it_ && !is_cleared()) next();
}

const MessageRef MessagesStandardIter::operator*() const {
    return current_chunk_it_->operator[](msg_idx_);
}

std::unique_ptr<const MessageRef> MessagesStandardIter::operator->() const {
    return current_chunk_it_->messages(msg_idx_);
}

MessagesStandardIter& MessagesStandardIter::operator++() {
    this->next();
    return *this;
}

MessagesStandardIter MessagesStandardIter::operator++(int) {
    auto res = *this;
    this->next();
    return res;
}

void MessagesStandardIter::next() {
    if (current_chunk_it_ == end_chunk_it_) return;
    next_any();
    while (current_chunk_it_ != end_chunk_it_ && !is_cleared()) next_any();
}

void MessagesStandardIter::next_any() {
    if (current_chunk_it_ == end_chunk_it_) return;
    auto chunk_ref = *current_chunk_it_;
    ++msg_idx_;
    if (msg_idx_ >= chunk_ref.size()) {
        // Advance to the next chunk
        ++current_chunk_it_;
        msg_idx_ = 0;
    }
}

bool MessagesStandardIter::operator==(const MessagesStandardIter& other) const {
    return (current_chunk_it_ == other.current_chunk_it_ &&
            end_chunk_it_ == other.end_chunk_it_ && msg_idx_ == other.msg_idx_);
}

bool MessagesStandardIter::operator!=(const MessagesStandardIter& other) const {
    return !this->operator==(other);
}

bool MessagesStandardIter::is_cleared() {
    if (current_chunk_it_ == end_chunk_it_) return false;
    const auto chunk_ref = *current_chunk_it_;
    if (!chunk_ref.valid()) return false;
    return (msg_idx_ < chunk_ref.size());
}

std::string MessagesStandardIter::to_string() const {
    std::stringstream ss;
    ss << "MessagesStandardIter: [curr_chunk_it = "
       << current_chunk_it_.to_string() << ", msg_idx = " << msg_idx_
       << ", end_chunk_it = " << end_chunk_it_.to_string() << "]";
    return ss.str();
}

// =========================================================
// ========= Reader::MessagesStandardRange =========================
// =========================================================

MessagesStandardRange::MessagesStandardRange(const ChunksIter begin_it,
                                             const ChunksIter end_it)
    : begin_chunk_it_(begin_it), end_chunk_it_(end_it) {}

MessagesStandardIter MessagesStandardRange::begin() const {
    return MessagesStandardIter(begin_chunk_it_, end_chunk_it_, 0);
}

MessagesStandardIter MessagesStandardRange::end() const {
    return MessagesStandardIter(end_chunk_it_, end_chunk_it_, 0);
}

std::string MessagesStandardRange::to_string() const {
    std::stringstream ss;
    ss << "MessagesStandardRange: [bit = " << begin_chunk_it_.to_string()
       << ", eit = " << end_chunk_it_.to_string() << "]";
    return ss.str();
}

// ==========================================================
// ========= Reader =========================================
// ==========================================================

MessagesStreamingRange Reader::messages() {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts(), end_ts(), {}, this);
}

MessagesStreamingRange Reader::messages(const ts_t start_ts,
                                        const ts_t end_ts) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts, end_ts, {}, this);
}

MessagesStreamingRange Reader::messages(
    const std::vector<uint32_t>& stream_ids) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts(), end_ts(), stream_ids, this);
}

MessagesStreamingRange Reader::messages(const std::vector<uint32_t>& stream_ids,
                                        const ts_t start_ts,
                                        const ts_t end_ts) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts, end_ts, stream_ids, this);
}

nonstd::optional<ts_t> Reader::ts_by_message_idx(uint32_t stream_id,
                                                 uint32_t message_idx) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    if (!chunks_.has_message_idx()) {
        return nonstd::nullopt;
    }
    // TODO: Check for message_count existence
    ChunkInfoNode* cin =
        chunks_.get_info_by_message_idx(stream_id, message_idx);
    if (!cin) return nonstd::nullopt;

    if (!verify_chunk(cin->offset)) {
        return nonstd::nullopt;
    }

    auto chunk_msg_index = message_idx - cin->message_start_idx;

    // shortcuting and not reading the chunk content if it's very first message
    // and we already checked validity
    if (chunk_msg_index == 0) {
        return {chunks_.get(cin->offset)->start_ts};
    }

    // reading chunk data to get message timestamp
    ChunkRef cref(cin->offset, this);
    if (chunk_msg_index < cref.size()) {
        return {cref.messages(chunk_msg_index)->ts()};
    }

    return nonstd::nullopt;
}

MessagesStandardRange Reader::messages_standard() {
    return MessagesStandardRange(chunks().begin(), chunks().end());
}

ChunksRange Reader::chunks() {
    return ChunksRange(0, file_.metadata_offset(), this);
}

Reader::Reader(const std::string& file) : file_{file} {
    if (!file_.valid()) {
        std::cerr << "ERROR: While openning OSF file. Expected valid() but "
                     "got file_ = "
                  << file_.to_string() << std::endl;
        throw std::logic_error("provided OSF file is not a valid OSF file.");
    }

    chunks_base_offset_ = file_.chunks_offset();

    read_metadata();

    read_chunks_info();
}

Reader::Reader(OsfFile& osf_file) : Reader(osf_file.filename()) {}

void Reader::read_metadata() {
    metadata_buf_.resize(FLATBUFFERS_PREFIX_LENGTH);

    file_.seek(file_.metadata_offset());
    file_.read(metadata_buf_.data(), FLATBUFFERS_PREFIX_LENGTH);
    size_t meta_size = get_prefixed_size(metadata_buf_.data());

    const size_t full_meta_size =
        meta_size + FLATBUFFERS_PREFIX_LENGTH + CRC_BYTES_SIZE;

    metadata_buf_.resize(full_meta_size);

    // no-op here
    file_.seek(file_.metadata_offset() + FLATBUFFERS_PREFIX_LENGTH);

    file_.read(metadata_buf_.data() + FLATBUFFERS_PREFIX_LENGTH,
               meta_size + CRC_BYTES_SIZE);

    if (!check_prefixed_size_block_crc(metadata_buf_.data(), full_meta_size)) {
        throw std::logic_error("ERROR: Invalid metadata block in OSF file.");
    }

    auto metadata =
        ouster::osf::gen::GetSizePrefixedMetadata(metadata_buf_.data());
    auto entries = metadata->entries();
    for (uint32_t i = 0; i < entries->size(); ++i) {
        auto entry = entries->Get(i);
        MetadataEntryRef meta_ref(reinterpret_cast<const uint8_t*>(entry));
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

    // Get chunks states
    std::vector<uint64_t> chunk_offsets{};
    if (metadata->chunks() && metadata->chunks()->size() > 0) {
        for (uint32_t i = 0; i < metadata->chunks()->size(); ++i) {
            auto co = metadata->chunks()->Get(i);
            chunks_.add(co->offset(), ts_t{co->start_ts()}, ts_t{co->end_ts()});
            chunk_offsets.push_back(co->offset());
        }
    }

    // Assign next_offsets links
    if (!chunk_offsets.empty()) {
        std::sort(chunk_offsets.begin(), chunk_offsets.end());
        for (size_t i = 0; i < chunk_offsets.size() - 1; ++i) {
            chunks_.get(chunk_offsets[i])->next_offset = chunk_offsets[i + 1];
        }
    }

    // NOTE: Left here for debugging
    // print_metadata_entries();
}

void Reader::read_chunks_info() {
    // Check that it has StreamingInfo and thus a valid StreamingLayout OSF
    // see RFC0018 for details
    auto streaming_info = meta_store_.get<osf::StreamingInfo>();
    if (!streaming_info) {
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

    chunks_.link_stream_chunks();
}

// TODO[pb]: MetadataStore to_string() ?
void Reader::print_metadata_entries() {
    std::cout << "Reader::print_metadata_entries:\n";
    int i = 0;
    for (const auto& me : meta_store_.entries()) {
        std::cout << "    entry[" << i++ << "] = " << me.second->to_string()
                  << std::endl;
    }
}

std::string Reader::id() const {
    if (auto metadata = get_osf_metadata_from_buf(metadata_buf_.data())) {
        if (metadata->id()) {
            return metadata->id()->str();
        }
    }
    return std::string{};
}

ts_t Reader::start_ts() const {
    if (auto metadata = get_osf_metadata_from_buf(metadata_buf_.data())) {
        return ts_t{metadata->start_ts()};
    }
    return ts_t{};
}

ts_t Reader::end_ts() const {
    if (auto metadata = get_osf_metadata_from_buf(metadata_buf_.data())) {
        return ts_t{metadata->end_ts()};
    }
    return ts_t{};
}

bool Reader::has_stream_info() const { return chunks_.has_info(); }

bool Reader::verify_chunk(uint64_t chunk_offset) {
    auto cs = chunks_.get(chunk_offset);
    if (!cs) return false;
    if (cs->status == ChunkValidity::UNKNOWN) {
        auto chunk_buf = file_.read_chunk(chunks_base_offset_ + chunk_offset);
        cs->status =
            osf::check_osf_chunk_buf(chunk_buf->data(), chunk_buf->size())
                ? ChunkValidity::VALID
                : ChunkValidity::INVALID;
    }
    return (cs->status == ChunkValidity::VALID);
}

// =========================================================
// ========= MessageRef ====================================
// =========================================================

uint32_t MessageRef::id() const {
    const ouster::osf::v2::StampedMessage* sm =
        reinterpret_cast<const ouster::osf::v2::StampedMessage*>(buf_);
    return sm->id();
}

MessageRef::ts_t MessageRef::ts() const {
    const ouster::osf::v2::StampedMessage* sm =
        reinterpret_cast<const ouster::osf::v2::StampedMessage*>(buf_);
    return ts_t(sm->ts());
}

bool MessageRef::is(const std::string& type_str) const {
    auto meta = meta_provider_.get(id());
    return (meta != nullptr) && (meta->type() == type_str);
}

bool MessageRef::operator==(const MessageRef& other) const {
    return (buf_ == other.buf_);
}

bool MessageRef::operator!=(const MessageRef& other) const {
    return !this->operator==(other);
}

std::string MessageRef::to_string() const {
    std::stringstream ss;
    const ouster::osf::v2::StampedMessage* sm =
        reinterpret_cast<const ouster::osf::v2::StampedMessage*>(buf_);
    ss << "MessageRef: [id = " << id() << ", ts = " << ts().count()
       << ", buffer = "
       << osf::to_string(sm->buffer()->Data(), sm->buffer()->size(), 100)
       << "]";
    return ss.str();
}

std::vector<uint8_t> MessageRef::buffer() const {
    const ouster::osf::gen::StampedMessage* sm =
        reinterpret_cast<const ouster::osf::gen::StampedMessage*>(buf_);

    if (sm->buffer() == nullptr) {
        return {};
    }

    return {sm->buffer()->data(), sm->buffer()->data() + sm->buffer()->size()};
}

// =======================================================
// =========== ChunkRef ==================================
// =======================================================

ChunkRef::ChunkRef()
    : chunk_offset_{std::numeric_limits<uint64_t>::max()},
      reader_{nullptr},
      chunk_buf_{nullptr} {}

ChunkRef::ChunkRef(const uint64_t offset, Reader* reader)
    : chunk_offset_(offset), reader_(reader) {
    if (!reader->file_.is_memory_mapped()) {
        // NOTE[pb]: We just rely on OS file operations caching thus not
        // trying to cache the read chunks in reader, however we might
        // reconsider it if we will discover the our caching might give
        // us better results (for now it's as is)
        chunk_buf_ = reader->file_.read_chunk(reader_->chunks_base_offset_ +
                                              chunk_offset_);
    }
    // Always expects "verified" chunk offset. See Reader::verify_chunk()
    assert(reader_->chunks_.get(chunk_offset_)->status !=
           ChunkValidity::UNKNOWN);
}

size_t ChunkRef::size() const {
    if (!valid()) return 0;
    const ouster::osf::v2::Chunk* chunk = get_chunk_from_buf(get_chunk_ptr());
    if (chunk->messages()) {
        return chunk->messages()->size();
    }
    return 0;
}

bool ChunkRef::valid() const {
    return (state()->status == ChunkValidity::VALID);
}

std::unique_ptr<const MessageRef> ChunkRef::messages(size_t msg_idx) const {
    if (!valid()) return nullptr;
    const ouster::osf::v2::Chunk* chunk = get_chunk_from_buf(get_chunk_ptr());
    if (!chunk->messages() || msg_idx >= chunk->messages()->size())
        return nullptr;
    const ouster::osf::v2::StampedMessage* m = chunk->messages()->Get(msg_idx);
    return std::make_unique<const MessageRef>(
        reinterpret_cast<const uint8_t*>(m), reader_->meta_store_, chunk_buf_);
}

const MessageRef ChunkRef::operator[](size_t msg_idx) const {
    const ouster::osf::v2::Chunk* chunk = get_chunk_from_buf(get_chunk_ptr());
    const ouster::osf::v2::StampedMessage* m = chunk->messages()->Get(msg_idx);
    return MessageRef(reinterpret_cast<const uint8_t*>(m), reader_->meta_store_,
                      chunk_buf_);
}

MessagesChunkIter ChunkRef::begin() const {
    return MessagesChunkIter(*this, 0);
}

MessagesChunkIter ChunkRef::end() const {
    return MessagesChunkIter(*this, size());
}

bool ChunkRef::operator==(const ChunkRef& other) const {
    return (chunk_offset_ == other.chunk_offset_ && reader_ == other.reader_);
}

bool ChunkRef::operator!=(const ChunkRef& other) const {
    return !this->operator==(other);
}

std::string ChunkRef::to_string() const {
    std::stringstream ss;
    auto chunk_state = state();
    ss << "ChunkRef: ["
       << "msgs_size = " << size() << ", state = ("
       << (chunk_state ? osf::to_string(*chunk_state) : "no state") << ")"
       << ", chunk_buf_ = "
       << (chunk_buf_ ? "size=" + std::to_string(chunk_buf_->size())
                      : "nullptr")
       << "]";
    return ss.str();
}

const uint8_t* ChunkRef::get_chunk_ptr() const {
    if (reader_->file_.is_memory_mapped()) {
        return reader_->file_.buf() + reader_->chunks_base_offset_ +
               chunk_offset_;
    }
    if (chunk_buf_ && !chunk_buf_->empty()) {
        return chunk_buf_->data();
    }

    return nullptr;
}

// ==========================================================
// ========= MessagesChunkIter ==============================
// ==========================================================

MessagesChunkIter::MessagesChunkIter() : chunk_ref_{}, msg_idx_{0} {}

MessagesChunkIter::MessagesChunkIter(const MessagesChunkIter& other)
    : chunk_ref_(other.chunk_ref_), msg_idx_(other.msg_idx_) {}

MessagesChunkIter::MessagesChunkIter(const ChunkRef chunk_ref,
                                     const size_t msg_idx)
    : chunk_ref_(chunk_ref), msg_idx_(msg_idx) {}

bool MessagesChunkIter::operator==(const MessagesChunkIter& other) const {
    return (chunk_ref_ == other.chunk_ref_ && msg_idx_ == other.msg_idx_);
}

bool MessagesChunkIter::operator!=(const MessagesChunkIter& other) const {
    return !this->operator==(other);
}

std::string MessagesChunkIter::to_string() const {
    std::stringstream ss;
    ss << "MessagesChunkIter: [chunk_ref = " << chunk_ref_.to_string()
       << ", msg_idx = " << msg_idx_ << "]";
    return ss.str();
}

const MessageRef MessagesChunkIter::operator*() const {
    return chunk_ref_[msg_idx_];
}

std::unique_ptr<const MessageRef> MessagesChunkIter::operator->() const {
    return chunk_ref_.messages(msg_idx_);
}

MessagesChunkIter& MessagesChunkIter::operator++() {
    this->next();
    return *this;
}

MessagesChunkIter MessagesChunkIter::operator++(int) {
    auto res = *this;
    this->next();
    return res;
}

MessagesChunkIter& MessagesChunkIter::operator--() {
    this->prev();
    return *this;
}

MessagesChunkIter MessagesChunkIter::operator--(int) {
    auto res = *this;
    this->prev();
    return res;
}

void MessagesChunkIter::next() {
    if (msg_idx_ < chunk_ref_.size()) ++msg_idx_;
}

void MessagesChunkIter::prev() {
    if (msg_idx_ > 0) --msg_idx_;
}

// ==========================================================
// ========= SreeamingReader::MessagesStreamingIter =========
// ==========================================================

// Simplest hash to better compare priority que states with stream ids
uint32_t calc_stream_ids_hash(const std::vector<uint32_t>& stream_ids) {
    uint32_t b = 378551;
    uint32_t a = 63689;
    uint32_t hash = 0;
    std::vector<uint32_t> tmp_stream_ids{stream_ids};
    std::sort(tmp_stream_ids.begin(), tmp_stream_ids.end());
    for (std::size_t i = 0; i < tmp_stream_ids.size(); ++i) {
        hash = hash * a + tmp_stream_ids[i];
        a *= b;
    }
    return hash;
}

MessagesStreamingIter::MessagesStreamingIter()
    : curr_ts_{},
      end_ts_{},
      stream_ids_{},
      stream_ids_hash_{},
      reader_{nullptr},
      curr_chunks_{} {}

MessagesStreamingIter::MessagesStreamingIter(const MessagesStreamingIter& other)
    : curr_ts_{other.curr_ts_},
      end_ts_{other.end_ts_},
      stream_ids_{other.stream_ids_},
      stream_ids_hash_{other.stream_ids_hash_},
      reader_{other.reader_},
      curr_chunks_{other.curr_chunks_} {}

MessagesStreamingIter::MessagesStreamingIter(
    const ts_t start_ts, const ts_t end_ts,
    const std::vector<uint32_t>& stream_ids, Reader* reader)
    : curr_ts_{start_ts},
      end_ts_{end_ts},
      stream_ids_{stream_ids},
      stream_ids_hash_{calc_stream_ids_hash(stream_ids_)},
      reader_{reader} {
    if (curr_ts_ == end_ts_) return;

    if (stream_ids_.empty()) {
        for (const auto& sm : reader_->chunks_.stream_chunks()) {
            stream_ids_.push_back(sm.first);
        }
    }

    // Per every stream_id open the first valid chunk in [start_ts, end_ts)
    // range. Steps:
    //  1. find first chunk by start_ts (lower bound)
    //  2. if chunk is valid open it, otherwise step 5
    //  3. find first message within chunk in [start_ts, end_ts) range
    //  4. addd opened chunk (offset, msg_idx) to the queue of streams we are
    //     reading
    //  5. move to the next chunk within stream, and continue from Step 2.
    for (const auto stream_id : stream_ids_) {
        // 1. find first chunk by start_ts (lower bound)
        auto* cs = reader_->chunks_.get_by_lower_bound_ts(stream_id, start_ts);
        bool filled = false;
        while (cs != nullptr && cs->start_ts < end_ts && !filled) {
            auto curr_offset = cs->offset;
            if (reader_->verify_chunk(curr_offset)) {
                // 2. if chunk is valid open it, otherwise step 5
                ChunkRef cref{curr_offset, reader_};
                for (size_t msg_idx = 0; msg_idx < cref.size(); ++msg_idx) {
                    // 3. find first message withing chunk in [start_ts, end_ts)
                    //    range
                    if (cref[msg_idx].ts() >= start_ts &&
                        cref[msg_idx].ts() < end_ts) {
                        // 4. addd opened chunk (offset, msg_idx) to the queue
                        //    of streams we are reading
                        curr_chunks_.emplace(cref, msg_idx);
                        filled = true;
                        break;
                    }
                }
            }
            // 5. move to the next chunk within stream, and continue from
            //    Step 2
            cs = reader_->chunks_.next_by_stream(curr_offset);
        }
    }

    if (!curr_chunks_.empty()) {
        const auto& curr_item = curr_chunks_.top();
        curr_ts_ = curr_item.first[curr_item.second].ts();
    } else {
        curr_ts_ = end_ts_;
    }
}

const MessageRef MessagesStreamingIter::operator*() const {
    const auto& curr_item = curr_chunks_.top();
    return curr_item.first[curr_item.second];
}

std::unique_ptr<const MessageRef> MessagesStreamingIter::operator->() const {
    const auto& curr_item = curr_chunks_.top();
    return curr_item.first.messages(curr_item.second);
}

// It's not a full equality because priority_queue requires either gnarly
// hacks to access internal collection or re-implementation.
// But for the purpose of checking the iterator position in a collection
// and checking boundaries it should be somewhat "correct"
//
// TODO[pb]: This should be revisited later with priority_queue
// re-implementation that will be easier to work with for our multi-streams
// case.
bool MessagesStreamingIter::operator==(
    const MessagesStreamingIter& other) const {
    return (curr_ts_ == other.curr_ts_ && end_ts_ == other.end_ts_ &&
            reader_ == other.reader_ &&
            stream_ids_hash_ == other.stream_ids_hash_ &&
            curr_chunks_.size() == other.curr_chunks_.size() &&
            (curr_chunks_.empty() ||
             curr_chunks_.top() == other.curr_chunks_.top()));
}

bool MessagesStreamingIter::operator!=(
    const MessagesStreamingIter& other) const {
    return !this->operator==(other);
}

MessagesStreamingIter& MessagesStreamingIter::operator++() {
    this->next();
    return *this;
}

MessagesStreamingIter MessagesStreamingIter::operator++(int) {
    auto res = *this;
    this->next();
    return res;
}

void MessagesStreamingIter::next() {
    if (curr_ts_ >= end_ts_) return;

    const auto curr_item = curr_chunks_.top();
    curr_chunks_.pop();

    const ChunkRef& cref = curr_item.first;
    size_t msg_idx = curr_item.second;

    if (msg_idx + 1 < cref.size()) {
        // Traversing current chunk
        ++msg_idx;
        if (cref[msg_idx].ts() < end_ts_) {
            curr_chunks_.emplace(cref, msg_idx);
        }
    } else {
        // Looking for the next chunk of the current stream_id
        // const auto curr_stream_id = cref[msg_idx].id();
        auto next_chunk_state =
            reader_->chunks_.next_by_stream(curr_item.first.offset());
        if (next_chunk_state) {
            auto next_chunk_info =
                reader_->chunks_.get_info(next_chunk_state->offset);
            if (next_chunk_info == nullptr) {
                throw std::logic_error(
                    "ERROR: Can't iterate by streams without StreamingInfo "
                    "available.");
            }
            if (next_chunk_state->start_ts < end_ts_) {
                if (reader_->verify_chunk(next_chunk_state->offset)) {
                    ChunkRef cref{next_chunk_state->offset, reader_};
                    for (size_t msg_idx = 0; msg_idx < cref.size(); ++msg_idx) {
                        if (cref[msg_idx].ts() < curr_ts_) {
                            throw std::logic_error(
                                "ERROR: Can't have decreasing by timestamp "
                                "messages in StreamingLayout");
                        }
                        if (cref[msg_idx].ts() < end_ts_) {
                            curr_chunks_.emplace(cref, msg_idx);
                            break;
                        }
                    }
                }
            }
        }
    }

    if (!curr_chunks_.empty()) {
        const auto& curr_item = curr_chunks_.top();
        const auto next_ts = curr_item.first[curr_item.second].ts();
        if (next_ts < curr_ts_) {
            throw std::logic_error(
                "ERROR: Can't have decreasing by timestamp messages "
                "in StreamingLayout");
        }
        curr_ts_ = next_ts;
    } else {
        curr_ts_ = end_ts_;
    }
}

/// NOTE: Debug function, will be removed after some time ...
void MessagesStreamingIter::print_and_finish() {
    while (!curr_chunks_.empty()) {
        auto& top = curr_chunks_.top();
        std::cout << "(( ts = " << top.first[top.second].ts().count()
                  << ", id = " << top.first[top.second].id()
                  << ", msg_idx = " << top.second
                  << ", cref = " << top.first.to_string() << std::endl;
        curr_chunks_.pop();
    }
}

std::string MessagesStreamingIter::to_string() const {
    std::stringstream ss;
    ss << "MessagesStreamingIter: [curr_ts = " << curr_ts_.count()
       << ", end_ts = " << end_ts_.count()
       << ", curr_chunks_.size = " << curr_chunks_.size()
       << ", stream_ids_hash_ = " << stream_ids_hash_;
    if (!curr_chunks_.empty()) {
        const auto& curr_item = curr_chunks_.top();
        ss << ", top = (ts = " << curr_item.first[curr_item.second].ts().count()
           << ", id = " << curr_item.first[curr_item.second].id() << ")";
    }
    ss << "]";
    return ss.str();
}

// =========================================================
// ========= StreamingReader::MessagesStreamingRange =======
// =========================================================

MessagesStreamingRange::MessagesStreamingRange(
    const ts_t start_ts, const ts_t end_ts,
    const std::vector<uint32_t>& stream_ids, Reader* reader)
    : start_ts_(start_ts),
      end_ts_(end_ts),
      stream_ids_{stream_ids},
      reader_{reader} {}

MessagesStreamingIter MessagesStreamingRange::begin() const {
    return MessagesStreamingIter(start_ts_, end_ts_ + ts_t{1}, stream_ids_,
                                 reader_);
}

MessagesStreamingIter MessagesStreamingRange::end() const {
    return MessagesStreamingIter(end_ts_ + ts_t{1}, end_ts_ + ts_t{1},
                                 stream_ids_, reader_);
}

std::string MessagesStreamingRange::to_string() const {
    std::stringstream ss;
    ss << "MessagesStreamingRange: [start_ts = " << start_ts_.count()
       << ", end_ts = " << end_ts_.count() << "]";
    return ss.str();
}

}  // namespace osf
}  // namespace ouster