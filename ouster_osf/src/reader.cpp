/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/reader.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <nonstd/optional.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/impl/logging.h"
#include "ouster/core/types.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/callback_osf_file.h"
#include "ouster/osf/crc32.h"
#include "ouster/osf/file.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/memory_mapped_osf_file.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/metadata.h"

// these includes must be last as it somehow pulls in weird
// things that break stuff in windows
#include <curl/curl.h>
#include <curl/easy.h>

#include "ouster/core/impl/curl_ca.h"

using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace osf {

namespace {

inline const ouster::sdk::osf::v2::Chunk* get_chunk_from_buf(const uint8_t* buf) {
    return ouster::sdk::osf::v2::GetSizePrefixedChunk(buf);
}

class Curl {
   public:
    Curl() {
        curl_global_init(CURL_GLOBAL_ALL);
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        curl_handle_ = curl_easy_init();
        curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, &Curl::write_memory_callback);
        curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, this);
        ouster::sdk::core::impl::configure_ca_bundle(curl_handle_);
    }

    Curl(Curl&&) = delete;
    Curl(Curl&) = delete;
    Curl& operator=(const Curl&) = delete;
    Curl& operator=(Curl&&) = delete;

    ~Curl() {
        curl_easy_cleanup(curl_handle_);
        curl_global_cleanup();
    }

    std::vector<uint8_t>& request(const std::string& url, size_t start, size_t length) const {
        curl_easy_setopt(curl_handle_, CURLOPT_URL, url.c_str());
        int timeout_seconds = 10;
        curl_easy_setopt(curl_handle_, CURLOPT_TIMEOUT, timeout_seconds);
        curl_easy_setopt(curl_handle_, CURLOPT_HEADER, 0);
        curl_easy_setopt(curl_handle_, CURLOPT_NOBODY, 0);
        curl_easy_setopt(curl_handle_, CURLOPT_HTTPHEADER, 0);
        curl_easy_setopt(curl_handle_, CURLOPT_FOLLOWLOCATION, 1L);

        std::string range = std::to_string(start) + "-" + std::to_string(start + length - 1);
        curl_easy_setopt(curl_handle_, CURLOPT_RANGE, range.c_str());

        // NOLINTNEXTLINE(google-runtime-int)
        long http_code = 0;
        buffer_.clear();
        buffer_.reserve(length);
        auto res = curl_easy_perform(curl_handle_);
        if (res == CURLE_SEND_ERROR) {
            // Specific versions of curl does't play well with the sensor
            // http server. When CURLE_SEND_ERROR happens for the first time
            // silently re-attempting the http request resolves the problem.
            res = curl_easy_perform(curl_handle_);
        }
        if (res != CURLE_OK) {
            throw std::runtime_error("CurlClient::execute_request failed for the url: [" + url +
                                     "] with the error message: " + curl_easy_strerror(res));
        }
        curl_easy_getinfo(curl_handle_, CURLINFO_RESPONSE_CODE, &http_code);
        if (200 <= http_code && http_code < 300) {
            // HTTP 2XX means a successful response
            return buffer_;
        }

        throw std::runtime_error(std::string("CurlClient::execute_request failed for url: [" + url +
                                             "] with the code: [" + std::to_string(http_code) +
                                             std::string("] - and return: ")));
    }

    size_t get_size(const std::string& url) {
        curl_easy_setopt(curl_handle_, CURLOPT_URL, url.c_str());
        int timeout_seconds = 10;
        curl_easy_setopt(curl_handle_, CURLOPT_TIMEOUT, timeout_seconds);
        curl_easy_setopt(curl_handle_, CURLOPT_HEADER, 1);
        curl_easy_setopt(curl_handle_, CURLOPT_NOBODY, 1);
        curl_easy_setopt(curl_handle_, CURLOPT_FOLLOWLOCATION, 1L);

        // NOLINTNEXTLINE(google-runtime-int)
        long http_code = 0;
        auto res = curl_easy_perform(curl_handle_);
        if (res == CURLE_SEND_ERROR) {
            // Specific versions of curl does't play well with the sensor
            // http server. When CURLE_SEND_ERROR happens for the first time
            // silently re-attempting the http request resolves the problem.
            res = curl_easy_perform(curl_handle_);
        }
        if (res != CURLE_OK) {
            throw std::runtime_error("CurlClient::execute_request failed for the url: [" + url +
                                     "] with the error message: " + curl_easy_strerror(res));
        }
        curl_easy_getinfo(curl_handle_, CURLINFO_RESPONSE_CODE, &http_code);
        if (200 <= http_code && http_code < 300) {
            // HTTP 2XX means a successful response
            curl_off_t result{};
            curl_easy_getinfo(curl_handle_, CURLINFO_CONTENT_LENGTH_DOWNLOAD_T, &result);
            return result;
        }

        throw std::runtime_error(std::string("CurlClient::execute_request failed for url: [" + url +
                                             "] with the code: [" + std::to_string(http_code) +
                                             std::string("] - and return: ")));
    }

    static size_t write_memory_callback(void* contents, size_t element_size, size_t elements_count,
                                        void* user_pointer) {
        size_t size_increment = element_size * elements_count;
        auto* client = static_cast<Curl*>(user_pointer);
        auto size_current = client->buffer_.size();
        client->buffer_.resize(size_current + size_increment);
        memcpy(&client->buffer_[size_current], contents, size_increment);
        return size_increment;
    }

   private:
    mutable CURL* curl_handle_;
    mutable std::vector<uint8_t> buffer_;
};

template <typename KeyT, typename ValueT>
class LRUCache {
   public:
    using key_value_pair_t = typename std::pair<KeyT, ValueT>;
    using list_iterator_t = typename std::list<key_value_pair_t>::iterator;

    explicit LRUCache(size_t max_size) : max_size_(max_size) {}

    void put(const KeyT& key, const ValueT& value) {
        auto it = cache_items_map_.find(key);
        cache_items_list_.push_front(key_value_pair_t(key, value));
        if (it != cache_items_map_.end()) {
            cache_items_list_.erase(it->second);
            cache_items_map_.erase(it);
        }
        cache_items_map_[key] = cache_items_list_.begin();

        if (cache_items_map_.size() > max_size_) {
            auto last = cache_items_list_.end();
            last--;
            cache_items_map_.erase(last->first);
            cache_items_list_.pop_back();
        }
    }

    const ValueT& get(const KeyT& key) {
        auto it = cache_items_map_.find(key);
        if (it == cache_items_map_.end()) {
            throw std::range_error("There is no such key in cache");
        } else {
            cache_items_list_.splice(cache_items_list_.begin(), cache_items_list_, it->second);
            return it->second->second;
        }
    }

    bool exists(const KeyT& key) const {
        return cache_items_map_.find(key) != cache_items_map_.end();
    }

    size_t size() const {
        return cache_items_map_.size();
    }

   private:
    std::list<key_value_pair_t> cache_items_list_;
    std::map<KeyT, list_iterator_t> cache_items_map_;
    size_t max_size_;
    size_t current_size_ = 0;
};

std::unique_ptr<OsfFile> make_file(const std::string& file) {
    if (file.find("://") != std::string::npos) {
        auto curl = std::make_shared<Curl>();
        LRUCache<std::pair<uint64_t, uint64_t>, OsfBuffer> cache(100);
        auto callback = [curl, file, cache](OsfOffset offset) mutable {
            try {
                return cache.get({offset.offset(), offset.size()});
            } catch (std::range_error& err) {
                auto buf = OsfBuffer();
                auto& data = curl->request(file, offset.offset(), offset.size());
                std::vector<uint8_t> moved;
                std::swap(moved, data);
                buf.load_data(std::move(moved));

                cache.put({offset.offset(), offset.size()}, buf);
                return buf;
            }
        };
        return std::make_unique<CallbackOsfFile>(callback, curl->get_size(file));
    }
    return std::make_unique<MemoryMappedOsfFile>(file);
}

}  // namespace

// ==========================================================
// ========= Reader::ChunksIter =============================
// ==========================================================

ChunksIter::ChunksIter(const ChunksIter& other)

    = default;

ChunksIter::ChunksIter(const uint64_t begin_addr, const uint64_t end_addr, Reader* reader)
    : current_addr_(begin_addr), end_addr_(end_addr), reader_(reader) {
    if (current_addr_ != end_addr_ && !is_cleared()) {
        next();
    }
}

ChunkRef ChunksIter::operator*() const {
    if (current_addr_ == end_addr_) {
        throw std::logic_error("ERROR: Can't dereference end iterator.");
    }
    return ChunkRef(current_addr_, reader_);
}

std::unique_ptr<ChunkRef> ChunksIter::operator->() const {
    return std::make_unique<ChunkRef>(current_addr_, reader_);
}

ChunksIter& ChunksIter::operator++() {
    this->next();
    return *this;
}

void ChunksIter::next() {
    if (current_addr_ == end_addr_) {
        return;
    }
    next_any();
    while (current_addr_ != end_addr_ && !is_cleared()) {
        next_any();
    }
}

void ChunksIter::next_any() {
    if (current_addr_ == end_addr_) {
        return;
    }
    auto next_chunk = reader_->chunks_.next(current_addr_);
    if (next_chunk != nullptr) {
        current_addr_ = next_chunk->offset;
    } else {
        current_addr_ = end_addr_;
    }
}

bool ChunksIter::is_cleared() {
    if (current_addr_ == end_addr_) {
        return false;
    }
    auto chunk_state = reader_->chunks_.get(current_addr_);
    if (chunk_state == nullptr) {
        return false;
    }
    OsfOffset offset(current_addr_, chunk_state->size);
    auto chunk_buffer = reader_->get_chunk(offset);
    return reader_->verify_chunk(offset, chunk_buffer);
}

bool ChunksIter::operator==(const ChunksIter& other) const {
    return (current_addr_ == other.current_addr_ && end_addr_ == other.end_addr_ &&
            reader_ == other.reader_);
}

bool ChunksIter::operator!=(const ChunksIter& other) const {
    return !this->operator==(other);
}

std::string ChunksIter::to_string() const {
    std::stringstream string_stream;
    string_stream << "ChunksIter: [ca = " << current_addr_ << ", ea = " << end_addr_ << "]";
    return string_stream.str();
}

bool ChunksIter::done() const {
    return current_addr_ == end_addr_;
}

// =======================================================
// ========= Reader::ChunksRange =========================
// =======================================================

ChunksRange::ChunksRange(const uint64_t begin_addr, const uint64_t end_addr, Reader* reader)
    : begin_addr_(begin_addr), end_addr_(end_addr), reader_(reader) {}

ChunksIter ChunksRange::begin() const {
    return ChunksIter(begin_addr_, end_addr_, reader_);
}

ChunksIter ChunksRange::end() const {
    return ChunksIter(end_addr_, end_addr_, reader_);
}

std::string ChunksRange::to_string() const {
    std::stringstream string_stream;
    string_stream << "ChunksRange: [ba = " << begin_addr_ << ", ea = " << end_addr_ << "]";
    return string_stream.str();
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

MessagesStreamingRange Reader::messages(const ts_t start_ts, const ts_t end_ts) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts, end_ts, {}, this);
}

MessagesStreamingRange Reader::messages(const std::vector<uint32_t>& stream_ids) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts(), end_ts(), stream_ids, this);
}

MessagesStreamingRange Reader::messages(const std::vector<uint32_t>& stream_ids,
                                        const ts_t start_ts, const ts_t end_ts) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    return MessagesStreamingRange(start_ts, end_ts, stream_ids, this);
}

nonstd::optional<ts_t> Reader::ts_by_message_idx(uint32_t stream_id, uint32_t message_idx) {
    if (!has_stream_info()) {
        throw std::logic_error(
            "ERROR: Can't iterate by streams without StreamingInfo "
            "available.");
    }
    if (!chunks_.has_message_idx()) {
        return nonstd::nullopt;
    }
    // TODO: Check for message_count existence
    ChunkInfoNode* cin = chunks_.get_info_by_message_idx(stream_id, message_idx);
    if (cin == nullptr) {
        return nonstd::nullopt;
    }

    OsfOffset temp_offset = {cin->offset, chunks_.get(cin->offset)->size};
    OsfBuffer buf = get_chunk(temp_offset);

    if (!verify_chunk(temp_offset, buf)) {
        return nonstd::nullopt;
    }

    auto chunk_msg_index = message_idx - cin->message_start_idx;

    // shortcuting and not reading the chunk content if it's very first message
    // and we already checked validity
    if (chunk_msg_index == 0) {
        return {chunks_.get(cin->offset)->start_ts};
    }

    // reading chunk data to get message timestamp
    ChunkRef cref(cin->offset, buf, this);
    if (chunk_msg_index < cref.size()) {
        return {cref.messages(chunk_msg_index)->ts()};
    }

    return nonstd::nullopt;
}

ChunksRange Reader::chunks() {
    return ChunksRange(0, file_->metadata_offset().offset(), this);
}

Reader::Reader(const std::string& file, const error_handler_t& error_handler)
    : ReaderBase(make_file(file), error_handler) {}

Reader::Reader(std::unique_ptr<OsfFile> osf_file, const error_handler_t& error_handler)
    : ReaderBase(std::move(osf_file), error_handler) {}

OsfBuffer Reader::get_chunk(OsfOffset offset) {
    return file_->read(file_->chunks_offset(), offset);
}

// =========================================================
// ========= MessageRef ====================================
// =========================================================
MessageRef::MessageRef(const OsfBuffer& buf, const MetadataStore& meta_provider,
                       error_handler_t error_handler)
    : buf_(buf), meta_provider_(meta_provider), error_handler_{std::move(error_handler)} {}

uint32_t MessageRef::id() const {
    const ouster::sdk::osf::v2::StampedMessage* stamped_message =
        reinterpret_cast<const ouster::sdk::osf::v2::StampedMessage*>(buf_.data());
    return stamped_message->id();
}

MessageRef::ts_t MessageRef::ts() const {
    const ouster::sdk::osf::v2::StampedMessage* stamped_message =
        reinterpret_cast<const ouster::sdk::osf::v2::StampedMessage*>(buf_.data());
    return ts_t(stamped_message->ts());
}

OsfBuffer MessageRef::buf() const {
    return buf_;
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
    std::stringstream string_stream;
    const ouster::sdk::osf::v2::StampedMessage* stamped_message =
        reinterpret_cast<const ouster::sdk::osf::v2::StampedMessage*>(buf_.data());
    string_stream << "MessageRef: [id = " << id() << ", ts = " << ts().count() << ", buffer = "
                  << osf::to_string(stamped_message->buffer()->Data(),
                                    stamped_message->buffer()->size(), 100)
                  << "]";
    return string_stream.str();
}

std::vector<uint8_t> MessageRef::buffer() const {
    const ouster::sdk::osf::impl::gen::StampedMessage* stamped_message =
        reinterpret_cast<const ouster::sdk::osf::impl::gen::StampedMessage*>(buf_.data());

    if (stamped_message->buffer() == nullptr) {
        return {};
    }

    // FIXME[tws] a copy
    return {stamped_message->buffer()->data(),
            stamped_message->buffer()->data() + stamped_message->buffer()->size()};
}

const error_handler_t& MessageRef::error_handler() const {
    return error_handler_;
}

// =======================================================
// =========== ChunkRef ==================================
// =======================================================

ChunkRef::ChunkRef(const uint64_t offset, Reader* reader) : chunk_offset_(offset), reader_(reader) {
    OsfOffset temp_offset = {offset, reader_->chunks_.get(chunk_offset_)->size};
    chunk_buf_ = reader->file_->read(reader_->file_->chunks_offset(), temp_offset);
    // Always expects "verified" chunk offset. See Reader::verify_chunk()
    assert(reader_->chunks_.get(chunk_offset_)->status != ChunkValidity::UNKNOWN);
}

ChunkRef::ChunkRef(const uint64_t offset, const OsfBuffer& buf, Reader* reader)
    : chunk_offset_(offset), reader_(reader), chunk_buf_(buf) {
    // Always expects "verified" chunk offset. See Reader::verify_chunk()
    assert(reader_->chunks_.get(chunk_offset_)->status != ChunkValidity::UNKNOWN);
}

ChunkState* ChunkRef::state() {
    return reader_->chunks_.get(chunk_offset_);
}

const ChunkState* ChunkRef::state() const {
    return reader_->chunks_.get(chunk_offset_);
}

ChunkInfoNode* ChunkRef::info() {
    return reader_->chunks_.get_info(chunk_offset_);
}

const ChunkInfoNode* ChunkRef::info() const {
    return reader_->chunks_.get_info(chunk_offset_);
}

size_t ChunkRef::size() const {
    if (!valid()) {
        return 0;
    }
    const ouster::sdk::osf::v2::Chunk* chunk = get_chunk_from_buf(chunk_buf_.data());
    if (chunk->messages() != nullptr) {
        return chunk->messages()->size();
    }
    return 0;
}

bool ChunkRef::valid() const {
    return (state()->status == ChunkValidity::VALID);
}

std::unique_ptr<const MessageRef> ChunkRef::messages(size_t msg_idx) const {
    try {
        if (!valid()) {
            return nullptr;
        }
        const ouster::sdk::osf::v2::Chunk* chunk = get_chunk_from_buf(chunk_buf_.data());
        if ((chunk->messages() == nullptr) || msg_idx >= chunk->messages()->size()) {
            return nullptr;
        }
        const ouster::sdk::osf::v2::StampedMessage* message = chunk->messages()->Get(msg_idx);
        OsfBuffer message_buf;
        uint64_t offset = reinterpret_cast<const uint8_t*>(message) - chunk_buf_.data();
        message_buf.load_data(chunk_buf_, offset, message->buffer()->size());
        return std::make_unique<const MessageRef>(message_buf, reader_->meta_store_,
                                                  reader_->error_handler_);
    } catch (const std::runtime_error& e) {
        reader_->error_handler_(Severity::OUSTER_WARNING, e.what());
        return nullptr;
    }
}

MessageRef ChunkRef::operator[](size_t msg_idx) const {
    auto temp_result = messages(msg_idx);  // for bounds checking
    if (temp_result == nullptr) {
        throw std::out_of_range("ERROR: Message index out of range in ChunkRef.");
    }

    return *temp_result;
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
    std::stringstream string_stream;
    auto chunk_state = state();
    string_stream << "ChunkRef: ["
                  << "msgs_size = " << size() << ", state = ("
                  << ((chunk_state != nullptr) ? osf::to_string(*chunk_state) : "no state") << ")"
                  << ", chunk_buf_ = "
                  << (chunk_buf_.has_value() ? "size=" + std::to_string(chunk_buf_.size())
                                             : "nullptr")
                  << "]";
    return string_stream.str();
}

uint64_t ChunkRef::offset() const {
    return chunk_offset_;
}

ts_t ChunkRef::start_ts() const {
    return state()->start_ts;
}

ts_t ChunkRef::end_ts() const {
    return state()->end_ts;
}

// ==========================================================
// ========= MessagesChunkIter ==============================
// ==========================================================

MessagesChunkIter::MessagesChunkIter(const MessagesChunkIter& other) = default;

MessagesChunkIter::MessagesChunkIter(ChunkRef chunk_ref, const size_t msg_idx)
    : chunk_ref_(std::move(chunk_ref)), msg_idx_(msg_idx) {}

bool MessagesChunkIter::operator==(const MessagesChunkIter& other) const {
    return (chunk_ref_ == other.chunk_ref_ && msg_idx_ == other.msg_idx_);
}

bool MessagesChunkIter::operator!=(const MessagesChunkIter& other) const {
    return !this->operator==(other);
}

std::string MessagesChunkIter::to_string() const {
    std::stringstream string_stream;
    string_stream << "MessagesChunkIter: [chunk_ref = " << chunk_ref_.to_string()
                  << ", msg_idx = " << msg_idx_ << "]";
    return string_stream.str();
}

MessageRef MessagesChunkIter::operator*() const {
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
    if (msg_idx_ < chunk_ref_.size()) {
        ++msg_idx_;
    }
}

void MessagesChunkIter::prev() {
    if (msg_idx_ > 0) {
        --msg_idx_;
    }
}

bool MessagesChunkIter::done() const {
    return (msg_idx_ >= chunk_ref_.size());
}

// ==========================================================
// ========= SreeamingReader::MessagesStreamingIter =========
// ==========================================================

// Simplest hash to better compare priority que states with stream ids
uint32_t calc_stream_ids_hash(const std::vector<uint32_t>& stream_ids) {
    uint32_t hash_b = 378551;
    uint32_t hash_a = 63689;
    uint32_t hash = 0;
    std::vector<uint32_t> tmp_stream_ids{stream_ids};
    std::sort(tmp_stream_ids.begin(), tmp_stream_ids.end());
    for (const uint32_t stream_id : tmp_stream_ids) {
        hash = hash * hash_a + stream_id;
        hash_a *= hash_b;
    }
    return hash;
}

bool MessagesStreamingIter::GreaterChunkType::operator()(const opened_chunk_type& a,
                                                         const opened_chunk_type& b) {
    return a.first[a.second].ts() > b.first[b.second].ts();
}

MessagesStreamingIter::MessagesStreamingIter()
    : curr_ts_{}, end_ts_{}, stream_ids_{}, stream_ids_hash_{}, reader_{nullptr}, curr_chunks_{} {}

MessagesStreamingIter::MessagesStreamingIter(const MessagesStreamingIter& other)

    = default;

MessagesStreamingIter::MessagesStreamingIter(const ts_t start_ts, const ts_t end_ts,
                                             const std::vector<uint32_t>& stream_ids,
                                             Reader* reader)
    : curr_ts_{start_ts},
      end_ts_{end_ts},
      stream_ids_{stream_ids},
      stream_ids_hash_{calc_stream_ids_hash(stream_ids_)},
      reader_{reader} {
    if (curr_ts_ == end_ts_) {
        return;
    }

    if (stream_ids_.empty()) {
        for (const auto& stream_mapping : reader_->chunks_.stream_chunks()) {
            stream_ids_.push_back(stream_mapping.first);
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
        auto* chunk_state = reader_->chunks_.get_by_lower_bound_ts(stream_id, start_ts);
        bool filled = false;
        while (chunk_state != nullptr && chunk_state->start_ts < end_ts && !filled) {
            auto curr_offset = chunk_state->offset;
            OsfOffset temp_offset = {curr_offset, reader_->chunks_.get(curr_offset)->size};
            OsfBuffer buf = reader_->get_chunk(temp_offset);
            if (reader_->verify_chunk(temp_offset, buf)) {
                // 2. if chunk is valid open it, otherwise step 5
                ChunkRef cref{curr_offset, buf, reader_};
                for (size_t msg_idx = 0; msg_idx < cref.size(); ++msg_idx) {
                    // 3. find first message within chunk in [start_ts, end_ts)
                    //    range
                    if (cref[msg_idx].ts() >= start_ts && cref[msg_idx].ts() < end_ts) {
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
            chunk_state = reader_->chunks_.next_by_stream(curr_offset);
        }
    }

    if (!curr_chunks_.empty()) {
        const auto& curr_item = curr_chunks_.top();
        curr_ts_ = curr_item.first[curr_item.second].ts();
    } else {
        curr_ts_ = end_ts_;
    }
}

MessageRef MessagesStreamingIter::operator*() const {
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
bool MessagesStreamingIter::operator==(const MessagesStreamingIter& other) const {
    return (curr_ts_ == other.curr_ts_ && end_ts_ == other.end_ts_ && reader_ == other.reader_ &&
            stream_ids_hash_ == other.stream_ids_hash_ &&
            curr_chunks_.size() == other.curr_chunks_.size() &&
            (curr_chunks_.empty() || curr_chunks_.top() == other.curr_chunks_.top()));
}

bool MessagesStreamingIter::operator!=(const MessagesStreamingIter& other) const {
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
    if (curr_ts_ >= end_ts_) {
        return;
    }
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
        auto next_chunk_state = reader_->chunks_.next_by_stream(curr_item.first.offset());
        if (next_chunk_state != nullptr) {
            auto next_chunk_info = reader_->chunks_.get_info(next_chunk_state->offset);
            if (next_chunk_info == nullptr) {
                throw std::logic_error(
                    "ERROR: Can't iterate by streams without StreamingInfo "
                    "available.");
            }
            if (next_chunk_state->start_ts < end_ts_) {
                OsfOffset temp_offset = {next_chunk_state->offset,
                                         reader_->chunks_.get(next_chunk_state->offset)->size};
                OsfBuffer buf = reader_->file_->read(reader_->file_->chunks_offset(), temp_offset);
                if (reader_->verify_chunk(temp_offset, buf)) {
                    ChunkRef cref{next_chunk_state->offset, buf, reader_};
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

std::string MessagesStreamingIter::to_string() const {
    std::stringstream string_stream;
    string_stream << "MessagesStreamingIter: [curr_ts = " << curr_ts_.count()
                  << ", end_ts = " << end_ts_.count()
                  << ", curr_chunks_.size = " << curr_chunks_.size()
                  << ", stream_ids_hash_ = " << stream_ids_hash_;
    if (!curr_chunks_.empty()) {
        const auto& curr_item = curr_chunks_.top();
        string_stream << ", top = (ts = " << curr_item.first[curr_item.second].ts().count()
                      << ", id = " << curr_item.first[curr_item.second].id() << ")";
    }
    string_stream << "]";
    return string_stream.str();
}

// =========================================================
// ========= StreamingReader::MessagesStreamingRange =======
// =========================================================

MessagesStreamingRange::MessagesStreamingRange(const ts_t start_ts, const ts_t end_ts,
                                               const std::vector<uint32_t>& stream_ids,
                                               Reader* reader)
    : start_ts_(start_ts), end_ts_(end_ts), stream_ids_{stream_ids}, reader_{reader} {}

MessagesStreamingIter MessagesStreamingRange::begin() const {
    return MessagesStreamingIter(start_ts_, end_ts_ + ts_t{1}, stream_ids_, reader_);
}

MessagesStreamingIter MessagesStreamingRange::end() const {
    return MessagesStreamingIter(end_ts_ + ts_t{1}, end_ts_ + ts_t{1}, stream_ids_, reader_);
}

std::string MessagesStreamingRange::to_string() const {
    std::stringstream string_stream;
    string_stream << "MessagesStreamingRange: [start_ts = " << start_ts_.count()
                  << ", end_ts = " << end_ts_.count() << "]";
    return string_stream.str();
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
