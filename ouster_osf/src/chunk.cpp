#include "ouster/osf/chunk.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/osf/basics.h"
#include "ouster/visibility.h"

using StreamChunksMap =
    std::unordered_map<uint32_t, std::shared_ptr<std::vector<uint64_t>>>;

namespace ouster {
namespace sdk {
namespace osf {

ChunkState::ChunkState(uint64_t off, ts_t start, ts_t end)
    : offset(off), size(0), start_ts(start), end_ts(end) {}

// =======================================================
// =========== ChunksPile ================================
// =======================================================
ChunksPile::ChunksPile(const std::vector<ChunkState>& chunk_states,
                       uint64_t last_chunk_buf_offset) {
    for (const auto& chunk_state : chunk_states) {
        pile_.emplace(chunk_state.offset, chunk_state);
        chunk_offsets_.push_back(chunk_state.offset);
    }
    link_chunks(last_chunk_buf_offset);
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
    if (!has_message_idx()) {
        return nullptr;
    }

    auto schunks = stream_chunks_.find(stream_id);
    if (schunks == stream_chunks_.end()) {
        return nullptr;
    }

    // out of bounds
    auto lci = get_info(schunks->second->back());
    if (message_idx >= lci->message_start_idx + lci->message_count) {
        return nullptr;
    }

    auto lower_bound_it = std::lower_bound(
        schunks->second->begin(), schunks->second->end(), message_idx,
        [&](uint64_t a, uint32_t m_idx) {
            const auto* ci = get_info(a);
            return ci->message_start_idx + ci->message_count - 1 < m_idx;
        });

    return get_info(*lower_bound_it);
}

ChunkState* ChunksPile::get_by_lower_bound_ts(uint32_t stream_id,
                                              const ts_t timestamp) {
    auto schunks = stream_chunks_.find(stream_id);
    if (schunks == stream_chunks_.end()) {
        return nullptr;
    }
    auto lb_offset =
        std::lower_bound(schunks->second->begin(), schunks->second->end(),
                         timestamp, [&](uint64_t a, const ts_t timestamp) {
                             return get(a)->end_ts < timestamp;
                         });
    if (lb_offset == schunks->second->end()) {
        return nullptr;
    }
    return get(*lb_offset);
}

ChunkState* ChunksPile::next(uint64_t offset) {
    auto chunk = get(offset);
    if (chunk == nullptr) {
        return nullptr;
    }
    return get(chunk->next_offset);
}

ChunkState* ChunksPile::next_by_stream(uint64_t offset) {
    auto chunk_info = get_info(offset);
    if (chunk_info == nullptr) {
        return nullptr;
    }
    auto result = get(chunk_info->next_offset);
    return result;
}

ChunkState* ChunksPile::first() {
    // Note - chunk_offsets must be sorted.
    if (chunk_offsets_.empty()) {
        return nullptr;
    }
    return get(chunk_offsets_.front());
}

size_t ChunksPile::size() const { return pile_.size(); }

bool ChunksPile::has_message_idx() const {
    // return true if we have no chunks (we're functionally indexed)
    if (pile_.empty()) {
        return true;
    }
    // rely on the fact that message_count in the ChunkInfo, if present
    // during Writing/Chunk building, can't be 0 (by construction in
    // ChunkBuilder and StreamingLayoutCW)
    // In other words we can't have Chunks with 0 messages written to OSF
    // file
    return (!pile_info_.empty()) &&
           pile_info_.begin()->second.message_count > 0;
}

StreamChunksMap& ChunksPile::stream_chunks() { return stream_chunks_; }

// NOTE - this computes chunk sizes with the assumption that chunks are
// contiguous and continue up to the beginning of the metadata.
OUSTER_API_FUNCTION
void ChunksPile::link_chunks(uint64_t last_chunk_buf_offset) {
    if (!chunk_offsets_.empty()) {
        std::sort(chunk_offsets_.begin(), chunk_offsets_.end());
        for (size_t i = 0; i < chunk_offsets_.size() - 1; ++i) {
            auto temp_chunk = get(chunk_offsets_[i]);
            temp_chunk->next_offset = chunk_offsets_[i + 1];
            temp_chunk->size = chunk_offsets_[i + 1] - chunk_offsets_[i];
        }
        auto last_chunk = get(chunk_offsets_.back());
        last_chunk->size = last_chunk_buf_offset - chunk_offsets_.back();
    }
}

std::vector<uint64_t> ChunksPile::chunk_offsets() const {
    return chunk_offsets_;
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

    if (!pile_info_.empty()) {
        // Do the next_offset links by streams
        auto curr_chunk = first();
        if (curr_chunk == nullptr) {
            throw std::runtime_error(
                "Invalid OSF file: could not find the first Chunk.");
        }

        while (curr_chunk != nullptr) {
            auto ci = get_info(curr_chunk->offset);
            if (ci == nullptr) {
                throw std::logic_error("ERROR: Have a missing chunk info");
            }
            if (stream_chunks_.count(ci->stream_id) != 0u) {
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
    std::stringstream stream;
    stream << "{offset = " << chunk_state.offset
           << ", next_offset = " << chunk_state.next_offset
           << ", start_ts = " << chunk_state.start_ts.count()
           << ", end_ts = " << chunk_state.end_ts.count()
           << ", status = " << static_cast<int>(chunk_state.status) << "}";
    return stream.str();
}

std::string to_string(const ChunkInfoNode& chunk_info) {
    std::stringstream stream;
    stream << "{offset = " << chunk_info.offset
           << ", next_offset = " << chunk_info.next_offset
           << ", stream_id = " << chunk_info.stream_id
           << ", message_count = " << chunk_info.message_count
           << ", message_start_idx = " << chunk_info.message_start_idx << "}";
    return stream.str();
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
