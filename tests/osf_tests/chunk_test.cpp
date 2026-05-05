
#include "ouster/osf/chunk.h"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

namespace ouster {
namespace sdk {
namespace osf {

TEST(ChunksPile, defaultConstructor) {
    ChunksPile pile;
    EXPECT_EQ(pile.has_message_idx(),
              true);  // true if there is nothing in the pile
    EXPECT_EQ(pile.size(), 0);
    EXPECT_EQ(pile.first(), nullptr);
    EXPECT_EQ(pile.get(12345), nullptr);
    EXPECT_EQ(pile.next_by_stream(0), nullptr);
}

TEST(ChunksPile, constructor) {
    uint64_t end_of_chunks_offset = 456;
    std::vector<ChunkState> chunk_states = {
        {100, 0ns, 0ns}, {200, 0ns, 0ns}, {300, 0ns, 0ns}};

    ChunksPile pile(chunk_states, end_of_chunks_offset);
    EXPECT_EQ(pile.size(), chunk_states.size());
    pile.link_stream_chunks();

    EXPECT_EQ(pile.first()->offset, chunk_states[0].offset);
    EXPECT_EQ(pile.first()->next_offset, chunk_states[1].offset);
    EXPECT_EQ(pile.first()->size,
              chunk_states[1].offset - chunk_states[0].offset);

    // check size of last chunk
    EXPECT_EQ(pile.get(chunk_states[2].offset)->size,
              end_of_chunks_offset - chunk_states[2].offset);

    // check index - no info was added, so there's no index
    EXPECT_EQ(pile.has_message_idx(), false);
}

TEST(ChunksPile, add_info) {
    // Modern OSFs will have "stream info" which relates a chunk (via offset) to
    // its stream id and a message count. This allows indexing into an OSF file
    // per-stream.
    uint64_t end_of_chunks_offset = 456;
    int message_count = 1;
    std::vector<uint64_t> chunk_offsets = {100, 200, 300, 400};
    std::vector<ChunkState> chunk_states;

    for (auto offset : chunk_offsets) {
        chunk_states.emplace_back(offset, std::chrono::nanoseconds(offset),
                                  std::chrono::nanoseconds(offset + 10));
    }

    ChunksPile pile(chunk_states, end_of_chunks_offset);
    for (auto offset : chunk_offsets) {
        // interleave chunks among two different streams (id 0 and 1)
        int stream_id = offset % 200 == 0 ? 1 : 0;
        pile.add_info(offset, stream_id, message_count);
        auto info = pile.get_info(offset);
        ASSERT_EQ(info->offset, offset);
        ASSERT_EQ(info->stream_id, stream_id);
        ASSERT_EQ(info->message_count, message_count);
    }

    pile.link_stream_chunks();
    ASSERT_NE(pile.get(100), nullptr);
    ASSERT_NE(pile.next(100), nullptr);

    // next_by_stream returns a ChunkState, but uses the ChunkInfoNode
    // "pile_info_" map internally.
    EXPECT_EQ(pile.next_by_stream(100)->offset, 300);
    EXPECT_EQ(pile.next_by_stream(200)->offset, 400);

    // In contrast to ChunkState, "next_offset" for a ChunkInfoNode refers to
    // the next ChunkInfoNode with the same stream id.
    EXPECT_EQ(pile.get_info(100)->next_offset, 300);
    EXPECT_EQ(pile.get_info(200)->next_offset, 400);

    // get_info_by_message_idx returns the ChunkInfoNode that corresponds to the
    // given stream id and message index (starting at 0 per stream, dictated by
    // the number of messages per chunk.)
    EXPECT_EQ(pile.get_info_by_message_idx(0, 0)->offset, 100);
    EXPECT_EQ(pile.get_info_by_message_idx(1, 0)->offset, 200);
    EXPECT_EQ(pile.get_info_by_message_idx(0, 1)->offset, 300);
    EXPECT_EQ(pile.get_info_by_message_idx(1, 1)->offset, 400);
    EXPECT_EQ(pile.get_info_by_message_idx(0, 2), nullptr);
    EXPECT_EQ(pile.get_info_by_message_idx(1, 2), nullptr);

    // get_by_lower_bound_ts returns the next ChunkState that has an end_ts that
    // occurs after the given ts.
    EXPECT_EQ(pile.get_by_lower_bound_ts(0, 90ns)->offset, 100);
    EXPECT_EQ(pile.get_by_lower_bound_ts(0, 120ns)->offset, 300);
    EXPECT_EQ(pile.get_by_lower_bound_ts(0, 330ns), nullptr);
    EXPECT_EQ(pile.get_by_lower_bound_ts(1, 90ns)->offset, 200);
    EXPECT_EQ(pile.get_by_lower_bound_ts(1, 220ns)->offset, 400);
    EXPECT_EQ(pile.get_by_lower_bound_ts(1, 420ns), nullptr);

    EXPECT_EQ(pile.size(), chunk_offsets.size());
    EXPECT_EQ(pile.first()->offset, chunk_offsets[0]);
    EXPECT_EQ(pile.first()->next_offset, chunk_offsets[1]);
    EXPECT_EQ(pile.first()->size, chunk_offsets[1] - chunk_offsets[0]);

    // has_message_idx returns true if "stream info" was present
    // in the file.
    EXPECT_EQ(pile.has_message_idx(), true);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
