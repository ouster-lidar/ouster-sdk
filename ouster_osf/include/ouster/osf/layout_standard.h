/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file layout_standard.h
 * @brief OSF Standard Layout strategy.
 *
 */
#pragma once

#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

constexpr uint32_t STANDARD_DEFAULT_CHUNK_SIZE =
    5 * 1024 * 1024;  // not strict ...

/**
 * Standard Layout chunking strategy
 *
 * When messages laid out into chunks in the order as they come and not
 * exceeding `chunk_size` (if possible). However if a single
 * message size is bigger than specified `chunk_size` it's still recorded.
 */
class StandardLayoutCW : public ChunksWriter {
   public:
    StandardLayoutCW(Writer& writer,
                     uint32_t chunk_size = STANDARD_DEFAULT_CHUNK_SIZE);
    void saveMessage(const uint32_t stream_id, const ts_t ts,
                     const std::vector<uint8_t>& msg_buf) override;

    void finish() override;

    uint32_t chunk_size() const override { return chunk_size_; }

   private:
    void finish_chunk();

    const uint32_t chunk_size_;
    ChunkBuilder chunk_builder_{};

    Writer& writer_;
};

}  // namespace osf
}  // namespace ouster