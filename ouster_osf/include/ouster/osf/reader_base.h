/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file reader_base.h
 * @brief OSF file Reader base
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ouster/error_handler.h"
#include "ouster/osf/chunk.h"
#include "ouster/osf/file.h"
#include "ouster/osf/metadata.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

using ouster::sdk::core::default_error_handler;
using ouster::sdk::core::error_handler_t;
using ouster::sdk::core::Severity;

/**
 * %OSF Reader that simply reads sequentially messages from the OSF file.
 *
 * @todo Add filtered reads, and other nice things...
 */
class OUSTER_API_CLASS ReaderBase {
   public:
    /**
     * Creates reader from %OSF file resource.
     *
     * @param[in] osf_file The OsfFile object to use to read from.
     * @param[in] error_handler An optional callback that serves as an error
     * handler.
     */
    OUSTER_API_FUNCTION
    ReaderBase(std::unique_ptr<OsfFile> osf_file,
               const error_handler_t& error_handler = default_error_handler);

    /**
     * Whether OSF contains the message counts that are needed for
     * ``ts_by_message_idx()``
     *
     * Message counts was added a bit later to the OSF core
     * (ChunkInfo struct), so this function will be obsolete over time.
     *
     * @return Whether OSF contains the message counts that are needed for
     *         ``ts_by_message_idx()``
     */
    OUSTER_API_FUNCTION
    bool has_message_idx() const;

    /**
     * Whether OSF contains the message timestamp index in the metadata
     * necessary to quickly collate and jump to a specific message time."
     *
     * @return Whether OSF contains the message timestamp index
     */
    OUSTER_API_FUNCTION
    bool has_timestamp_idx() const;

    /**
     * Return the metadata id.
     *
     * @return The metadata id.
     */
    OUSTER_API_FUNCTION
    std::string metadata_id() const;

    /**
     * Return the lowest timestamp in the ChunksIter.
     *
     * @return The lowest timestamp in the ChunksIter.
     */
    OUSTER_API_FUNCTION
    ts_t start_ts() const;

    /**
     * Return the highest timestamp in the ChunksIter.
     *
     * @return The highest timestamp in the ChunksIter.
     */
    OUSTER_API_FUNCTION
    ts_t end_ts() const;

    /**
     * Return all metadata entries as a MetadataStore
     *
     * @return All of the metadata entries as a MetadataStore.
     */
    OUSTER_API_FUNCTION
    const MetadataStore& meta_store() const;

    /**
     * If the chunks can be read by stream and in non-decreasing timestamp
     * order.
     *
     * @return The chunks can be read by stream and timestamps are sane.
     */
    OUSTER_API_FUNCTION
    bool has_stream_info() const;

    /**
     * Get the OSF file format version.
     *
     * @return The OSF file format version of this file.
     */
    OUSTER_API_FUNCTION
    ouster::sdk::core::Version version() const;

    /**
     * Checks the flatbuffers validity of a chunk by chunk offset.
     *
     * @param[in] offset Specify the chunk to verify via offset.
     * @param[in] buf A buffer containing the chunk to verify.
     * @return The validity of the chunk.
     */
    OUSTER_API_FUNCTION
    bool verify_chunk(OsfOffset offset, OsfBuffer& buf);

   protected:
    /**
     * Read, parse and store all of the flatbuffer related metadata.
     *
     * @throws std::logic_error Exception on invalid metadata block.
     */
    void read_metadata();

    /**
     * Internal OsfFile object used to read the OSF file.
     */
    std::unique_ptr<OsfFile> file_;

    /**
     * File version.
     */
    ouster::sdk::core::Version version_;

    /**
     * Internal MetadataStore object to hold all of the
     * metadata entries.
     */
    MetadataStore meta_store_{};

    /**
     * Internal ChunksPile object to hold all of the
     * chunks.
     */
    ChunksPile chunks_{};

    /**
     * Internal indicator of if this file has streaming info
     */
    bool has_streaming_info_{false};

    /**
     * Absolute offset to the beginning of the chunks in a file.
     */
    uint64_t chunks_base_offset_{0};

    ts_t start_ts_{};
    ts_t end_ts_{};

    /**
     * A function that can serve as a user-provided error handler.
     */
    error_handler_t error_handler_;

    // NOTE: These classes need an access to private member `chunks_` ...
    friend class ChunkRef;
    friend struct ChunksIter;
    friend struct MessagesStreamingIter;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
