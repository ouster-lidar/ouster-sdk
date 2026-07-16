/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file metadata_stream.h
 * @brief Stream containing sensor info.
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/frame_set_source.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Represents the metadata entry associated with a FrameSetSourceMetadataStream
 */
class OUSTER_API_CLASS FrameSetSourceMetadataStreamMeta
    : public MetadataEntryHelper<FrameSetSourceMetadataStreamMeta> {
   public:
    /**
     * Create a FrameSetSourceMetadataStreamMeta.
     */
    OUSTER_API_FUNCTION
    FrameSetSourceMetadataStreamMeta();

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a FrameSetSourceMetadataStreamMeta object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new FrameSetSourceMetadataStreamMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const ouster::sdk::osf::OsfBuffer& buf);

    /**
     * Get the string representation for the FrameSetSourceMetadataStreamMeta
     * object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the
     * FrameSetSourceMetadataStreamMeta object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;
};

/**
 * Templated struct for returning the OSF type string.
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<FrameSetSourceMetadataStreamMeta> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string
     * "ouster/v1/os_sensor/ScanSourceMetadataStreamMeta".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/ScanSourceMetadataStreamMeta";
    }
};

/**
 * MetadataStream that encodes sensor info config objects into the messages.
 */
class OUSTER_API_CLASS FrameSetSourceMetadataStream
    : public MessageStream<FrameSetSourceMetadataStreamMeta, FrameSetSourceMetadataSet> {
   protected:
    friend class Writer;
    friend class MessageRef;

    void save(const FrameSetSourceMetadataSet& frame_set_source_metadata_set, ts_t timestamp);

    /**
     * Encode/serialize the object to the buffer of bytes.
     */
    static std::vector<uint8_t> make_msg(
        const FrameSetSourceMetadataSet& frame_set_source_metadata_set);

    /**
     * Decode/deserialize the object from bytes buffer using the concrete
     * metadata type for the stream.
     */
    static std::unique_ptr<obj_type> decode_msg(const MessageRef& msg, const meta_type& meta,
                                                const MetadataStore& meta_provider);

   public:
    /**
     * @param[in] writer The writer object to use to write messages out.
     */
    OUSTER_API_FUNCTION
    FrameSetSourceMetadataStream(Writer& writer);

    /**
     * Deserialize the object from a buffer of bytes.
     * @param[in] buf buffer to deserialize from
     * @return the deserialized object
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<obj_type> from_buffer(const std::vector<uint8_t>& buf);
    /**
     * Return the concrete metadata type.
     * This has templated types.
     *
     * @return The concrete metadata type.
     */
    OUSTER_API_FUNCTION
    const meta_type& meta() const {
        return meta_;
    };

   private:
    /**
     * The internal writer object to use to write messages out.
     */
    Writer& writer_;

    /**
     * The internal concrete metadata type.
     */
    meta_type meta_;

    /**
     * The internal flatbuffer id for the stream.
     */
    uint32_t stream_meta_id_{0};
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace osf {

OUSTER_DEPRECATED_TYPE(ScanSourceMetadataStream, FrameSetSourceMetadataStream,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanSourceMetadataStreamMeta, FrameSetSourceMetadataStreamMeta,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
