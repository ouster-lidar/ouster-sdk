/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_info_stream.h
 * @brief Stream containing sensor info.
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Represents the metadata entry associated with a SensorInfoStream.
 */
class OUSTER_API_CLASS SensorInfoStreamMeta
    : public MetadataEntryHelper<SensorInfoStreamMeta> {
   public:
    /**
     * Create a SensorInfoStreamMeta.
     */
    OUSTER_API_FUNCTION
    SensorInfoStreamMeta();

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a SensorInfoStreamMeta object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new SensorInfoStreamMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(
        const ouster::sdk::osf::OsfBuffer buf);

    /**
     * Get the string representation for the SensorInfoStreamMeta object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the SensorInfoStreamMeta
     * object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;
};

/**
 * Templated struct for returning the OSF type string.
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<SensorInfoStreamMeta> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string
     * "ouster/v1/os_sensor/SensorInfoStreamMeta".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/SensorInfoStreamMeta";
    }
};

/// Decoded SensorInfo message
struct SensorInfoMessage {
    /// decoded SensorInfo structure
    ouster::sdk::core::SensorInfo sensor_info;

    /// metadata id for the lidar sensor associated with this SensorInfo
    uint32_t lidar_sensor_id;

    /// metadata id for the scan stream associated with this SensorInfo
    uint32_t scan_stream_id;
};

/**
 * SensorInfoStream that encodes sensor info config objects into the messages.
 */
class OUSTER_API_CLASS SensorInfoStream
    : public MessageStream<SensorInfoStreamMeta, SensorInfoMessage> {
   protected:
    friend class Writer;
    friend class MessageRef;

    // Access key pattern used to only allow friends to call our constructor
    struct Token {};

    void save(const obj_type& config, ts_t timestamp);

    /**
     * Encode/serialize the object to the buffer of bytes.
     */
    static std::vector<uint8_t> make_msg(const obj_type& msg);

    /**
     * Decode/deserialize the object from bytes buffer using the concrete
     * metadata type for the stream.
     */
    static std::unique_ptr<obj_type> decode_msg(
        const MessageRef& msg, const meta_type& meta,
        const MetadataStore& meta_provider);

   public:
    /**
     * @param[in] key Private class used to prevent non-friends from calling
     * this.
     * @param[in] writer The writer object to use to write messages out.
     */
    OUSTER_API_FUNCTION
    SensorInfoStream(Token key, Writer* writer);

    /**
     * Deserialize the object from a buffer of bytes.
     * @param[in] buf buffer to deserialize from
     * @return the deserialized object
     */
    static std::unique_ptr<obj_type> from_buffer(
        const std::vector<uint8_t>& buf);
    /**
     * Return the concrete metadata type.
     * This has templated types.
     *
     * @return The concrete metadata type.
     */
    OUSTER_API_FUNCTION
    const meta_type& meta() const { return meta_; };

   private:
    /**
     * The internal writer object to use to write messages out.
     */
    Writer* writer_{nullptr};

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
