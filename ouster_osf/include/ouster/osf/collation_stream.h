/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/frame_set.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Metadata entry for CollationStream to store reference to a sensor and
 * field_types
 *
 * OSF type:
 *   ouster/v1/os_sensor/CollationStream
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/collation_stream.fbs
 */
class OUSTER_API_CLASS CollationStreamMeta : public MetadataEntryHelper<CollationStreamMeta> {
   public:
    /**
     * Default constructor
     */
    OUSTER_API_FUNCTION CollationStreamMeta();

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a CollationStreamMeta object from a byte array.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new CollationStreamMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const OsfBuffer& buf);

    /**
     * Get the string representation for the CollationStreamMeta object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the CollationStreamMeta object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;
};

/**
 * Templated struct for returning the OSF type string.
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<CollationStreamMeta> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/CollationStream".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/CollationStream";
    }
};

/// Frame id typedef: a unified sensor index + message index pair. The sensor
/// index is the sensor's position in the file's registration order (the same
/// order exposed by sensor_info()); the message index is the frame's position
/// within that sensor's frame stream.
using FrameUniqueId = std::pair<uint32_t /*sensor_id*/, uint64_t /*frame_idx*/>;

/// Resolve a frame (lidar or camera) from its FrameUniqueId. The collation
/// stream stores only index references, not the frames themselves, so an
/// external functor resolves each one. The concrete frame type is determined by
/// the referenced sensor. Returns nullptr for a null / unresolvable slot.
using ResolveFrameFn = std::function<std::shared_ptr<ouster::sdk::core::Frame>(FrameUniqueId)>;

/// Sentinel message index (in the unified CollationMsg::frame_ids) for a null
/// slot.
constexpr uint64_t INVALID_COLLATION_FRAME_INDEX = 0xFFFFFFFFFFFFFFFFull;

/// Sentinel scan id for a null slot in the legacy, lidar-only
/// CollationMsg::scan_ids field. This is the exact {sensor_id, scan_idx}
/// sentinel that SDK versions predating camera support both write and skip on
/// read, so emitting it keeps those older readers from dereferencing a missing
/// frame.
constexpr FrameUniqueId LEGACY_INVALID_SCAN_ID = FrameUniqueId{0xFFFFu, 0u};

/**
 * CollationStream that encodes FrameSet objects into the messages.
 *
 * Object type: ouster::sdk::core::FrameSet
 * Meta type: CollationStreamMeta (marker only)
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/collation_stream.fbs
 */
class OUSTER_API_CLASS CollationStream
    : public MessageStream<CollationStreamMeta, ouster::sdk::core::FrameSet> {
   protected:
    friend class Writer;
    friend class MessageRef;

    /**
     * Saves the object to the writer applying the coding/serialization
     * algorithm defined in make_msg() function. The function is the same for
     * all streams types ...
     *
     * @param[in] receive_ts The receive timestamp to use for the collation.
     * @param[in] sensor_ts The sensor timestamp to use for the collation.
     * @param[in] collation The frame collation to write.
     * @param[in] frame_indices Per-slot message index, positionally aligned to
     *                          the collation's frames (slot k references the
     *                          k-th registered sensor, by unified index). Use
     *                          INVALID_COLLATION_FRAME_INDEX for a null slot.
     *                          Written to the unified `frame_ids` field.
     * @param[in] legacy_scan_ids Backward-compatible, lidar-only collation
     *                          entries written to the legacy `scan_ids` field:
     *                          one per lidar slot, in per-lidar index order,
     *                          with a null slot encoded as
     *                          {sensor_id=0xFFFF, scan_idx=0} (the sentinel
     *                          understood by SDKs that predate camera support).
     */
    void save(const ouster::sdk::osf::ts_t receive_ts, const ouster::sdk::osf::ts_t sensor_ts,
              const obj_type& collation, const std::vector<uint64_t>& frame_indices,
              const std::vector<FrameUniqueId>& legacy_scan_ids);

    /**
     * Encode/serialize the object to the buffer of bytes.
     *
     * @param[in] collation The frame collation to turn into a buffer.
     * @param[in] frame_indices Per-slot (unified) message index (see save()).
     * @param[in] legacy_scan_ids Lidar-only legacy collation entries (see
     *                          save()).
     * @return The byte vector representation of the collation.
     */
    std::vector<uint8_t> make_msg(const obj_type& collation,
                                  const std::vector<uint64_t>& frame_indices,
                                  const std::vector<FrameUniqueId>& legacy_scan_ids);

    /**
     * Decode/deserialize the object from bytes buffer using the concrete
     * metadata type for the stream.
     *
     * @param[in] msg The MessageRef to decode into an instance of the data type
     * associated with this data stream.
     * @param[in] meta The concrete metadata type to use for decoding.
     * @param[in] meta_provider Provides the file's metadata entries.
     * @param[in] resolve_frame Functor resolving a FrameUniqueId (unified sensor
     *                          index + message index) into a Frame (lidar or
     *                          camera). Returns nullptr for a null slot.
     * @return Pointer to the decoded object.
     */
    static std::unique_ptr<obj_type> decode_msg(const MessageRef& msg, const meta_type& meta,
                                                const MetadataStore& meta_provider,
                                                const ResolveFrameFn& resolve_frame);

   public:
    /**
     * @param[in] writer The writer object to use to write messages out.
     */
    OUSTER_API_FUNCTION
    CollationStream(Writer& writer);

    /**
     * Return the concrete metadata type.
     * This has templated types.
     *
     * @return The concrete metadata type.
     */
    OUSTER_API_FUNCTION
    const meta_type& meta() const {
        return meta_;
    }

   private:
    /**
     * The internal writer object to use to write messages out.
     */
    Writer& writer_;

    /**
     * The internal concrete metadata type.
     */
    meta_type meta_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
