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

/// Frame id typedef, which is a sensor id + frame index pair.
using FrameUniqueId = std::pair<uint32_t /*sensor_id*/, uint64_t /*frame_idx*/>;

/// Resolve frame functor.
/// Collation stream does not encode frame set sources directly, so we need an
/// external functor to resolve them.
using ResolveFrameFn = std::function<std::shared_ptr<ouster::sdk::core::LidarFrame>(FrameUniqueId)>;

/// Special case of frame id which denotes a missing LidarFrame
constexpr FrameUniqueId INVALID_FRAME_UID = FrameUniqueId{0xFFFF, 0};

/**
 * CollationStream that encodes FrameSet objects into the messages.
 *
 * Object type: ouster::sdk::core::FrameSet
 * Meta type: CollationStreamMeta (sensor_meta_id, field_types)
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_scan_stream.fbs
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
     * @param[in] receive_ts The receive timestamp to use for the lidar frame.
     * @param[in] sensor_ts The sensor timestamp to use for the lidar frame.
     * @param[in] collation The lidar frame collation to write.
     * @param[in] frame_uids The lidar frame unique ids to reference.
     */
    void save(const ouster::sdk::osf::ts_t receive_ts, const ouster::sdk::osf::ts_t sensor_ts,
              const obj_type& collation, const std::vector<FrameUniqueId>& frame_uids);

    /**
     * Encode/serialize the object to the buffer of bytes.
     *
     * @param[in] collation The lidar frame collation to turn into a buffer.
     * @param[in] frame_uids The lidar frame ids to reference.
     * @return The byte vector representation of lidar_frame.
     */
    std::vector<uint8_t> make_msg(const obj_type& collation,
                                  const std::vector<FrameUniqueId>& frame_uids);

    /**
     * Decode/deserialize the object from bytes buffer using the concrete
     * metadata type for the stream.
     *
     * @param[in] msg The MessageRef to decode into an instance of the data type
     * associated with this data stream.
     * @param[in] meta The concrete metadata type to use for decoding.
     * @param[in] meta_provider Used to reconstruct any references to other
     *                          metadata entries dependencies
     *                          (like sensor_meta_id)
     * @param[in] resolve_frame functor resolving frame ids into lidar frames
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

#include "ouster/core/deprecation.h"

namespace ouster {
namespace sdk {
namespace osf {

OUSTER_DEPRECATED_TYPE(ScanId, FrameUniqueId, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ResolveScanFn, ResolveFrameFn, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_CONSTEXP(INVALID_SCAN_ID, INVALID_FRAME_UID, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
