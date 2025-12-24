/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/lidar_scan_set.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"
#include "ouster/visibility.h"

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
class OUSTER_API_CLASS CollationStreamMeta
    : public MetadataEntryHelper<CollationStreamMeta> {
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
    static std::unique_ptr<MetadataEntry> from_buffer(const OsfBuffer buf);

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

/// Scan id typedef, which is a sensor id + scan index pair.
using ScanId = std::pair<uint32_t /*sensor_id*/, uint64_t /*scan_idx*/>;

/// Resolve scan functor.
/// Collation stream does not encode scan sources directly, so we need an
/// external functor to resolve them.
using ResolveScanFn =
    std::function<std::shared_ptr<ouster::sdk::core::LidarScan>(ScanId)>;

/// Special case of scan id which denotes a missing LidarScan
constexpr ScanId INVALID_SCAN_ID = ScanId{0xFFFF, 0};

/**
 * CollationStream that encodes LidarScanSet objects into the messages.
 *
 * Object type: ouster::sdk::core::LidarScanSet
 * Meta type: CollationStreamMeta (sensor_meta_id, field_types)
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_scan_stream.fbs
 */
class OUSTER_API_CLASS CollationStream
    : public MessageStream<CollationStreamMeta,
                           ouster::sdk::core::LidarScanSet> {
   protected:
    friend class Writer;
    friend class MessageRef;

    // Access key pattern used to only allow friends to call our constructor
    struct Token {};

    /**
     * Saves the object to the writer applying the coding/serialization
     * algorithm defined in make_msg() function. The function is the same for
     * all streams types ...
     *
     * @param[in] receive_ts The receive timestamp to use for the lidar scan.
     * @param[in] sensor_ts The sensor timestamp to use for the lidar scan.
     * @param[in] collation The lidar scan collation to write.
     * @param[in] scan_ids The lidar scan ids to reference.
     */
    void save(const ouster::sdk::osf::ts_t receive_ts,
              const ouster::sdk::osf::ts_t sensor_ts, const obj_type& collation,
              const std::vector<ScanId>& scan_ids);

    /**
     * Encode/serialize the object to the buffer of bytes.
     *
     * @param[in] collation The lidar scan collation to turn into a buffer.
     * @param[in] scan_ids The lidar scan ids to reference.
     * @return The byte vector representation of lidar_scan.
     */
    std::vector<uint8_t> make_msg(const obj_type& collation,
                                  const std::vector<ScanId>& scan_ids);

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
     * @param[in] resolve_scan functor resolving scan ids into lidar scans
     * @return Pointer to the decoded object.
     */
    static std::unique_ptr<obj_type> decode_msg(
        const MessageRef& msg, const meta_type& meta,
        const MetadataStore& meta_provider, const ResolveScanFn& resolve_scan);

   public:
    /**
     * @param[in] key Private class used to prevent non-friends from calling
     * this.
     * @param[in] writer The writer object to use to write messages out.
     */
    OUSTER_API_FUNCTION
    CollationStream(Token key, Writer& writer);

    /**
     * Return the concrete metadata type.
     * This has templated types.
     *
     * @return The concrete metadata type.
     */
    OUSTER_API_FUNCTION
    const meta_type& meta() const { return meta_; }

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
