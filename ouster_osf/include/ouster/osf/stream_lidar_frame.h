/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file stream_lidar_frame.h
 * @brief Stream of LidarFrame
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

#include "ouster/core/frame.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Cast `src` to a subset of fields with possible different underlying
 * ChanFieldTypes.
 *
 * The result is a copy of `src` carrying exactly @p field_types: fields that
 * `src` holds under a different ChanFieldType (or field class, or extra
 * dimensions) are cast into the requested one, fields `src` already holds as
 * requested are copied unchanged, and everything else is dropped. Every other
 * part of the frame — dimensions, timestamp / pose / status headers, objects,
 * the associated sensor_info(), and any headers specific to the concrete frame
 * type — comes across untouched.
 *
 * The copy is made through Frame::clone(), so the returned frame has the same
 * concrete type as @p src: hand it a LidarFrame and you get a LidarFrame, hand
 * it a CameraFrame and you get a CameraFrame.
 *
 * @throws std::invalid_argument if a requested field is missing from @p src, or
 *         if a field needing a cast cannot be cast (casting is supported for
 *         two-dimensional fields only).
 *
 * @param[in] src The frame to cast.
 * @param[in] field_types The field types to cast the frame to.
 * @return a copy of `src` with transformed fields.
 */
OUSTER_API_FUNCTION
std::shared_ptr<ouster::sdk::core::Frame> slice_and_cast(
    const ouster::sdk::core::Frame& src,
    const ouster::sdk::core::LidarFrameFieldTypes& field_types);

/**
 * Cast `ls_src` LidarFrame to a subset of fields with possible different
 * underlying ChanFieldTypes.
 *
 * Convenience overload returning a LidarFrame by value; see
 * slice_and_cast(const core::Frame&, const core::LidarFrameFieldTypes&) for
 * the semantics.
 *
 * @throws std::invalid_argument Exception on trying to slice a frame with only
 *                               a subset of the requested frames
 *
 * @param[in] ls_src The LidarFrame to cast.
 * @param[in] field_types The field types to cast the LidarFrame to.
 * @return a copy of `ls_src` with transformed fields.
 */
OUSTER_API_FUNCTION
ouster::sdk::core::LidarFrame slice_and_cast(
    const ouster::sdk::core::LidarFrame& ls_src,
    const ouster::sdk::core::LidarFrameFieldTypes& field_types);

/**
 * Metadata entry for LidarFrameStream to store reference to a sensor and
 * field_types
 *
 * OSF type:
 *   ouster/v1/os_sensor/LidarScanStream
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/lidar_scan_stream.fbs
 */
class OUSTER_API_CLASS LidarFrameStreamMeta : public MetadataEntryHelper<LidarFrameStreamMeta> {
   public:
    /**
     * @param[in] sensor_meta_id Reference to LidarSensor metadata that
     *                           describes the sensor configuration.
     * @param[in] field_types LidarFrame fields specs, this argument is
     * optional.
     */
    OUSTER_API_FUNCTION
    LidarFrameStreamMeta(const uint32_t sensor_meta_id,
                         const ouster::sdk::core::LidarFrameFieldTypes& field_types = {});

    /**
     * Return the sensor meta id.
     *
     * @return The sensor meta id.
     */
    OUSTER_API_FUNCTION
    uint32_t sensor_meta_id() const;

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a LidarFrameStreamMeta object from a byte array.
     *
     * @todo Figure out why this wasn't just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new LidarFrameStreamMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const OsfBuffer& buf);

    /**
     * Get the string representation for the LidarFrameStreamMeta object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the LidarFrameStreamMeta object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;

   private:
    /**
     * Internal store of the sensor id.
     *
     * Flat Buffer Reference:
     *   fb/os_sensor/lidar_scan_stream.fbs :: LidarFrameStream :: sensor_id
     */
    uint32_t sensor_meta_id_{0};

    /**
     * Internal store of the field types.
     *
     * Flat Buffer Reference:
     *   fb/os_sensor/lidar_scan_stream.fbs :: LidarFrameStream :: field_types
     */
    ouster::sdk::core::LidarFrameFieldTypes field_types_;
};

/** @defgroup OSFTraitsLidarFrameStreamMeta Templated struct for traits.*/

/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsLidarFrameStreamMeta
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<LidarFrameStreamMeta> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/LidarScanStream".
     */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/LidarScanStream";
    }
};

/**
 * LidarFrameStream that encodes LidarFrame objects into the messages.
 *
 * Object type: ouster::sdk::core::LidarFrame
 * Meta type: LidarFrameStreamMeta (sensor_meta_id, field_types)
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_scan_stream.fbs
 */
class OUSTER_API_CLASS LidarFrameStream
    : public MessageStream<LidarFrameStreamMeta, ouster::sdk::core::LidarFrame> {
   protected:
    friend class Writer;
    friend class MessageRef;

    /**
     * Saves the object to the writer applying the coding/serialization
     * algorithm defined in make_msg() function. The function is the same for
     * all streams types ...
     *
     * @todo [pb]: Probably should be abstracted/extracted from all streams
     * we also might want to have the corresponding function to read back
     * sequentially from Stream that doesn't seem like fit into this model...
     *
     * @param[in] receive_ts The receive timestamp to use for the lidar frame.
     * @param[in] sensor_ts The sensor timestamp to use for the lidar frame.
     * @param[in] lidar_frame The lidar frame to write.
     * @param[in] field_types Field types of the lidar frame to write.
     */
    void save(const ouster::sdk::osf::ts_t receive_ts, const ouster::sdk::osf::ts_t sensor_ts,
              const obj_type& lidar_frame,
              const ouster::sdk::core::LidarFrameFieldTypes& field_types);

    /**
     * Encode/serialize the object to the buffer of bytes.
     *
     * @param[in] lidar_frame The lidar frame to turn into a vector of bytes.
     * @param[in] field_types Field types of the lidar frame to write.
     * @return The byte vector representation of lidar_frame.
     */
    std::vector<uint8_t> make_msg(const obj_type& lidar_frame,
                                  const ouster::sdk::core::LidarFrameFieldTypes& field_types);

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
     * @param[in] fields List of fields to decode. All are decoded if not
     *                   provided. None are decoded with an empty array.
     * @return Pointer to the decoded object.
     */
    OUSTER_API_FUNCTION static std::unique_ptr<obj_type> decode_msg(
        const MessageRef& msg, const meta_type& meta, const MetadataStore& meta_provider,
        const nonstd::optional<std::vector<std::string>>& fields = {});

   public:
    /**
     * @param[in] writer The writer object to use to write messages out.
     * @param[in] sensor_meta_id The sensor to use.
     * @param[in] field_types LidarFrame fields specs, this argument is
     * optional.
     */
    OUSTER_API_FUNCTION
    LidarFrameStream(Writer& writer, const uint32_t sensor_meta_id,
                     const ouster::sdk::core::LidarFrameFieldTypes& field_types = {});

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

    /**
     * The internal flatbuffer id for the metadata.
     */
    uint32_t sensor_meta_id_{0};

    /**
     * The internal LidarInfo data.
     */
    ouster::sdk::core::LidarInfo sensor_info_;

    /**
     * The internal field_types data.
     */
    ouster::sdk::core::LidarFrameFieldTypes field_types_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
