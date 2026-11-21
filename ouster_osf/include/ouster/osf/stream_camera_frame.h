/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file stream_camera_frame.h
 * @brief Stream of CameraFrame
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

#include "ouster/core/camera_frame.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Metadata entry for CameraFrameStream. Holds a back-reference to the footer
 * sensor metadata entry (a CameraSensor entry, which stores this camera's
 * CameraInfo) describing this camera - mirroring LidarFrameStreamMeta.
 *
 * OSF type:
 *   ouster/v1/os_sensor/CameraFrameStream
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/camera_frame_stream.fbs
 */
class OUSTER_API_CLASS CameraFrameStreamMeta : public MetadataEntryHelper<CameraFrameStreamMeta> {
   public:
    /**
     * @param[in] sensor_meta_id Reference to the CameraSensor metadata entry
     *                           holding this camera's CameraInfo. Defaults to 0
     *                           for backward compatibility with entries that
     *                           carry no back-reference.
     */
    OUSTER_API_FUNCTION
    explicit CameraFrameStreamMeta(uint32_t sensor_meta_id = 0);

    /**
     * Return the sensor meta id (the footer CameraSensor entry describing this
     * camera).
     *
     * @return The sensor meta id.
     */
    OUSTER_API_FUNCTION
    uint32_t sensor_meta_id() const;

    /** @copydoc MetadataEntry::buffer */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a CameraFrameStreamMeta object from a byte array.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new CameraFrameStreamMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const OsfBuffer& buf);

    /** @copydoc MetadataEntry::repr */
    OUSTER_API_FUNCTION
    std::string repr() const override;

   private:
    /// Reference to the footer CameraSensor entry holding this camera's info.
    uint32_t sensor_meta_id_{0};
};

/**
 * Templated struct for returning the OSF type string.
 */
template <>
struct OUSTER_API_CLASS MetadataTraits<CameraFrameStreamMeta> {
    /** @return The OSF type string "ouster/v1/os_sensor/CameraFrameStream". */
    OUSTER_API_FUNCTION
    static const std::string type() {
        return "ouster/v1/os_sensor/CameraFrameStream";
    }
};

/**
 * CameraFrameStream that encodes CameraFrame objects into messages.
 *
 * Object type: ouster::sdk::core::CameraFrame
 * Meta type: CameraFrameStreamMeta
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/camera_frame_stream.fbs
 */
class OUSTER_API_CLASS CameraFrameStream
    : public MessageStream<CameraFrameStreamMeta, ouster::sdk::core::CameraFrame> {
   protected:
    friend class Writer;
    friend class MessageRef;

    /**
     * Saves the camera frame to the writer using make_msg() serialization.
     *
     * @param[in] receive_ts The receive timestamp to use for the camera frame.
     * @param[in] sensor_ts The sensor timestamp to use for the camera frame.
     * @param[in] camera_frame The camera frame to write.
     */
    void save(ouster::sdk::osf::ts_t receive_ts, ouster::sdk::osf::ts_t sensor_ts,
              const obj_type& camera_frame);

    /**
     * Encode/serialize the camera frame to the buffer of bytes.
     *
     * @param[in] camera_frame The camera frame to serialize.
     * @return The byte vector representation of camera_frame.
     */
    std::vector<uint8_t> make_msg(const obj_type& camera_frame);

    /**
     * Decode/deserialize the camera frame from a bytes buffer.
     *
     * @param[in] msg The MessageRef to decode.
     * @param[in] meta The concrete metadata type to use for decoding.
     * @param[in] meta_provider Metadata store used to reconstruct references.
     * @param[in] fields List of fields to decode. All are decoded if not
     *                   provided. None are decoded with an empty array.
     * @throws std::runtime_error if a requested field is not in the frame.
     * @return Pointer to the decoded CameraFrame.
     */
    OUSTER_API_FUNCTION static std::unique_ptr<obj_type> decode_msg(
        const MessageRef& msg, const meta_type& meta, const MetadataStore& meta_provider,
        const nonstd::optional<std::vector<std::string>>& fields = {});

   public:
    /**
     * @param[in] writer The writer object to use to write messages out.
     * @param[in] sensor_meta_id Reference to the footer CameraSensor metadata
     *                           entry holding this camera's CameraInfo.
     * @param[in] fields_to_write The field names to actually save from each
     *                            camera frame. If empty, every field on the
     *                            frame is written (mirrors add_sensor() for
     *                            lidars). Missing fields are skipped.
     */
    OUSTER_API_FUNCTION
    explicit CameraFrameStream(Writer& writer, uint32_t sensor_meta_id = 0,
                               std::vector<std::string> fields_to_write = {});

    /**
     * Return the concrete metadata type.
     *
     * @return The concrete metadata type.
     */
    OUSTER_API_FUNCTION
    const meta_type& meta() const {
        return meta_;
    }

   private:
    // Reference is intentional: the stream does not own the writer and must
    // outlive it; converting to a pointer would change ownership semantics.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    Writer& writer_;
    meta_type meta_;
    // Field names to write from each frame; empty means "all fields".
    std::vector<std::string> desired_fields_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
