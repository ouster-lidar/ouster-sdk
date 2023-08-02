/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file stream_lidar_scan.h
 * @brief Stream of LidarScan
 *
 */
#pragma once

#include "ouster/osf/basics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

// Cast `ls_src` LidarScan to a subset of fields with possible different
// underlying ChanFieldTypes.
// @return a copy of `ls_src` with transformed fields
LidarScan slice_with_cast(const LidarScan& ls_src,
                          const LidarScanFieldTypes& field_types);

// Zeros field
struct zero_field {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest) {
        field_dest.setZero();
    }
};

/**
 * Metadata entry for LidarScanStream to store reference to a sensor and
 * field_types
 *
 * @verbatim
 * Fields:
 *   sensor_meta_id: metadata_ref - reference to LidarSensor metadata that
 *                                  describes the sensor configuration.
 *   field_types: LidarScan fields specs
 *
 * OSF type:
 *   ouster/v1/os_sensor/LidarScanStream
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_scan_stream.fbs
 * @endverbatim
 *
 */
class LidarScanStreamMeta : public MetadataEntryHelper<LidarScanStreamMeta> {
   public:
    LidarScanStreamMeta(const uint32_t sensor_meta_id,
                        const LidarScanFieldTypes field_types = {})
        : sensor_meta_id_{sensor_meta_id},
          field_types_{field_types.begin(), field_types.end()} {}

    uint32_t sensor_meta_id() const { return sensor_meta_id_; }

    const LidarScanFieldTypes& field_types() const { return field_types_; }

    // Simplified with MetadataEntryHelper<LidarScanStreamMeta>: type()+clone()
    // std::string type() const override;
    // std::unique_ptr<MetadataEntry> clone() const override;

    std::vector<uint8_t> buffer() const final;

    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    std::string repr() const override;

   private:
    uint32_t sensor_meta_id_{0};
    LidarScanFieldTypes field_types_;
};

template <>
struct MetadataTraits<LidarScanStreamMeta> {
    static const std::string type() {
        return "ouster/v1/os_sensor/LidarScanStream";
    }
};

/**
 * LidarScanStream that encodes LidarScan objects into the messages.
 *
 * @verbatim
 * Object type: ouster::sensor::LidarScan
 * Meta type: LidarScanStreamMeta (sensor_meta_id, field_types)
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/lidar_scan_stream.fbs
 * @endverbatim
 *
 */
class LidarScanStream : public MessageStream<LidarScanStreamMeta, LidarScan> {
   public:
    LidarScanStream(Writer& writer, const uint32_t sensor_meta_id,
                    const LidarScanFieldTypes& field_types = {});

    /**
     * Saves the object to the writer applying the coding/serizlization
     * algorithm defined in make_msg() function. The function is the same for
     * all streams types ...
     *
     * @todo [pb]: Probably should be abstracted/extracted from all streams
     * we also might want to have the corresponding function to read back
     * sequentially from Stream that doesn't seem like fit into this model...
     */
    void save(const ouster::osf::ts_t ts, const obj_type& lidar_scan);

    /** Encode/serialize the object to the buffer of bytes */
    std::vector<uint8_t> make_msg(const obj_type& lidar_scan);

    /**
     * Decode/deserialize the object from bytes buffer using the concrete
     * metadata type for the stream.
     * metadata_provider is used to reconstruct any references to other
     * metadata entries dependencies (like sensor_meta_id)
     */
    static std::unique_ptr<obj_type> decode_msg(
        const std::vector<uint8_t>& buf, const meta_type& meta,
        const MetadataStore& meta_provider);

    const meta_type& meta() const { return meta_; }

   private:
    Writer& writer_;

    meta_type meta_;

    uint32_t stream_meta_id_{0};

    uint32_t sensor_meta_id_{0};

    sensor::sensor_info sensor_info_;
};

}  // namespace osf
}  // namespace ouster
