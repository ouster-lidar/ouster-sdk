/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file meta_extrinsics.h
 * @brief Metadata entry Extrinsics
 *
 */
#pragma once

#include <iostream>
#include <memory>

#include "ouster/osf/metadata.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {

/**
 * Metadata entry to store sensor Extrinsics.
 *
 * @verbatim
 * Fields:
 *   extrinsics: mat4d - 4x4 homogeneous transform
 *   ref_meta_id: uint32_t - reference to other metadata entry, typically
 *                           LidarSensor
 *   name: string - named id if needed, to support multiple extrinsics per
 *                  object (i.e. LidarSensor, or Gps) with name maybe used
 *                  to associate extrinsics to some external system of
 *                  records or just name the source originator of the
 *                  extrinsics information.
 *
 * OSF type:
 *   ouster/v1/os_sensor/Extrinsics
 *
 * Flatbuffer definition file:
 *   fb/os_sensor/extrinsics.fbs
 * @endverbatim
 *
 */
class Extrinsics : public MetadataEntryHelper<Extrinsics> {
   public:
    explicit Extrinsics(const mat4d& extrinsics, uint32_t ref_meta_id = 0,
                        const std::string& name = "")
        : extrinsics_(extrinsics), ref_meta_id_{ref_meta_id}, name_{name} {}
    const mat4d& extrinsics() const { return extrinsics_; }
    const std::string& name() const { return name_; }
    uint32_t ref_meta_id() const { return ref_meta_id_; }

    std::vector<uint8_t> buffer() const final;

    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    std::string repr() const override;

   private:
    mat4d extrinsics_;
    uint32_t ref_meta_id_;
    std::string name_;
};

template <>
struct MetadataTraits<Extrinsics> {
    static const std::string type() { return "ouster/v1/os_sensor/Extrinsics"; }
};

}  // namespace osf
}  // namespace ouster