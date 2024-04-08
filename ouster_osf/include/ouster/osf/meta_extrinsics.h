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
 * OSF type:
 *   ouster/v1/os_sensor/Extrinsics
 *
 * Flat Buffer Reference:
 *   fb/os_sensor/extrinsics.fbs
 */
class Extrinsics : public MetadataEntryHelper<Extrinsics> {
   public:
    /**
     * @param[in] extrinsics ///< The extrinsic matrix to store
     *                       ///< mat4d - 4x4 homogeneous transform
     * @param[in] ref_meta_id The flat buffer metadata(not sensor_info)
     *                        reference id
     * @param[in] name ///< Named id if needed, to support multiple extrinsics
     *                 ///< perobject (i.e. LidarSensor, or Gps) with name
     *                 ///< maybe usedto associate extrinsics to some external
     *                 ///< system of records or just name the source
     *                 ///< originator of the extrinsics information.
     */
    explicit Extrinsics(const mat4d& extrinsics, uint32_t ref_meta_id = 0,
                        const std::string& name = "");

    /**
     * Get the extrinsics matrix.
     *
     * @return The eigen extrinsics matrix.
     */
    const mat4d& extrinsics() const;

    /**
     * Get the extrinsics name.
     *
     * @return The extrinsics name.
     */
    const std::string& name() const;

    /**
     * Get the reference metadata id.
     *
     * @return The reference metadata id.
     */
    uint32_t ref_meta_id() const;

    /**
     * @copydoc MetadataEntry::buffer
     */
    std::vector<uint8_t> buffer() const final;

    /**
     * Create an Extrinsics object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The byte vector to construct an Extrinsics object from.
     * @return The new Extrinsics cast as a MetadataEntry
     */
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf);

    /**
     * Get the string representation for the Extrinsics object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the Extrinsics object.
     */
    std::string repr() const override;

   private:
    /**
     * The internal extrinsics array.
     *
     * Flat Buffer Reference:
     *   fb/os_sensor/extrinsics.fbs :: Extrinsics :: extrinsics
     */
    mat4d extrinsics_;

    /**
     * The internal flatbuffer metadata reference id.
     *
     * Flat Buffer Reference:
     *   fb/os_sensor/extrinsics.fbs :: Extrinsics :: ref_id
     */
    uint32_t ref_meta_id_;

    /**
     * The internal name for the extrinsics array.
     *
     * Flat Buffer Reference: fb/os_sensor/extrinsics.fbs :: Extrinsics :: name
     */
    std::string name_;
};

/** @defgroup OSFTraitsExtrinsics OSF Templated traits struct. */

/**
 * Templated struct for returning the OSF type string.
 *
 * @ingroup OSFTraitsExtrinsics
 */
template <>
struct MetadataTraits<Extrinsics> {
    /**
     * Return the OSF type string.
     *
     * @return The OSF type string "ouster/v1/os_sensor/Extrinsics".
     */
    static const std::string type() { return "ouster/v1/os_sensor/Extrinsics"; }
};

}  // namespace osf
}  // namespace ouster
