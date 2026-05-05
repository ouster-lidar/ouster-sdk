/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file zone_monitor.h
 * @brief type definitions for zone monitor
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/typedefs.h"
#include "ouster/zone.h"

namespace ouster {
namespace sdk {
namespace core {

class SensorInfo;

/**
 * @brief Specifies which files to include in a ZoneSet zip archive.
 */
enum class ZoneSetOutputFilter {
    STL,         ///< Include only STL files in the zip archive
    ZRB,         ///< Include only ZRB files in the zip archive
    STL_AND_ZRB  ///< Include STL and ZRB files in the zip archive
};

/**
 * @brief Configuration for a complete set of zones to be monitored.
 *
 * Represents a collection of zones, including active zones and
 * sensor extrinsics. Provides serialization and deserialization utilities
 * to/from zip archives (files or memory), as well as conversion to JSON.
 */
struct OUSTER_API_CLASS ZoneSet {
    /**
     * Map of zones available for monitoring.
     * Max 128 zones.
     */
    std::unordered_map<uint32_t, ouster::sdk::core::Zone> zones{};

    std::vector<uint32_t> power_on_live_ids{};  ///< Default active zones
                                                ///< after configuration.

    mat4d sensor_to_body_transform =
        mat4d::Zero();  ///< sensor to body transform

    std::string label;  ///< Optional label for the ZoneSet

    /**
     * Renders all zones in this ZoneSet.
     * @note Do not use this method if you intend to use STL-based zones on the
     * sensor, as the sensor will render them as needed. This updates each
     * Zone's ZRB and sets the ZoneSet's type to ZRB.
     * @param[in] sensor_info SensorInfo containing beam configuration and
     *            sensor to body transform for rendering.
     */
    OUSTER_API_FUNCTION
    void render(const SensorInfo& sensor_info);

    /**
     * Instantiates ZoneSet from sensor info and zone set type.
     */
    OUSTER_API_FUNCTION
    ZoneSet() = default;

    /**
     * Instantiates ZoneSet from a zip file.
     *
     * @throws std::invalid_argument zip archive does not exist or contains
     *         invalid configuration.
     *
     * @param[in] zip_path path to the zip file
     */
    OUSTER_API_FUNCTION
    explicit ZoneSet(const std::string& zip_path);

    /**
     * Instantiates ZoneSet from a zip archive in memory.
     *
     * @throws std::invalid_argument zip archive is malformed or contains
     *         invalid configuration.
     *
     * @param[in] zip_bytes binary blob containing zip archive
     */
    OUSTER_API_FUNCTION
    explicit ZoneSet(const std::vector<uint8_t>& zip_bytes);

    /**
     * Serializes this ZoneSet to a zip file
     *
     * @param[in] zip_path the file path to write the .zip archive to.
     * @param[in] zone_set_output_filter specifies which files to include in the
     * resulting zip archive.
     *
     * @throws std::runtime_error on failing to produce zip archive or missing
     *         critical data.
     */
    OUSTER_API_FUNCTION
    void save(const std::string& zip_path,
              ZoneSetOutputFilter zone_set_output_filter) const;

    /**
     * Serializes this ZoneSet as a zip blob
     *
     * @param[in] zone_set_output_filter specifies which files to include in the
     * resulting zip archive.
     * @throws std::runtime_error on failing to produce zip archive or missing
     *         critical data.
     *
     * @return binary blob containing zip archive
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> to_zip_blob(
        ZoneSetOutputFilter zone_set_output_filter) const;

    /**
     * Retrieves zone monitor metadata as json.
     *
     * @param[in] zone_set_output_filter specifies which files to include in the
     * JSON output.
     * @return json string containing zone monitor metadata
     */
    OUSTER_API_FUNCTION std::string to_json(
        ZoneSetOutputFilter zone_set_output_filter) const;

    /**
     * Equality operator for ZoneSet.
     *
     * @param[in] lhs Left-hand side ZoneSet to compare.
     * @param[in] rhs Right-hand side ZoneSet to compare.
     * @return true if both ZoneSets are equal, false otherwise.
     */
    friend OUSTER_API_FUNCTION bool operator==(const ZoneSet& lhs,
                                               const ZoneSet& rhs);

    /**
     * Inequality operator for ZoneSet.
     *
     * @param[in] lhs Left-hand side ZoneSet to compare.
     * @param[in] rhs Right-hand side ZoneSet to compare.
     * @return true if both ZoneSets are not equal, false otherwise.
     */
    friend OUSTER_API_FUNCTION bool operator!=(const ZoneSet& lhs,
                                               const ZoneSet& rhs);

   private:
    void check_invariants() const;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
