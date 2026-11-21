/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <string>

#include "nonstd/optional.hpp"
#include "ouster/core/beam_config.h"
#include "ouster/core/stl.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zrb.h"

namespace ouster {
namespace sdk {
namespace core {

/// Output to trigger on the sensor when a zone is triggered.
enum class ZoneTriggerOutput {
    NONE = 0,   ///< No output on trigger
    OSSD_DUAL,  ///< Output OSSD on trigger
    GPIO_0,     ///< Output on GPIO 0 on trigger
    GPIO_1      ///< Output on GPIO 1 on trigger
};

/**
 * This class represents a physical area defined by a Mesh, along with
 * metadata and rendered products associated with that area.
 */
class OUSTER_API_CLASS Zone {
   public:
    /// Specified by FW
    static constexpr uint32_t MAX_TRIANGLES = 2048;

    /// Optional STL defining the zone's 3D geometry
    nonstd::optional<Stl> stl;
    /// Optional ZRB containing rendered range images for the zone
    nonstd::optional<Zrb> zrb;

    // Trigger Config
    uint32_t point_count{};              ///< Minimum area (in points) to trigger an alert
    uint32_t frame_count{};              ///< Minimum time (in frames) to trigger an alert
    ZoneTriggerOutput trigger_output{};  ///< Output trigger type when this zone is triggered

    /// Zone operation mode
    enum class ZoneMode : uint8_t {
        NONE = 0,       ///< No mode specified
        OCCUPANCY = 1,  ///< Occupancy mode specifies the zone shall be
                        ///< triggered when an object enters the zone
        VACANCY = 2     ///< Vacancy mode specifies the zone shall be triggered
                        ///< when an object leaves the zone
    };

    ZoneMode mode{ZoneMode::NONE};  ///< The current operation mode of the zone

    /**
     * Converts to a ZoneMode enum value from its corresponding string
     * representation.
     * @param[in] str The string to convert.
     * @param[out] mode The resulting ZoneMode enum value.
     * @return true if the conversion was successful, false otherwise.
     */
    OUSTER_API_FUNCTION
    static bool string_to_zonemode(const std::string& str, Zone::ZoneMode& mode);

    std::string label;  ///< Optional label for the ZoneSet

    OUSTER_API_FUNCTION
    Zone();  ///< Default constructor

    /**
     * Renders the zone using the provided beam configuration. Zone STLs
     * are limited to MAX_TRIANGLES triangles to ensure rendering can be
     * performed in a reasonable amount of time. If the STL contains more than
     * MAX_TRIANGLES triangles, the render function will return false and the
     * zone will not be rendered.
     * @param[in] beam_config The beam configuration to use for rendering.
     * @return true if the rendering was successful.
     */
    OUSTER_API_FUNCTION
    bool render(const BeamConfig& beam_config);

    OUSTER_API_IGNORE
    void check_invariants() const;  ///< Checks the invariants of the Zone
};

/**
 * Converts a ZoneTriggerOutput enum value to its corresponding string representation.
 * @param[in] zone_trigger The ZoneTriggerOutput enum value to convert.
 * @return The string representation of the ZoneTriggerOutput.
 */
OUSTER_API_FUNCTION
std::string to_string(ZoneTriggerOutput zone_trigger);

/**
 * Converts to a ZoneTriggerOutput enum value from its corresponding string
 * representation.
 * @param[in] str The string to convert.
 * @param[out] mode The resulting ZoneTriggerOutput enum value.
 * @return true if the conversion was successful, false otherwise.
 */
OUSTER_API_FUNCTION
bool string_to_zonetriggeroutput(const std::string& str, ZoneTriggerOutput& mode);

/**
 * Converts a ZoneMode enum value to its corresponding string representation.
 * @param[in] zone_mode The ZoneMode enum value to convert.
 * @return The string representation of the ZoneMode.
 */
OUSTER_API_FUNCTION
std::string to_string(Zone::ZoneMode zone_mode);

/**
 * Compares two Zone objects for equality.
 * @param[in] lhs The first Zone object.
 * @param[in] rhs The second Zone object.
 * @return true if the two Zone objects are equal, false otherwise.
 */
OUSTER_API_FUNCTION
bool operator==(const Zone& lhs, const Zone& rhs);

/**
 * Compares two Zone objects for inequality.
 * @param[in] lhs The first Zone object.
 * @param[in] rhs The second Zone object.
 * @return true if the two Zone objects are not equal, false otherwise.
 */
OUSTER_API_FUNCTION
bool operator!=(const Zone& lhs, const Zone& rhs);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
