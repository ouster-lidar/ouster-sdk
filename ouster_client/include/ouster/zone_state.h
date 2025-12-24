/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
// TODO: some of these values (e.g. error flags, trigger type etc) have to be
//       clarified with firmware once sensor implementation gets stable, and
//       we should add enums and/or documentation to explain them -- Tim T.
#pragma pack(push, 1)

/**
 * @brief Represents the state of a zone.
 *
 * Contains metadata for each zone, including trigger status,
 * error flags, and range information.
 */
struct OUSTER_API_CLASS ZoneState {
    // NOTE field descriptions are ripped straight from the requirements doc.
    uint8_t live{0};            ///< 1 for all live zones, regardless of zone
                                ///< alert trigger state (empty slots will be 0)
    uint8_t id{0};              ///< Index of ROM config file.
    uint8_t error_flags{0};     ///< Error flags: missing data, low confidence
                                ///< pixels, zone obscured
    uint8_t trigger_type{0};    ///< Occupancy/Non-Occ/etc.
    uint8_t trigger_status{0};  ///< Trigger status, currently:
                                ///< 0x0 Deasserted, 0x1 Asserted.
    uint32_t triggered_frames{0};  ///< Saturating count of frames triggered
                                   ///< consecutively. Resets on desassertion.
    uint32_t count{0};  ///< Saturating latest count of points in the zone.
                        ///< Updates live, regardless of trigger status.
    uint32_t occlusion_count{
        0};  ///< Count of points below minimum range in the zone.
    uint32_t invalid_count{0};  ///< Count of invalid points in the zone.
    uint32_t max_count{0};      ///< Maximum count of points in the zone.

    uint32_t min_range{
        0};  ///< Minimum range of points in the zone in millimeters.
    uint32_t max_range{
        0};  ///< Maximum range of points in the zone in millimeters.
    uint32_t mean_range{0};  ///< Mean of points in the zone in millimeters.
};
#pragma pack(pop)

}  // namespace core
}  // namespace sdk
}  // namespace ouster
