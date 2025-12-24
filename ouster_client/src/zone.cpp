#include "ouster/zone.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "ouster/beam_config.h"
#include "ouster/ray.h"
#include "ouster/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {

void Zone::check_invariants() const {
    if (point_count == 0) {
        throw std::logic_error("Zone: point_count must be in [1, 262143]");
    }
    if (frame_count == 0) {
        throw std::logic_error("Zone: frame_count must be in [1, 65535]");
    }
    if (!stl && !zrb) {
        throw std::logic_error("Zone: must have either STL or ZRB");
    }
    if (mode != ZoneMode::OCCUPANCY && mode != ZoneMode::VACANCY) {
        throw std::logic_error("Zone: mode must be OCCUPANCY or VACANCY");
    }
    if (stl) {
        if (stl->blob().empty()) {
            throw std::logic_error("Zone: STL blob cannot be empty");
        }
        if (stl->coordinate_frame == Stl::CoordinateFrame::NONE) {
            throw std::logic_error(
                "Zone: STL coordinate frame must be BODY or SENSOR");
        }
    }
    if (zrb) {
        if ((zrb->far_range_mm != 0).count() < point_count) {
            throw std::logic_error(
                "Zone: ZRB far range image has fewer nonzero pixels than "
                "point_count");
        }
    }
}

// Zone
Zone::Zone() : stl{}, zrb{} {}

bool Zone::string_to_zonemode(const std::string& str, Zone::ZoneMode& mode) {
    // NOTE - no "NONE" because that is the default value and should not be set
    if (str == "OCCUPANCY") {
        mode = Zone::ZoneMode::OCCUPANCY;
    } else if (str == "VACANCY") {
        mode = Zone::ZoneMode::VACANCY;
    } else {
        return false;
    }
    return true;
}

bool Zone::render(const BeamConfig& config) {
    check_invariants();
    if (!stl) {
        std::cerr << "Zone: Error rendering zone, no STL provided.\n";
        return false;
    }
    auto mesh = stl->to_mesh();
    if (mesh.triangles().empty()) {
        std::cerr << "Zone: Error rendering zone, STL has no triangles.\n";
        return false;
    }

    zrb =
        Zrb(config.n_rows, config.n_cols, config.m_per_zmbin,
            config.serial_number, stl->hash(), config.beam_to_lidar_transform,
            config.lidar_to_sensor_transform, config.sensor_to_body_transform);

    // Pre-select LUT references to avoid branch in inner loop
    const auto& beam_offsets =
        (stl->coordinate_frame == Stl::CoordinateFrame::BODY)
            ? config.lut->offset
            : config.lut_no_sensor_to_body_transform->offset;
    const auto& beam_directions =
        (stl->coordinate_frame == Stl::CoordinateFrame::BODY)
            ? config.lut->direction
            : config.lut_no_sensor_to_body_transform->direction;

    uint32_t pixels_with_intersections = 0;
    for (uint32_t row = 0; row < config.n_rows; row++) {
        // Left side is az+, middle is az=0, right is az-
        for (uint32_t col = 0; col < config.n_cols; col++) {
            Ray beam;
            auto pixel_offset = (row * config.n_cols) + col;
            beam.offset = beam_offsets.row(pixel_offset).cast<float>();
            beam.direction =
                beam_directions.row(pixel_offset).cast<float>() * 1000.0f;
            BoundsF bounds_meters;
            if (mesh.closest_and_farthest_intersections(beam, bounds_meters)) {
                pixels_with_intersections++;
            } else {
                bounds_meters = {0.f, 0.f};
            }
            // NOLINTBEGIN(cppcoreguidelines-narrowing-conversions)
            double near_mm = std::round(bounds_meters.first * 1000.0);
            double far_mm = std::round(bounds_meters.second * 1000.0);
            if (near_mm > UINT32_MAX || far_mm > UINT32_MAX) {
                throw std::logic_error("Zone::render: range overflow");
            }
            zrb->near_range_mm(row, col) = near_mm;
            zrb->far_range_mm(row, col) = far_mm;
            // NOLINTEND(cppcoreguidelines-narrowing-conversions)
        }
    }

    if (pixels_with_intersections && pixels_with_intersections < point_count) {
        throw std::logic_error("Zone: area of rendered zone (" +
                               std::to_string(pixels_with_intersections) +
                               ") is smaller than "
                               "point_count (" +
                               std::to_string(point_count) +
                               ") specified in zone.");
    }
    return pixels_with_intersections > 0;
}

std::string to_string(Zone::ZoneMode zone_mode) {
    switch (zone_mode) {
        case Zone::ZoneMode::NONE:
            return "NONE";
        case Zone::ZoneMode::OCCUPANCY:
            return "OCCUPANCY";
        case Zone::ZoneMode::VACANCY:
            return "VACANCY";
        default:
            break;
    };
    return "UNKNOWN";
}

bool operator==(const Zone& lhs, const Zone& rhs) {
    return lhs.point_count == rhs.point_count &&
           lhs.frame_count == rhs.frame_count && lhs.mode == rhs.mode &&
           lhs.stl == rhs.stl && lhs.zrb == rhs.zrb;
}

bool operator!=(const Zone& lhs, const Zone& rhs) { return !(lhs == rhs); }

}  // namespace core
}  // namespace sdk
}  // namespace ouster
