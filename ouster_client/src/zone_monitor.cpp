/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/zone_monitor.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <ios>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonschema/jsonschema.hpp>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "ouster/beam_config.h"
#include "ouster/compat_ops.h"
#include "ouster/impl/logging.h"
#include "ouster/impl/threadpool.h"
#include "ouster/json_tools.h"
#include "ouster/sha256.h"
#include "ouster/typedefs.h"
#include "ouster/types.h"
#include "ouster/vector_streambuf.h"
#include "ouster/zone.h"
#include "ouster/zrb.h"
#include "zip.h"

namespace core = ouster::sdk::core;
using core::BeamConfig;
using core::VectorStreamBuf;
using core::Zip;
using core::Zone;
using core::Zrb;

namespace ouster {
namespace sdk {
namespace core {

namespace {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays)
constexpr char METADATA_SCHEMA[] = R"({
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "label": {
      "type": "string",
      "maxLength": 64,
      "default": ""
    },
    "version": {
      "type": "object",
      "patternProperties": {
        ".*": {
          "type": "integer",
          "minimum": 1,
          "maximum": 65535
        }
      },
      "required": [
        "metadata",
        "file_naming"
      ],
      "additionalProperties": false
    },
    "power_on_live_ids": {
      "type": "array",
      "items": {
        "type": "integer",
        "minimum": 0,
        "maximum": 127
      },
      "minItems": 0,
      "maxItems": 16
    },
    "zones": {
      "type": "object",
      "patternProperties": {
        "^(12[0-7]|1[01][0-9]|[0-9]?[0-9])$": {
          "type": "object",
          "properties": {
            "label": {
              "type": "string",
              "maxLength": 64,
              "default": ""
            },
            "point_count": {
              "type": "integer",
              "minimum": 1,
              "maximum": 262143
            },
            "frame_count": {
              "type": "integer",
              "minimum": 1,
              "maximum": 65535
            },
            "mode": {
              "enum": [
                "VACANCY",
                "OCCUPANCY"
              ]
            },
            "stl": {
              "type": "object",
              "properties": {
                "file_name": {
                  "type": "string",
                  "pattern": "^([a-zA-Z0-9_\\-][a-zA-Z0-9_\\- ]*\\.(stl|STL))?$",
                  "default": ""
                },
                "coordinate_frame": {
                  "enum": [
                    "SENSOR",
                    "BODY"
                  ],
                  "default": "BODY"
                },
                "hash": {
                  "type": "string",
                  "pattern": "^[0-9a-fA-F]{64}$"
                }
              },
              "required": [
                "file_name"
              ],
              "additionalProperties": false
            },
            "zrb": {
              "type": "object",
              "properties": {
                "file_name": {
                  "type": "string",
                  "pattern": "^((12[0-7]|1[01][0-9]|[0-9]?[0-9])\\.(zrb|ZRB))?$",
                  "default": ""
                },
                "hash": {
                  "type": "string",
                  "pattern": "^[0-9a-fA-F]{64}$"
                }
              },
              "required": [
                "hash"
              ],
              "additionalProperties": false
            }
          },
          "required": [
            "point_count",
            "frame_count",
            "mode"
          ],
          "oneOf": [
            {
              "required": [
                "stl"
              ],
              "not": {
                "required": [
                  "zrb"
                ]
              }
            },
            {
              "required": [
                "zrb"
              ],
              "not": {
                "required": [
                  "stl"
                ]
              }
            },
            {
              "required": [
                "stl",
                "zrb"
              ]
            }
          ],
          "additionalProperties": false
        }
      },
      "additionalProperties": false,
      "minItems": 1,
      "maxItems": 128
    },
    "sensor_to_body_transform": {
      "type": "array",
      "items": {
        "type": "number"
      },
      "minItems": 16,
      "maxItems": 16
    }
  },
  "required": [
    "version",
    "power_on_live_ids",
    "zones",
    "sensor_to_body_transform"
  ],
  "additionalProperties": false
})";

const auto SCHEMA_JSON =
    jsoncons::json::parse(static_cast<const char*>(METADATA_SCHEMA));
const auto SCHEMA = jsoncons::jsonschema::make_schema(SCHEMA_JSON);
const auto VALIDATOR =
    jsoncons::jsonschema::json_validator<jsoncons::json>(SCHEMA);

bool parse_and_validate_zone_set_config_zip(
    const std::vector<uint8_t>& zip_bytes, ZoneSet& zone_set_config,
    ValidatorIssues& issues) {
    Zip zip(zip_bytes);
    jsoncons::json metadata =
        jsoncons::json::parse(zip.get_file_as_string("metadata.json"));
    VALIDATOR.validate(metadata);

    // Set label
    zone_set_config.label = metadata.get_value_or<std::string>("label", "");

    // Parse power on active ids
    zone_set_config.power_on_live_ids =
        metadata.at("power_on_live_ids").as<std::vector<unsigned int>>();

    // Parse sensor to body transform
    std::array<double, 16> sensor_to_body_transform_data =
        metadata.at("sensor_to_body_transform").as<std::array<double, 16>>();
    zone_set_config.sensor_to_body_transform =
        mat4d_from_array(sensor_to_body_transform_data);

    // At least one zone must be defined per the schema so it's okay to throw if
    // the zones key is missing
    for (auto& zone_json_it : metadata.at("zones").object_range()) {
        int zone_id = std::stoi(zone_json_it.key());
        auto& zone_json = zone_json_it.value();
        ouster::sdk::core::Zone zone{};

        // TODO[tws] encapsulate this in a function
        // Set zone trigger params
        zone.point_count = zone_json.at("point_count").as<int>();
        zone.frame_count = zone_json.at("frame_count").as<int>();
        Zone::ZoneMode zone_mode{};
        if (!Zone::string_to_zonemode(zone_json.at("mode").as<std::string>(),
                                      zone_mode)) {
            std::stringstream err;
            err << "Invalid zone mode string for zone " << zone_id;
            issues.critical.emplace_back(zone_json_it.key(), err.str());
            continue;
        }
        zone.mode = zone_mode;

        // Set label
        zone.label = zone_json.get_value_or<std::string>("label", "");

        // TODO[tws] encapsulate this in a function
        // Set zone STL
        if (zone_json.contains("stl")) {
            auto stl_json = zone_json.at("stl");
            auto file_name = stl_json.at("file_name").as<std::string>();
            auto coordinate_frame_str =
                stl_json.at("coordinate_frame").as<std::string>();

            zone.stl = ouster::sdk::core::Stl(zip.get_file(file_name));
            zone.stl->filename = file_name;
            if (!ouster::sdk::core::Stl::string_to_coordinate_frame(
                    coordinate_frame_str, zone.stl->coordinate_frame)) {
                std::stringstream err;
                err << "Invalid coordinate frame string for zone " << zone_id;
                issues.critical.emplace_back(zone_json_it.key(), err.str());
                continue;
            }
        }

        // TODO[tws] encapsulate this in a function
        // Set zone ZRB
        if (zone_json.contains("zrb")) {
            auto file_name =
                zone_json.at("zrb").at("file_name").as<std::string>();
            zone.zrb = ouster::sdk::core::Zrb(zip.get_file(file_name));
        }
        // TODO[tws] error if zone.stl or zone.zrb are not set?

        zone_set_config.zones.emplace(zone_id, zone);
    }

    return issues.critical.empty();
}

}  // namespace

void ZoneSet::check_invariants() const {
    if (sensor_to_body_transform == mat4d::Zero()) {
        throw std::logic_error(
            "ZoneSet: sensor_to_body_transform must be set.");
    }

    int64_t first_zrb_n_rows = -1;
    int64_t first_zrb_n_cols = -1;
    for (const auto& pair : zones) {
        uint32_t zone_id = pair.first;
        const Zone& zone = pair.second;
        try {
            zone.check_invariants();
        } catch (const std::exception& e) {
            std::stringstream err;
            err << "ZoneSet: Zone " << zone_id
                << " failed invariant check: " << e.what();
            throw std::logic_error(err.str());
        }

        if (!(zone.stl || zone.zrb)) {
            throw std::logic_error(
                "ZoneSet: all Zones must have either an STL or ZRB file.");
        }

        if (zone.zrb) {
            if (first_zrb_n_rows == -1 && first_zrb_n_cols == -1) {
                first_zrb_n_rows = zone.zrb->near_range_mm.rows();
                first_zrb_n_cols = zone.zrb->near_range_mm.cols();
            } else {
                if (zone.zrb->near_range_mm.rows() != first_zrb_n_rows ||
                    zone.zrb->near_range_mm.cols() != first_zrb_n_cols) {
                    throw std::logic_error(
                        "ZoneSet: all ZRBs must have the same resolution.");
                }
            }
        }
    }
}

ZoneSet::ZoneSet(
    const std::string&
        zip_path) {  // NOLINT(cppcoreguidelines-pro-type-member-init)
    auto zip_bytes = ouster::sdk::core::get_file_as_bytes(zip_path);
    // TODO[tws] dedupe with below
    ValidatorIssues issues;
    if (!parse_and_validate_zone_set_config_zip(zip_bytes, *this, issues)) {
        std::stringstream msg;
        msg << "Failed to create ZoneSet due to the following errors: "
            << std::endl
            << issues.to_string();

        throw std::invalid_argument(msg.str());
    }
}

ZoneSet::ZoneSet(
    const std::vector<uint8_t>&
        zip_bytes) {  // NOLINT(cppcoreguidelines-pro-type-member-init)
    ValidatorIssues issues;
    if (!parse_and_validate_zone_set_config_zip(zip_bytes, *this, issues)) {
        std::stringstream msg;
        msg << "Failed to create ZoneSet due to the following errors: "
            << std::endl
            << issues.to_string();

        throw std::invalid_argument(msg.str());
    }
}

void ZoneSet::render(const SensorInfo& sensor_info) {
    // Change the type to ZRB since after rendering all zones will have ZRBs
    float m_per_zmbin = ouster::sdk::core::DEFAULT_M_PER_ZMBIN;
    BeamConfig beam_config(
        sensor_info.w(), sensor_info.beam_altitude_angles,
        sensor_info.beam_azimuth_angles, sensor_info.beam_to_lidar_transform,
        sensor_info.lidar_to_sensor_transform, sensor_to_body_transform,
        m_per_zmbin, sensor_info.sn);

    Threadpool<void> pool;
    std::vector<std::future<void>> items(zones.size());

    int future_idx = 0;
    for (auto& iter : zones) {
        items[future_idx++] = pool.enqueue([&]() {
            auto& zone = iter.second;
            if (zone.zrb && !zone.stl) {
                logger().info(
                    "ZoneSet::render: rendering not required for zone {}, "
                    "which has a ZRB but no STL",
                    iter.first);
            } else {
                std::vector<uint8_t> buf;
                VectorStreamBuf sbuf(&buf);
                std::ostream out(&sbuf);

                // TODO[tws] maybe change zone.render to return optional zrb,
                // since the bool would be redundant
                if (zone.render(beam_config)) {
                    auto& zrb = zone.zrb.value();
                    zrb.save(out);
                    if (out.fail()) {
                        throw std::runtime_error(
                            "ZoneSet::render: failed to write rendered ZRB to "
                            "buffer for zone " +
                            std::to_string(iter.first) + ".");
                    }
                } else {
                    throw std::runtime_error("ZoneSet::render: zone " +
                                             std::to_string(iter.first) +
                                             " was out of sensor FOV.");
                }
            }

            zone.zrb->serial_number = sensor_info.sn;
        });
    }

    for (auto& item : items) {
        item.get();
    }
}

std::vector<uint8_t> ZoneSet::to_zip_blob(
    ZoneSetOutputFilter zone_set_output_filter) const {
    std::string metadata =
        to_json(zone_set_output_filter);  // calls check_invariants()
    VALIDATOR.validate(jsoncons::json::parse(metadata));

    Zip zip;
    zip.add_file("metadata.json", metadata);

    for (const auto& pair : zones) {
        std::string zone_id = std::to_string(pair.first);
        // TODO[tws] check that at least one of stl or zrb is present
        auto& zone = pair.second;
        auto& stl = zone.stl;
        auto& zrb = zone.zrb;
        if (stl && zone_set_output_filter != ZoneSetOutputFilter::ZRB) {
            auto file_name =
                stl->filename.empty() ? zone_id + ".stl" : stl->filename;
            std::vector<uint8_t> blob = stl->blob();
            if (blob.empty()) {
                throw std::runtime_error("Zone " + zone_id +
                                         " has an empty stl blob");
            }
            zip.add_file(file_name, blob);
        }

        if (zrb && zone_set_output_filter != ZoneSetOutputFilter::STL) {
            auto file_name = zone_id + ".zrb";
            std::vector<uint8_t> blob = zrb->blob();
            if (blob.empty()) {
                throw std::runtime_error("Zone " + zone_id +
                                         " has an empty zrb blob");
            }
            zip.add_file(file_name, blob);
        }
    }

    return zip.to_bytes();
}

void ZoneSet::save(const std::string& zip_path,
                   ZoneSetOutputFilter zone_set_output_filter) const {
    std::vector<uint8_t> zip =
        to_zip_blob(zone_set_output_filter);  // calls check_invariants()

    std::ofstream file(zip_path, std::ios::binary | std::ios::out);
    file.write(reinterpret_cast<char*>(zip.data()),
               static_cast<std::streamsize>(zip.size()));
}

std::string ZoneSet::to_json(ZoneSetOutputFilter zone_set_output_filter) const {
    check_invariants();
    jsoncons::json result;
    result["label"] = label;
    result["version"]["metadata"] = 1;
    result["version"]["file_naming"] = 1;
    result["power_on_live_ids"] = power_on_live_ids;
    result["zones"] = jsoncons::json::object();

    for (const auto& pair : zones) {
        std::string id = std::to_string(pair.first);
        const Zone& zone = pair.second;

        result["zones"][id]["point_count"] = zone.point_count;
        result["zones"][id]["frame_count"] = zone.frame_count;
        result["zones"][id]["mode"] = to_string(zone.mode);
        result["zones"][id]["label"] = zone.label;

        // TODO[tws] dedupe stl/zrb?
        auto& stl = zone.stl;
        auto& zrb = zone.zrb;
        if (stl && zone_set_output_filter != ZoneSetOutputFilter::ZRB) {
            auto file_name =
                stl->filename.empty() ? id + ".stl" : stl->filename;
            result["zones"][id]["stl"]["file_name"] = file_name;
            result["zones"][id]["stl"]["coordinate_frame"] =
                to_string(stl->coordinate_frame);
            result["zones"][id]["stl"]["hash"] = stl->hash().str();
        }
        if (zrb && zone_set_output_filter != ZoneSetOutputFilter::STL) {
            auto file_name = id + ".zrb";
            result["zones"][id]["zrb"]["file_name"] = file_name;
            result["zones"][id]["zrb"]["hash"] = zrb->hash().str();
        }
    }

    result["sensor_to_body_transform"] =
        mat4d_to_array(sensor_to_body_transform);

    std::string out;
    result.dump_pretty(out);
    return out;
}

bool operator==(const ZoneSet& lhs, const ZoneSet& rhs) {
    // Note that sensor_info_ptr_ is not compared
    return lhs.zones == rhs.zones &&
           lhs.power_on_live_ids == rhs.power_on_live_ids &&
           lhs.sensor_to_body_transform == rhs.sensor_to_body_transform;
}

bool operator!=(const ZoneSet& lhs, const ZoneSet& rhs) {
    return !(lhs == rhs);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
