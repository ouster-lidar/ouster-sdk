/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/open_source.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <functional>
#include <ios>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <map>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

#include "ouster/scan_source_utils.h"

using ouster::sdk::core::IoType;

namespace ouster {
namespace sdk {

namespace impl {
std::map<core::IoType, ScanBuilderT>& get_builders() {
    static std::map<core::IoType, ScanBuilderT> builders;
    return builders;
}

std::map<core::IoType, PacketBuilderT>& get_packet_builders() {
    static std::map<core::IoType, PacketBuilderT> builders;
    return builders;
}
}  // namespace impl

static std::string read_file(const std::string& file_name) {
    std::ifstream ifs(file_name.c_str(),
                      std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type file_size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(file_size);
    ifs.read(bytes.data(), file_size);

    return std::string(bytes.data(), file_size);
}

namespace core {
void populate_extrinsics(
    std::string extrinsics_file,
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> extrinsics,
    std::vector<std::shared_ptr<SensorInfo>>& sensor_infos) {
    // try and load from file if set
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> final_extrinsics;
    if (!extrinsics_file.empty()) {
        std::string json = read_file(extrinsics_file);
        auto data = jsoncons::json::parse(json);
        size_t extrinsic_count = 0;
        if (data.contains("transforms")) {
            for (const auto& obj : data["transforms"].array_range()) {
                auto serial = obj["source_frame"].as<std::string>();
                auto x = obj["p_x"].as<double>();
                auto y = obj["p_y"].as<double>();
                auto z = obj["p_z"].as<double>();
                auto qx = obj["q_x"].as<double>();
                auto qy = obj["q_y"].as<double>();
                auto qz = obj["q_z"].as<double>();
                auto qw = obj["q_w"].as<double>();

                // check if it matches any of our SNs, if so apply it
                Eigen::Matrix<double, 4, 4, Eigen::RowMajor> mat =
                    Eigen::Matrix<double, 4, 4, Eigen::RowMajor>::Identity();

                auto rot = Eigen::Quaternion<double>(qw, qx, qy, qz)
                               .normalized()
                               .toRotationMatrix();
                mat.topLeftCorner(3, 3) = rot;
                mat(0, 3) = x;
                mat(1, 3) = y;
                mat(2, 3) = z;
                for (auto& info : sensor_infos) {
                    if (std::to_string(info->sn) == serial) {
                        info->extrinsic = mat;
                        extrinsic_count++;
                        break;
                    }
                }
            }
        }
        if (extrinsic_count != sensor_infos.size()) {
            printf(
                "warning: loaded extrinsics from '%s' doesn't match to the "
                "count of"
                " sensors. provided %lu, expected: %lu",
                extrinsics_file.c_str(), extrinsic_count, sensor_infos.size());
        }
    }

    if (!extrinsics.empty()) {
        for (size_t i = 0; i < std::min(extrinsics.size(), sensor_infos.size());
             i++) {
            sensor_infos[i]->extrinsic = extrinsics[i];
        }

        if (extrinsics.size() < sensor_infos.size()) {
            printf(
                "warning: loaded extrinsics doesn't match to the count of"
                " sensors. provided %lu, expected: %lu",
                extrinsics.size(), sensor_infos.size());
        }
    }
}

std::vector<std::vector<FieldType>> resolve_field_types(
    const std::vector<std::shared_ptr<SensorInfo>>& metadata, bool raw_headers,
    bool raw_fields,
    const nonstd::optional<std::vector<std::string>>& field_names) {
    std::vector<std::vector<FieldType>> all_ftypes;
    for (const auto& meta : metadata) {
        auto ftypes = get_field_types(*meta);
        bool dual = (meta->num_returns() == 2);

        if (raw_fields) {
            ftypes.emplace_back(ChanField::RAW32_WORD1, ChanFieldType::UINT32);
            if (meta->format.udp_profile_lidar ==
                UDPProfileLidar::RNG15_RFL8_NIR8_DUAL) {
                ftypes.emplace_back(ChanField::RAW32_WORD2,
                                    ChanFieldType::UINT32);
            } else if (meta->format.udp_profile_lidar !=
                       UDPProfileLidar::RNG15_RFL8_NIR8) {
                ftypes.emplace_back(ChanField::RAW32_WORD2,
                                    ChanFieldType::UINT32);
                ftypes.emplace_back(ChanField::RAW32_WORD3,
                                    ChanFieldType::UINT32);
                if (dual) {
                    ftypes.emplace_back(ChanField::RAW32_WORD4,
                                        ChanFieldType::UINT32);
                }
            }
        }

        if (raw_headers) {
            auto packet_format = PacketFormat(*meta);
            auto h = packet_format.pixels_per_column;
            auto raw_headers_space =
                (packet_format.packet_header_size +
                 packet_format.packet_footer_size +
                 packet_format.col_header_size + packet_format.col_footer_size);
            auto size_bytes = raw_headers_space / h;
            auto dtype = ChanFieldType::UINT8;
            if (size_bytes >= 2) {
                dtype = ChanFieldType::UINT32;
            } else if (size_bytes >= 1) {
                dtype = ChanFieldType::UINT16;
            }
            ftypes.emplace_back(ChanField::RAW_HEADERS, dtype);
        }

        // Finally apply field names
        if (field_names) {
            std::vector<FieldType> real_field_types;
            // Only parse fields we asked for in field_names
            for (const auto& field_type : ftypes) {
                for (const auto& name : field_names.value()) {
                    if (field_type.name == name) {
                        real_field_types.push_back(field_type);
                        break;
                    }
                }
            }

            // Error if we cant find a field with the requested name
            for (const auto& name : field_names.value()) {
                bool found = false;
                for (const auto& field_type : ftypes) {
                    if (field_type.name == name) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    throw std::runtime_error(
                        "Requested field '" + name +
                        "' does exist not in packet format.");  // todo print
                                                                // packet format
                                                                // name?
                }
            }

            ftypes = real_field_types;
        }
        all_ftypes.push_back(ftypes);
    }
    return all_ftypes;
}
}  // namespace core

core::AnyScanSource open_source(
    const std::string& name,
    const std::function<void(ScanSourceOptions&)>& options, bool collate,
    int sensor_idx) {
    std::vector<std::string> list = {name};
    return open_source(list, options, collate, sensor_idx);
}

core::AnyScanSource open_source(
    const std::vector<std::string>& names,
    const std::function<void(ScanSourceOptions&)>& options, bool collate,
    int sensor_idx) {
    ScanSourceOptions opts;
    if (options) {
        options(opts);
    }
    auto type = ouster::sdk::core::io_type(names[0]);
    for (size_t i = 1; i < names.size(); i++) {
        if (ouster::sdk::core::io_type(names[i]) != type) {
            throw std::runtime_error("All sources must have the same type.");
        }
    }
    if (impl::get_builders().find(type) != impl::get_builders().end()) {
        auto source =
            impl::get_builders()[type](names, opts, collate, sensor_idx);
        return core::AnyScanSource(std::move(source));
    } else {
        if (type == IoType::OSF || type == IoType::PCAP ||
            type == IoType::SENSOR) {
            throw std::runtime_error(
                "Could not open scan source. Unhandled source type " +
                to_string(type) +
                ". Perhaps you forgot to include the requisite source type or "
                "built the library without the correct build flags.");
        } else {
            throw std::runtime_error(
                "Could not open scan source. Unhandled source type " +
                to_string(type) + ".");
        }
    }
}

core::AnyPacketSource open_packet_source(
    const std::string& name,
    const std::function<void(PacketSourceOptions&)>& options) {
    std::vector<std::string> list = {name};
    return open_packet_source(list, options);
}

core::AnyPacketSource open_packet_source(
    const std::vector<std::string>& names,
    const std::function<void(PacketSourceOptions&)>& options) {
    PacketSourceOptions opts;
    if (options) {
        options(opts);
    }
    auto type = ouster::sdk::core::io_type(names[0]);
    if (impl::get_packet_builders().find(type) !=
        impl::get_packet_builders().end()) {
        return core::AnyPacketSource(
            impl::get_packet_builders()[type](names, opts));
    } else {
        if (type == IoType::PCAP || type == IoType::SENSOR) {
            throw std::runtime_error(
                "Could not open packet source. Unhandled source type " +
                to_string(type) +
                ". Perhaps you forgot to include the requisite source type or "
                "built the library without the correct build flags.");
        } else {
            throw std::runtime_error(
                "Could not open packet source. Unhandled source type " +
                to_string(type) + ".");
        }
    }
}

void PacketSourceOptions::check(const char* source_type) const {
    extrinsics_file.check("extrinsics_file", source_type);
    extrinsics.check("extrinsics", source_type);
    soft_id_check.check("soft_id_check", source_type);
    meta.check("meta", source_type);
    lidar_port.check("lidar_port", source_type);
    imu_port.check("imu_port", source_type);
    config_timeout.check("config_timeout", source_type);
    buffer_time_sec.check("buffer_time_sec", source_type);
    index.check("index", source_type);
    timeout.check("timeout", source_type);
    sensor_info.check("sensor_info", source_type);
    do_not_reinitialize.check("do_not_reinitialize", source_type);
    no_auto_udp_dest.check("no_auto_udp_dest", source_type);
    sensor_config.check("sensor_config", source_type);
    reuse_ports.check("reuse_ports", source_type);
}

void ScanSourceOptions::check(const char* source_type) const {
    extrinsics_file.check("extrinsics_file", source_type);
    extrinsics.check("extrinsics", source_type);
    field_names.check("field_names", source_type);
    soft_id_check.check("soft_id_check", source_type);
    index.check("index", source_type);
    meta.check("meta", source_type);
    lidar_port.check("lidar_port", source_type);
    imu_port.check("imu_port", source_type);
    do_not_reinitialize.check("do_not_reinitialize", source_type);
    no_auto_udp_dest.check("no_auto_udp_dest", source_type);
    timeout.check("timeout", source_type);
    config_timeout.check("config_timeout", source_type);
    queue_size.check("queue_size", source_type);
    sensor_info.check("sensor_info", source_type);
    raw_headers.check("raw_headers", source_type);
    raw_fields.check("raw_fields", source_type);
    sensor_config.check("sensor_config", source_type);
    reuse_ports.check("reuse_ports", source_type);
}

PacketSourceOptions::PacketSourceOptions(ScanSourceOptions& options)
    : extrinsics_file(options.extrinsics_file.move()),
      extrinsics(options.extrinsics.move()),
      soft_id_check(options.soft_id_check.move()),
      meta(options.meta.move()),
      lidar_port(options.lidar_port.move()),
      imu_port(options.imu_port.move()),
      timeout(options.timeout.move()),
      index(options.index.move()),
      sensor_info(options.sensor_info.move()),
      do_not_reinitialize(options.do_not_reinitialize.move()),
      no_auto_udp_dest(options.no_auto_udp_dest.move()),
      sensor_config(options.sensor_config.move()),
      reuse_ports(options.reuse_ports.move()) {}

PacketSourceOptions::PacketSourceOptions() = default;
}  // namespace sdk
}  // namespace ouster
