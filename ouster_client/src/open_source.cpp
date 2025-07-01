/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/open_source.h"

#include <Eigen/Geometry>
#include <fstream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>

#include "ouster/scan_source_utils.h"

namespace ouster {

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

static std::string read_file(const std::string& fileName) {
    std::ifstream ifs(fileName.c_str(),
                      std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(fileSize);
    ifs.read(bytes.data(), fileSize);

    return std::string(bytes.data(), fileSize);
}

void populate_extrinsics(
    std::string extrinsics_file,
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> extrinsics,
    std::vector<std::shared_ptr<sensor::sensor_info>>& sensor_infos) {
    // try and load from file if set
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> final_extrinsics;
    if (extrinsics_file.length()) {
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

    if (extrinsics.size()) {
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
    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>& metadata,
    bool raw_headers, bool raw_fields,
    const nonstd::optional<std::vector<std::string>>& field_names) {
    std::vector<std::vector<FieldType>> all_ftypes;
    for (const auto& m : metadata) {
        auto ftypes = get_field_types(*m);
        bool dual = (m->num_returns() == 2);

        if (raw_fields) {
            ftypes.push_back(FieldType(sensor::ChanField::RAW32_WORD1,
                                       sensor::ChanFieldType::UINT32));
            if (m->format.udp_profile_lidar !=
                ouster::sensor::UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8) {
                ftypes.push_back(FieldType(sensor::ChanField::RAW32_WORD2,
                                           sensor::ChanFieldType::UINT32));
                ftypes.push_back(FieldType(sensor::ChanField::RAW32_WORD3,
                                           sensor::ChanFieldType::UINT32));
            }
            if (dual) {
                ftypes.push_back(FieldType(sensor::ChanField::RAW32_WORD4,
                                           sensor::ChanFieldType::UINT32));
            }
        }

        if (raw_headers) {
            auto pf = ouster::sensor::packet_format(*m);
            auto h = pf.pixels_per_column;
            auto raw_headers_space =
                (pf.packet_header_size + pf.packet_footer_size +
                 pf.col_header_size + pf.col_footer_size);
            auto size_bytes = raw_headers_space / h;
            auto dtype = sensor::ChanFieldType::UINT8;
            if (size_bytes >= 2) {
                dtype = sensor::ChanFieldType::UINT32;
            } else if (size_bytes >= 1) {
                dtype = sensor::ChanFieldType::UINT16;
            }
            ftypes.push_back(FieldType(sensor::ChanField::RAW_HEADERS, dtype));
        }

        // Finally apply field names
        if (field_names) {
            std::vector<FieldType> real_field_types;
            // Only parse fields we asked for in field_names
            for (const auto& ft : ftypes) {
                for (const auto& name : field_names.value()) {
                    if (ft.name == name) {
                        real_field_types.push_back(ft);
                        break;
                    }
                }
            }

            // Error if we cant find a field with the requested name
            for (const auto& name : field_names.value()) {
                bool found = false;
                for (const auto& ft : ftypes) {
                    if (ft.name == name) {
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
    auto type = ouster::core::io_type(names[0]);
    if (impl::get_builders().find(type) != impl::get_builders().end()) {
        auto source = impl::get_builders()[type](names, opts);
        if (sensor_idx >= 0) {
            source = new ouster::core::Singler(
                std::unique_ptr<ouster::core::ScanSource>(source), sensor_idx);
        } else if (collate) {
            source = new ouster::core::Collator(
                std::unique_ptr<ouster::core::ScanSource>(source));
        }
        return core::AnyScanSource(
            std::unique_ptr<ouster::core::ScanSource>(source));
    } else {
        throw std::runtime_error(
            "Source type expected to be a sensor hostname, ip address, or a "
            ".pcap, .osf, or .bag file.");
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
    auto type = ouster::core::io_type(names[0]);
    if (impl::get_packet_builders().find(type) !=
        impl::get_packet_builders().end()) {
        return core::AnyPacketSource(std::unique_ptr<core::PacketSource>(
            impl::get_packet_builders()[type](names, opts)));
    } else {
        throw std::runtime_error(
            "Source type expected to be a sensor hostname, ip address, or a "
            ".pcap, or .bag file.");
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
      sensor_config(options.sensor_config.move()) {}

PacketSourceOptions::PacketSourceOptions() {}
}  // namespace ouster
