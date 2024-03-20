/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/operations.h"

#include <atomic>
#include <csignal>
#include <iostream>

#include "compat_ops.h"
#include "fb_utils.h"
#include "json/json.h"
#include "json_utils.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/file.h"
#include "ouster/osf/meta_extrinsics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/pcap_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

std::string dump_metadata(const std::string& file, bool full) {
    OsfFile osf_file(file);
    auto osf_header = get_osf_header_from_buf(osf_file.get_header_chunk_ptr());

    Json::Value root{};

    root["header"]["size"] = static_cast<Json::UInt64>(osf_file.size());
    root["header"]["version"] = static_cast<Json::Int>(osf_file.version());
    root["header"]["status"] = to_string(osf_header->status());
    root["header"]["metadata_offset"] =
        static_cast<Json::UInt64>(osf_file.metadata_offset());
    root["header"]["chunks_offset"] =
        static_cast<Json::UInt64>(osf_file.chunks_offset());

    Reader reader(file);

    root["metadata"]["id"] = reader.id();
    root["metadata"]["start_ts"] =
        static_cast<Json::UInt64>(reader.start_ts().count());
    root["metadata"]["end_ts"] =
        static_cast<Json::UInt64>(reader.end_ts().count());

    auto osf_metadata =
        get_osf_metadata_from_buf(osf_file.get_metadata_chunk_ptr());

    if (full) {
        root["metadata"]["chunks"] = Json::arrayValue;
        for (size_t i = 0; i < osf_metadata->chunks()->size(); ++i) {
            auto osf_chunk = osf_metadata->chunks()->Get(i);
            Json::Value chunk{};
            chunk["start_ts"] =
                static_cast<Json::UInt64>(osf_chunk->start_ts());
            chunk["end_ts"] = static_cast<Json::UInt64>(osf_chunk->end_ts());
            chunk["offset"] = static_cast<Json::UInt64>(osf_chunk->offset());
            root["metadata"]["chunks"].append(chunk);
        }
    }

    const MetadataStore& meta_store = reader.meta_store();

    root["metadata"]["entries"] = Json::arrayValue;

    for (const auto& me : meta_store.entries()) {
        Json::Value meta_element{};
        meta_element["id"] = static_cast<Json::Int>(me.first);
        meta_element["type"] = me.second->type();

        if (full) {
            const std::string me_str = me.second->repr();
            Json::Value me_obj{};
            if (parse_json(me_str, me_obj)) {
                meta_element["buffer"] = me_obj;
            } else {
                meta_element["buffer"] = me_str;
            }
        }

        root["metadata"]["entries"].append(meta_element);
    }

    return json_string(root);
}

void parse_and_print(const std::string& file, bool with_decoding) {
    OsfFile osf_file{file};

    using ouster::osf::LidarScanStream;
    using ouster::osf::LidarSensor;

    std::cout << "OSF v2:" << std::endl;
    std::cout << "  file = " << osf_file.to_string() << std::endl;

    ouster::osf::Reader reader(osf_file);

    int ls_c = 0;
    int other_c = 0;

    // TODO[pb]: Remove the SIGINT handlers from C++ wrapped function used in
    //           Python bindings
    // https://pybind11.readthedocs.io/en/stable/faq.html#how-can-i-properly-handle-ctrl-c-in-long-running-functions
    thread_local std::atomic_bool quit{false};
    auto sig = std::signal(SIGINT, [](int) { quit = true; });

    for (const auto msg : reader.messages_standard()) {
        if (msg.is<LidarScanStream>()) {
            std::cout << "  Ls     ts: " << msg.ts().count()
                      << ", stream_id = " << msg.id();
            ++ls_c;

            // Example of code to get an object
            if (with_decoding) {
                auto obj_ls = msg.decode_msg<LidarScanStream>();
                if (obj_ls) {
                    std::cout << " [D]";
                }
            }

            std::cout << std::endl;

        } else {
            // UNKNOWN Message
            std::cout << "  UK     ts: " << msg.ts().count()
                      << ", stream_id = " << msg.id() << std::endl;
            ++other_c;
        }

        if (quit) {
            std::cout << "Stopped early via SIGINT!" << std::endl;
            break;
        }
    }

    // restore signal handler
    std::signal(SIGINT, sig);

    std::cout << "\nSUMMARY (OSF v2): \n";
    std::cout << "  lidar_scan     (Ls)     count = " << ls_c << std::endl;
    std::cout << "  other (NOT IMPLEMENTED) count = " << other_c << std::endl;
}

int64_t backup_osf_file_metablob(const std::string& osf_file_name,
                                 const std::string& backup_file_name) {
    uint64_t metadata_offset = 0;
    {
        OsfFile osf_file{osf_file_name};
        metadata_offset = osf_file.metadata_offset();
    }

    // Backup the current metadata blob
    return copy_file_trailing_bytes(osf_file_name, backup_file_name,
                                    metadata_offset);
}

int64_t restore_osf_file_metablob(const std::string& osf_file_name,
                                  const std::string& backup_file_name) {
    uint64_t metadata_offset = 0;
    {
        OsfFile osf_file{osf_file_name};
        metadata_offset = osf_file.metadata_offset();
    }
    truncate_file(osf_file_name, metadata_offset);
    auto result = append_binary_file(osf_file_name, backup_file_name);

    if (result > 0) {
        finish_osf_file(osf_file_name, metadata_offset,
                        result - metadata_offset);
    } else {
        return -1;
    }
    return result;
}

/**
 * Internal simplification function for generating modified
 * metadata flatbuffer blobs.
 *
 * @param[in] file_name Filename of the OSF file to modify
 * @param[in] new_metadata List of new sensor infos to populate
 * @return The generated flatbuffer metadata blob
 */
flatbuffers::FlatBufferBuilder _generate_modify_metadata_fbb(
    const std::string& file_name,
    const std::vector<ouster::sensor::sensor_info>& new_metadata) {
    auto metadata_fbb = flatbuffers::FlatBufferBuilder(32768);
    Reader reader(file_name);

    std::string metadata_id = reader.id();
    ts_t start_ts = reader.start_ts();
    ts_t end_ts = reader.end_ts();

    /// @todo on OsfFile refactor, make a copy constructor for MetadataStore
    MetadataStore new_meta_store;
    auto old_meta_store = reader.meta_store();
    std::cout << "Looking for non sensor info metadata in old metastore"
              << std::endl;
    for (const auto& entry : old_meta_store.entries()) {
        std::cout << "Found: " << entry.second->type() << " ";
        /// @todo figure out why there isnt an easy def for this
        if (entry.second->type() != "ouster/v1/os_sensor/LidarSensor") {
            new_meta_store.add(*entry.second);
            std::cout << "Is non sensor_info, adding" << std::endl;
        } else {
            std::cout << std::endl;
        }
    }

    for (const auto& entry : new_metadata) {
        new_meta_store.add(LidarSensor(entry));
    }

    std::vector<flatbuffers::Offset<ouster::osf::gen::MetadataEntry>> entries =
        new_meta_store.make_entries(metadata_fbb);

    std::vector<ouster::osf::gen::ChunkOffset> chunks{};
    for (const auto& entry : reader.chunks()) {
        chunks.emplace_back(entry.start_ts().count(), entry.end_ts().count(),
                            entry.offset());
    }

    auto metadata = ouster::osf::gen::CreateMetadataDirect(
        metadata_fbb, metadata_id.c_str(),
        !chunks.empty() ? start_ts.count() : 0,
        !chunks.empty() ? end_ts.count() : 0, &chunks, &entries);

    metadata_fbb.FinishSizePrefixed(metadata,
                                    ouster::osf::gen::MetadataIdentifier());
    return metadata_fbb;
}

int64_t osf_file_modify_metadata(
    const std::string& file_name,
    const std::vector<ouster::sensor::sensor_info>& new_metadata) {
    std::string temp_dir;
    std::string temp_path;
    int64_t saved_bytes = -2;
    uint64_t metadata_offset = 0;

    // Scope the reading portion so that we dont run into read write file
    // locks
    {
        OsfFile osf_file{file_name};
        metadata_offset = osf_file.metadata_offset();
    }

    auto metadata_fbb = _generate_modify_metadata_fbb(file_name, new_metadata);

    truncate_file(file_name, metadata_offset);
    saved_bytes = builder_to_file(metadata_fbb, file_name, true);
    finish_osf_file(file_name, metadata_offset, saved_bytes);

    return saved_bytes;
}

bool pcap_to_osf(const std::string& pcap_filename,
                 const std::string& meta_filename, int lidar_port,
                 const std::string& osf_filename, int chunk_size) {
    std::cout << "Converting: " << std::endl
              << "  PCAP file: " << pcap_filename << std::endl
              << "  with json file: " << meta_filename << std::endl
              << "  to OSF file: " << osf_filename << std::endl
              << "  chunk_size: "
              << (chunk_size ? std::to_string(chunk_size) : "DEFAULT")
              << std::endl;

    PcapRawSource pcap_source{pcap_filename};

    std::string sensor_metadata = read_text_file(meta_filename);

    auto info = sensor::parse_metadata(sensor_metadata);

    std::cout << "Using sensor data:\n"
              << "  lidar_port = " << lidar_port << std::endl;

    std::cout << "Processing PCAP packects to OSF messages ";

    Writer writer{osf_filename, "ouster-cli osf from_pcap",
                  static_cast<uint32_t>(chunk_size)};

    std::cout << "(chunk_size: " << writer.chunk_size() << "): ..."
              << std::endl;

    auto field_types = get_field_types(info);

    // Overwrite field_types for Legacy UDP profile, so to reduce the LidarScan
    // encoding sizes (saves about ~15% of disk/bandwidth)
    if (info.format.udp_profile_lidar ==
        sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        field_types.clear();
        field_types.emplace_back(sensor::ChanField::RANGE,
                                 sensor::ChanFieldType::UINT32);
        field_types.emplace_back(sensor::ChanField::SIGNAL,
                                 sensor::ChanFieldType::UINT16);
        field_types.emplace_back(sensor::ChanField::REFLECTIVITY,
                                 sensor::ChanFieldType::UINT16);
        field_types.emplace_back(sensor::ChanField::NEAR_IR,
                                 sensor::ChanFieldType::UINT16);
    }
    std::cout << "LidarScan field_types: " << ouster::to_string(field_types)
              << std::endl;

    auto sensor_meta_id = writer.addMetadata<LidarSensor>(sensor_metadata);
    auto ls_stream =
        writer.createStream<LidarScanStream>(sensor_meta_id, field_types);

    int ls_cnt = 0;

    if (lidar_port > 0) {
        pcap_source.addLidarDataHandler(
            lidar_port, info,
            [&ls_cnt, &ls_stream](const osf::ts_t ts, const LidarScan& ls) {
                ls_cnt++;
                ls_stream.save(ts, ls);
            });
    }

    // TODO[pb]: Remove the SIGINT handlers from C++ wrapped function used in
    //           Python bindings
    // https://pybind11.readthedocs.io/en/stable/faq.html#how-can-i-properly-handle-ctrl-c-in-long-running-functions
    thread_local std::atomic_bool quit{false};
    auto sig = std::signal(SIGINT, [](int) { quit = true; });

    pcap_source.runWhile(
        [](const sensor_utils::packet_info&) { return !quit; });

    // restore signal handler
    std::signal(SIGINT, sig);

    writer.close();

    std::cout << "Saved to OSF file:" << std::endl
              << "  Lidar Scan messages: " << ls_cnt << std::endl;

    return true;
}

}  // namespace osf
}  // namespace ouster
