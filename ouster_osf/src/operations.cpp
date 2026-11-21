/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/operations.h"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_parser.hpp>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "ouster/core/camera_info.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/signals.h"
#include "ouster/core/types.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/file.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/meta_camera_sensor.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_frame.h"
#include "ouster/osf/stream_osf_file.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace {
/**
 * Internal helper turning a SensorInfo into the footer metadata entry that
 * stores it: lidars live in LidarSensor entries and cameras in CameraSensor
 * entries, matching what Writer::add_sensor() writes.
 *
 * @throws std::invalid_argument if the sensor kind is not lidar or camera.
 *
 * @param[in] info The sensor metadata to wrap
 * @return The metadata entry holding @p info
 */
std::unique_ptr<MetadataEntry> make_sensor_entry(const ouster::sdk::core::SensorInfo& info) {
    if (const auto* lidar = dynamic_cast<const ouster::sdk::core::LidarInfo*>(&info)) {
        return std::make_unique<LidarSensor>(*lidar);
    }
    if (const auto* camera = dynamic_cast<const ouster::sdk::core::CameraInfo*>(&info)) {
        return std::make_unique<CameraSensor>(*camera);
    }
    throw std::invalid_argument(
        "osf_file_modify_metadata: received a SensorInfo that is neither a "
        "LidarInfo nor a CameraInfo");
}

/**
 * Internal simplification function for generating modified
 * metadata flatbuffer blobs.
 *
 * @param[in] file_name Filename of the OSF file to modify
 * @param[in] new_metadata List of new sensor infos to populate
 * @return The generated flatbuffer metadata blob
 */
flatbuffers::FlatBufferBuilder generate_modify_metadata_fbb(
    const std::string& file_name,
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& new_metadata) {
    auto metadata_fbb = flatbuffers::FlatBufferBuilder(32768);
    Reader reader(file_name);

    std::string metadata_id = reader.metadata_id();
    ts_t start_ts = reader.start_ts();
    ts_t end_ts = reader.end_ts();

    /// @todo on OsfFile refactor, make a copy constructor for MetadataStore
    MetadataStore new_meta_store;
    auto old_meta_store = reader.meta_store();
    ouster::sdk::core::logger().info("Looking for non sensor info metadata in old metastore");
    // Sensor metadata is stored in LidarSensor (lidars) and CameraSensor
    // (cameras) footer entries; both kinds are replaced wholesale, everything
    // else is carried over untouched.
    std::vector<uint32_t> replaced_ids;
    for (const auto& entry : old_meta_store.entries()) {
        const std::string type = entry.second->type();
        if (type == metadata_type<LidarSensor>() || type == metadata_type<CameraSensor>()) {
            replaced_ids.push_back(entry.first);
        } else {
            new_meta_store.add(*entry.second);
            ouster::sdk::core::logger().info("Is non sensor_info, adding");
        }
    }

    for (size_t i = 0; i < new_metadata.size(); i++) {
        auto entry = make_sensor_entry(*new_metadata[i]);
        // Frame streams and Extrinsics reference their sensor by footer id, so
        // hand each new entry the id of the entry it replaces rather than
        // letting the store assign a fresh one.
        if (i < replaced_ids.size()) {
            entry->set_id(replaced_ids[i]);
        }
        new_meta_store.add(*entry);
    }
    // NOLINTBEGIN(misc-include-cleaner)
    std::vector<flatbuffers::Offset<ouster::sdk::osf::impl::gen::MetadataEntry>> entries =
        impl::make_entries(new_meta_store, metadata_fbb);

    std::vector<ouster::sdk::osf::impl::gen::ChunkOffset> chunks{};
    for (const auto& entry : reader.chunks()) {
        chunks.emplace_back(entry.start_ts().count(), entry.end_ts().count(), entry.offset());
    }

    auto metadata = ouster::sdk::osf::impl::gen::CreateMetadataDirect(
        metadata_fbb, metadata_id.c_str(), !chunks.empty() ? start_ts.count() : 0,
        !chunks.empty() ? end_ts.count() : 0, &chunks, &entries);

    metadata_fbb.FinishSizePrefixed(metadata, ouster::sdk::osf::impl::gen::MetadataIdentifier());
    return metadata_fbb;
    // NOLINTEND(misc-include-cleaner)
}
}  // namespace

std::string dump_metadata(const std::string& file, bool full) {
    Reader reader(file);
    auto& osf_file = reader.file();
    auto osf_header = impl::get_osf_header_from_buf(reader.file().get_header_chunk().data());

    jsoncons::json root;

    root["header"]["size"] = osf_header->file_length();
    root["header"]["version"] = osf_file.version().to_string();
    root["header"]["status"] = impl::to_string(osf_header->status());
    root["header"]["metadata_offset"] = osf_file.metadata_offset().offset();
    root["header"]["chunks_offset"] = osf_file.chunks_offset().offset();

    root["metadata"]["id"] = reader.metadata_id();
    root["metadata"]["start_ts"] = static_cast<uint64_t>(reader.start_ts().count());
    root["metadata"]["end_ts"] = static_cast<uint64_t>(reader.end_ts().count());

    if (full) {
        jsoncons::json chunks(jsoncons::json_array_arg);

        auto osf_chunk = reader.chunks_.first();
        while (osf_chunk != nullptr) {
            jsoncons::json chunk;
            chunk["start_ts"] = osf_chunk->start_ts;
            chunk["end_ts"] = osf_chunk->end_ts;
            chunk["offset"] = osf_chunk->offset;
            chunks.emplace_back(chunk);

            osf_chunk = reader.chunks_.next(osf_chunk->offset);
        }
        root["metadata"]["chunks"] = chunks;
    }

    const MetadataStore& meta_store = reader.meta_store();
    jsoncons::json entries(jsoncons::json_array_arg);
    for (const auto& metadata_entry : meta_store.entries()) {
        jsoncons::json meta_element;
        meta_element["id"] = static_cast<int>(metadata_entry.first);
        meta_element["type"] = metadata_entry.second->type();
        if (full) {
            const std::string temp_data = metadata_entry.second->repr();
            std::istringstream temp(temp_data);

            jsoncons::json_decoder<jsoncons::json> temp_decoder;
            jsoncons::json_stream_reader reader(temp, temp_decoder);

            std::error_code temp_error_code;
            reader.read(temp_error_code);
            if (!temp_error_code) {
                meta_element["buffer"] = temp_decoder.get_result();
            } else {
                jsoncons::json buffer = temp_data;
                meta_element["buffer"] = buffer;
            }
        }

        entries.emplace_back(meta_element);
    }
    root["metadata"]["entries"] = entries;

    std::string out;
    root.dump(out);
    return out;
}

void parse_and_print(const std::string& file, bool with_decoding) {
    using ouster::sdk::osf::LidarFrameStream;
    using ouster::sdk::osf::LidarSensor;

    std::cout << "OSF v2:" << std::endl;
    std::cout << "  file = " << file << std::endl;

    ouster::sdk::osf::Reader reader(file);

    int ls_c = 0;
    int other_c = 0;

    for (const auto msg : reader.messages()) {
        if (msg.is<LidarFrameStream>()) {
            std::cout << "  Ls     ts: " << msg.ts().count() << ", stream_id = " << msg.id();
            ++ls_c;

            // Example of code to get an object
            if (with_decoding) {
                auto obj_ls = msg.decode_msg<LidarFrameStream>();
                if (obj_ls) {
                    std::cout << " [D]";
                }
            }

            std::cout << std::endl;

        } else {
            // UNKNOWN Message
            std::cout << "  UK     ts: " << msg.ts().count() << ", stream_id = " << msg.id()
                      << std::endl;
            ++other_c;
        }

        if (ouster::sdk::core::signaled()) {
            std::cout << "Stopped early via SIGINT!" << std::endl;
            break;
        }
    }

    std::cout << "\nSUMMARY (OSF v2): \n";
    std::cout << "  lidar_frame     (Ls)     count = " << ls_c << std::endl;
    std::cout << "  other (NOT IMPLEMENTED) count = " << other_c << std::endl;
}

int64_t backup_osf_file_metablob(const std::string& osf_file_name,
                                 const std::string& backup_file_name) {
    uint64_t metadata_offset = 0;
    {
        StreamOsfFile osf_file{osf_file_name};
        metadata_offset = osf_file.metadata_offset().offset();
    }

    // Backup the current metadata blob
    return copy_file_trailing_bytes(osf_file_name, backup_file_name, metadata_offset);
}

int64_t restore_osf_file_metablob(const std::string& osf_file_name,
                                  const std::string& backup_file_name) {
    uint64_t metadata_offset = 0;
    auto version = OsfFile::CURRENT_VERSION;
    {
        StreamOsfFile osf_file{osf_file_name};
        version = osf_file.version();
        metadata_offset = osf_file.metadata_offset().offset();
    }
    if (metadata_offset == 0) {
        throw std::runtime_error("File had invalid metadata and cannot be restored.");
    }
    truncate_file(osf_file_name, metadata_offset);
    auto result = append_binary_file(osf_file_name, backup_file_name);

    if (result > 0) {
        impl::finish_osf_file(osf_file_name, metadata_offset, result - metadata_offset, version);
    } else {
        return -1;
    }
    return result;
}

int64_t osf_file_modify_metadata(
    const std::string& file_name,
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& new_metadata) {
    int64_t saved_bytes = -2;
    uint64_t metadata_offset = 0;
    ouster::sdk::core::Version version = ouster::sdk::osf::OsfFile::CURRENT_VERSION;

    // Scope the reading portion so that we don't run into read write file
    // locks
    {
        StreamOsfFile osf_file{file_name};
        version = osf_file.version();
        metadata_offset = osf_file.metadata_offset().offset();
    }

    auto metadata_fbb = generate_modify_metadata_fbb(file_name, new_metadata);

    truncate_file(file_name, metadata_offset);
    saved_bytes = static_cast<int64_t>(impl::builder_to_file(metadata_fbb, file_name, true));
    impl::finish_osf_file(file_name, metadata_offset, saved_bytes, version);

    return saved_bytes;
}

int64_t osf_file_modify_metadata(const std::string& file_name,
                                 const std::vector<ouster::sdk::core::LidarInfo>& new_metadata) {
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> infos;
    infos.reserve(new_metadata.size());
    for (const auto& info : new_metadata) {
        infos.push_back(std::make_shared<ouster::sdk::core::LidarInfo>(info));
    }
    return osf_file_modify_metadata(file_name, infos);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
