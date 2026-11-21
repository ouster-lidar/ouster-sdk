/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file operations.h
 * @brief High level OSF operations
 *
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/sensor_info.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Outputs OSF v2 metadata + header info in JSON format.
 *
 * @param[in] file OSF file (only v2 supported)
 * @param[in] full flag print full information (i.e. chunks_offset and decoded
 *             metas)
 * @return JSON formatted string of the OSF metadata + header
 */
OUSTER_API_FUNCTION
std::string dump_metadata(const std::string& file, bool full = true);

/**
 * Reads OSF file and prints (STDOUT) messages types, timestamps and
 * overall statistics per message type.
 *
 * @param[in] file OSF file
 * @param[in] with_decoding decode known messages (used to time a
 *                          reading + decoding together)
 */
OUSTER_API_FUNCTION
void parse_and_print(const std::string& file, bool with_decoding = false);

/**
 * Backup the metadata blob in an OSF file.
 *
 * @param[in] osf_file_name The OSF file to backup from.
 * @param[in] backup_file_name The path to store the metadata blob backup.
 * @return The number of the bytes written to the backup file.
 */
OUSTER_API_FUNCTION
int64_t backup_osf_file_metablob(const std::string& osf_file_name,
                                 const std::string& backup_file_name);

/**
 * Restore an OSF metadata blob from a backup file.
 *
 * @param[in] osf_file_name The OSF file to restore to.
 * @param[in] backup_file_name The path to the metadata blob backup.
 * @return The number of the bytes written to the OSF file.
 */
OUSTER_API_FUNCTION
int64_t restore_osf_file_metablob(const std::string& osf_file_name,
                                  const std::string& backup_file_name);

/**
 * Modify an OSF files sensor metadata.
 *
 * Replaces every sensor metadata entry in the footer with @p new_metadata,
 * dispatching each entry on its concrete type: a
 * ouster::sdk::core::LidarInfo is written as a LidarSensor entry and a
 * ouster::sdk::core::CameraInfo as a CameraSensor entry. The new entries
 * reuse the footer ids of the entries they replace, in order, so frame streams
 * and Extrinsics that reference a sensor by id keep resolving.
 *
 * @throws std::invalid_argument if an entry is neither a LidarInfo nor a
 *         CameraInfo.
 *
 * @param[in] file_name The OSF file to modify.
 * @param[in] new_metadata The new metadata for the OSF file
 * @return The number of the bytes written to the OSF file.
 */
OUSTER_API_FUNCTION
int64_t osf_file_modify_metadata(
    const std::string& file_name,
    const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& new_metadata);

/**
 * Modify an OSF files LidarInfo metadata.
 *
 * @param[in] file_name The OSF file to modify.
 * @param[out] new_metadata The new metadata for the OSF file
 * @return The number of the bytes written to the OSF file.
 */
OUSTER_API_FUNCTION
int64_t osf_file_modify_metadata(const std::string& file_name,
                                 const std::vector<ouster::sdk::core::LidarInfo>& new_metadata);

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
