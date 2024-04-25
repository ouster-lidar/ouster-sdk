/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file operations.h
 * @brief High level OSF operations
 *
 */
#pragma once

#include <string>

#include "ouster/osf/basics.h"
#include "ouster/osf/metadata.h"
#include "ouster/types.h"

/// @todo fix parameter directions in api doc
namespace ouster {
namespace osf {

/**
 * Outputs OSF v2 metadata + header info in JSON format.
 *
 * @param[in] file OSF file (only v2 supported)
 * @param[in] full flag print full information (i.e. chunks_offset and decoded
 *             metas)
 * @return JSON formatted string of the OSF metadata + header
 */
std::string dump_metadata(const std::string& file, bool full = true);

/**
 * Reads OSF file and prints (STDOUT) messages types, timestamps and
 * overall statistics per message type.
 *
 * @param[in] file OSF file
 * @param[in] with_decoding decode known messages (used to time a
 *                          reading + decoding together)
 */
void parse_and_print(const std::string& file, bool with_decoding = false);

/**
 * Backup the metadata blob in an OSF file.
 *
 * @param[in] osf_file_name The OSF file to backup from.
 * @param[in] backup_file_name The path to store the metadata blob backup.
 * @return The number of the bytes written to the backup file.
 */
int64_t backup_osf_file_metablob(const std::string& osf_file_name,
                                 const std::string& backup_file_name);

/**
 * Restore an OSF metadata blob from a backup file.
 *
 * @param[in] osf_file_name The OSF file to restore to.
 * @param[in] backup_file_name The path to the metadata blob backup.
 * @return The number of the bytes written to the OSF file.
 */
int64_t restore_osf_file_metablob(const std::string& osf_file_name,
                                  const std::string& backup_file_name);

/**
 * Modify an OSF files sensor_info metadata.
 *
 * @param[in] file_name The OSF file to modify.
 * @param[in] new_metadata The new metadata for the OSF file
 * @return The number of the bytes written to the OSF file.
 */
int64_t osf_file_modify_metadata(
    const std::string& file_name,
    const std::vector<ouster::sensor::sensor_info>& new_metadata);

}  // namespace osf
}  // namespace ouster
