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
 * @param file OSF file (only v2 supported)
 * @param full flag print full information (i.e. chunks_offset and decoded
 * metas)
 * @return JSON formatted string of the OSF metadata + header
 *
 */
std::string dump_metadata(const std::string& file, bool full = true);

/**
 * Reads OSF file and prints (STDOUT) messages types, timestamps and
 * overall statistics per message type.
 *
 * @param file OSF file
 * @param with_decoding decode known messages (used to time a
 *  reading + decoding together)
 *
 */
void parse_and_print(const std::string& file, bool with_decoding = false);

/**
 * Backup the metadata blob in an OSF file.
 *
 * @param osf_file_name[in] The OSF file to backup from.
 * @param backup_file_name[in] The path to store the metadata blob backup.
 * @return The number of the bytes written to the backup file.
 */
int64_t backup_osf_file_metablob(const std::string& osf_file_name,
                                 const std::string& backup_file_name);

/**
 * Restore an OSF metadata blob from a backup file.
 *
 * @param osf_file_name[in] The OSF file to restore to.
 * @param backup_file_name[in] The path to the metadata blob backup.
 * @return The number of the bytes written to the OSF file.
 */
int64_t restore_osf_file_metablob(const std::string& osf_file_name,
                                  const std::string& backup_file_name);

/**
 * Modify an OSF files sensor_info metadata.
 *
 * @param file_name[in] The OSF file to modify.
 * @param new_metadata[in] The new metadata for the OSF file
 * @return The number of the bytes written to the OSF file.
 */
int64_t osf_file_modify_metadata(
    const std::string& file_name,
    const std::vector<ouster::sensor::sensor_info>& new_metadata);

/**
 * Convert pcap with a single sensor stream to OSF.
 * @todo fix api comments
 */
bool pcap_to_osf(const std::string& pcap_filename,
                 const std::string& meta_filename, int lidar_port,
                 const std::string& osf_filename, int chunk_size = 0);

}  // namespace osf
}  // namespace ouster
