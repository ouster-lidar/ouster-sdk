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
 * Convert pcap with a single sensor stream to OSF.
 */
bool pcap_to_osf(const std::string& pcap_filename,
                 const std::string& meta_filename, int lidar_port,
                 const std::string& osf_filename, int chunk_size = 0);

}  // namespace osf
}  // namespace ouster
