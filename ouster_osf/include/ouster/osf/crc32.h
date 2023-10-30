/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file crc32.h
 * @brief crc32 utility
 *
 */
#pragma once

#include <stddef.h>

#include <cstdint>

namespace ouster {
namespace osf {

/// Size of the CRC field in a buffer
const uint32_t CRC_BYTES_SIZE = 4;

/**
 * Caclulate CRC value for the buffer of given size. (ZLIB version)
 * @param buf pointer to the data buffer
 * @param size size of the buffer in bytes
 * @return CRC32 value
 */
uint32_t crc32(const uint8_t* buf, uint32_t size);

/**
 * Caclulate and append CRC value for the buffer of given size and append
 * it to the initial crc value. (ZLIB version)
 * @param initial_crc initial crc value to append to
 * @param buf pointer to the data buffer
 * @param size size of the buffer in bytes
 * @return CRC32 value
 */
uint32_t crc32(uint32_t initial_crc, const uint8_t* buf, uint32_t size);

}  // namespace osf
}  // namespace ouster