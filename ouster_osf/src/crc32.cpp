/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/crc32.h"

#include <zlib.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace ouster {
namespace osf {

const uint32_t CRC_INITIAL_VALUE = 0L;

// =============== ZLIB functions wrappers ====================

uint32_t crc32(const uint8_t* buf, uint32_t size) {
    return crc32_z(CRC_INITIAL_VALUE, buf, size);
}

uint32_t crc32(uint32_t initial_crc, const uint8_t* buf, uint32_t size) {
    return crc32_z(initial_crc, buf, size);
}

}  // namespace osf
}  // namespace ouster