#pragma once

#include <cstdint>
#include <cstdlib>

#include "lidar_globals.h"

namespace ouster {
namespace sdk {
namespace core {
constexpr size_t C_ZONE_MON_PIXEL_BOUNDS_BYTES =
    2 * sizeof(uint16_t);                       // 16b near / 16b far
constexpr size_t C_ZONE_MON_MAX_N_COLS = 2048;  // Currently cap suport at 2048
constexpr size_t C_ZONE_MON_MAX_N_ROWS =
    N_PIXELS_MAX;  // Currently cap suport at 128
constexpr size_t C_ZONE_MON_MAX_ACTIVE_ZONES = 16;
constexpr size_t C_ZONE_MON_MAX_AVAILABLE_ZONES = 128;
constexpr size_t C_ZONE_MON_ACTIVE_ZONE_SET_SIZE =
    C_ZONE_MON_MAX_N_COLS * C_ZONE_MON_MAX_N_ROWS * C_ZONE_MON_MAX_ACTIVE_ZONES;
constexpr size_t C_ZONE_MON_ACTIVE_ZONE_SET_SIZE_BYTES =
    C_ZONE_MON_ACTIVE_ZONE_SET_SIZE * C_ZONE_MON_PIXEL_BOUNDS_BYTES;
constexpr size_t C_ZONE_MON_ACTIVE_ZONE_SET_SIZE_DRAM =
    C_ZONE_MON_ACTIVE_ZONE_SET_SIZE_BYTES / sizeof(uint64_t);
constexpr size_t C_ZONE_MON_BUFFERED_SET_COUNT = 2;
constexpr size_t C_ZONE_MON_BUFFER_SIZE_BYTES =
    C_ZONE_MON_ACTIVE_ZONE_SET_SIZE_BYTES * C_ZONE_MON_BUFFERED_SET_COUNT;

constexpr size_t C_ZONE_MON_ZONE_ID_W = 8;
constexpr size_t C_ZONE_MON_ALL_ZONE_IDS_W =
    C_ZONE_MON_ZONE_ID_W * C_ZONE_MON_MAX_ACTIVE_ZONES;
constexpr size_t C_ZONE_MON_MODE_W = 2;
constexpr size_t C_ZONE_MON_ALL_MODES_W =
    C_ZONE_MON_MODE_W * C_ZONE_MON_MAX_ACTIVE_ZONES;
constexpr size_t C_ZONE_MON_AREA_THRESH_W = 16;
constexpr size_t C_ZONE_MON_ALL_AREA_THRESHS_W =
    C_ZONE_MON_AREA_THRESH_W * C_ZONE_MON_MAX_ACTIVE_ZONES;
constexpr size_t C_ZONE_MON_TIME_THRESH_W = 8;
constexpr size_t C_ZONE_MON_ALL_TIME_THRESHS_W =
    C_ZONE_MON_TIME_THRESH_W * C_ZONE_MON_MAX_ACTIVE_ZONES;
constexpr size_t C_ZONE_MON_HASH_W = 256;
}  // namespace core
}  // namespace sdk
}  // namespace ouster
