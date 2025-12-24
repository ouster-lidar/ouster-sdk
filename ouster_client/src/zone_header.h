#pragma once

#include <array>
#include <bitset>
#include <cstdint>

#include "ouster/sha256.h"
#include "ouster/zrb.h"

namespace ouster {
namespace sdk {
namespace core {

using ZoneCacheVersion = std::array<char, 7>;
using SerialNumber = std::array<char, 16>;

SerialNumber serial_number_from_int(uint64_t serial_number);

#pragma pack(push, 1)
struct CacheHeaderInfo {
    constexpr static ZoneCacheVersion ZONE_CACHE_VERSION = {"ZONE03"};
    ZoneCacheVersion version;
    Sha256 hash{};

    CacheHeaderInfo();
};  // __attribute__((packed));

struct CacheRenderMetadata {
    SerialNumber serial_number{};
    uint32_t n_cols{};
    uint32_t n_rows{};
    float m_per_zmbin{};
    Sha256 stl_hash{};
    Sha256 bounds_hash{};
    std::array<float, 16> beam_to_lidar{};
    std::array<float, 16> lidar_to_sensor{};
    std::array<float, 16> sensor_to_body{};
    std::bitset<2048> valid_col_mask;

    CacheRenderMetadata() = default;
    CacheRenderMetadata(const Zrb& zrb,
                        const std::bitset<2048>& valid_col_mask_init,
                        SerialNumber serial_number_init);
};  // __attribute__((packed));

struct CacheHeader {
    CacheHeaderInfo info{};
    CacheRenderMetadata meta{};

    CacheHeader() = default;
    CacheHeader(const Zrb& zrb, const std::bitset<2048>& valid_col_mask_init,
                SerialNumber serial_number_init);
};  // __attribute__((packed));
#pragma pack(pop)

static_assert(sizeof(CacheHeaderInfo) == 39,
              "CacheHeaderInfo size should be 39 bytes");
static_assert(sizeof(CacheRenderMetadata) == 540,
              "CacheRenderMetadata size should be 540 bytes");

}  // namespace core
}  // namespace sdk
}  // namespace ouster
