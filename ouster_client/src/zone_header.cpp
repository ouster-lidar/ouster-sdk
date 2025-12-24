#include "zone_header.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

namespace ouster {
namespace sdk {
namespace core {

namespace {
constexpr double TRANSFORM_HASH_PRECISION = 1e-6;

std::array<float, 16> canonicalize_transform(const mat4d& transform) {
    std::array<float, 16> result{};
    for (int i = 0; i < 16; ++i) {
        double value = transform.data()[i];
        double rounded = std::round(value / TRANSFORM_HASH_PRECISION) *
                         TRANSFORM_HASH_PRECISION;
        result[i] = static_cast<float>(rounded);
    }
    return result;
}
}  // namespace

SerialNumber serial_number_from_int(uint64_t serial_number_int) {
    std::string sn_str = std::to_string(serial_number_int);
    SerialNumber serial_number{0};
    std::copy_n(sn_str.begin(), std::min(sn_str.size(), serial_number.size()),
                serial_number.begin());
    return serial_number;
}

constexpr ZoneCacheVersion CacheHeaderInfo::ZONE_CACHE_VERSION;

CacheHeaderInfo::CacheHeaderInfo() : version{ZONE_CACHE_VERSION} {}

// Cache definitions
CacheRenderMetadata::CacheRenderMetadata(
    const Zrb& zrb, const std::bitset<2048>& valid_col_mask_init,
    SerialNumber serial_number_init)

    : serial_number{serial_number_init},
      n_cols{static_cast<uint32_t>(zrb.near_range_mm.cols())},
      n_rows{static_cast<uint32_t>(zrb.near_range_mm.rows())},
      m_per_zmbin{zrb.m_per_zmbin_},
      beam_to_lidar{canonicalize_transform(zrb.beam_to_lidar_transform)},
      lidar_to_sensor{canonicalize_transform(zrb.lidar_to_sensor_transform)},
      sensor_to_body{canonicalize_transform(zrb.sensor_to_body_transform)},
      valid_col_mask{valid_col_mask_init} {
    if (zrb.stl_hash) {
        this->stl_hash = zrb.stl_hash.value();
    } else {
        this->stl_hash = Sha256();
    }
    if (zrb.beam_to_lidar_transform.isZero()) {
        throw std::logic_error(
            "CacheRenderMetadata: beam_to_lidar_transform not set");
    }
    if (zrb.lidar_to_sensor_transform.isZero()) {
        throw std::logic_error(
            "CacheRenderMetadata: lidar_to_sensor_transform not set");
    }
    if (zrb.sensor_to_body_transform.isZero()) {
        throw std::logic_error(
            "CacheRenderMetadata: sensor_to_body_transform not set");
    }
}

CacheHeader::CacheHeader(const Zrb& zrb,
                         const std::bitset<2048>& valid_col_mask_init,
                         SerialNumber serial_number_init)
    : info(CacheHeaderInfo()),
      meta(CacheRenderMetadata(zrb, valid_col_mask_init, serial_number_init)) {}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
