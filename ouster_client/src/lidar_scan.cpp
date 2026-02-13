/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ios>
#include <limits>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/cartesian.h"
#include "ouster/impl/logging.h"
#include "ouster/packet.h"
#include "ouster/types.h"
#include "ouster/visibility.h"
#include "ouster/xyzlut.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ouster {
namespace sdk {
namespace core {

// clang-format off
/**
 * Flags for frame_status
 */
enum FrameStatusMasks : uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_MASK = 0x0f,  ///< Mask to get thermal shutdown status
    FRAME_STATUS_SHOT_LIMITING_MASK = 0xf0      ///< Mask to get shot limting status
};

//! @cond Doxygen_Suppress
enum FrameStatusShifts: uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT = 0,    ///< No shift for thermal shutdown
    FRAME_STATUS_SHOT_LIMITING_SHIFT = 4        /// shift 4 for shot limiting
};
//! @endcond

// clang-format on

LidarScan::LidarScan() = default;
LidarScan::LidarScan(const LidarScan&) = default;
LidarScan::LidarScan(LidarScan&&) = default;
LidarScan& LidarScan::operator=(const LidarScan&) = default;
LidarScan& LidarScan::operator=(LidarScan&&) = default;
LidarScan::~LidarScan() = default;
namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

static const Table<std::string, ChanFieldType, 5> LEGACY_FIELD_SLOTS{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT16},
     {ChanField::NEAR_IR, ChanFieldType::UINT16},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {ChanField::FLAGS, ChanFieldType::UINT8}}};

static const Table<std::string, ChanFieldType, 10> DUAL_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 6> SINGLE_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 4> LB_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 4> LB_WINDOW_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 5> ZM_LB_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
}};

static const Table<std::string, ChanFieldType, 6> ZM_SINGLE_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
}};

static const Table<std::string, ChanFieldType, 5> FIVE_WORD_SLOTS{{
    {ChanField::RAW32_WORD1, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD2, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD3, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD4, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD5, ChanFieldType::UINT32},
}};

static const Table<std::string, ChanFieldType, 8> DUAL_LB_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 0> OFF_PROFILE_SLOTS{{}};

struct OUSTER_API_CLASS DefaultFieldsEntry {
    const std::pair<std::string, ChanFieldType>* fields;
    size_t n_fields;
};

using ouster::sdk::core::impl::MAX_NUM_PROFILES;

// clang-format off
Table<UDPProfileLidar, DefaultFieldsEntry, MAX_NUM_PROFILES> default_scan_fields{
    {{UDPProfileLidar::LEGACY,
      {LEGACY_FIELD_SLOTS.data(), LEGACY_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
      {DUAL_FIELD_SLOTS.data(), DUAL_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
      {SINGLE_FIELD_SLOTS.data(), SINGLE_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG15_RFL8_NIR8,
      {LB_FIELD_SLOTS.data(), LB_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG15_RFL8_WIN8,
      {LB_WINDOW_FIELD_SLOTS.data(), LB_WINDOW_FIELD_SLOTS.size()}},
     {UDPProfileLidar::FIVE_WORD_PIXEL,
      {FIVE_WORD_SLOTS.data(), FIVE_WORD_SLOTS.size()}},
     {UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL,
      {DUAL_LB_SLOTS.data(), DUAL_LB_SLOTS.size()}},
     {UDPProfileLidar::RNG15_RFL8_NIR8_DUAL,
      {DUAL_LB_SLOTS.data(), DUAL_LB_SLOTS.size()}},
     {UDPProfileLidar::OFF,
      {OFF_PROFILE_SLOTS.data(), OFF_PROFILE_SLOTS.size()}},
     {UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16,
      {ZM_LB_FIELD_SLOTS.data(), ZM_LB_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16,
      {ZM_SINGLE_FIELD_SLOTS.data(), ZM_SINGLE_FIELD_SLOTS.size()}},
}};
// clang-format on

static LidarScanFieldTypes lookup_scan_fields(UDPProfileLidar profile) {
    auto end = impl::default_scan_fields.end();
    auto it = std::find_if(impl::default_scan_fields.begin(), end,
                           [profile](const auto& key_value) {
                               return key_value.first == profile;
                           });

    if (it == end || it->first == UDPProfileLidar::UNKNOWN) {
        throw std::invalid_argument("Unknown lidar udp profile");
    }

    auto entry = it->second;
    LidarScanFieldTypes field_types;
    for (size_t i = 0; i < entry.n_fields; i++) {
        // everything is pixel field for now
        field_types.emplace_back(entry.fields[i].first, entry.fields[i].second,
                                 std::vector<size_t>(),
                                 FieldClass::PIXEL_FIELD);
    }
    return field_types;
}

bool raw_headers_enabled(const PacketFormat& packet_format,
                         const LidarScan& lidar_scan) {
    using ouster::sdk::core::logger;
    if (!lidar_scan.has_field(ChanField::RAW_HEADERS)) {
        return false;
    }

    auto raw_headers_ft = lidar_scan.field(ChanField::RAW_HEADERS).tag();
    // ensure that we can pack headers into the size of a single RAW_HEADERS
    // column
    if (packet_format.pixels_per_column * field_type_size(raw_headers_ft) <
        (packet_format.packet_header_size + packet_format.col_header_size +
         packet_format.col_footer_size + packet_format.packet_footer_size)) {
        logger().debug(
            "WARNING: Can't fit RAW_HEADERS into a column of {} {} "
            "values",
            packet_format.pixels_per_column, to_string(raw_headers_ft));
        return false;
    }
    return true;
}

}  // namespace impl

static FieldDescriptor get_field_type_descriptor(const LidarScan& scan,
                                                 const FieldType& field_type) {
    if (field_type.field_class == FieldClass::PIXEL_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(scan.h);
        dims.push_back(scan.w);
        dims.insert(dims.end(), field_type.extra_dims.begin(),
                    field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else if (field_type.field_class == FieldClass::COLUMN_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(scan.w);
        dims.insert(dims.end(), field_type.extra_dims.begin(),
                    field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else if (field_type.field_class == FieldClass::PACKET_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(scan.packet_count());
        dims.insert(dims.end(), field_type.extra_dims.begin(),
                    field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else {  // FieldClass::SCAN_FIELD
        return FieldDescriptor::array(field_type.element_type,
                                      field_type.extra_dims);
    }
}

LidarScan::LidarScan(const DataFormat& format)
    : LidarScan{format.columns_per_frame, format.pixels_per_column,
                get_field_types(format, Version(0, 0, 0)),
                format.columns_per_packet} {}

LidarScan::LidarScan(const SensorInfo& info)
    : LidarScan{info.format.columns_per_frame, info.format.pixels_per_column,
                get_field_types(info), info.format.columns_per_packet} {
    sensor_info = std::make_shared<SensorInfo>(info);
}

LidarScan::LidarScan(std::shared_ptr<SensorInfo> info) : LidarScan{*info} {
    sensor_info = info;
}

LidarScan::LidarScan(std::shared_ptr<SensorInfo> info,
                     const std::vector<FieldType>& field_types)
    : LidarScan{info->format.columns_per_frame, info->format.pixels_per_column,
                field_types, info->format.columns_per_packet} {
    sensor_info = info;
}

// specify sensor:: namespace for doxygen matching
LidarScan::LidarScan(size_t w, size_t h, LidarScanFieldTypes field_types,
                     size_t columns_per_packet)
    : packet_count_{(w + columns_per_packet - 1) /
                    columns_per_packet},  // equivalent to
                                          // int(ceil(w/columns_per_packet))
      w{w},
      h{h},
      columns_per_packet_(columns_per_packet) {
    if (w * h == 0) {
        throw std::invalid_argument(
            "Cannot construct non-empty LidarScan with "
            "zero width or height");
    }

    for (const auto& field_type : field_types) {
        add_field(field_type);
    }

    timestamp_ = Field{fd_array<uint64_t>(w), FieldClass::COLUMN_FIELD};
    measurement_id_ = Field{fd_array<uint16_t>(w), FieldClass::COLUMN_FIELD};
    status_ = Field{fd_array<uint32_t>(w), FieldClass::COLUMN_FIELD};
    packet_timestamp_ =
        Field{fd_array<uint64_t>(packet_count_), FieldClass::PACKET_FIELD};
    pose_ = Field{fd_array<double>(w, 4, 4), {}};
    alert_flags_ =
        Field{fd_array<uint8_t>(packet_count_), FieldClass::PACKET_FIELD};

    // Initialize all the poses to identity
    auto ptr = pose_.get<double>();
    auto ident = mat4d::Identity().eval();
    for (size_t i = 0; i < w; ++i) {
        memcpy(&ptr[i * 4 * 4], ident.data(), 4 * 4 * sizeof(double));
    }
}

LidarScan::LidarScan(const LidarScan& ls_src,
                     const LidarScanFieldTypes& field_types)
    : packet_count_(ls_src.packet_count_),
      w(ls_src.w),
      h(ls_src.h),
      columns_per_packet_(ls_src.columns_per_packet_),
      frame_status(ls_src.frame_status),
      frame_id(ls_src.frame_id),
      sensor_info(ls_src.sensor_info) {
    for (const auto& field_type : field_types) {
        const std::string& name = field_type.name;
        FieldDescriptor dst_desc = get_field_type_descriptor(*this, field_type);
        if (ls_src.has_field(name)) {
            // either copy cast or error
            const auto& src_field = ls_src.field(name);
            const auto& src_desc = src_field.desc();
            if (src_desc == dst_desc) {
                fields()[name] = ls_src.field(name);
            } else {
                // cast if the dimensions match
                if (dst_desc.shape != src_desc.shape) {
                    throw std::invalid_argument(
                        "Field '" + name +
                        "' from source scan has dimensions that don't match "
                        "desired.");
                }
                add_field(name, dst_desc, field_type.field_class);
                ouster::sdk::core::impl::visit_field(
                    *this, name, ouster::sdk::core::impl::copy_and_cast(),
                    ls_src, name);
            }
        } else {
            add_field(name, dst_desc, field_type.field_class);
        }
    }

    timestamp_ = ls_src.timestamp_;
    measurement_id_ = ls_src.measurement_id_;
    status_ = ls_src.status_;
    packet_timestamp_ = ls_src.packet_timestamp_;
    pose_ = ls_src.pose_;
}

LidarScan::LidarScan(size_t w, size_t h, UDPProfileLidar profile,
                     size_t columns_per_packet)
    : LidarScan{w, h, impl::lookup_scan_fields(profile), columns_per_packet} {}

LidarScan::LidarScan(size_t w, size_t h)
    : LidarScan{w, h, UDPProfileLidar::LEGACY, DEFAULT_COLUMNS_PER_PACKET} {}

ShotLimitingStatus LidarScan::shot_limiting() const {
    return static_cast<ShotLimitingStatus>(
        (frame_status & FrameStatusMasks::FRAME_STATUS_SHOT_LIMITING_MASK) >>
        FrameStatusShifts::FRAME_STATUS_SHOT_LIMITING_SHIFT);
}

ThermalShutdownStatus LidarScan::thermal_shutdown() const {
    return static_cast<ThermalShutdownStatus>(
        (frame_status & FrameStatusMasks::FRAME_STATUS_THERMAL_SHUTDOWN_MASK) >>
        FrameStatusShifts::FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT);
}

Field& LidarScan::field(const std::string& name) {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in LidarScan.");
    }
}

const Field& LidarScan::field(const std::string& name) const {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in LidarScan.");
    }
}

bool LidarScan::has_field(const std::string& name) const {
    return fields().count(name) > 0;
}

Field& LidarScan::add_field(const FieldType& type) {
    if (has_field(type.name)) {
        throw std::invalid_argument("Duplicated field '" + type.name + "'");
    }

    // just validate that we didnt add a 0 size pixel field
    if (type.field_class == FieldClass::PIXEL_FIELD) {
        // none of the dimensions should be zero
        for (const auto& dim : type.extra_dims) {
            if (dim == 0) {
                throw std::invalid_argument(
                    "Cannot add pixel field with 0 elements.");
            }
        }
    }

    // no other checking is necessary
    fields()[type.name] =
        Field(get_field_type_descriptor(*this, type), type.field_class);

    return fields()[type.name];
}

Field& LidarScan::add_field(const std::string& name, FieldDescriptor desc,
                            FieldClass field_class) {
    if (has_field(name)) {
        throw std::invalid_argument("Duplicated field '" + name + "'");
    }

    if (field_class == FieldClass::PIXEL_FIELD) {
        if (desc.shape.size() < 2) {
            throw std::invalid_argument(
                "Pixel fields must have at least 2 dimensions");
        }
        if (desc.shape[0] != h || desc.shape[1] != w) {
            throw std::invalid_argument(
                "Pixel field shape must match "
                "LidarScan's width and height. Was " +
                std::to_string(desc.shape[0]) + "x" +
                std::to_string(desc.shape[1]) + " vs " + std::to_string(h) +
                "x" + std::to_string(w));
        }
        // none of the dimensions should be zero
        for (const auto& dim : desc.shape) {
            if (dim == 0) {
                throw std::invalid_argument(
                    "Cannot add pixel field with 0 elements.");
            }
        }
    }

    if (field_class == FieldClass::COLUMN_FIELD) {
        if (desc.shape[0] != w) {
            throw std::invalid_argument(
                "Column field shape must match "
                "LidarScan's height. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " +
                std::to_string(w));
        }
    }

    if (field_class == FieldClass::PACKET_FIELD) {
        if (desc.shape[0] != packet_count_) {
            throw std::invalid_argument(
                "Packet field shape must match "
                "number of packets. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " +
                std::to_string(packet_count_));
        }
    }

    fields()[name] = Field{desc, field_class};

    return fields()[name];
}

Field LidarScan::del_field(const std::string& name) {
    if (!has_field(name)) {
        throw std::invalid_argument("Attempted deleting non existing field '" +
                                    name + "'");
    }

    Field ptr;
    field(name).swap(ptr);
    fields().erase(name);
    return ptr;
}

template <typename T>
Eigen::Ref<img_t<T>> LidarScan::field(const std::string& name) {
    return field(name);
}

template <typename T>
Eigen::Ref<const img_t<T>> LidarScan::field(const std::string& name) const {
    return field(name);
}

// explicitly instantiate for each supported field type

// clang-format off
template Eigen::Ref<img_t<uint8_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<uint16_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<uint32_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<uint64_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<int8_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<int16_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<int32_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<int64_t>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<float>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<img_t<double>> LidarScan::field(const std::string& field_name);
template Eigen::Ref<const img_t<uint8_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<uint16_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<uint32_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<uint64_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<int8_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<int16_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<int32_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<int64_t>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<float>> LidarScan::field(const std::string& field_name) const;
template Eigen::Ref<const img_t<double>> LidarScan::field(const std::string& field_name) const;
// clang-format on

static FieldType get_field_type(const std::string& name, const Field& field) {
    int offset = 0;
    if (field.field_class() == FieldClass::PIXEL_FIELD) {
        offset = 2;
    } else if ((field.field_class() == FieldClass::COLUMN_FIELD) ||
               (field.field_class() == FieldClass::PACKET_FIELD)) {
        offset = 1;
    }
    std::vector<size_t> extra_dims;
    extra_dims.insert(extra_dims.begin(), field.shape().begin() + offset,
                      field.shape().end());
    return FieldType(name, field.tag(), extra_dims, field.field_class());
}

FieldType LidarScan::field_type(const std::string& name) const {
    return get_field_type(name, field(name));
}

std::unordered_map<std::string, Field>& LidarScan::fields() { return fields_; }

const std::unordered_map<std::string, Field>& LidarScan::fields() const {
    return fields_;
}

Eigen::Ref<LidarScan::Header<uint64_t>> LidarScan::timestamp() {
    return timestamp_;
}

Eigen::Ref<const LidarScan::Header<uint64_t>> LidarScan::timestamp() const {
    return timestamp_;
}

Eigen::Ref<LidarScan::Header<uint64_t>> LidarScan::packet_timestamp() {
    return packet_timestamp_;
}

Eigen::Ref<const LidarScan::Header<uint64_t>> LidarScan::packet_timestamp()
    const {
    return packet_timestamp_;
}

Eigen::Ref<LidarScan::Header<uint8_t>> LidarScan::alert_flags() {
    return alert_flags_;
}

Eigen::Ref<const LidarScan::Header<uint8_t>> LidarScan::alert_flags() const {
    return alert_flags_;
}

ArrayView1<ZoneState> LidarScan::zones() {
    auto it = fields().find(ChanField::ZONE_STATES);

    if (it == fields().end()) {
        return ArrayView1<ZoneState>(nullptr, {0});
    }

    return it->second;
}

ConstArrayView1<ZoneState> LidarScan::zones() const {
    auto it = fields().find(ChanField::ZONE_STATES);

    if (it == fields().end()) {
        return ConstArrayView1<ZoneState>(nullptr, {0});
    }

    return it->second;
}

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace ouster::sdk::core::ChanField;
uint64_t LidarScan::get_first_valid_packet_timestamp() const {
    uint64_t time = get_first_valid_lidar_packet_timestamp();
    if (has_field(IMU_PACKET_TIMESTAMP)) {
        ConstArrayView<uint64_t, 1> data = field(IMU_PACKET_TIMESTAMP);
        for (size_t i = 0; i < data.size(); i++) {
            if (data(i) != 0) {
                time = (time == 0) ? data(i) : std::min(data(i), time);
                break;
            }
        }
    }
    if (has_field(ZONE_PACKET_TIMESTAMP)) {
        ConstArrayView<uint64_t, 1> data = field(ZONE_PACKET_TIMESTAMP);
        if (data(0) != 0) {
            time = (time == 0) ? data(0) : std::min(data(0), time);
        }
    }
    return time;
}

uint64_t LidarScan::get_last_valid_packet_timestamp() const {
    uint64_t time = get_last_valid_lidar_packet_timestamp();
    if (has_field(IMU_PACKET_TIMESTAMP)) {
        ConstArrayView<uint64_t, 1> data = field(IMU_PACKET_TIMESTAMP);
        for (int i = static_cast<int>(data.size()) - 1; i >= 0; i--) {
            if (data(i) != 0) {
                time = std::max(data(i), time);
                break;
            }
        }
    }
    if (has_field(ZONE_PACKET_TIMESTAMP)) {
        ConstArrayView<uint64_t, 1> data = field(ZONE_PACKET_TIMESTAMP);
        time = std::max(data(0), time);
    }
    return time;
}

uint64_t LidarScan::get_first_valid_lidar_packet_timestamp() const {
    int total_packets = packet_timestamp().size();
    int columns_per_packet = w / total_packets;

    for (int i = 0; i < total_packets; ++i) {
        if (status()
                .middleRows(i * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1; })
                .any()) {
            return packet_timestamp()[i];
        }
    }

    return 0;
}

uint64_t LidarScan::get_last_valid_lidar_packet_timestamp() const {
    int total_packets = packet_timestamp().size();
    int columns_per_packet = w / total_packets;

    for (int i = total_packets - 1; i >= 0; --i) {
        if (status()
                .middleRows(i * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1; })
                .any()) {
            return packet_timestamp()[i];
        }
    }

    return 0;
}

uint64_t LidarScan::get_first_valid_column_timestamp() const {
    auto col = get_first_valid_column();
    if (col < 0) {
        return 0;
    }
    return timestamp()[col];
}

uint64_t LidarScan::get_last_valid_column_timestamp() const {
    auto col = get_last_valid_column();
    if (col < 0) {
        return 0;
    }
    return timestamp()[col];
}

int LidarScan::get_first_valid_column() const {
    auto stat = status();
    for (int i = 0; i < stat.size(); ++i) {
        if ((stat[i] & 1) > 0) {
            return i;
        }
    }
    return -1;
}

int LidarScan::get_last_valid_column() const {
    auto stat = status();
    for (int i = stat.size() - 1; i >= 0; --i) {
        if ((stat[i] & 1) > 0) {
            return i;
        }
    }
    return -1;
}

Eigen::Ref<LidarScan::Header<uint16_t>> LidarScan::measurement_id() {
    return measurement_id_;
}

Eigen::Ref<const LidarScan::Header<uint16_t>> LidarScan::measurement_id()
    const {
    return measurement_id_;
}

Eigen::Ref<LidarScan::Header<uint32_t>> LidarScan::status() { return status_; }

Eigen::Ref<const LidarScan::Header<uint32_t>> LidarScan::status() const {
    return status_;
}

Field& LidarScan::pose() { return pose_; }

const Field& LidarScan::pose() const { return pose_; }

void LidarScan::set_column_pose(int index, const Matrix4dR& pose) {
    if (index < 0 || index >= static_cast<int>(w)) {
        throw std::out_of_range("Column index out of range");
    }
    std::memcpy(static_cast<char*>(pose_.get()) + index * 16 * sizeof(double),
                pose.data(), sizeof(double) * 16);
}

Matrix4dR LidarScan::get_column_pose(int index) const {
    if (index < 0 || index >= static_cast<int>(w)) {
        throw std::out_of_range("Column index out of range");
    }
    Matrix4dR out;
    std::memcpy(
        out.data(),
        static_cast<const char*>(pose_.get()) + index * 16 * sizeof(double),
        sizeof(double) * 16);
    return out;
}

bool LidarScan::complete(ColumnWindow window) const {
    const auto& status = this->status();
    auto start = window.first;
    auto end = window.second;

    if (start <= end) {
        return status.segment(start, end - start + 1)
            .unaryExpr([](uint32_t status_val) { return status_val & 0x01; })
            .isConstant(0x01);
    } else {
        return status.segment(0, end + 1)
                   .unaryExpr(
                       [](uint32_t status_val) { return status_val & 0x01; })
                   .isConstant(0x01) &&
               status.segment(start, this->w - start)
                   .unaryExpr(
                       [](uint32_t status_val) { return status_val & 0x01; })
                   .isConstant(0x01);
    }
}

bool LidarScan::complete() const {
    if (!sensor_info) {
        throw std::runtime_error(
            "LidarScan must have a valid SensorInfo in order to compute "
            "completeness");
    }
    return complete(sensor_info->format.column_window);
}

size_t LidarScan::packet_count() const { return packet_count_; }

bool LidarScan::equals(const LidarScan& other) const {
    return frame_id == other.frame_id && w == other.w && h == other.h &&
           frame_status == other.frame_status &&
           measurement_id_ == other.measurement_id_ &&
           timestamp_ == other.timestamp_ &&
           packet_timestamp_ == other.packet_timestamp_ &&
           pose() == other.pose() && fields() == other.fields();
}

bool operator==(const LidarScan& a, const LidarScan& b) { return a.equals(b); }

bool operator!=(const LidarScan& a, const LidarScan& b) { return !(a == b); }

LidarScanFieldTypes LidarScan::field_types() const {
    LidarScanFieldTypes field_types_vec;
    for (const auto& kv : fields()) {
        field_types_vec.push_back(get_field_type(kv.first, kv.second));
    }

    std::sort(field_types_vec.begin(), field_types_vec.end());
    return field_types_vec;
}

LidarScanFieldTypes get_field_types(UDPProfileLidar udp_profile_lidar) {
    // Get typical LidarScan to obtain field types
    return impl::lookup_scan_fields(udp_profile_lidar);
}

LidarScanFieldTypes get_field_types(const DataFormat& format) {
    return get_field_types(format, Version(0, 0, 0));
}

LidarScanFieldTypes get_field_types(const DataFormat& format,
                                    const Version& fw_version) {
    LidarScanFieldTypes field_types =
        impl::lookup_scan_fields(format.udp_profile_lidar);

    size_t imu_measurements =
        format.imu_packets_per_frame * format.imu_measurements_per_packet;

    /** TODO:
     * Theoretically would be good to move this to static storage like
     * the rest of the lookup tables, but extra dimensions complicate
     * the issue.
     * Potentially we can refactor the whole thing out and stop using
     * get_field_types on naked profiles without DataFormat, thus
     * eliminating the need for FieldType and just use FieldDescriptor?
     * -- Tim T.
     */
    using namespace ouster::sdk::core;
    using namespace ouster::sdk::core::ChanField;
    if (format.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        field_types.emplace_back(IMU_ACC, ChanFieldType::FLOAT32,
                                 std::vector<size_t>{imu_measurements, 3},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(IMU_GYRO, ChanFieldType::FLOAT32,
                                 std::vector<size_t>{imu_measurements, 3},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(IMU_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{imu_measurements},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(IMU_MEASUREMENT_ID, ChanFieldType::UINT16,
                                 std::vector<size_t>{imu_measurements},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(IMU_STATUS, ChanFieldType::UINT16,
                                 std::vector<size_t>{imu_measurements},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(
            IMU_PACKET_TIMESTAMP, ChanFieldType::UINT64,
            std::vector<size_t>{format.imu_packets_per_frame},
            FieldClass::SCAN_FIELD);
        field_types.emplace_back(
            POSITION_STRING, ChanFieldType::CHAR,
            std::vector<size_t>{format.imu_packets_per_frame,
                                NMEA_SENTENCE_LENGTH},
            FieldClass::SCAN_FIELD);
        field_types.emplace_back(
            POSITION_LAT_LONG, ChanFieldType::FLOAT64,
            std::vector<size_t>{format.imu_packets_per_frame, 2},
            FieldClass::SCAN_FIELD);
        field_types.emplace_back(
            POSITION_TIMESTAMP, ChanFieldType::UINT64,
            std::vector<size_t>{format.imu_packets_per_frame},
            FieldClass::SCAN_FIELD);
        field_types.emplace_back(
            IMU_ALERT_FLAGS, ChanFieldType::UINT8,
            std::vector<size_t>{format.imu_packets_per_frame},
            FieldClass::SCAN_FIELD);
    }

    if (format.zone_monitoring_enabled) {
        field_types.emplace_back(LIVE_ZONESET_HASH, ChanFieldType::UINT8,
                                 std::vector<size_t>{32},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(ZONE_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{1},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(ZONE_PACKET_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{1},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(ZONE_ALERT_FLAGS, ChanFieldType::UINT8,
                                 std::vector<size_t>{1},
                                 FieldClass::SCAN_FIELD);
        field_types.emplace_back(ZONE_STATES, ChanFieldType::ZONE_STATE,
                                 std::vector<size_t>{16},
                                 FieldClass::SCAN_FIELD);
    }

    // remove WINDOW if FW is < 3.2
    if (fw_version < Version(3, 2, 0)) {
        for (size_t i = 0; i < field_types.size(); i++) {
            if (field_types[i].name == ouster::sdk::core::ChanField::WINDOW) {
                field_types.erase(field_types.begin() + i);
                break;
            }
        }
    }

    return field_types;
}

LidarScanFieldTypes get_field_types(const SensorInfo& info) {
    return get_field_types(info.format, info.get_version());
}

std::string to_string(const FieldType& field_type) {
    std::string out = field_type.name;
    out += ": ";
    out += to_string(field_type.element_type);
    out += " (";
    int j = 0;
    for (const auto& dim : field_type.extra_dims) {
        if (j++ > 0) {
            out += ", ";
        }
        out += std::to_string(dim);
    }
    out += ") ";
    out += ouster::sdk::core::to_string(field_type.field_class);
    return out;
}

std::string to_string(const LidarScanFieldTypes& field_types) {
    std::stringstream string_stream;
    string_stream << "(";
    for (size_t i = 0; i < field_types.size(); ++i) {
        if (i > 0) {
            string_stream << ", ";
        }
        string_stream << to_string(field_types[i]);
    }
    string_stream << ")";
    return string_stream.str();
}

std::string to_string(const LidarScan& lidar_scan) {
    std::stringstream string_stream;
    LidarScanFieldTypes field_types = lidar_scan.field_types();
    string_stream << "LidarScan: {h = " << lidar_scan.h
                  << ", w = " << lidar_scan.w << ", packets_per_frame = "
                  << lidar_scan.packet_timestamp().size()
                  << ", fid = " << lidar_scan.frame_id << "," << std::endl
                  << " frame status = " << std::hex << lidar_scan.frame_status
                  << std::dec << ", thermal_shutdown status = "
                  << to_string(lidar_scan.thermal_shutdown())
                  << ", shot_limiting status = "
                  << to_string(lidar_scan.shot_limiting()) << "," << std::endl
                  << "  field_types = " << to_string(field_types) << ","
                  << std::endl;

    auto read_eigen = [](auto ref, std::stringstream& string_stream) {
        string_stream << "min: " << (double)ref.minCoeff()
                      << "; mean: " << ref.template cast<double>().mean()
                      << "; max: " << (double)ref.maxCoeff();
    };

    auto read_field = [&read_eigen](const Field& field_val,
                                    const std::string& name,
                                    std::stringstream& string_stream) {
        string_stream << "     " << name
                      << " type:" << to_string(field_val.tag()) << " shape: (";

        const auto& shape = field_val.shape();
        for (size_t i = 0; i < shape.size(); ++i) {
            string_stream << shape[i];
            if (i < shape.size() - 1) {
                string_stream << ", ";
            }
        }
        string_stream << ") ";

        if (field_val.bytes() > 0) {
            if (field_val.tag() == ChanFieldType::CHAR) {
                // Show first sentence (e.g., first NMEA string).
                const char* data = static_cast<const char*>(field_val.get());
                size_t max_len = NMEA_SENTENCE_LENGTH;
                const auto& shape = field_val.shape();
                if (!shape.empty() && shape.back() > 0) {
                    max_len = shape.back();
                }
                const char* end =
                    static_cast<const char*>(memchr(data, '\0', max_len));
                size_t len = (end != nullptr) ? static_cast<size_t>(end - data)
                                              : max_len;
                std::string preview;
                preview.reserve(len);
                for (size_t i = 0; i < len; ++i) {
                    switch (data[i]) {
                        case '\r':
                            preview += "\\r";
                            break;
                        case '\n':
                            preview += "\\n";
                            break;
                        case '\t':
                            preview += "\\t";
                            break;
                        case '\\':
                            preview += "\\\\";
                            break;
                        case '\b':
                            preview += "\\b";
                            break;
                        case '\f':
                            preview += "\\f";
                            break;
                        case '\v':
                            preview += "\\v";
                            break;
                        case '"':
                            preview += "\\\"";
                            break;
                        default:
                            preview += data[i];
                            break;
                    }
                }
                bool truncated = (len == max_len && end == nullptr);
                if (truncated) {
                    preview += "...";
                }
                string_stream << "\"" << preview << "\"";
            } else {
                // For numeric types, show min/mean/max stats
                // visit_field_2d will no-op for unsupported types (ZONE_STATE,
                // etc.)
                FieldView flat_view = field_val.reshape(1, field_val.size());
                impl::visit_field_2d(flat_view, read_eigen, string_stream);
            }
        }
        string_stream << std::endl;
    };

    for (auto&& kv : lidar_scan.fields()) {
        read_field(kv.second, kv.first, string_stream);
    }

    string_stream << "}";
    return string_stream.str();
}

ScanBatcher::ScanBatcher(const std::shared_ptr<SensorInfo>& info)
    : w_(info->format.columns_per_frame),
      h_(info->format.pixels_per_column),
      pf(get_format(*info)) {
    if (info->format.columns_per_packet == 0) {
        throw std::invalid_argument("unexpected columns_per_packet: 0");
    }
    // Calculate the number of packets required to have a complete scan
    expected_lidar_packets_ = info->format.lidar_packets_per_frame();
    if (info->format.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        expected_imu_packets_ = info->format.imu_packets_per_frame;
    }
    if (info->format.zone_monitoring_enabled) {
        expected_zone_packets_ = 1;
    }
    sensor_info_ = info;
}

ScanBatcher::ScanBatcher(const SensorInfo& info)
    : ScanBatcher(std::make_shared<SensorInfo>(info)) {}

namespace {

/*
 * Generic operation to set all columns in the range [start, end) to zero
 */
struct ZeroFieldCols {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& /*unused*/,
                    std::ptrdiff_t start, std::ptrdiff_t end) const {
        field.block(0, start, field.rows(), end - start).setZero();
    }
};

/*
 * Zero out all measurement block headers in range [start, end)
 */
void zero_header_cols(LidarScan& lidar_scan, std::ptrdiff_t start,
                      std::ptrdiff_t end) {
    lidar_scan.timestamp().segment(start, end - start).setZero();
    lidar_scan.measurement_id().segment(start, end - start).setZero();
    lidar_scan.status().segment(start, end - start).setZero();
}

/*
 * Generic operation to read a channel field from a packet measurement block
 * into a scan
 */
struct ParseFieldCol {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& field_name,
                    uint16_t m_id, const PacketFormat& packet_format,
                    const uint8_t* col_buf) const {
        // RAW_HEADERS field is populated separately because it has
        // a different processing scheme and doesn't fit into existing field
        // model (i.e. data packed per column rather than per pixel)
        if (field_name == ChanField::RAW_HEADERS) {
            return;
        }

        packet_format.col_field(col_buf, field_name, field.col(m_id).data(),
                                field.cols());
    }
};

uint64_t frame_status(const uint8_t thermal_shutdown,
                      const uint8_t shot_limiting) {
    uint64_t res = 0;

    // clang-format off
    res |= (thermal_shutdown & 0x0f)
        << FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT;  // right nibble is thermal
                                                 // shutdown status, apply mask
                                                 // for safety, then shift
    // clang-format on
    res |= (shot_limiting & 0x0f)
           << FRAME_STATUS_SHOT_LIMITING_SHIFT;  // right nibble is shot
                                                 // limiting, apply mask for
                                                 // safety, then shift
    return res;
}

/**
 * Pack the lidar packet and column headers and footer into a RAW_HEADERS field.
 */
struct PackRawHeadersCol {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> rh_field,
                    const std::string& /*unused*/,
                    const PacketFormat& packet_format, uint16_t col_idx,
                    const uint8_t* packet_buf) const {
        const uint8_t* col_buf = packet_format.nth_col(col_idx, packet_buf);
        const uint16_t m_id = packet_format.col_measurement_id(col_buf);

        using ColMajorView =
            Eigen::Map<const Eigen::Array<T, -1, 1, Eigen::ColMajor>>;

        const ColMajorView col_header_vec(
            reinterpret_cast<const T*>(col_buf),
            packet_format.col_header_size / sizeof(T));

        rh_field.block(0, m_id, col_header_vec.size(), 1) = col_header_vec;

        const ColMajorView col_footer_vec(
            reinterpret_cast<const T*>(col_buf + packet_format.col_size -
                                       packet_format.col_footer_size),
            packet_format.col_footer_size / sizeof(T));

        rh_field.block(col_header_vec.size(), m_id, col_footer_vec.size(), 1) =
            col_footer_vec;

        const ColMajorView packet_header_vec(
            reinterpret_cast<const T*>(packet_buf),
            packet_format.packet_header_size / sizeof(T));

        rh_field.block(col_header_vec.size() + col_footer_vec.size(), m_id,
                       packet_header_vec.size(), 1) = packet_header_vec;

        const ColMajorView packet_footer_vec(
            reinterpret_cast<const T*>(packet_format.footer(packet_buf)),
            packet_format.packet_footer_size / sizeof(T));

        rh_field.block(col_header_vec.size() + col_footer_vec.size() +
                           packet_header_vec.size(),
                       m_id, packet_footer_vec.size(), 1) = packet_footer_vec;
    }
};

}  // namespace

void ScanBatcher::parse_by_col(const uint8_t* packet_buf,
                               LidarScan& lidar_scan) {
    const bool raw_headers = impl::raw_headers_enabled(pf, lidar_scan);
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint64_t timestamp_val = pf.col_timestamp(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01) != 0u;

        // drop out-of-bounds data in case of misconfiguration
        if (m_id >= w_) {
            continue;
        }

        if (raw_headers) {
            // zero out missing columns if we jumped forward
            if (m_id >= next_headers_m_id_) {
                impl::visit_field(lidar_scan, ChanField::RAW_HEADERS,
                                  ZeroFieldCols{}, "", next_headers_m_id_,
                                  m_id);
                next_headers_m_id_ = m_id + 1;
            }

            impl::visit_field(lidar_scan, ChanField::RAW_HEADERS,
                              PackRawHeadersCol(), ChanField::RAW_HEADERS, pf,
                              icol, packet_buf);
        }

        // drop invalid
        if (!valid) {
            continue;
        }

        // zero out missing columns if we jumped forward
        if (m_id >= next_valid_m_id_) {
            impl::foreach_channel_field(lidar_scan, pf, ZeroFieldCols{},
                                        next_valid_m_id_, m_id);
            zero_header_cols(lidar_scan, next_valid_m_id_, m_id);
            next_valid_m_id_ = m_id + 1;
        }

        // write new header values
        lidar_scan.timestamp()[m_id] = timestamp_val;
        lidar_scan.measurement_id()[m_id] = m_id;
        lidar_scan.status()[m_id] = status;

        impl::foreach_channel_field(lidar_scan, pf, ParseFieldCol{}, m_id, pf,
                                    col_buf);
    }
}

/*
 * Faster version of ParseFieldCol that works by blocks instead and skips
 * extra checks
 */
template <int BlockDim>
struct ParseFieldBlock {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& field_name,
                    const PacketFormat& packet_format,
                    const uint8_t* packet_buf) const {
        packet_format.block_field<T, BlockDim>(field, field_name, packet_buf);
    }
};

void ScanBatcher::parse_by_block(const uint8_t* packet_buf,
                                 LidarScan& lidar_scan) {
    // zero out missing columns if we jumped forward
    const uint16_t first_m_id =
        pf.col_measurement_id(pf.nth_col(0, packet_buf));
    if (first_m_id >= next_valid_m_id_) {
        impl::foreach_channel_field(lidar_scan, pf, ZeroFieldCols{},
                                    next_valid_m_id_, first_m_id);
        zero_header_cols(lidar_scan, next_valid_m_id_, first_m_id);
        next_valid_m_id_ = first_m_id + pf.columns_per_packet;
    }

    // write new header values
    auto timestamp = lidar_scan.timestamp();
    auto measurement_id = lidar_scan.measurement_id();
    auto status = lidar_scan.status();
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const auto m_id = pf.col_measurement_id(col_buf);

        measurement_id[m_id] = m_id;
        timestamp[m_id] = pf.col_timestamp(col_buf);
        status[m_id] = pf.col_status(col_buf);
    }

    switch (pf.block_parsable()) {
        case 16:
            impl::foreach_channel_field(lidar_scan, pf, ParseFieldBlock<16>{},
                                        pf, packet_buf);
            break;
        case 8:
            impl::foreach_channel_field(lidar_scan, pf, ParseFieldBlock<8>{},
                                        pf, packet_buf);
            break;
        case 4:
            impl::foreach_channel_field(lidar_scan, pf, ParseFieldBlock<4>{},
                                        pf, packet_buf);
            break;
        default:
            throw std::invalid_argument("Invalid block dim for packet format");
    }
}

void ScanBatcher::batch_lidar_packet(const LidarPacket& packet,
                                     LidarScan& lidar_scan) {
    const uint8_t* packet_buf = packet.buf.data();

    // handling packet level data: packet_timestamp
    const uint8_t* col0_buf = pf.nth_col(0, packet_buf);
    const uint16_t packet_id =
        pf.col_measurement_id(col0_buf) / pf.columns_per_packet;
    if (packet_id < lidar_scan.packet_timestamp().rows()) {
        lidar_scan.packet_timestamp()[packet_id] = packet.host_timestamp;
        lidar_scan.alert_flags()[packet_id] = pf.alert_flags(packet_buf);
    }

    // handling column and pixel level data
    size_t block_parsable = pf.block_parsable();
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01) != 0u;

        if (!valid || m_id >= w_) {
            block_parsable = 0;
            break;
        }
    }

    // validate that we actually can block parse it
    // this requires that each block be block_parsable away from the end of scan
    if (block_parsable != 0) {
        for (int icol = 0; icol < pf.columns_per_packet;
             icol += block_parsable) {
            const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
            const uint16_t m_id = pf.col_measurement_id(col_buf);
            if (m_id + block_parsable > w_) {
                block_parsable = 0;
                break;
            }
        }
    }

    if ((block_parsable != 0) && !impl::raw_headers_enabled(pf, lidar_scan)) {
        parse_by_block(packet_buf, lidar_scan);
    } else {
        parse_by_col(packet_buf, lidar_scan);
    }

    batched_lidar_packets_++;
}

void ScanBatcher::batch_imu_packet(const ImuPacket& packet,
                                   LidarScan& lidar_scan) {
    using namespace ouster::sdk::core::ChanField;

    const uint8_t* buf = packet.buf.data();

    // get packet_id from imu measurement id
    size_t imu_first_m_id =
        pf.col_measurement_id(pf.imu_nth_measurement(0, buf));
    uint16_t packet_id =
        imu_first_m_id / (lidar_scan.w / pf.imu_packets_per_frame);
    size_t col_offset = packet_id * pf.imu_measurements_per_packet;

    // TODO: this pattern could be a method -- Tim T.
    FieldView imu_ts_fview = lidar_scan.has_field(IMU_TIMESTAMP)
                                 ? lidar_scan.field(IMU_TIMESTAMP)
                                 : FieldView{};
    FieldView imu_m_id_fview = lidar_scan.has_field(IMU_MEASUREMENT_ID)
                                   ? lidar_scan.field(IMU_MEASUREMENT_ID)
                                   : FieldView{};
    FieldView imu_status_fview = lidar_scan.has_field(IMU_STATUS)
                                     ? lidar_scan.field(IMU_STATUS)
                                     : FieldView{};

    for (size_t i = 0; i < pf.imu_measurements_per_packet; ++i) {
        const uint8_t* col_buf = pf.imu_nth_measurement(i, buf);
        if (imu_ts_fview) {
            ArrayView1<uint64_t> imu_timestamp = imu_ts_fview;
            imu_timestamp(col_offset + i) = pf.col_timestamp(col_buf);
        }
        if (imu_m_id_fview) {
            ArrayView1<uint16_t> imu_m_id = imu_m_id_fview;
            imu_m_id(col_offset + i) = pf.col_measurement_id(col_buf);
        }
        if (imu_status_fview) {
            ArrayView1<uint16_t> imu_status = imu_status_fview;
            imu_status(col_offset + i) = pf.col_status(col_buf) & 0x1;
        }
    }

    if (lidar_scan.has_field(IMU_ACC)) {
        pf.parse_accel(col_offset, buf, lidar_scan.field(IMU_ACC));
    }

    if (lidar_scan.has_field(IMU_GYRO)) {
        pf.parse_gyro(col_offset, buf, lidar_scan.field(IMU_GYRO));
    }

    if (lidar_scan.has_field(IMU_PACKET_TIMESTAMP)) {
        ArrayView1<uint64_t> packet_timestamp =
            lidar_scan.field(IMU_PACKET_TIMESTAMP);
        packet_timestamp(packet_id) = packet.host_timestamp;
    }

    if (lidar_scan.has_field(IMU_ALERT_FLAGS)) {
        ArrayView1<uint8_t> alert_flags = lidar_scan.field(IMU_ALERT_FLAGS);
        alert_flags(packet_id) = pf.alert_flags(buf);
    }

    auto sentence = pf.imu_nmea_sentence(buf);

    if (lidar_scan.has_field(POSITION_STRING)) {
        ArrayView2<char> nmea_sentences = lidar_scan.field(POSITION_STRING);
        std::memcpy(nmea_sentences.subview(packet_id).data(), sentence.data(),
                    sentence.size());
    }

    if (lidar_scan.has_field(POSITION_TIMESTAMP)) {
        ArrayView1<uint64_t> nmea_ts = lidar_scan.field(POSITION_TIMESTAMP);
        nmea_ts(packet_id) = pf.imu_nmea_ts(buf);
    }

    if (lidar_scan.has_field(POSITION_LAT_LONG)) {
        ArrayView2<double> lat_long = lidar_scan.field(POSITION_LAT_LONG);
        if (!parse_lat_long(sentence, lat_long(packet_id, 0),
                            lat_long(packet_id, 1))) {
            lat_long(packet_id, 0) = std::numeric_limits<double>::quiet_NaN();
            lat_long(packet_id, 1) = std::numeric_limits<double>::quiet_NaN();
        }
    }

    batched_imu_packets_++;
}

void ScanBatcher::batch_zone_packet(const ZonePacket& packet,
                                    LidarScan& lidar_scan) {
    using namespace ouster::sdk::core::ChanField;
    const uint8_t* buf = packet.buf.data();

    if (lidar_scan.has_field(ZONE_ALERT_FLAGS)) {
        ArrayView1<uint8_t> alert_flags = lidar_scan.field(ZONE_ALERT_FLAGS);
        alert_flags(0) = pf.alert_flags(buf);
    }
    if (lidar_scan.has_field(ZONE_TIMESTAMP)) {
        ArrayView1<uint64_t> zone_ts = lidar_scan.field(ZONE_TIMESTAMP);
        zone_ts(0) = pf.zone_timestamp(buf);
    }
    if (lidar_scan.has_field(ZONE_PACKET_TIMESTAMP)) {
        ArrayView1<uint64_t> zone_packet_ts =
            lidar_scan.field(ZONE_PACKET_TIMESTAMP);
        zone_packet_ts(0) = packet.host_timestamp;
    }
    if (lidar_scan.has_field(LIVE_ZONESET_HASH)) {
        std::array<uint8_t, 32> hash = pf.live_zoneset_hash(buf);
        std::memcpy(lidar_scan.field(LIVE_ZONESET_HASH), hash.data(),
                    sizeof(uint8_t) * hash.size());
    }
    if (lidar_scan.has_field(ZONE_STATES)) {
        ArrayView1<ZoneState> zones = lidar_scan.field(ZONE_STATES);
        for (size_t i = 0; i < zones.shape[0]; ++i) {
            const uint8_t* zone_ptr = pf.zone_nth_measurement(i, buf);
            ZoneState& zone = zones(i);

            zone.live = static_cast<uint8_t>(pf.zone_live(zone_ptr));
            zone.id = pf.zone_id(zone_ptr);
            zone.error_flags = pf.zone_error_flags(zone_ptr);
            zone.trigger_type = pf.zone_trigger_type(zone_ptr);
            zone.trigger_status = pf.zone_trigger_status(zone_ptr);
            zone.triggered_frames = pf.zone_triggered_frames(zone_ptr);
            zone.count = pf.zone_points_count(zone_ptr);
            zone.occlusion_count = pf.zone_occlusion_count(zone_ptr);
            zone.invalid_count = pf.zone_invalid_count(zone_ptr);
            zone.max_count = pf.zone_max_count(zone_ptr);
            zone.min_range = pf.zone_min_range(zone_ptr);
            zone.max_range = pf.zone_max_range(zone_ptr);
            zone.mean_range = pf.zone_mean_range(zone_ptr);
        }
    }
    batched_zone_packets_++;
}

bool ScanBatcher::operator()(const Packet& packet, LidarScan& lidar_scan) {
    if (packet.type() == PacketType::Imu &&
        pf.udp_profile_imu != UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        return false;
    }

    if (lidar_scan.w != w_ || lidar_scan.h != h_) {
        throw std::invalid_argument("unexpected scan dimensions");
    }
    if (static_cast<size_t>(lidar_scan.packet_timestamp().rows()) !=
        lidar_scan.w / pf.columns_per_packet) {
        throw std::invalid_argument("unexpected scan columns_per_packet: " +
                                    std::to_string(pf.columns_per_packet));
    }

    // process cached packet and packet ts, if present
    batch_cached_packet(lidar_scan);

    const uint8_t* packet_buf = packet.buf.data();
    const int64_t f_id = pf.frame_id(packet_buf);

    if (lidar_scan.frame_id == -1 || finished_scan_id_ >= 0) {
        // expecting to start batching a new scan
        if (finished_scan_id_ >= 0) {
            // drop duplicate or packets from previous frame
            if (finished_scan_id_ == f_id) {
                // drop old duplicate packets
                return false;
            } else if (finished_scan_id_ ==
                       ((f_id + 1) %
                        (static_cast<int64_t>(pf.max_frame_id) + 1))) {
                // drop reordered packets from the previous frame
                return false;
            }
        }
        finished_scan_id_ = -1;
        next_valid_m_id_ = 0;
        next_headers_m_id_ = 0;
        batched_lidar_packets_ = 0;
        batched_imu_packets_ = 0;
        batched_zone_packets_ = 0;
        lidar_scan.frame_id = f_id;
        zero_header_cols(lidar_scan, 0, w_);
        lidar_scan.packet_timestamp().setZero();
        const uint8_t f_thermal_shutdown = pf.thermal_shutdown(packet_buf);
        const uint8_t f_shot_limiting = pf.shot_limiting(packet_buf);
        lidar_scan.frame_status =
            frame_status(f_thermal_shutdown, f_shot_limiting);

        // The countdown values are supposed to be the same for all packets in a
        // given scan.
        lidar_scan.shutdown_countdown =
            pf.countdown_thermal_shutdown(packet_buf);
        lidar_scan.shot_limiting_countdown =
            pf.countdown_shot_limiting(packet_buf);
        lidar_scan.sensor_info = sensor_info_;
    } else if (lidar_scan.frame_id ==
               ((f_id + 1) % (static_cast<int64_t>(pf.max_frame_id) + 1))) {
        // drop reordered packets from the previous frame
        return false;
    } else if (lidar_scan.frame_id != f_id) {
        // got a packet from a new frame, release the old one
        finalize_scan(lidar_scan);

        // store packet buf and ts data to the cache for later processing
        cache_packet(packet);
        return true;
    }

    if (pf.udp_profile_lidar == UDPProfileLidar::LEGACY ||
        packet.type() == PacketType::Lidar) {
        batch_lidar_packet(packet.as<LidarPacket>(), lidar_scan);
    } else if (pf.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA &&
               packet.type() == PacketType::Imu) {
        batch_imu_packet(packet.as<ImuPacket>(), lidar_scan);
    } else if (packet.type() == PacketType::Zone) {
        batch_zone_packet(packet.as<ZonePacket>(), lidar_scan);
    }

    // if we have enough packets and are packet-complete release the scan
    if (check_scan_complete(lidar_scan)) {
        finalize_scan(lidar_scan);
        return true;
    }

    return false;
}

void ScanBatcher::cache_packet(const Packet& packet) {
    cache_ = std::make_unique<Packet>(packet);
    cached_packet_ = true;
}

void ScanBatcher::batch_cached_packet(LidarScan& lidar_scan) {
    if (cached_packet_ && cache_) {
        cached_packet_ = false;
        this->operator()(*cache_, lidar_scan);
    }
}

bool ScanBatcher::check_scan_complete(const LidarScan& lidar_scan) const {
    bool lidar_batching_finished =
        (pf.udp_profile_lidar == UDPProfileLidar::OFF) ||
        (batched_lidar_packets_ >= expected_lidar_packets_ &&
         static_cast<size_t>(lidar_scan.packet_timestamp().count()) ==
             expected_lidar_packets_);
    bool imu_batching_finished = batched_imu_packets_ >= expected_imu_packets_;
    bool zone_batching_finished =
        batched_zone_packets_ >= expected_zone_packets_;

    return lidar_batching_finished && imu_batching_finished &&
           zone_batching_finished;
}

void ScanBatcher::finalize_scan(LidarScan& lidar_scan) {
    if (next_valid_m_id_ < w_) {
        impl::foreach_channel_field(lidar_scan, pf, ZeroFieldCols{},
                                    next_valid_m_id_, w_);
    }

    if (impl::raw_headers_enabled(pf, lidar_scan)) {
        impl::visit_field(lidar_scan, ChanField::RAW_HEADERS, ZeroFieldCols{},
                          "", next_headers_m_id_, w_);
    }

    finished_scan_id_ = lidar_scan.frame_id;

    // reset counts
    batched_lidar_packets_ = 0;
    batched_imu_packets_ = 0;
    batched_zone_packets_ = 0;
}

void ScanBatcher::reset() { cached_packet_ = false; }

size_t ScanBatcher::batched_packets() {
    return batched_lidar_packets_ + batched_imu_packets_ +
           batched_zone_packets_;
}

FieldType::FieldType() = default;

FieldType::FieldType(const std::string& name_in, ChanFieldType element_type_in,
                     const std::vector<size_t> extra_dims_in,
                     FieldClass class_in)
    : name(name_in),
      element_type(element_type_in),
      extra_dims(extra_dims_in),
      field_class(class_in) {}

bool operator==(const FieldType& a, const FieldType& b) {
    return a.name == b.name && a.element_type == b.element_type &&
           a.field_class == b.field_class && a.extra_dims == b.extra_dims;
}

bool operator!=(const FieldType& a, const FieldType& b) {
    return a.name != b.name || a.element_type != b.element_type ||
           a.field_class != b.field_class || a.extra_dims != b.extra_dims;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
