/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/lidar_frame.h"

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

#include "ouster/core/impl/cartesian.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/impl/table.h"
#include "ouster/core/packet.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"
#include "ouster/core/xyzlut.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ouster::sdk::core::impl::float3x16_t;

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

LidarFrame::LidarFrame() = default;
LidarFrame::LidarFrame(const LidarFrame&) = default;
LidarFrame::LidarFrame(LidarFrame&&) noexcept = default;
LidarFrame& LidarFrame::operator=(const LidarFrame&) = default;
LidarFrame& LidarFrame::operator=(LidarFrame&&) noexcept = default;
LidarFrame::~LidarFrame() = default;

namespace impl {

using ouster::impl::Table;

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

static const Table<std::string, ChanFieldType, 6> RGB_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::RGB, ChanFieldType::FLOAT16},  // this is turned into a 3d array
    {ChanField::FLAGS, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 10> DUAL_RGB_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::RGB, ChanFieldType::FLOAT16},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
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

static const Table<std::string, ChanFieldType, 6> ZM_LB_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 7> ZM_SINGLE_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
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

static const Table<std::string, ChanFieldType, 10> ZM_DUAL_FIELD_SLOTS{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::FLAGS, ChanFieldType::UINT8},
    {ChanField::FLAGS2, ChanFieldType::UINT8},
    {ChanField::ZONE_MASK, ChanFieldType::UINT16},
    {ChanField::WINDOW, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 0> OFF_PROFILE_SLOTS{{}};

struct OUSTER_API_CLASS DefaultFieldsEntry {
    const std::pair<std::string, ChanFieldType>* fields;
    size_t n_fields;
};

// clang-format off
Table<UDPProfileLidar, DefaultFieldsEntry, MAX_NUM_PROFILES> default_frame_fields{
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
     {UDPProfileLidar::RNG19_RFL8_SIG16_ZONE16_DUAL,
      {ZM_DUAL_FIELD_SLOTS.data(), ZM_DUAL_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16,
      {RGB_FIELD_SLOTS.data(), RGB_FIELD_SLOTS.size()}},
     {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL,
      {DUAL_RGB_FIELD_SLOTS.data(), DUAL_RGB_FIELD_SLOTS.size()}},
}};
// clang-format on

namespace {

LidarFrameFieldTypes lookup_frame_fields(UDPProfileLidar profile) {
    auto end = impl::default_frame_fields.end();
    auto it = std::find_if(impl::default_frame_fields.begin(), end,
                           [profile](const auto& key_value) { return key_value.first == profile; });

    if (it == end || it->first == UDPProfileLidar::UNKNOWN) {
        throw std::invalid_argument("Unknown lidar udp profile");
    }

    auto entry = it->second;
    LidarFrameFieldTypes field_types;
    for (size_t i = 0; i < entry.n_fields; i++) {
        // everything is pixel field for now
        field_types.emplace_back(entry.fields[i].first, entry.fields[i].second,
                                 std::vector<size_t>(), FieldClass::PIXEL_FIELD);
    }

    // update RGB to 3D since the table doesn't support this information
    for (auto& field_type : field_types) {
        if (field_type.name == "RGB") {
            field_type.extra_dims.push_back(3);
        }
    }

    return field_types;
}

}  // namespace

bool raw_headers_enabled(const PacketFormat& packet_format, const LidarFrame& lidar_frame) {
    using ouster::sdk::core::logger;
    if (!lidar_frame.has_field(ChanField::RAW_HEADERS)) {
        return false;
    }

    auto raw_headers_ft = lidar_frame.field(ChanField::RAW_HEADERS).tag();
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

namespace {

FieldDescriptor get_field_type_descriptor(const LidarFrame& frame, const FieldType& field_type) {
    if (field_type.field_class == FieldClass::PIXEL_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(frame.h);
        dims.push_back(frame.w);
        dims.insert(dims.end(), field_type.extra_dims.begin(), field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else if (field_type.field_class == FieldClass::COLUMN_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(frame.w);
        dims.insert(dims.end(), field_type.extra_dims.begin(), field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else if (field_type.field_class == FieldClass::PACKET_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(frame.packet_count());
        dims.insert(dims.end(), field_type.extra_dims.begin(), field_type.extra_dims.end());
        return FieldDescriptor::array(field_type.element_type, dims);
    } else {  // FieldClass::FRAME_FIELD
        return FieldDescriptor::array(field_type.element_type, field_type.extra_dims);
    }
}

}  // namespace

LidarFrame::LidarFrame(const DataFormat& format)
    : LidarFrame{format.pixels_per_column, format.columns_per_frame,
                 get_field_types(format, Version(0, 0, 0)), format.columns_per_packet} {}

LidarFrame::LidarFrame(const SensorInfo& info) : LidarFrame(std::make_shared<SensorInfo>(info)) {}

LidarFrame::LidarFrame(std::shared_ptr<SensorInfo> info)
    : LidarFrame{info->format.pixels_per_column, info->format.columns_per_frame,
                 get_field_types(*info), info->format.columns_per_packet} {
    sensor_info = std::move(info);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

LidarFrame::LidarFrame(std::shared_ptr<SensorInfo> info, const std::vector<FieldType>& field_types)
    : LidarFrame{info->format.pixels_per_column, info->format.columns_per_frame, field_types,
                 info->format.columns_per_packet} {
    sensor_info = std::move(info);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

LidarFrame::LidarFrame(size_t h, size_t w, const LidarFrameFieldTypes& field_types,
                       size_t columns_per_packet)
    : w(w), h(h) {
    if (w * h == 0) {
        throw std::invalid_argument(
            "Cannot construct LidarFrame with "
            "zero width or height");
    }
    if (columns_per_packet == 0) {
        throw std::invalid_argument("columns_per_packet must be greater than 0");
    }

    // equivalent to int(ceil(w/columns_per_packet))
    this->packet_count_ = (w + columns_per_packet - 1) / columns_per_packet;

    for (const auto& field_type : field_types) {
        add_field(field_type);
    }

    timestamp_ = Field{fd_array<uint64_t>(w), FieldClass::COLUMN_FIELD};
    measurement_id_ = Field{fd_array<uint16_t>(w), FieldClass::COLUMN_FIELD};
    status_ = Field{fd_array<uint32_t>(w), FieldClass::COLUMN_FIELD};
    packet_timestamp_ = Field{fd_array<uint64_t>(packet_count_), FieldClass::PACKET_FIELD};
    body_to_world_ = Field{fd_array<double>(w, 4, 4), {}};
    alert_flags_ = Field{fd_array<uint8_t>(packet_count_), FieldClass::PACKET_FIELD};

    // Initialize all the poses to identity
    auto ptr = body_to_world_.get<double>();
    auto ident = mat4d::Identity().eval();
    for (size_t i = 0; i < w; ++i) {
        memcpy(&ptr[i * 4 * 4], ident.data(), static_cast<size_t>(4 * 4) * sizeof(double));
    }
}

LidarFrame::LidarFrame(const LidarFrame& other, const LidarFrameFieldTypes& fields)
    : timestamp_(other.timestamp_),
      measurement_id_(other.measurement_id_),
      status_(other.status_),
      packet_timestamp_(other.packet_timestamp_),
      body_to_world_(other.body_to_world_),
      alert_flags_(other.alert_flags_),
      packet_count_(other.packet_count_),
      w(other.w),
      h(other.h),
      frame_status(other.frame_status),
      frame_id(other.frame_id),
      sensor_info(other.sensor_info) {
    for (const auto& field_type : fields) {
        const std::string& name = field_type.name;
        FieldDescriptor dst_desc = get_field_type_descriptor(*this, field_type);
        if (other.has_field(name)) {
            // either copy cast or error
            const auto& src_field = other.field(name);
            const auto& src_desc = src_field.desc();
            if (src_desc == dst_desc) {
                this->fields()[name] = other.field(name);
            } else {
                // cast if the dimensions match
                if (dst_desc.shape != src_desc.shape) {
                    throw std::invalid_argument(
                        "Field '" + name +
                        "' from source frame has dimensions that don't match "
                        "desired.");
                }
                add_field(name, dst_desc, field_type.field_class);
                ouster::sdk::core::impl::visit_field(
                    *this, name, ouster::sdk::core::impl::copy_and_cast(), other, name);
            }
        } else {
            add_field(name, dst_desc, field_type.field_class);
        }
    }

    objects() = other.objects();
}

LidarFrame::LidarFrame(size_t h, size_t w, UDPProfileLidar profile, size_t columns_per_packet)
    : LidarFrame{h, w, impl::lookup_frame_fields(profile), columns_per_packet} {}

LidarFrame::LidarFrame(size_t h, size_t w)
    : LidarFrame{h, w, impl::lookup_frame_fields(UDPProfileLidar::LEGACY),
                 DEFAULT_COLUMNS_PER_PACKET} {}

ShotLimitingStatus LidarFrame::shot_limiting() const {
    return static_cast<ShotLimitingStatus>(
        (frame_status & FrameStatusMasks::FRAME_STATUS_SHOT_LIMITING_MASK) >>
        FrameStatusShifts::FRAME_STATUS_SHOT_LIMITING_SHIFT);
}

ThermalShutdownStatus LidarFrame::thermal_shutdown() const {
    return static_cast<ThermalShutdownStatus>(
        (frame_status & FrameStatusMasks::FRAME_STATUS_THERMAL_SHUTDOWN_MASK) >>
        FrameStatusShifts::FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT);
}

Field& LidarFrame::field(const std::string& name) {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in LidarFrame.");
    }
}

const Field& LidarFrame::field(const std::string& name) const {
    try {
        return fields().at(name);
    } catch (std::out_of_range& e) {
        throw std::out_of_range("Field '" + name + "' not found in LidarFrame.");
    }
}

bool LidarFrame::has_field(const std::string& name) const {
    return fields().count(name) > 0;
}

Field& LidarFrame::add_field(const FieldType& type) {
    if (has_field(type.name)) {
        throw std::invalid_argument("Duplicated field '" + type.name + "'");
    }

    // just validate that we didnt add a 0 size pixel field
    if (type.field_class == FieldClass::PIXEL_FIELD) {
        // none of the dimensions should be zero
        for (const auto& dim : type.extra_dims) {
            if (dim == 0) {
                throw std::invalid_argument("Cannot add pixel field with 0 elements.");
            }
        }
    }

    // no other checking is necessary
    fields()[type.name] = Field(get_field_type_descriptor(*this, type), type.field_class);

    return fields()[type.name];
}

Field& LidarFrame::add_field(const std::string& name, FieldDescriptor desc,
                             FieldClass field_class) {
    if (has_field(name)) {
        throw std::invalid_argument("Duplicated field '" + name + "'");
    }

    if (field_class == FieldClass::PIXEL_FIELD) {
        if (desc.shape.size() < 2) {
            throw std::invalid_argument("Pixel fields must have at least 2 dimensions");
        }
        if (desc.shape[0] != h || desc.shape[1] != w) {
            throw std::invalid_argument(
                "Pixel field shape must match "
                "LidarFrame's width and height. Was " +
                std::to_string(desc.shape[0]) + "x" + std::to_string(desc.shape[1]) + " vs " +
                std::to_string(h) + "x" + std::to_string(w));
        }
        // none of the dimensions should be zero
        for (const auto& dim : desc.shape) {
            if (dim == 0) {
                throw std::invalid_argument("Cannot add pixel field with 0 elements.");
            }
        }
    }

    if (field_class == FieldClass::COLUMN_FIELD) {
        if (desc.shape[0] != w) {
            throw std::invalid_argument(
                "Column field shape must match "
                "LidarFrame's height. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " + std::to_string(w));
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

Field LidarFrame::del_field(const std::string& name) {
    if (!has_field(name)) {
        throw std::invalid_argument("Attempted deleting non existing field '" + name + "'");
    }

    Field ptr;
    field(name).swap(ptr);
    fields().erase(name);
    return ptr;
}

// LidarFrame::field<T>(...) is defined inline in lidar_scan.h to work around
// an Apple Clang mangling bug for the dependent default StrideType expression
// in Eigen::Ref. See the note in lidar_scan.h. (float16_t and other types are
// covered by implicit instantiation wherever they are used.)

namespace {

FieldType get_field_type(const std::string& name, const Field& field) {
    int offset = 0;
    if (field.field_class() == FieldClass::PIXEL_FIELD) {
        offset = 2;
    } else if ((field.field_class() == FieldClass::COLUMN_FIELD) ||
               (field.field_class() == FieldClass::PACKET_FIELD)) {
        offset = 1;
    }
    std::vector<size_t> extra_dims;
    extra_dims.insert(extra_dims.begin(), field.shape().begin() + offset, field.shape().end());
    return FieldType(name, field.tag(), extra_dims, field.field_class());
}

}  // namespace

FieldType LidarFrame::field_type(const std::string& name) const {
    return get_field_type(name, field(name));
}

std::unordered_map<std::string, Field>& LidarFrame::fields() {
    return fields_;
}

const std::unordered_map<std::string, Field>& LidarFrame::fields() const {
    return fields_;
}

std::unordered_map<std::string, std::vector<Object>>& LidarFrame::objects() {
    return objects_;
}

const std::unordered_map<std::string, std::vector<Object>>& LidarFrame::objects() const {
    return objects_;
}

Eigen::Ref<LidarFrame::Header<uint64_t>> LidarFrame::timestamp() {
    return timestamp_;
}

Eigen::Ref<const LidarFrame::Header<uint64_t>> LidarFrame::timestamp() const {
    return timestamp_;
}

Eigen::Ref<LidarFrame::Header<uint64_t>> LidarFrame::packet_timestamp() {
    return packet_timestamp_;
}

Eigen::Ref<const LidarFrame::Header<uint64_t>> LidarFrame::packet_timestamp() const {
    return packet_timestamp_;
}

Eigen::Ref<LidarFrame::Header<uint8_t>> LidarFrame::alert_flags() {
    return alert_flags_;
}

Eigen::Ref<const LidarFrame::Header<uint8_t>> LidarFrame::alert_flags() const {
    return alert_flags_;
}

ArrayView1<ZoneState> LidarFrame::zones() {
    auto it = fields().find(ChanField::ZONE_STATES);

    if (it == fields().end()) {
        return ArrayView1<ZoneState>(nullptr, {0});
    }

    return it->second;
}

ConstArrayView1<ZoneState> LidarFrame::zones() const {
    auto it = fields().find(ChanField::ZONE_STATES);

    if (it == fields().end()) {
        return ConstArrayView1<ZoneState>(nullptr, {0});
    }

    return it->second;
}

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace ouster::sdk::core::ChanField;

namespace {
nonstd::optional<uint64_t> min_optional(nonstd::optional<uint64_t> a,
                                        nonstd::optional<uint64_t> b) {
    if (!a) {
        return b;
    }
    if (!b) {
        return a;
    }
    return std::min(*a, *b);
}

nonstd::optional<uint64_t> max_optional(nonstd::optional<uint64_t> a,
                                        nonstd::optional<uint64_t> b) {
    if (!a) {
        return b;
    }
    if (!b) {
        return a;
    }
    return std::max(*a, *b);
}

uint64_t ts_value_or_throw(nonstd::optional<uint64_t> time) {
    if (!time) {
        throw std::runtime_error("No valid packets in LidarFrame");
    }
    return *time;
}
}  // namespace

uint64_t LidarFrame::get_first_valid_packet_timestamp() const {
    nonstd::optional<uint64_t> time = get_first_valid_lidar_packet_timestamp_internal();
    time = min_optional(time, get_first_valid_imu_packet_timestamp_internal());
    time = min_optional(time, get_valid_zone_packet_timestamp_internal());
    return ts_value_or_throw(time);
}

uint64_t LidarFrame::get_first_valid_packet_timestamp(PacketType stream) const {
    switch (stream) {
        case PacketType::Lidar:
            return ts_value_or_throw(get_first_valid_lidar_packet_timestamp_internal());
        case PacketType::Imu:
            return ts_value_or_throw(get_first_valid_imu_packet_timestamp_internal());
        case PacketType::Zone:
            return ts_value_or_throw(get_valid_zone_packet_timestamp_internal());
        case PacketType::Unknown:
        default:
            throw std::runtime_error("No valid packets in LidarFrame");
    }
}

uint64_t LidarFrame::get_last_valid_packet_timestamp() const {
    nonstd::optional<uint64_t> time = get_last_valid_lidar_packet_timestamp_internal();
    time = max_optional(time, get_last_valid_imu_packet_timestamp_internal());
    time = max_optional(time, get_valid_zone_packet_timestamp_internal());
    return ts_value_or_throw(time);
}

uint64_t LidarFrame::get_last_valid_packet_timestamp(PacketType stream) const {
    switch (stream) {
        case PacketType::Lidar:
            return ts_value_or_throw(get_last_valid_lidar_packet_timestamp_internal());
        case PacketType::Imu:
            return ts_value_or_throw(get_last_valid_imu_packet_timestamp_internal());
        case PacketType::Zone:
            return ts_value_or_throw(get_valid_zone_packet_timestamp_internal());
        case PacketType::Unknown:
        default:
            throw std::runtime_error("No valid packets in LidarFrame");
    }
}

uint64_t LidarFrame::get_min_valid_packet_timestamp() const {
    nonstd::optional<uint64_t> time = get_min_valid_lidar_packet_timestamp_internal();
    time = min_optional(time, get_min_valid_imu_packet_timestamp_internal());
    time = min_optional(time, get_valid_zone_packet_timestamp_internal());
    return ts_value_or_throw(time);
}

uint64_t LidarFrame::get_min_valid_packet_timestamp(PacketType stream) const {
    switch (stream) {
        case PacketType::Lidar:
            return ts_value_or_throw(get_min_valid_lidar_packet_timestamp_internal());
        case PacketType::Imu:
            return ts_value_or_throw(get_min_valid_imu_packet_timestamp_internal());
        case PacketType::Zone:
            return ts_value_or_throw(get_valid_zone_packet_timestamp_internal());
        case PacketType::Unknown:
        default:
            throw std::runtime_error("No valid packets in LidarFrame");
    }
}

uint64_t LidarFrame::get_max_valid_packet_timestamp() const {
    nonstd::optional<uint64_t> time = get_max_valid_lidar_packet_timestamp_internal();
    time = max_optional(time, get_max_valid_imu_packet_timestamp_internal());
    time = max_optional(time, get_valid_zone_packet_timestamp_internal());
    return ts_value_or_throw(time);
}

uint64_t LidarFrame::get_max_valid_packet_timestamp(PacketType stream) const {
    switch (stream) {
        case PacketType::Lidar:
            return ts_value_or_throw(get_max_valid_lidar_packet_timestamp_internal());
        case PacketType::Imu:
            return ts_value_or_throw(get_max_valid_imu_packet_timestamp_internal());
        case PacketType::Zone:
            return ts_value_or_throw(get_valid_zone_packet_timestamp_internal());
        case PacketType::Unknown:
        default:
            throw std::runtime_error("No valid packets in LidarFrame");
    }
}

uint64_t LidarFrame::get_first_valid_lidar_packet_timestamp() const {
    return get_first_valid_lidar_packet_timestamp_internal().value_or(0);
}

uint64_t LidarFrame::get_last_valid_lidar_packet_timestamp() const {
    return get_last_valid_lidar_packet_timestamp_internal().value_or(0);
}

nonstd::optional<uint64_t> LidarFrame::get_first_valid_lidar_packet_timestamp_internal() const {
    int total_packets = static_cast<int>(packet_timestamp().size());
    int columns_per_packet = static_cast<int>(w) / total_packets;

    for (int i = 0; i < total_packets; ++i) {
        if (status()
                .middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1; })
                .any()) {
            return packet_timestamp()[i];
        }
    }

    return nonstd::nullopt;
}

nonstd::optional<uint64_t> LidarFrame::get_last_valid_lidar_packet_timestamp_internal() const {
    int total_packets = static_cast<int>(packet_timestamp().size());
    int columns_per_packet = static_cast<int>(w) / total_packets;

    for (int i = total_packets - 1; i >= 0; --i) {
        if (status()
                .middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1; })
                .any()) {
            return packet_timestamp()[i];
        }
    }

    return nonstd::nullopt;
}

nonstd::optional<uint64_t> LidarFrame::get_min_valid_lidar_packet_timestamp_internal() const {
    nonstd::optional<uint64_t> time;
    int total_packets = static_cast<int>(packet_timestamp().size());
    int columns_per_packet = static_cast<int>(w) / total_packets;

    for (int i = 0; i < total_packets; ++i) {
        if (status()
                .middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1; })
                .any()) {
            const uint64_t packet_time = packet_timestamp()[i];
            time = time ? std::min(*time, packet_time) : packet_time;
        }
    }
    return time;
}

nonstd::optional<uint64_t> LidarFrame::get_max_valid_lidar_packet_timestamp_internal() const {
    nonstd::optional<uint64_t> time;
    int total_packets = static_cast<int>(packet_timestamp().size());
    int columns_per_packet = static_cast<int>(w) / total_packets;

    for (int i = 0; i < total_packets; ++i) {
        if (status()
                .middleRows(static_cast<Eigen::Index>(i) * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t status_val) { return status_val & 1; })
                .any()) {
            const uint64_t packet_time = packet_timestamp()[i];
            time = time ? std::max(*time, packet_time) : packet_time;
        }
    }
    return time;
}

nonstd::optional<uint64_t> LidarFrame::get_first_valid_imu_packet_timestamp_internal() const {
    if (has_field(IMU_PACKET_TIMESTAMP) && has_field(IMU_STATUS)) {
        ConstArrayView<uint64_t, 1> data = field(IMU_PACKET_TIMESTAMP);
        Eigen::Ref<const Header<uint16_t>> status = field(IMU_STATUS);
        int icolumns_per_packet = static_cast<int>(status.size() / data.size());
        for (size_t i = 0; i < data.size(); i++) {
            if (status
                    .middleRows(static_cast<Eigen::Index>(i * icolumns_per_packet),
                                icolumns_per_packet)
                    .unaryExpr([](uint16_t status_val) { return status_val & 1; })
                    .any()) {
                return data(i);
            }
        }
    }
    return nonstd::nullopt;
}

nonstd::optional<uint64_t> LidarFrame::get_last_valid_imu_packet_timestamp_internal() const {
    if (has_field(IMU_PACKET_TIMESTAMP) && has_field(IMU_STATUS)) {
        ConstArrayView<uint64_t, 1> data = field(IMU_PACKET_TIMESTAMP);
        ConstArrayView<uint16_t, 1> status = field(IMU_STATUS);
        Eigen::Map<const Header<uint16_t>> estatus(status.data(),
                                                   static_cast<Eigen::Index>(status.size()));
        int icolumns_per_packet = static_cast<int>(status.size() / data.size());
        for (int i = static_cast<int>(data.size()) - 1; i >= 0; i--) {
            if (estatus
                    .middleRows(static_cast<Eigen::Index>(i) * icolumns_per_packet,
                                icolumns_per_packet)
                    .unaryExpr([](uint16_t status_val) { return status_val & 1; })
                    .any()) {
                return data(i);
            }
        }
    }
    return nonstd::nullopt;
}

nonstd::optional<uint64_t> LidarFrame::get_min_valid_imu_packet_timestamp_internal() const {
    nonstd::optional<uint64_t> time;
    if (has_field(IMU_PACKET_TIMESTAMP) && has_field(IMU_STATUS)) {
        ConstArrayView<uint64_t, 1> data = field(IMU_PACKET_TIMESTAMP);
        ConstArrayView<uint16_t, 1> status = field(IMU_STATUS);
        Eigen::Map<const Header<uint16_t>> estatus(status.data(),
                                                   static_cast<Eigen::Index>(status.size()));
        int icolumns_per_packet = static_cast<int>(status.size() / data.size());
        for (size_t i = 0; i < data.size(); i++) {
            if (estatus
                    .middleRows(static_cast<Eigen::Index>(i * icolumns_per_packet),
                                icolumns_per_packet)
                    .unaryExpr([](uint16_t status_val) { return status_val & 1; })
                    .any()) {
                const uint64_t packet_time = data(i);
                time = time ? std::min(*time, packet_time) : packet_time;
            }
        }
    }
    return time;
}

nonstd::optional<uint64_t> LidarFrame::get_max_valid_imu_packet_timestamp_internal() const {
    nonstd::optional<uint64_t> time;
    if (has_field(IMU_PACKET_TIMESTAMP) && has_field(IMU_STATUS)) {
        ConstArrayView<uint64_t, 1> data = field(IMU_PACKET_TIMESTAMP);
        ConstArrayView<uint16_t, 1> status = field(IMU_STATUS);
        Eigen::Map<const Header<uint16_t>> estatus(status.data(),
                                                   static_cast<Eigen::Index>(status.size()));
        int icolumns_per_packet = static_cast<int>(status.size() / data.size());
        for (size_t i = 0; i < data.size(); i++) {
            if (estatus
                    .middleRows(static_cast<Eigen::Index>(i * icolumns_per_packet),
                                icolumns_per_packet)
                    .unaryExpr([](uint16_t status_val) { return status_val & 1; })
                    .any()) {
                const uint64_t packet_time = data(i);
                time = time ? std::max(*time, packet_time) : packet_time;
            }
        }
    }
    return time;
}

nonstd::optional<uint64_t> LidarFrame::get_valid_zone_packet_timestamp_internal() const {
    if (has_field(ZONE_PACKET_TIMESTAMP)) {
        ConstArrayView<uint64_t, 1> data = field(ZONE_PACKET_TIMESTAMP);
        if (data(0) != 0) {
            return data(0);
        }
    }
    return nonstd::nullopt;
}

uint64_t column_timestamp_at_destaggered_pixel(
    size_t row, size_t col, const std::vector<int>& pixel_shift_by_row,
    const Eigen::Ref<const Eigen::Array<uint64_t, Eigen::Dynamic, 1>>& column_timestamps) {
    const size_t width = column_timestamps.size();
    if (row >= pixel_shift_by_row.size() || col >= width) {
        throw std::invalid_argument("row or column is out of range");
    }

    const int w = static_cast<int>(width);
    const int offset = (w + pixel_shift_by_row[row] % w) % w;
    const int staggered_col = (static_cast<int>(col) - offset + w) % w;
    return column_timestamps[staggered_col];
}

int LidarFrame::get_first_valid_column() const {
    auto stat = status();
    for (int i = 0; i < stat.size(); ++i) {
        if ((stat[i] & 1) > 0) {
            return i;
        }
    }
    throw std::runtime_error("No valid columns in LidarFrame");
}

int LidarFrame::get_last_valid_column() const {
    auto stat = status();
    for (int i = static_cast<int>(stat.size()) - 1; i >= 0; --i) {
        if ((stat[i] & 1) > 0) {
            return i;
        }
    }
    throw std::runtime_error("No valid columns in LidarFrame");
}

Eigen::Ref<LidarFrame::Header<uint16_t>> LidarFrame::measurement_id() {
    return measurement_id_;
}

Eigen::Ref<const LidarFrame::Header<uint16_t>> LidarFrame::measurement_id() const {
    return measurement_id_;
}

Eigen::Ref<LidarFrame::Header<uint32_t>> LidarFrame::status() {
    return status_;
}

Eigen::Ref<const LidarFrame::Header<uint32_t>> LidarFrame::status() const {
    return status_;
}

Field& LidarFrame::body_to_world() {
    return body_to_world_;
}

const Field& LidarFrame::body_to_world() const {
    return body_to_world_;
}

Field& LidarFrame::pose() {
    return body_to_world_;
}

const Field& LidarFrame::pose() const {
    return body_to_world_;
}

void LidarFrame::set_column_pose(int index, const Matrix4dR& pose) {
    if (index < 0 || index >= static_cast<int>(w)) {
        throw std::out_of_range("Column index out of range");
    }
    std::memcpy(
        static_cast<char*>(body_to_world_.get()) + static_cast<size_t>(index) * 16 * sizeof(double),
        pose.data(), sizeof(double) * 16);
}

Matrix4dR LidarFrame::get_column_pose(int index) const {
    if (index < 0 || index >= static_cast<int>(w)) {
        throw std::out_of_range("Column index out of range");
    }
    Matrix4dR out;
    std::memcpy(out.data(),
                static_cast<const char*>(body_to_world_.get()) +
                    static_cast<size_t>(index) * 16 * sizeof(double),
                sizeof(double) * 16);
    return out;
}

bool LidarFrame::complete(ColumnWindow window) const {
    const auto& status = this->status();
    auto start = window.first;
    auto end = window.second;

    if (start <= end) {
        return status.segment(start, end - start + 1)
            .unaryExpr([](uint32_t status_val) { return status_val & 0x01; })
            .isConstant(0x01);
    } else {
        return status.segment(0, end + 1)
                   .unaryExpr([](uint32_t status_val) { return status_val & 0x01; })
                   .isConstant(0x01) &&
               status.segment(start, this->w - start)
                   .unaryExpr([](uint32_t status_val) { return status_val & 0x01; })
                   .isConstant(0x01);
    }
}

bool LidarFrame::complete() const {
    if (!sensor_info) {
        throw std::runtime_error(
            "LidarFrame must have a valid SensorInfo in order to compute "
            "completeness");
    }
    return complete(sensor_info->format.column_window);
}

size_t LidarFrame::packet_count() const {
    return packet_count_;
}

bool LidarFrame::equals(const LidarFrame& other) const {
    return frame_id == other.frame_id && w == other.w && h == other.h &&
           frame_status == other.frame_status && measurement_id_ == other.measurement_id_ &&
           timestamp_ == other.timestamp_ && packet_timestamp_ == other.packet_timestamp_ &&
           body_to_world() == other.body_to_world() && fields() == other.fields() &&
           objects() == other.objects();
}

bool operator==(const LidarFrame& a, const LidarFrame& b) {
    return a.equals(b);
}

bool operator!=(const LidarFrame& a, const LidarFrame& b) {
    return !(a == b);
}

LidarFrameFieldTypes LidarFrame::field_types() const {
    LidarFrameFieldTypes field_types_vec;
    for (const auto& kv : fields()) {
        field_types_vec.push_back(get_field_type(kv.first, kv.second));
    }

    std::sort(field_types_vec.begin(), field_types_vec.end());
    return field_types_vec;
}

LidarFrameFieldTypes get_field_types(const DataFormat& format, const Version& fw_version) {
    LidarFrameFieldTypes field_types = impl::lookup_frame_fields(format.udp_profile_lidar);

    size_t imu_measurements =
        static_cast<size_t>(format.imu_packets_per_frame) * format.imu_measurements_per_packet;

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
                                 std::vector<size_t>{imu_measurements, 3}, FieldClass::FRAME_FIELD);
        field_types.emplace_back(IMU_GYRO, ChanFieldType::FLOAT32,
                                 std::vector<size_t>{imu_measurements, 3}, FieldClass::FRAME_FIELD);
        field_types.emplace_back(IMU_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{imu_measurements}, FieldClass::FRAME_FIELD);
        field_types.emplace_back(IMU_MEASUREMENT_ID, ChanFieldType::UINT16,
                                 std::vector<size_t>{imu_measurements}, FieldClass::FRAME_FIELD);
        field_types.emplace_back(IMU_STATUS, ChanFieldType::UINT16,
                                 std::vector<size_t>{imu_measurements}, FieldClass::FRAME_FIELD);
        field_types.emplace_back(IMU_PACKET_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{format.imu_packets_per_frame},
                                 FieldClass::FRAME_FIELD);
        field_types.emplace_back(
            POSITION_STRING, ChanFieldType::CHAR,
            std::vector<size_t>{format.imu_packets_per_frame, NMEA_SENTENCE_LENGTH},
            FieldClass::FRAME_FIELD);
        field_types.emplace_back(POSITION_LAT_LONG, ChanFieldType::FLOAT64,
                                 std::vector<size_t>{format.imu_packets_per_frame, 2},
                                 FieldClass::FRAME_FIELD);
        field_types.emplace_back(POSITION_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{format.imu_packets_per_frame},
                                 FieldClass::FRAME_FIELD);
        field_types.emplace_back(IMU_ALERT_FLAGS, ChanFieldType::UINT8,
                                 std::vector<size_t>{format.imu_packets_per_frame},
                                 FieldClass::FRAME_FIELD);
    }

    if (format.zone_monitoring_enabled) {
        field_types.emplace_back(LIVE_ZONESET_HASH, ChanFieldType::UINT8, std::vector<size_t>{32},
                                 FieldClass::FRAME_FIELD);
        field_types.emplace_back(ZONE_TIMESTAMP, ChanFieldType::UINT64, std::vector<size_t>{1},
                                 FieldClass::FRAME_FIELD);
        field_types.emplace_back(ZONE_PACKET_TIMESTAMP, ChanFieldType::UINT64,
                                 std::vector<size_t>{1}, FieldClass::FRAME_FIELD);
        field_types.emplace_back(ZONE_ALERT_FLAGS, ChanFieldType::UINT8, std::vector<size_t>{1},
                                 FieldClass::FRAME_FIELD);
        field_types.emplace_back(ZONE_STATES, ChanFieldType::ZONE_STATE, std::vector<size_t>{16},
                                 FieldClass::FRAME_FIELD);
    }

    // remove WINDOW if FW is < 3.2 or if zone and < 3.2.1
    bool is_zone = false;
    if (format.udp_profile_lidar == UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16 ||
        format.udp_profile_lidar == UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16) {
        is_zone = true;
    }
    if (fw_version < Version(3, 2, 0) || (fw_version < Version(3, 2, 1) && is_zone)) {
        for (size_t i = 0; i < field_types.size(); i++) {
            if (field_types[i].name == ouster::sdk::core::ChanField::WINDOW) {
                field_types.erase(field_types.begin() + static_cast<std::ptrdiff_t>(i));
                break;
            }
        }
    }

    return field_types;
}

LidarFrameFieldTypes get_field_types(const SensorInfo& info) {
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

std::string to_string(const LidarFrameFieldTypes& field_types) {
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

std::string to_string(const LidarFrame& lidar_frame) {
    std::stringstream string_stream;
    LidarFrameFieldTypes field_types = lidar_frame.field_types();
    string_stream << "LidarFrame: {h = " << lidar_frame.h << ", w = " << lidar_frame.w
                  << ", packets_per_frame = " << lidar_frame.packet_timestamp().size()
                  << ", fid = " << lidar_frame.frame_id << "," << std::endl
                  << " frame status = " << std::hex << lidar_frame.frame_status << std::dec
                  << ", thermal_shutdown status = " << to_string(lidar_frame.thermal_shutdown())
                  << ", shot_limiting status = " << to_string(lidar_frame.shot_limiting()) << ","
                  << std::endl
                  << "  field_types = " << to_string(field_types) << "," << std::endl;

    auto read_eigen = [](auto ref, std::stringstream& string_stream) {
        string_stream << "min: " << static_cast<double>(ref.minCoeff())
                      << "; mean: " << ref.template cast<double>().mean()
                      << "; max: " << static_cast<double>(ref.maxCoeff());
    };

    auto read_field = [&read_eigen](const Field& field_val, const std::string& name,
                                    std::stringstream& string_stream) {
        string_stream << "     " << name << " type:" << to_string(field_val.tag()) << " shape: (";

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
                const char* end = static_cast<const char*>(memchr(data, '\0', max_len));
                size_t len = (end != nullptr) ? static_cast<size_t>(end - data) : max_len;
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

    for (auto&& kv : lidar_frame.fields()) {
        read_field(kv.second, kv.first, string_stream);
    }

    string_stream << "}";
    return string_stream.str();
}

FrameBatcher::FrameBatcher(const std::shared_ptr<SensorInfo>& info)
    : pf(get_format(*info)), cache_{pf}, last_init_id_{info->init_id}, sensor_info_{info} {
    if (info->format.columns_per_packet == 0) {
        throw std::invalid_argument("unexpected columns_per_packet: 0");
    }
    if (info->format.pixels_per_column == 0) {
        throw std::invalid_argument("unexpected pixels_per_column: 0");
    }
    // Calculate the number of packets required to have a complete frame
    expected_lidar_packets_ = info->format.lidar_packets_per_frame();
    if (info->format.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        expected_imu_packets_ = info->format.imu_packets_per_frame;
    }
    if (info->format.zone_monitoring_enabled) {
        expected_zone_packets_ = 1;
    }
}

FrameBatcher::FrameBatcher(const SensorInfo& info)
    : FrameBatcher(std::make_shared<SensorInfo>(info)) {}

namespace {

/*
 * Zero out all measurement block headers in range [start, end)
 */
void zero_header_cols(LidarFrame& lidar_frame, std::ptrdiff_t start, std::ptrdiff_t end) {
    lidar_frame.timestamp().segment(start, end - start).setZero();
    lidar_frame.measurement_id().segment(start, end - start).setZero();
    lidar_frame.status().segment(start, end - start).setZero();
}

/*
 * Generic operation to read a channel field from a packet measurement block
 * into a frame
 */
struct ParseFieldCol {
    template <typename T, size_t NDim>
    void operator()(ArrayView<T, NDim> field, const std::string& field_name, uint16_t m_id,
                    const PacketFormat& packet_format, const uint8_t* col_buf) const {
        // RAW_HEADERS field is populated separately because it has
        // a different processing scheme and doesn't fit into existing field
        // model (i.e. data packed per column rather than per pixel)
        if (field_name == ChanField::RAW_HEADERS) {
            return;
        }

        // RGB is stored in LidarFrame as H x W x 3 float16_t values, but the
        // packet helpers currently read/write each pixel as one packed 3x16-bit
        // element. This cast relies on those layouts matching.
        // TODO: teach PacketFormat about multi-element fields directly.
        if (NDim == 3) {
            packet_format.col_field(
                col_buf, field_name,
                reinterpret_cast<float3x16_t*>(field.subview(keep(), m_id).data()), field.shape[1]);
        } else {
            packet_format.col_field(col_buf, field_name, field.subview(keep(), m_id).data(),
                                    field.shape[1]);
        }
    }
};

uint64_t frame_status(const uint8_t thermal_shutdown, const uint8_t shot_limiting) {
    uint64_t res = 0;

    // clang-format off
    res |= (thermal_shutdown & 0x0f)
        << FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT;  // right nibble is thermal
                                                 // shutdown status, apply mask
                                                 // for safety, then shift
    // clang-format on
    res |= (shot_limiting & 0x0f) << FRAME_STATUS_SHOT_LIMITING_SHIFT;  // right nibble is shot
                                                                        // limiting, apply mask for
                                                                        // safety, then shift
    return res;
}

/**
 * Pack the lidar packet and column headers and footer into a RAW_HEADERS field.
 */
struct PackRawHeadersCol {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> rh_field, const std::string& /*unused*/,
                    const PacketFormat& packet_format, uint16_t col_idx,
                    const uint8_t* packet_buf) const {
        const uint8_t* col_buf = packet_format.nth_col(col_idx, packet_buf);
        const uint16_t m_id = packet_format.col_measurement_id(col_buf);

        using ColMajorView = Eigen::Map<const Eigen::Array<T, -1, 1, Eigen::ColMajor>>;

        const ColMajorView col_header_vec(reinterpret_cast<const T*>(col_buf),
                                          packet_format.col_header_size / sizeof(T));

        rh_field.block(0, m_id, col_header_vec.size(), 1) = col_header_vec;

        const ColMajorView col_footer_vec(
            reinterpret_cast<const T*>(col_buf + packet_format.col_size -
                                       packet_format.col_footer_size),
            packet_format.col_footer_size / sizeof(T));

        rh_field.block(col_header_vec.size(), m_id, col_footer_vec.size(), 1) = col_footer_vec;

        const ColMajorView packet_header_vec(reinterpret_cast<const T*>(packet_buf),
                                             packet_format.packet_header_size / sizeof(T));

        rh_field.block(col_header_vec.size() + col_footer_vec.size(), m_id,
                       packet_header_vec.size(), 1) = packet_header_vec;

        const ColMajorView packet_footer_vec(
            reinterpret_cast<const T*>(packet_format.footer(packet_buf)),
            packet_format.packet_footer_size / sizeof(T));

        rh_field.block(col_header_vec.size() + col_footer_vec.size() + packet_header_vec.size(),
                       m_id, packet_footer_vec.size(), 1) = packet_footer_vec;
    }
};

void memset_float16(void* mem, uint16_t val, size_t count) {
    auto dst = static_cast<ouster::sdk::core::float16_t*>(mem);
    const ouster::sdk::core::float16_t fill{val};
    std::fill_n(dst, count, fill);
}

void zero_field(LidarFrame& frame, const std::string& name, std::ptrdiff_t start,
                std::ptrdiff_t end, bool check_exists = true) {
    if (check_exists && !frame.has_field(name)) {
        return;
    }

    if (start == end) {
        return;
    }

    auto& field = frame.field(name);

    // zero the specified area of each column
    auto slice_size = (end - start) * field.desc().element_size;
    auto start_ptr = static_cast<uint8_t*>(field.get<void>());
    auto row_size = field.desc().element_size;
    for (size_t i = 1; i < field.shape().size(); i++) {
        row_size *= field.shape()[i];
    }
    auto byte_offset = field.desc().element_size * start;
    for (size_t i = 2; i < field.shape().size(); i++) {
        slice_size *= field.shape()[i];
        byte_offset *= field.shape()[i];
    }
    start_ptr += byte_offset;
    // if the field is a float16, special case the zeroing to NaN
    if (field.tag() == ChanFieldType::FLOAT16) {
        for (size_t i = 0; i < field.shape()[0]; i++) {
            memset_float16(start_ptr + (i * row_size), 0x7e00, slice_size >> 1);
        }
        return;
    }

    for (size_t i = 0; i < field.shape()[0]; i++) {
        memset(start_ptr + i * row_size, 0, slice_size);
    }
}

void zero_fields(LidarFrame& frame, const PacketFormat& packet_format, std::ptrdiff_t start,
                 std::ptrdiff_t end) {
    for (const auto& field_type : packet_format) {
        if (!frame.has_field(field_type.first)) {
            continue;
        }

        zero_field(frame, field_type.first, start, end, false);
    }
}

}  // namespace

void FrameBatcher::parse_by_col(const uint8_t* packet_buf, LidarFrame& lidar_frame) {
    const bool raw_headers = impl::raw_headers_enabled(pf, lidar_frame);
    for (uint32_t icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint64_t timestamp_val = pf.col_timestamp(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01) != 0u;

        // drop out-of-bounds data in case of misconfiguration
        if (m_id >= lidar_frame.w) {
            continue;
        }

        if (raw_headers) {
            // zero out missing columns if we jumped forward
            if (m_id >= next_headers_m_id_) {
                zero_field(lidar_frame, ChanField::RAW_HEADERS, next_headers_m_id_, m_id);
                next_headers_m_id_ = m_id + 1;
            }

            impl::visit_field(lidar_frame, ChanField::RAW_HEADERS, PackRawHeadersCol(),
                              ChanField::RAW_HEADERS, pf, icol, packet_buf);
        }

        // drop invalid
        if (!valid) {
            continue;
        }

        // zero out missing columns if we jumped forward
        if (m_id >= next_valid_m_id_) {
            zero_fields(lidar_frame, pf, next_valid_m_id_, m_id);
            zero_header_cols(lidar_frame, next_valid_m_id_, m_id);
            next_valid_m_id_ = m_id + 1;
        }

        // write new header values
        lidar_frame.timestamp()[m_id] = timestamp_val;
        lidar_frame.measurement_id()[m_id] = m_id;
        lidar_frame.status()[m_id] = status;

        impl::foreach_channel_field_ndim(lidar_frame, pf, ParseFieldCol{}, m_id, pf, col_buf);
    }
}

/*
 * Faster version of ParseFieldCol that works by blocks instead and skips
 * extra checks
 */
template <int BlockDim>
struct ParseFieldBlock {
    template <typename T, size_t NDim>
    void operator()(ArrayView<T, NDim> field, const std::string& field_name,
                    const PacketFormat& packet_format, const uint8_t* packet_buf) const {
        // RGB is stored in LidarFrame as H x W x 3 float16_t values, but the
        // packet helpers currently read/write each pixel as one packed 3x16-bit
        // element. This cast relies on those layouts matching.
        // TODO: teach PacketFormat about multi-element fields directly.
        if (NDim == 3) {
            packet_format.block_field<float3x16_t, BlockDim>(
                reinterpret_cast<impl::float3x16_t*>(field.data()), field.shape[1], field_name,
                packet_buf);
        } else {
            packet_format.block_field<T, BlockDim>(field.data(), field.shape[1], field_name,
                                                   packet_buf);
        }
    }
};

void FrameBatcher::parse_by_block(const uint8_t* packet_buf, LidarFrame& lidar_frame) {
    // zero out missing columns if we jumped forward
    const uint16_t first_m_id = pf.col_measurement_id(pf.nth_col(0, packet_buf));
    if (first_m_id >= next_valid_m_id_) {
        zero_fields(lidar_frame, pf, next_valid_m_id_, first_m_id);
        zero_header_cols(lidar_frame, next_valid_m_id_, first_m_id);
        next_valid_m_id_ = first_m_id + pf.columns_per_packet;
    }

    // write new header values
    auto timestamp = lidar_frame.timestamp();
    auto measurement_id = lidar_frame.measurement_id();
    auto status = lidar_frame.status();
    for (uint32_t icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const auto m_id = pf.col_measurement_id(col_buf);

        measurement_id[m_id] = m_id;
        timestamp[m_id] = pf.col_timestamp(col_buf);
        status[m_id] = pf.col_status(col_buf);
    }

    switch (pf.block_parsable()) {
        case 16:
            impl::foreach_channel_field_ndim(lidar_frame, pf, ParseFieldBlock<16>{}, pf,
                                             packet_buf);
            break;
        case 8:
            impl::foreach_channel_field_ndim(lidar_frame, pf, ParseFieldBlock<8>{}, pf, packet_buf);
            break;
        case 4:
            impl::foreach_channel_field_ndim(lidar_frame, pf, ParseFieldBlock<4>{}, pf, packet_buf);
            break;
        default:
            throw std::invalid_argument("Invalid block dim for packet format");
    }
}

void FrameBatcher::batch_lidar_packet(const LidarPacket& packet, LidarFrame& lidar_frame) {
    const uint8_t* packet_buf = packet.buf.data();

    // handling packet level data: packet_timestamp
    const uint8_t* col0_buf = pf.nth_col(0, packet_buf);
    const uint16_t packet_id = pf.col_measurement_id(col0_buf) / pf.columns_per_packet;
    if (packet_id < lidar_frame.packet_timestamp().rows()) {
        lidar_frame.packet_timestamp()[packet_id] = packet.host_timestamp;
        lidar_frame.alert_flags()[packet_id] = pf.alert_flags(packet_buf);
    }

    // handling column and pixel level data
    size_t block_parsable = pf.block_parsable();
    for (uint32_t icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01) != 0u;

        if (!valid || m_id >= lidar_frame.w) {
            block_parsable = 0;
            break;
        }
    }

    // validate that we actually can block parse it
    // this requires that each block be block_parsable away from the end of
    // frame
    if (block_parsable != 0) {
        for (uint32_t icol = 0; icol < pf.columns_per_packet; icol += block_parsable) {
            const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
            const uint16_t m_id = pf.col_measurement_id(col_buf);
            if (m_id + block_parsable > lidar_frame.w) {
                block_parsable = 0;
                break;
            }
        }
    }

    if ((block_parsable != 0) && !impl::raw_headers_enabled(pf, lidar_frame)) {
        parse_by_block(packet_buf, lidar_frame);
    } else {
        parse_by_col(packet_buf, lidar_frame);
    }

    batched_lidar_packets_++;
}

void FrameBatcher::batch_imu_packet(const ImuPacket& packet, LidarFrame& lidar_frame) {
    using namespace ouster::sdk::core::ChanField;
    const uint8_t* buf = packet.buf.data();

    // get packet_id from imu measurement id
    size_t imu_first_m_id = pf.col_measurement_id(pf.imu_nth_measurement(0, buf));
    uint16_t packet_id = imu_first_m_id / (lidar_frame.w / pf.imu_packets_per_frame);
    size_t col_offset = packet_id * pf.imu_measurements_per_packet;

    // TODO: this pattern could be a method -- Tim T.
    FieldView imu_ts_fview =
        lidar_frame.has_field(IMU_TIMESTAMP) ? lidar_frame.field(IMU_TIMESTAMP) : FieldView{};
    FieldView imu_m_id_fview = lidar_frame.has_field(IMU_MEASUREMENT_ID)
                                   ? lidar_frame.field(IMU_MEASUREMENT_ID)
                                   : FieldView{};
    FieldView imu_status_fview =
        lidar_frame.has_field(IMU_STATUS) ? lidar_frame.field(IMU_STATUS) : FieldView{};

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

    if (lidar_frame.has_field(IMU_ACC)) {
        pf.parse_accel(col_offset, buf, lidar_frame.field(IMU_ACC));
    }

    if (lidar_frame.has_field(IMU_GYRO)) {
        pf.parse_gyro(col_offset, buf, lidar_frame.field(IMU_GYRO));
    }

    if (lidar_frame.has_field(IMU_PACKET_TIMESTAMP)) {
        ArrayView1<uint64_t> packet_timestamp = lidar_frame.field(IMU_PACKET_TIMESTAMP);
        packet_timestamp(packet_id) = packet.host_timestamp;
    }

    if (lidar_frame.has_field(IMU_ALERT_FLAGS)) {
        ArrayView1<uint8_t> alert_flags = lidar_frame.field(IMU_ALERT_FLAGS);
        alert_flags(packet_id) = pf.alert_flags(buf);
    }

    auto sentence = pf.imu_nmea_sentence(buf);

    if (lidar_frame.has_field(POSITION_STRING)) {
        ArrayView2<char> nmea_sentences = lidar_frame.field(POSITION_STRING);
        std::memcpy(nmea_sentences.subview(packet_id).data(), sentence.data(), sentence.size());
    }

    if (lidar_frame.has_field(POSITION_TIMESTAMP)) {
        ArrayView1<uint64_t> nmea_ts = lidar_frame.field(POSITION_TIMESTAMP);
        nmea_ts(packet_id) = pf.imu_nmea_ts(buf);
    }

    if (lidar_frame.has_field(POSITION_LAT_LONG)) {
        ArrayView2<double> lat_long = lidar_frame.field(POSITION_LAT_LONG);
        if (!parse_lat_long(sentence, lat_long(packet_id, 0), lat_long(packet_id, 1))) {
            lat_long(packet_id, 0) = std::numeric_limits<double>::quiet_NaN();
            lat_long(packet_id, 1) = std::numeric_limits<double>::quiet_NaN();
        }
    }

    batched_imu_packets_++;
}

void FrameBatcher::batch_zone_packet(const ZonePacket& packet, LidarFrame& lidar_frame) {
    using namespace ouster::sdk::core::ChanField;
    const uint8_t* buf = packet.buf.data();

    if (lidar_frame.has_field(ZONE_ALERT_FLAGS)) {
        ArrayView1<uint8_t> alert_flags = lidar_frame.field(ZONE_ALERT_FLAGS);
        alert_flags(0) = pf.alert_flags(buf);
    }
    if (lidar_frame.has_field(ZONE_TIMESTAMP)) {
        ArrayView1<uint64_t> zone_ts = lidar_frame.field(ZONE_TIMESTAMP);
        zone_ts(0) = pf.zone_timestamp(buf);
    }
    if (lidar_frame.has_field(ZONE_PACKET_TIMESTAMP)) {
        ArrayView1<uint64_t> zone_packet_ts = lidar_frame.field(ZONE_PACKET_TIMESTAMP);
        zone_packet_ts(0) = packet.host_timestamp;
    }
    if (lidar_frame.has_field(LIVE_ZONESET_HASH)) {
        std::array<uint8_t, 32> hash = pf.live_zoneset_hash(buf);
        std::memcpy(lidar_frame.field(LIVE_ZONESET_HASH), hash.data(),
                    sizeof(uint8_t) * hash.size());
    }
    if (lidar_frame.has_field(ZONE_STATES)) {
        ArrayView1<ZoneState> zones = lidar_frame.field(ZONE_STATES);
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

void FrameBatcher::batch_packet(const Packet& packet, LidarFrame& lidar_frame) {
    if (pf.udp_profile_lidar == UDPProfileLidar::LEGACY || packet.type() == PacketType::Lidar) {
        batch_lidar_packet(packet.as<LidarPacket>(), lidar_frame);
    } else if (pf.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA &&
               packet.type() == PacketType::Imu) {
        batch_imu_packet(packet.as<ImuPacket>(), lidar_frame);
    } else if (packet.type() == PacketType::Zone) {
        batch_zone_packet(packet.as<ZonePacket>(), lidar_frame);
    }
}

void FrameBatcher::start_frame(int64_t f_id, const uint8_t* packet_buf, LidarFrame& lidar_frame) {
    // expecting to start batching a new frame
    finished_frame_id_ = -1;
    next_valid_m_id_ = 0;
    next_headers_m_id_ = 0;
    batched_lidar_packets_ = 0;
    batched_imu_packets_ = 0;
    batched_zone_packets_ = 0;
    lidar_frame.frame_id = f_id;
    zero_header_cols(lidar_frame, static_cast<ptrdiff_t>(0), static_cast<ptrdiff_t>(lidar_frame.w));
    lidar_frame.packet_timestamp().setZero();

    // zero IMU timestamps, statuses, and measurement ids in case this frame is
    // re-used
    for (auto& field_name :
         {IMU_PACKET_TIMESTAMP, IMU_TIMESTAMP, IMU_MEASUREMENT_ID, IMU_STATUS, IMU_ALERT_FLAGS,
          IMU_ACC, IMU_GYRO, ZONE_ALERT_FLAGS, ZONE_PACKET_TIMESTAMP, ZONE_TIMESTAMP,
          LIVE_ZONESET_HASH, ZONE_STATES}) {
        if (lidar_frame.has_field(field_name)) {
            lidar_frame.field(field_name).set_zero();
        }
    }

    auto f_thermal_shut = static_cast<uint8_t>(pf.thermal_shutdown(packet_buf));
    auto f_shot_limiting = static_cast<uint8_t>(pf.shot_limiting(packet_buf));
    lidar_frame.frame_status = frame_status(f_thermal_shut, f_shot_limiting);

    // The countdown values are supposed to be the same for all packets in a
    // given frame.
    lidar_frame.shutdown_countdown = pf.countdown_thermal_shutdown(packet_buf);
    lidar_frame.shot_limiting_countdown = pf.countdown_shot_limiting(packet_buf);
    lidar_frame.sensor_info = sensor_info_;
}

bool FrameBatcher::batch_with_caching(const Packet& packet, LidarFrame& lidar_frame) {
    cache_packet(packet);

    while (!cache_.empty()) {
        auto& top = cache_.top();

        const uint8_t* packet_buf = top->buf.data();
        const int64_t f_id = pf.frame_id(packet_buf);

        if (finished_frame_id_ >= 0 && pf.frame_id_difference(finished_frame_id_, f_id) <= 0) {
            // packet from an old frame, drop it and keep looking
            dropped_packets_++;
            cache_.pop();
            continue;
        }

        if (lidar_frame.frame_id == -1 || finished_frame_id_ >= 0) {
            start_frame(f_id, packet_buf, lidar_frame);
        }

        auto f_id_diff = pf.frame_id_difference(lidar_frame.frame_id, f_id);
        if (f_id_diff < 0) {
            // packet from older frame (which shouldn't happen because we have a
            // priority queue, but just in case), drop it and keep looking
            dropped_packets_++;
            cache_.pop();
        } else if (f_id_diff > 0) {
            if (cache_.size() >= max_cache_size_) {
                // we hit the cache size limit and still haven't found
                // a packet for the current frame, so we finalize what we have
                // and move on
                finalize_frame(lidar_frame);
                return true;
            }
            // packet belongs to a future frame
            return false;
        } else {
            batch_packet(*top, lidar_frame);
            cache_.pop();

            // if we have enough packets and are packet-complete release the
            // frame
            if (check_frame_complete(lidar_frame)) {
                finalize_frame(lidar_frame);
                return true;
            }
        }
    }

    return false;
}

bool FrameBatcher::handle_init_id_change(const Packet& packet, LidarFrame& lidar_frame) {
    // If the init id has changed, it's because the sensor configuration has
    // changed or reinitialized. The frame id counter will have changed too,
    // so we'll need to clear the cache.
    last_init_id_ = pf.init_id(packet.buf.data());
    if (lidar_frame.frame_id == -1 || finished_frame_id_ >= 0) {
        // if we have no active frame, just start the new one
        reset();
        reset_frame_ = false;  // but we don't want to start the _next_ packet
        const uint8_t* packet_buf = packet.buf.data();
        const int64_t f_id = pf.frame_id(packet_buf);
        start_frame(f_id, packet_buf, lidar_frame);
        batch_packet(packet, lidar_frame);
        if (check_frame_complete(lidar_frame)) {
            finalize_frame(lidar_frame);
            return true;
        }
        return false;
    }

    // if we have an active frame, finalize it before resetting
    finalize_frame(lidar_frame);
    reset();

    // we have to cache this packet if we don't want to lose it
    cache_packet(packet);
    return true;
}

bool FrameBatcher::batch(const Packet& packet, LidarFrame& lidar_frame) {
    if (reset_frame_) {
        // This will trigger the start of a new frame.
        lidar_frame.frame_id = -1;
        reset_frame_ = false;
    }

    if (packet.type() == PacketType::Imu &&
        pf.udp_profile_imu != UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        return false;
    }

    if (lidar_frame.w != sensor_info_->format.columns_per_frame ||
        lidar_frame.h != sensor_info_->format.pixels_per_column) {
        throw std::invalid_argument("unexpected frame dimensions");
    }
    if (static_cast<size_t>(lidar_frame.packet_timestamp().rows()) !=
        lidar_frame.w / pf.columns_per_packet) {
        throw std::invalid_argument("unexpected frame columns_per_packet: " +
                                    std::to_string(pf.columns_per_packet));
    }

    if (pf.udp_profile_lidar != UDPProfileLidar::LEGACY &&
        pf.init_id(packet.buf.data()) != last_init_id_) {
        return handle_init_id_change(packet, lidar_frame);
    }

    const uint8_t* packet_buf = packet.buf.data();
    const int64_t f_id = pf.frame_id(packet_buf);

    if (cache_.empty()) {
        if (finished_frame_id_ >= 0 && pf.frame_id_difference(finished_frame_id_, f_id) <= 0) {
            // packet from an old frame, drop it
            dropped_packets_++;
            return false;
        }

        if (lidar_frame.frame_id == -1 || finished_frame_id_ >= 0) {
            // Start a new frame if we just finished one.
            start_frame(f_id, packet_buf, lidar_frame);
            batch_packet(packet, lidar_frame);
            if (check_frame_complete(lidar_frame)) {
                finalize_frame(lidar_frame);
                return true;
            }
            return false;
        }
    }

    if (lidar_frame.frame_id == f_id && finished_frame_id_ < 0) {
        // If this packet belongs to the current frame, add it.
        batch_packet(packet, lidar_frame);
        if (check_frame_complete(lidar_frame)) {
            finalize_frame(lidar_frame);
            return true;
        }
        return false;
    }

    return batch_with_caching(packet, lidar_frame);
}

bool FrameBatcher::operator()(const Packet& packet, LidarFrame& lidar_frame) {
    return batch(packet, lidar_frame);
}

void FrameBatcher::cache_packet(const Packet& packet) {
    cache_.push(std::make_unique<Packet>(packet));
}

bool FrameBatcher::check_frame_complete(const LidarFrame& lidar_frame) const {
    bool lidar_batching_finished =
        (pf.udp_profile_lidar == UDPProfileLidar::OFF) ||
        (batched_lidar_packets_ >= expected_lidar_packets_ &&
         static_cast<size_t>(lidar_frame.packet_timestamp().count()) == expected_lidar_packets_);
    bool imu_batching_finished = batched_imu_packets_ >= expected_imu_packets_;
    bool zone_batching_finished = batched_zone_packets_ >= expected_zone_packets_;
    bool complete = lidar_batching_finished && imu_batching_finished && zone_batching_finished;
    return complete;
}

void FrameBatcher::finalize_frame(LidarFrame& lidar_frame) {
    if (next_valid_m_id_ < lidar_frame.w) {
        zero_fields(lidar_frame, pf, next_valid_m_id_, static_cast<std::ptrdiff_t>(lidar_frame.w));
    }

    if (impl::raw_headers_enabled(pf, lidar_frame)) {
        zero_field(lidar_frame, ChanField::RAW_HEADERS, next_headers_m_id_,
                   static_cast<std::ptrdiff_t>(lidar_frame.w));
    }

    if (lidar_frame.sensor_info->init_id == last_init_id_ &&
        lidar_frame.frame_id <= last_frame_id_ && pf.header_type == HeaderType::FUSA) {
        throw std::runtime_error("32-bit frame id did not increase since the last frame");
    }

    finished_frame_id_ = lidar_frame.frame_id;
    last_frame_id_ = lidar_frame.frame_id;

    // reset counts
    batched_lidar_packets_ = 0;
    batched_imu_packets_ = 0;
    batched_zone_packets_ = 0;
}

void FrameBatcher::reset() {
    reset_frame_ = true;
    finished_frame_id_ = -1;
    next_valid_m_id_ = 0;
    next_headers_m_id_ = 0;
    batched_lidar_packets_ = 0;
    batched_imu_packets_ = 0;
    batched_zone_packets_ = 0;
    while (!cache_.empty()) {
        cache_.pop();
    }
}

size_t FrameBatcher::batched_packets() const {
    return batched_lidar_packets_ + batched_imu_packets_ + batched_zone_packets_;
}

size_t FrameBatcher::dropped_packets() const {
    return dropped_packets_;
}

void FrameBatcher::set_max_cache_size(size_t max_cache_size) {
    if (max_cache_size == 0) {
        throw std::invalid_argument("max_cache_size must be > 0");
    }
    max_cache_size_ = max_cache_size;
}

size_t FrameBatcher::get_max_cache_size() const {
    return max_cache_size_;
}

FieldType::FieldType() = default;

FieldType::FieldType(std::string name_in, ChanFieldType element_type_in,
                     std::vector<size_t> extra_dims_in, FieldClass class_in)
    : name(std::move(name_in)),
      element_type(element_type_in),
      extra_dims(std::move(extra_dims_in)),
      field_class(class_in) {}

bool operator==(const FieldType& a, const FieldType& b) {
    return a.name == b.name && a.element_type == b.element_type && a.field_class == b.field_class &&
           a.extra_dims == b.extra_dims;
}

bool operator!=(const FieldType& a, const FieldType& b) {
    return a.name != b.name || a.element_type != b.element_type || a.field_class != b.field_class ||
           a.extra_dims != b.extra_dims;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
