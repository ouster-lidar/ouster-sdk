/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>
#include <vector>

#include "ouster/impl/lidar_scan_impl.h"
#include "ouster/impl/logging.h"
#include "ouster/strings.h"
#include "ouster/types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ouster::strings;

namespace ouster {

// clang-format off
/**
 * Flags for frame_status
 */
enum frame_status_masks : uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_MASK = 0x0f,  ///< Mask to get thermal shutdown status
    FRAME_STATUS_SHOT_LIMITING_MASK = 0xf0      ///< Mask to get shot limting status
};

//! @cond Doxygen_Suppress
enum frame_status_shifts: uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT = 0,    ///< No shift for thermal shutdown
    FRAME_STATUS_SHOT_LIMITING_SHIFT = 4        /// shift 4 for shot limiting
};
//! @endcond

// clang-format on
using sensor::ChanFieldType;
using sensor::UDPProfileLidar;

LidarScan::LidarScan() = default;
LidarScan::LidarScan(const LidarScan&) = default;
LidarScan::LidarScan(LidarScan&&) = default;
LidarScan& LidarScan::operator=(const LidarScan&) = default;
LidarScan& LidarScan::operator=(LidarScan&&) = default;
LidarScan::~LidarScan() = default;
namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

static const Table<std::string, ChanFieldType, 5> legacy_field_slots{
    {{sensor::ChanField::RANGE, ChanFieldType::UINT32},
     {sensor::ChanField::SIGNAL, ChanFieldType::UINT16},
     {sensor::ChanField::NEAR_IR, ChanFieldType::UINT16},
     {sensor::ChanField::REFLECTIVITY, ChanFieldType::UINT8},
     {sensor::ChanField::FLAGS, ChanFieldType::UINT8}}};

static const Table<std::string, ChanFieldType, 9> dual_field_slots{{
    {sensor::ChanField::RANGE, ChanFieldType::UINT32},
    {sensor::ChanField::RANGE2, ChanFieldType::UINT32},
    {sensor::ChanField::SIGNAL, ChanFieldType::UINT16},
    {sensor::ChanField::SIGNAL2, ChanFieldType::UINT16},
    {sensor::ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {sensor::ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {sensor::ChanField::FLAGS, ChanFieldType::UINT8},
    {sensor::ChanField::FLAGS2, ChanFieldType::UINT8},
    {sensor::ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<std::string, ChanFieldType, 5> single_field_slots{{
    {sensor::ChanField::RANGE, ChanFieldType::UINT32},
    {sensor::ChanField::SIGNAL, ChanFieldType::UINT16},
    {sensor::ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {sensor::ChanField::FLAGS, ChanFieldType::UINT8},
    {sensor::ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<std::string, ChanFieldType, 4> lb_field_slots{{
    {sensor::ChanField::RANGE, ChanFieldType::UINT32},
    {sensor::ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {sensor::ChanField::NEAR_IR, ChanFieldType::UINT16},
    {sensor::ChanField::FLAGS, ChanFieldType::UINT8},
}};

static const Table<std::string, ChanFieldType, 5> five_word_slots{{
    {sensor::ChanField::RAW32_WORD1, ChanFieldType::UINT32},
    {sensor::ChanField::RAW32_WORD2, ChanFieldType::UINT32},
    {sensor::ChanField::RAW32_WORD3, ChanFieldType::UINT32},
    {sensor::ChanField::RAW32_WORD4, ChanFieldType::UINT32},
    {sensor::ChanField::RAW32_WORD5, ChanFieldType::UINT32},
}};

static const Table<std::string, ChanFieldType, 7> fusa_two_word_slots{{
    {sensor::ChanField::RANGE, ChanFieldType::UINT32},
    {sensor::ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {sensor::ChanField::NEAR_IR, ChanFieldType::UINT16},
    {sensor::ChanField::RANGE2, ChanFieldType::UINT32},
    {sensor::ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {sensor::ChanField::FLAGS, ChanFieldType::UINT8},
    {sensor::ChanField::FLAGS2, ChanFieldType::UINT8},
}};

struct DefaultFieldsEntry {
    const std::pair<std::string, ChanFieldType>* fields;
    size_t n_fields;
};

using ouster::sensor::impl::MAX_NUM_PROFILES;
// clang-format off
Table<UDPProfileLidar, DefaultFieldsEntry, MAX_NUM_PROFILES> default_scan_fields{
    {{UDPProfileLidar::PROFILE_LIDAR_LEGACY,
      {legacy_field_slots.data(), legacy_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
      {dual_field_slots.data(), dual_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
      {single_field_slots.data(), single_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
      {lb_field_slots.data(), lb_field_slots.size()}},
     {UDPProfileLidar::PROFILE_FIVE_WORD_PIXEL,
      {five_word_slots.data(), five_word_slots.size()}},
     {UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL,
      {fusa_two_word_slots.data(), fusa_two_word_slots.size()}},
}};
// clang-format on

static LidarScanFieldTypes lookup_scan_fields(UDPProfileLidar profile) {
    auto end = impl::default_scan_fields.end();
    auto it =
        std::find_if(impl::default_scan_fields.begin(), end,
                     [profile](const auto& kv) { return kv.first == profile; });

    if (it == end || it->first == 0)
        throw std::invalid_argument("Unknown lidar udp profile");

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

bool raw_headers_enabled(const sensor::packet_format& pf, const LidarScan& ls) {
    using ouster::sensor::logger;
    if (!ls.has_field(sensor::ChanField::RAW_HEADERS)) {
        return false;
    }

    auto raw_headers_ft = ls.field(sensor::ChanField::RAW_HEADERS).tag();
    // ensure that we can pack headers into the size of a single RAW_HEADERS
    // column
    if (pf.pixels_per_column * sensor::field_type_size(raw_headers_ft) <
        (pf.packet_header_size + pf.col_header_size + pf.col_footer_size +
         pf.packet_footer_size)) {
        logger().debug(
            "WARNING: Can't fit RAW_HEADERS into a column of {} {} "
            "values",
            pf.pixels_per_column, to_string(raw_headers_ft));
        return false;
    }
    return true;
}

}  // namespace impl

static FieldDescriptor get_field_type_descriptor(const LidarScan& scan,
                                                 const FieldType& ft) {
    if (ft.field_class == FieldClass::PIXEL_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(scan.h);
        dims.push_back(scan.w);
        dims.insert(dims.end(), ft.extra_dims.begin(), ft.extra_dims.end());
        return FieldDescriptor::array(ft.element_type, dims);
    } else if (ft.field_class == FieldClass::COLUMN_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(scan.w);
        dims.insert(dims.end(), ft.extra_dims.begin(), ft.extra_dims.end());
        return FieldDescriptor::array(ft.element_type, dims);
    } else if (ft.field_class == FieldClass::PACKET_FIELD) {
        std::vector<size_t> dims;
        dims.push_back(scan.packet_count());
        dims.insert(dims.end(), ft.extra_dims.begin(), ft.extra_dims.end());
        return FieldDescriptor::array(ft.element_type, dims);
    } else {  // FieldClass::SCAN_FIELD
        return FieldDescriptor::array(ft.element_type, ft.extra_dims);
    }
}

LidarScan::LidarScan(const sensor::sensor_info& info)
    : LidarScan{info.format.columns_per_frame, info.format.pixels_per_column,
                info.format.udp_profile_lidar, info.format.columns_per_packet} {
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

    for (const auto& ft : field_types) {
        add_field(ft);
    }

    timestamp_ = Field{fd_array<uint64_t>(w), FieldClass::COLUMN_FIELD};
    measurement_id_ = Field{fd_array<uint16_t>(w), FieldClass::COLUMN_FIELD};
    status_ = Field{fd_array<uint32_t>(w), FieldClass::COLUMN_FIELD};
    packet_timestamp_ =
        Field{fd_array<uint64_t>(packet_count_), FieldClass::PACKET_FIELD};
    pose_ = Field{fd_array<double>(w, 4, 4), {}};
    alert_flags_ =
        Field{fd_array<uint8_t>(packet_count_), FieldClass::PACKET_FIELD};

    /**
     * These may be unnecessary to set to identity
     */
    for (size_t i = 0; i < w; ++i) {
        Eigen::Ref<img_t<double>> pose = pose_.subview(i);
        pose = mat4d::Identity();
    }
}

LidarScan::LidarScan(const LidarScan& ls_src,
                     const LidarScanFieldTypes& field_types)
    : packet_count_(ls_src.packet_count_),
      w(ls_src.w),
      h(ls_src.h),
      columns_per_packet_(ls_src.columns_per_packet_),
      frame_status(ls_src.frame_status),
      frame_id(ls_src.frame_id) {
    for (const auto& ft : field_types) {
        const std::string& name = ft.name;
        FieldDescriptor dst_desc = get_field_type_descriptor(*this, ft);
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
                add_field(name, dst_desc, ft.field_class);
                ouster::impl::visit_field(
                    *this, name, ouster::impl::copy_and_cast(), ls_src, name);
            }
        } else {
            add_field(name, dst_desc, ft.field_class);
        }
    }

    timestamp_ = ls_src.timestamp_;
    measurement_id_ = ls_src.measurement_id_;
    status_ = ls_src.status_;
    packet_timestamp_ = ls_src.packet_timestamp_;
    pose_ = ls_src.pose_;
}

LidarScan::LidarScan(size_t w, size_t h, sensor::UDPProfileLidar profile,
                     size_t columns_per_packet)
    : LidarScan{w, h, impl::lookup_scan_fields(profile), columns_per_packet} {}

LidarScan::LidarScan(size_t w, size_t h)
    : LidarScan{w, h, UDPProfileLidar::PROFILE_LIDAR_LEGACY,
                DEFAULT_COLUMNS_PER_PACKET} {}

sensor::ShotLimitingStatus LidarScan::shot_limiting() const {
    return static_cast<sensor::ShotLimitingStatus>(
        (frame_status & frame_status_masks::FRAME_STATUS_SHOT_LIMITING_MASK) >>
        frame_status_shifts::FRAME_STATUS_SHOT_LIMITING_SHIFT);
}

sensor::ThermalShutdownStatus LidarScan::thermal_shutdown() const {
    return static_cast<sensor::ThermalShutdownStatus>(
        (frame_status &
         frame_status_masks::FRAME_STATUS_THERMAL_SHUTDOWN_MASK) >>
        frame_status_shifts::FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT);
}

Field& LidarScan::field(const std::string& name) { return fields().at(name); }

const Field& LidarScan::field(const std::string& name) const {
    return fields().at(name);
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
    if (has_field(name))
        throw std::invalid_argument("Duplicated field '" + name + "'");

    if (field_class == FieldClass::PIXEL_FIELD) {
        if (desc.shape.size() < 2)
            throw std::invalid_argument(
                "Pixel fields must have at least 2 dimensions");
        if (desc.shape[0] != h || desc.shape[1] != w)
            throw std::invalid_argument(
                "Pixel field shape must match "
                "LidarScan's width and height. Was " +
                std::to_string(desc.shape[0]) + "x" +
                std::to_string(desc.shape[1]) + " vs " + std::to_string(h) +
                "x" + std::to_string(w));
        // none of the dimensions should be zero
        for (const auto& dim : desc.shape) {
            if (dim == 0) {
                throw std::invalid_argument(
                    "Cannot add pixel field with 0 elements.");
            }
        }
    }

    if (field_class == FieldClass::COLUMN_FIELD) {
        if (desc.shape[0] != w)
            throw std::invalid_argument(
                "Column field shape must match "
                "LidarScan's height. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " +
                std::to_string(w));
    }

    if (field_class == FieldClass::PACKET_FIELD) {
        if (desc.shape[0] != packet_count_)
            throw std::invalid_argument(
                "Packet field shape must match "
                "number of packets. Width was " +
                std::to_string(desc.shape[0]) + " vs required width of " +
                std::to_string(packet_count_));
    }

    fields()[name] = Field{desc, field_class};

    return fields()[name];
}

// TODO: verify this is sane with python bindings, might be hard to keep alive
Field LidarScan::del_field(const std::string& name) {
    if (!has_field(name))
        throw std::invalid_argument("Attempted deleting non existing field '" +
                                    name + "'");

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
template Eigen::Ref<img_t<uint8_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<uint16_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<uint32_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<uint64_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<int8_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<int16_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<int32_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<int64_t>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<float>> LidarScan::field(const std::string& f);
template Eigen::Ref<img_t<double>> LidarScan::field(const std::string& f);
template Eigen::Ref<const img_t<uint8_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<uint16_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<uint32_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<uint64_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<int8_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<int16_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<int32_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<int64_t>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<float>> LidarScan::field(const std::string& f) const;
template Eigen::Ref<const img_t<double>> LidarScan::field(const std::string& f) const;
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

uint64_t LidarScan::get_first_valid_packet_timestamp() const {
    int total_packets = packet_timestamp().size();
    int columns_per_packet = w / total_packets;

    for (int i = 0; i < total_packets; ++i) {
        if (status()
                .middleRows(i * columns_per_packet, columns_per_packet)
                .unaryExpr([](uint32_t s) { return s & 1; })
                .any())
            return packet_timestamp()[i];
    }

    return 0;
}

uint64_t LidarScan::get_first_valid_column_timestamp() const {
    auto stat = status();
    for (int i = 0; i < timestamp().size(); i++) {
        if ((stat[i] & 1) > 0) {
            return timestamp()[i];
        }
    }
    return 0;
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

bool LidarScan::complete(sensor::ColumnWindow window) const {
    const auto& status = this->status();
    auto start = window.first;
    auto end = window.second;

    if (start <= end) {
        return status.segment(start, end - start + 1)
            .unaryExpr([](uint32_t s) { return s & 0x01; })
            .isConstant(0x01);
    } else {
        return status.segment(0, end)
                   .unaryExpr([](uint32_t s) { return s & 0x01; })
                   .isConstant(0x01) &&
               status.segment(start, this->w - start)
                   .unaryExpr([](uint32_t s) { return s & 0x01; })
                   .isConstant(0x01);
    }
}

size_t LidarScan::packet_count() const { return packet_count_; }

bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.frame_id == b.frame_id && a.w == b.w && a.h == b.h &&
           a.frame_status == b.frame_status &&
           a.measurement_id_ == b.measurement_id_ &&
           a.timestamp_ == b.timestamp_ &&
           a.packet_timestamp_ == b.packet_timestamp_ && a.pose() == b.pose() &&
           a.fields() == b.fields();
}

LidarScanFieldTypes LidarScan::field_types() const {
    LidarScanFieldTypes ft;
    for (const auto& kv : fields()) {
        ft.push_back(get_field_type(kv.first, kv.second));
    }

    std::sort(ft.begin(), ft.end());
    return ft;
}

LidarScanFieldTypes get_field_types(UDPProfileLidar udp_profile_lidar) {
    // Get typical LidarScan to obtain field types
    return impl::lookup_scan_fields(udp_profile_lidar);
}

LidarScanFieldTypes get_field_types(const sensor::sensor_info& info) {
    // Get typical LidarScan to obtain field types
    return get_field_types(info.format.udp_profile_lidar);
}

std::string to_string(const FieldType& field_type) {
    std::string out = field_type.name;
    out += ": ";
    out += to_string(field_type.element_type);
    out += " (";
    int j = 0;
    for (const auto& d : field_type.extra_dims) {
        if (j++ > 0) out += ", ";
        out += std::to_string(d);
    }
    out += ") ";
    out += ouster::to_string(field_type.field_class);
    return out;
}

std::string to_string(const LidarScanFieldTypes& field_types) {
    std::stringstream ss;
    ss << "(";
    for (size_t i = 0; i < field_types.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << to_string(field_types[i]);
    }
    ss << ")";
    return ss.str();
}

std::string to_string(const LidarScan& ls) {
    std::stringstream ss;
    LidarScanFieldTypes field_types = ls.field_types();
    ss << "LidarScan: {h = " << ls.h << ", w = " << ls.w
       << ", packets_per_frame = " << ls.packet_timestamp().size()
       << ", fid = " << ls.frame_id << "," << std::endl
       << " frame status = " << std::hex << ls.frame_status << std::dec
       << ", thermal_shutdown status = " << to_string(ls.thermal_shutdown())
       << ", shot_limiting status = " << to_string(ls.shot_limiting()) << ","
       << std::endl
       << "  field_types = " << to_string(field_types) << "," << std::endl;

    auto read_eigen = [](auto ref, std::stringstream& ss) {
        ss << "min: " << (double)ref.minCoeff()
           << "; mean: " << ref.template cast<double>().mean()
           << "; max: " << (double)ref.maxCoeff();
    };

    auto read_field = [&read_eigen](const Field& f, const std::string& name,
                                    std::stringstream& ss) {
        ss << "    " << name << " type:" << to_string(f.tag()) << " shape: (";

        const auto& shape = f.shape();
        for (size_t i = 0; i < shape.size(); ++i) {
            ss << shape[i];
            if (i < shape.size() - 1) {
                ss << ", ";
            }
        }
        ss << ") ";

        if (f.bytes() > 0) {
            FieldView flat_view = f.reshape(1, f.size());
            impl::visit_field_2d(flat_view, read_eigen, ss);
        }
        ss << std::endl;
    };

    for (auto&& kv : ls.fields()) {
        read_field(kv.second, kv.first, ss);
    }

    ss << "}";
    return ss.str();
}

XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    const mat4d& beam_to_lidar_transform,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg) {
    if (w <= 0 || h <= 0)
        throw std::invalid_argument("lut dimensions must be greater than zero");

    if ((azimuth_angles_deg.size() != h || altitude_angles_deg.size() != h) &&
        (azimuth_angles_deg.size() != w * h ||
         altitude_angles_deg.size() != w * h)) {
        throw std::invalid_argument("unexpected scan dimensions");
    }

    double beam_to_lidar_euclidean_distance_mm = beam_to_lidar_transform(0, 3);
    if (beam_to_lidar_transform(2, 3) != 0) {
        beam_to_lidar_euclidean_distance_mm =
            std::sqrt(std::pow(beam_to_lidar_transform(0, 3), 2) +
                      std::pow(beam_to_lidar_transform(2, 3), 2));
    }

    XYZLut lut;

    Eigen::ArrayXd encoder(w * h);   // theta_e
    Eigen::ArrayXd azimuth(w * h);   // theta_a
    Eigen::ArrayXd altitude(w * h);  // phi

    if (azimuth_angles_deg.size() == h && altitude_angles_deg.size() == h) {
        // OS sensor
        const double azimuth_radians = M_PI * 2.0 / w;

        // populate angles for each pixel
        for (size_t v = 0; v < w; v++) {
            for (size_t u = 0; u < h; u++) {
                size_t i = u * w + v;
                encoder(i) = 2.0 * M_PI - (v * azimuth_radians);
                azimuth(i) = -azimuth_angles_deg[u] * M_PI / 180.0;
                altitude(i) = altitude_angles_deg[u] * M_PI / 180.0;
            }
        }

    } else if (azimuth_angles_deg.size() == w * h &&
               altitude_angles_deg.size() == w * h) {
        // DF sensor
        // populate angles for each pixel
        for (size_t v = 0; v < w; v++) {
            for (size_t u = 0; u < h; u++) {
                size_t i = u * w + v;
                encoder(i) = 0;
                azimuth(i) = azimuth_angles_deg[i] * M_PI / 180.0;
                altitude(i) = altitude_angles_deg[i] * M_PI / 180.0;
            }
        }
    }

    // unit vectors for each pixel
    lut.direction = LidarScan::Points{w * h, 3};
    lut.direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
    lut.direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
    lut.direction.col(2) = altitude.sin();

    // offsets due to beam origin
    lut.offset = LidarScan::Points{w * h, 3};
    lut.offset.col(0) =
        encoder.cos() * beam_to_lidar_transform(0, 3) -
        lut.direction.col(0) * beam_to_lidar_euclidean_distance_mm;
    lut.offset.col(1) =
        encoder.sin() * beam_to_lidar_transform(0, 3) -
        lut.direction.col(1) * beam_to_lidar_euclidean_distance_mm;
    lut.offset.col(2) =
        -lut.direction.col(2) * beam_to_lidar_euclidean_distance_mm +
        beam_to_lidar_transform(2, 3);

    // apply the supplied transform
    auto rot = transform.topLeftCorner(3, 3).transpose();
    auto trans = transform.topRightCorner(3, 1).transpose();
    lut.direction.matrix() *= rot;
    lut.offset.matrix() *= rot;
    lut.offset.matrix() += trans.replicate(w * h, 1);

    // apply scaling factor
    lut.direction *= range_unit;
    lut.offset *= range_unit;

    return lut;
}

XYZLut make_xyz_lut(const sensor::sensor_info& sensor, bool use_extrinsics) {
    mat4d transform = sensor.lidar_to_sensor_transform;
    if (use_extrinsics) {
        // apply extrinsics after lidar_to_sensor_transform so the
        // resulting LUT will produce the coordinates in
        // "extrinsics frame" instead of "sensor frame"
        mat4d ext_transform = sensor.extrinsic;
        ext_transform(0, 3) /= sensor::range_unit;
        ext_transform(1, 3) /= sensor::range_unit;
        ext_transform(2, 3) /= sensor::range_unit;
        transform = ext_transform * sensor.lidar_to_sensor_transform;
    }
    return make_xyz_lut(
        sensor.format.columns_per_frame, sensor.format.pixels_per_column,
        sensor::range_unit, sensor.beam_to_lidar_transform, transform,
        sensor.beam_azimuth_angles, sensor.beam_altitude_angles);
}

LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut) {
    return cartesian(scan.field(sensor::ChanField::RANGE), lut);
}

LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const XYZLut& lut) {
    if (range.cols() * range.rows() != lut.direction.rows())
        throw std::invalid_argument("unexpected image dimensions");
    auto reshaped = Eigen::Map<const Eigen::Array<uint32_t, -1, 1>>(
        range.data(), range.cols() * range.rows());
    auto nooffset = lut.direction.colwise() * reshaped.cast<double>();
    return (reshaped == 0)
        .replicate<1, 3>()
        .select(nooffset, nooffset + lut.offset);
}

ScanBatcher::ScanBatcher(size_t w, const sensor::packet_format& pf)
    : w(w),
      h(pf.pixels_per_column),
      next_valid_m_id(0),
      next_headers_m_id(0),
      cache(pf.lidar_packet_size),
      cache_packet_ts(0),
      pf(pf) {
    if (pf.columns_per_packet == 0)
        throw std::invalid_argument("unexpected columns_per_packet: 0");
    // Since we don't know the azimuth window, assume it is full.
    const size_t desired_w =
        w / pf.columns_per_packet + (w % pf.columns_per_packet ? 1 : 0);
    expected_packets = desired_w;
}

ScanBatcher::ScanBatcher(const sensor::sensor_info& info)
    : ScanBatcher(info.format.columns_per_frame, sensor::get_format(info)) {
    // Calculate the number of packets required to have a complete scan
    int max_packets = expected_packets;
    if (info.format.column_window.second < info.format.column_window.first) {
        // the valid azimuth window wraps through 0
        int start_packet =
            info.format.column_window.second / pf.columns_per_packet;
        int end_packet =
            info.format.column_window.first / pf.columns_per_packet;
        expected_packets = start_packet + 1 + (max_packets - end_packet);
        // subtract one if start and end are in the same block
        if (start_packet == end_packet) {
            expected_packets -= 1;
        }
    } else {
        // no wrapping of azimuth the window through 0
        int start_packet =
            info.format.column_window.first / pf.columns_per_packet;
        int end_packet =
            info.format.column_window.second / pf.columns_per_packet;

        expected_packets = end_packet - start_packet + 1;
    }
}

namespace {

/*
 * Generic operation to set all columns in the range [start, end) to zero
 */
struct zero_field_cols {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string&,
                    std::ptrdiff_t start, std::ptrdiff_t end) const {
        field.block(0, start, field.rows(), end - start).setZero();
    }
};

/*
 * Zero out all measurement block headers in range [start, end)
 */
void zero_header_cols(LidarScan& ls, std::ptrdiff_t start, std::ptrdiff_t end) {
    ls.timestamp().segment(start, end - start).setZero();
    ls.measurement_id().segment(start, end - start).setZero();
    ls.status().segment(start, end - start).setZero();
}

/*
 * Generic operation to read a channel field from a packet measurement block
 * into a scan
 */
struct parse_field_col {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& f,
                    uint16_t m_id, const sensor::packet_format& pf,
                    const uint8_t* col_buf) const {
        // RAW_HEADERS field is populated separately because it has
        // a different processing scheme and doesn't fit into existing field
        // model (i.e. data packed per column rather than per pixel)
        if (f == sensor::ChanField::RAW_HEADERS) return;

        pf.col_field(col_buf, f, field.col(m_id).data(), field.cols());
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
struct pack_raw_headers_col {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> rh_field, const std::string&,
                    const sensor::packet_format& pf, uint16_t col_idx,
                    const uint8_t* packet_buf) const {
        const uint8_t* col_buf = pf.nth_col(col_idx, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);

        using ColMajorView =
            Eigen::Map<const Eigen::Array<T, -1, 1, Eigen::ColMajor>>;

        const ColMajorView col_header_vec(reinterpret_cast<const T*>(col_buf),
                                          pf.col_header_size / sizeof(T));

        rh_field.block(0, m_id, col_header_vec.size(), 1) = col_header_vec;

        const ColMajorView col_footer_vec(
            reinterpret_cast<const T*>(col_buf + pf.col_size -
                                       pf.col_footer_size),
            pf.col_footer_size / sizeof(T));

        rh_field.block(col_header_vec.size(), m_id, col_footer_vec.size(), 1) =
            col_footer_vec;

        const ColMajorView packet_header_vec(
            reinterpret_cast<const T*>(packet_buf),
            pf.packet_header_size / sizeof(T));

        rh_field.block(col_header_vec.size() + col_footer_vec.size(), m_id,
                       packet_header_vec.size(), 1) = packet_header_vec;

        const ColMajorView packet_footer_vec(
            reinterpret_cast<const T*>(pf.footer(packet_buf)),
            pf.packet_footer_size / sizeof(T));

        rh_field.block(col_header_vec.size() + col_footer_vec.size() +
                           packet_header_vec.size(),
                       m_id, packet_footer_vec.size(), 1) = packet_footer_vec;
    }
};

}  // namespace

void ScanBatcher::_parse_by_col(const uint8_t* packet_buf, LidarScan& ls) {
    const bool raw_headers = impl::raw_headers_enabled(pf, ls);
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint64_t ts = pf.col_timestamp(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01);

        // drop out-of-bounds data in case of misconfiguration
        if (m_id >= w) continue;

        if (raw_headers) {
            // zero out missing columns if we jumped forward
            if (m_id >= next_headers_m_id) {
                impl::visit_field(ls, sensor::ChanField::RAW_HEADERS,
                                  zero_field_cols{}, "", next_headers_m_id,
                                  m_id);
                next_headers_m_id = m_id + 1;
            }

            impl::visit_field(
                ls, sensor::ChanField::RAW_HEADERS, pack_raw_headers_col(),
                sensor::ChanField::RAW_HEADERS, pf, icol, packet_buf);
        }

        // drop invalid
        if (!valid) continue;

        // zero out missing columns if we jumped forward
        if (m_id >= next_valid_m_id) {
            impl::foreach_channel_field(ls, pf, zero_field_cols{},
                                        next_valid_m_id, m_id);
            zero_header_cols(ls, next_valid_m_id, m_id);
            next_valid_m_id = m_id + 1;
        }

        // write new header values
        ls.timestamp()[m_id] = ts;
        ls.measurement_id()[m_id] = m_id;
        ls.status()[m_id] = status;

        impl::foreach_channel_field(ls, pf, parse_field_col{}, m_id, pf,
                                    col_buf);
    }
}

/*
 * Faster version of parse_field_col that works by blocks instead and skips
 * extra checks
 */
template <int BlockDim>
struct parse_field_block {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, const std::string& f,
                    const sensor::packet_format& pf,
                    const uint8_t* packet_buf) const {
        pf.block_field<T, BlockDim>(field, f, packet_buf);
    }
};

void ScanBatcher::_parse_by_block(const uint8_t* packet_buf, LidarScan& ls) {
    // zero out missing columns if we jumped forward
    const uint16_t first_m_id =
        pf.col_measurement_id(pf.nth_col(0, packet_buf));
    if (first_m_id >= next_valid_m_id) {
        impl::foreach_channel_field(ls, pf, zero_field_cols{}, next_valid_m_id,
                                    first_m_id);
        zero_header_cols(ls, next_valid_m_id, first_m_id);
        next_valid_m_id = first_m_id + pf.columns_per_packet;
    }

    // write new header values
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint64_t ts = pf.col_timestamp(col_buf);
        const uint32_t status = pf.col_status(col_buf);

        ls.timestamp()[m_id] = ts;
        ls.measurement_id()[m_id] = m_id;
        ls.status()[m_id] = status;
    }

    switch (pf.block_parsable()) {
        case 16:
            impl::foreach_channel_field(ls, pf, parse_field_block<16>{}, pf,
                                        packet_buf);
            break;
        case 8:
            impl::foreach_channel_field(ls, pf, parse_field_block<8>{}, pf,
                                        packet_buf);
            break;
        case 4:
            impl::foreach_channel_field(ls, pf, parse_field_block<4>{}, pf,
                                        packet_buf);
            break;
        default:
            throw std::invalid_argument("Invalid block dim for packet format");
    }
}

bool ScanBatcher::operator()(const uint8_t* packet_buf, LidarScan& ls) {
    return this->operator()(packet_buf, 0, ls);
}

bool ScanBatcher::operator()(const ouster::sensor::LidarPacket& packet,
                             LidarScan& ls) {
    return (*this)(packet.buf.data(), packet.host_timestamp, ls);
}

void ScanBatcher::finalize_scan(LidarScan& ls, bool raw_headers) {
    impl::foreach_channel_field(ls, pf, zero_field_cols{}, next_valid_m_id, w);

    if (raw_headers) {
        impl::visit_field(ls, sensor::ChanField::RAW_HEADERS, zero_field_cols{},
                          "", next_headers_m_id, w);
    }
}

bool ScanBatcher::operator()(const uint8_t* packet_buf, uint64_t packet_ts,
                             LidarScan& ls) {
    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");
    if (static_cast<size_t>(ls.packet_timestamp().rows()) !=
        ls.w / pf.columns_per_packet)
        throw std::invalid_argument("unexpected scan columns_per_packet: " +
                                    std::to_string(pf.columns_per_packet));

    // process cached packet and packet ts
    if (cached_packet) {
        cached_packet = false;
        this->operator()(cache.data(), cache_packet_ts, ls);
    }

    const int64_t f_id = pf.frame_id(packet_buf);

    const bool raw_headers = impl::raw_headers_enabled(pf, ls);

    if (ls.frame_id == -1 || finished_scan_id >= 0) {
        // expecting to start batching a new scan
        if (finished_scan_id >= 0) {
            // drop duplicate or packets from previous frame
            if (finished_scan_id == f_id) {
                // drop old duplicate packets
                return false;
            } else if (finished_scan_id ==
                       ((f_id + 1) %
                        (static_cast<int64_t>(pf.max_frame_id) + 1))) {
                // drop reordered packets from the previous frame
                return false;
            }
        }
        finished_scan_id = -1;
        next_valid_m_id = 0;
        next_headers_m_id = 0;
        batched_packets = 0;
        ls.frame_id = f_id;
        zero_header_cols(ls, 0, w);
        ls.packet_timestamp().setZero();
        const uint8_t f_thermal_shutdown = pf.thermal_shutdown(packet_buf);
        const uint8_t f_shot_limiting = pf.shot_limiting(packet_buf);
        ls.frame_status = frame_status(f_thermal_shutdown, f_shot_limiting);

        // The countdown values are supposed to be the same for all packets in a
        // given scan.
        ls.shutdown_countdown = pf.countdown_thermal_shutdown(packet_buf);
        ls.shot_limiting_countdown = pf.countdown_shot_limiting(packet_buf);
    } else if (ls.frame_id ==
               ((f_id + 1) % (static_cast<int64_t>(pf.max_frame_id) + 1))) {
        // drop reordered packets from the previous frame
        return false;
    } else if (ls.frame_id != f_id) {
        // got a packet from a new frame, release the old one
        finished_scan_id = ls.frame_id;
        finalize_scan(ls, raw_headers);

        // store packet buf and ts data to the cache for later processing
        std::memcpy(cache.data(), packet_buf, cache.size());
        cache_packet_ts = packet_ts;
        cached_packet = true;
        return true;
    }

    batched_packets++;

    // handling packet level data: packet_timestamp
    const uint8_t* col0_buf = pf.nth_col(0, packet_buf);
    const uint16_t packet_id =
        pf.col_measurement_id(col0_buf) / pf.columns_per_packet;
    if (packet_id < ls.packet_timestamp().rows()) {
        ls.packet_timestamp()[packet_id] = packet_ts;
        ls.alert_flags()[packet_id] = pf.alert_flags(packet_buf);
    }

    // handling column and pixel level data
    bool happy_packet = true;
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01);

        if (!valid || m_id >= w) {
            happy_packet = false;
            break;
        }
    }

    if (pf.block_parsable() && happy_packet && !raw_headers) {
        _parse_by_block(packet_buf, ls);
    } else {
        _parse_by_col(packet_buf, ls);
    }

    // if we have enough packets and are packet-complete release the scan
    if (batched_packets >= expected_packets &&
        (size_t)ls.packet_timestamp().count() == expected_packets) {
        finished_scan_id = f_id;
        finalize_scan(ls, raw_headers);
        return true;
    }

    return false;
}

FieldType::FieldType() {}

FieldType::FieldType(const std::string& name_,
                     sensor::ChanFieldType element_type_,
                     const std::vector<size_t> extra_dims_, FieldClass class_)
    : name(name_),
      element_type(element_type_),
      extra_dims(extra_dims_),
      field_class(class_) {}

bool operator==(const FieldType& a, const FieldType& b) {
    return a.name == b.name && a.element_type == b.element_type &&
           a.field_class == b.field_class && a.extra_dims == b.extra_dims;
}

bool operator!=(const FieldType& a, const FieldType& b) {
    return a.name != b.name || a.element_type != b.element_type ||
           a.field_class != b.field_class || a.extra_dims != b.extra_dims;
}

namespace pose_util {
void dewarp(Eigen::Ref<Points> dewarped, const Eigen::Ref<const Points> points,
            const Eigen::Ref<const Poses> poses) {
    const size_t W = poses.rows();                  // Number of pose matrices
    const size_t H = points.rows() / poses.rows();  // Points per pose matrix

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for schedule(static)
#endif
    for (size_t w = 0; w < W; ++w) {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
            pose_matrix(poses.row(w).data());
        const Eigen::Matrix3d rotation = pose_matrix.topLeftCorner<3, 3>();
        const Eigen::Vector3d translation = pose_matrix.topRightCorner<3, 1>();

        for (size_t i = 0; i < H; ++i) {
            const Eigen::Index ix = i * W + w;
            Eigen::Map<const Eigen::Vector3d> s(points.row(ix).data());
            Eigen::Map<Eigen::Vector3d> p(dewarped.row(ix).data());
            p = rotation * s + translation;
        }
    }
}

Points dewarp(const Eigen::Ref<const Points> points,
              const Eigen::Ref<const Poses> poses) {
    Points dewarped(points.rows(), points.cols());
    dewarp(dewarped, points, poses);
    return dewarped;
}

void transform(Eigen::Ref<pose_util::Points> transformed,
               const Eigen::Ref<const pose_util::Points> points,
               const Eigen::Ref<const pose_util::Pose> pose) {
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> pose_matrix =
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
            pose.data());

    Eigen::Matrix3d rotation = pose_matrix.topLeftCorner<3, 3>();
    Eigen::Vector3d translation = pose_matrix.topRightCorner<3, 1>();

    transformed =
        (points * rotation.transpose()).rowwise() + translation.transpose();
}

Points transform(const Eigen::Ref<const Points> points,
                 const Eigen::Ref<const Pose> pose) {
    Points transformed(points.rows(), points.cols());
    transform(transformed, points, pose);
    return transformed;
}
}  // namespace pose_util
}  // namespace ouster
