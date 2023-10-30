/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>
#include <vector>

#include "logging.h"
#include "ouster/impl/lidar_scan_impl.h"
#include "ouster/types.h"

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
using sensor::ChanField;
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

static const Table<ChanField, ChanFieldType, 4> legacy_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT32},
     {ChanField::NEAR_IR, ChanFieldType::UINT32},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT32}}};

static const Table<ChanField, ChanFieldType, 7> dual_field_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<ChanField, ChanFieldType, 4> single_field_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT16},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<ChanField, ChanFieldType, 3> lb_field_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT16},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<ChanField, ChanFieldType, 5> five_word_slots{{
    {ChanField::RAW32_WORD1, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD2, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD3, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD4, ChanFieldType::UINT32},
    {ChanField::RAW32_WORD5, ChanFieldType::UINT32},
}};

static const Table<ChanField, ChanFieldType, 5> fusa_two_word_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
}};

struct DefaultFieldsEntry {
    const std::pair<ChanField, ChanFieldType>* fields;
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

static std::vector<std::pair<ChanField, ChanFieldType>> lookup_scan_fields(
    UDPProfileLidar profile) {
    auto end = impl::default_scan_fields.end();
    auto it =
        std::find_if(impl::default_scan_fields.begin(), end,
                     [profile](const auto& kv) { return kv.first == profile; });

    if (it == end || it->first == 0)
        throw std::invalid_argument("Unknown lidar udp profile");

    auto entry = it->second;
    return {entry.fields, entry.fields + entry.n_fields};
}

bool raw_headers_enabled(const sensor::packet_format& pf, const LidarScan& ls) {
    using ouster::sensor::logger;
    ChanFieldType raw_headers_ft = ls.field_type(ChanField::RAW_HEADERS);
    if (!raw_headers_ft) {
        return false;
    }
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

// specify sensor:: namespace for doxygen matching
LidarScan::LidarScan(size_t w, size_t h, LidarScanFieldTypes field_types,
                     size_t columns_per_packet)
    : timestamp_{Header<uint64_t>::Zero(w)},
      packet_timestamp_{Header<uint64_t>::Zero(w / columns_per_packet)},
      measurement_id_{Header<uint16_t>::Zero(w)},
      status_{Header<uint32_t>::Zero(w)},
      pose_(w, mat4d::Identity()),
      field_types_{std::move(field_types)},
      w{static_cast<std::ptrdiff_t>(w)},
      h{static_cast<std::ptrdiff_t>(h)} {
    for (const auto& ft : field_types_) {
        if (fields_.count(ft.first) > 0)
            throw std::invalid_argument("Duplicated fields found");
        fields_[ft.first] = impl::FieldSlot{ft.second, w, h};
    }
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

template <typename T,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
Eigen::Ref<img_t<T>> LidarScan::field(ChanField f) {
    return fields_.at(f).get<T>();
}

template <typename T,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
Eigen::Ref<const img_t<T>> LidarScan::field(ChanField f) const {
    return fields_.at(f).get<T>();
}

// explicitly instantiate for each supported field type
template Eigen::Ref<img_t<uint8_t>> LidarScan::field(ChanField f);
template Eigen::Ref<img_t<uint16_t>> LidarScan::field(ChanField f);
template Eigen::Ref<img_t<uint32_t>> LidarScan::field(ChanField f);
template Eigen::Ref<img_t<uint64_t>> LidarScan::field(ChanField f);
template Eigen::Ref<const img_t<uint8_t>> LidarScan::field(ChanField f) const;
template Eigen::Ref<const img_t<uint16_t>> LidarScan::field(ChanField f) const;
template Eigen::Ref<const img_t<uint32_t>> LidarScan::field(ChanField f) const;
template Eigen::Ref<const img_t<uint64_t>> LidarScan::field(ChanField f) const;

ChanFieldType LidarScan::field_type(ChanField f) const {
    return fields_.count(f) ? fields_.at(f).tag : ChanFieldType::VOID;
}

LidarScan::FieldIter LidarScan::begin() const { return field_types_.cbegin(); }

LidarScan::FieldIter LidarScan::end() const { return field_types_.cend(); }

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

std::vector<mat4d>& LidarScan::pose() { return pose_; }

const std::vector<mat4d>& LidarScan::pose() const { return pose_; }

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

bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.frame_id == b.frame_id && a.w == b.w && a.h == b.h &&
           a.frame_status == b.frame_status && a.fields_ == b.fields_ &&
           a.field_types_ == b.field_types_ &&
           (a.timestamp() == b.timestamp()).all() &&
           (a.measurement_id() == b.measurement_id()).all() &&
           (a.status() == b.status()).all() && a.pose() == b.pose();
}

LidarScanFieldTypes get_field_types(const LidarScan& ls) {
    return {ls.begin(), ls.end()};
}

LidarScanFieldTypes get_field_types(UDPProfileLidar udp_profile_lidar) {
    // Get typical LidarScan to obtain field types
    return impl::lookup_scan_fields(udp_profile_lidar);
}

LidarScanFieldTypes get_field_types(const sensor::sensor_info& info) {
    // Get typical LidarScan to obtain field types
    return get_field_types(info.format.udp_profile_lidar);
}

std::string to_string(const LidarScanFieldTypes& field_types) {
    std::stringstream ss;
    ss << "(";
    for (size_t i = 0; i < field_types.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << sensor::to_string(field_types[i].first) << ":"
           << sensor::to_string(field_types[i].second);
    }
    ss << ")";
    return ss.str();
}

std::string to_string(const LidarScan& ls) {
    std::stringstream ss;
    LidarScanFieldTypes field_types(ls.begin(), ls.end());
    ss << "LidarScan: {h = " << ls.h << ", w = " << ls.w
       << ", packets_per_frame = " << ls.packet_timestamp().size()
       << ", fid = " << ls.frame_id << "," << std::endl
       << " frame status =  " << std::hex << ls.frame_status << std::dec
       << ", thermal_shutdown status = " << to_string(ls.thermal_shutdown())
       << ", shot_limiting status = " << to_string(ls.shot_limiting()) << ","
       << std::endl
       << "  field_types = " << to_string(field_types) << "," << std::endl;

    if (!field_types.empty()) {
        ss << "  fields = [" << std::endl;
        img_t<uint64_t> key{ls.h, ls.w};
        for (const auto& ft : ls) {
            impl::visit_field(ls, ft.first, impl::read_and_cast(), key);
            ss << "    " << to_string(ft.first) << ":" << to_string(ft.second)
               << " = (";
            ss << key.minCoeff() << "; " << key.mean() << "; "
               << key.maxCoeff();
            ss << ")," << std::endl;
        }
        ss << "  ]," << std::endl;
    }

    const auto& ts = ls.timestamp();
    ss << "  timestamp = (" << ts.minCoeff() << "; " << ts.mean() << "; "
       << ts.maxCoeff() << ")," << std::endl;
    const auto& packet_ts = ls.packet_timestamp();
    ss << "  packet_timestamp = (" << packet_ts.minCoeff() << "; "
       << packet_ts.mean() << "; " << packet_ts.maxCoeff() << ")," << std::endl;
    const auto& mid = ls.measurement_id().cast<uint64_t>();
    ss << "  measurement_id = (" << mid.minCoeff() << "; " << mid.mean() << "; "
       << mid.maxCoeff() << ")," << std::endl;
    const auto& st = ls.status().cast<uint64_t>();
    ss << "  status = (" << st.minCoeff() << "; " << st.mean() << "; "
       << st.maxCoeff() << ")" << std::endl;
    ss << "  poses = (size: " << ls.pose().size() << ")" << std::endl;
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

LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut) {
    return cartesian(scan.field(ChanField::RANGE), lut);
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
      next_valid_packet_id(0),
      cache(pf.lidar_packet_size),
      cache_packet_ts(0),
      pf(pf) {
    if (pf.columns_per_packet == 0)
        throw std::invalid_argument("unexpected columns_per_packet: 0");
}

ScanBatcher::ScanBatcher(const sensor::sensor_info& info)
    : ScanBatcher(info.format.columns_per_frame, sensor::get_format(info)) {}

namespace {

/*
 * Generic operation to set all columns in the range [start, end) to zero
 */
struct zero_field_cols {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField, std::ptrdiff_t start,
                    std::ptrdiff_t end) const {
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
    void operator()(Eigen::Ref<img_t<T>> field, ChanField f, uint16_t m_id,
                    const sensor::packet_format& pf,
                    const uint8_t* col_buf) const {
        // user defined fields that we shouldn't change
        if (f >= ChanField::CUSTOM0 && f <= ChanField::CUSTOM9) return;

        // RAW_HEADERS field is populated separately because it has
        // a different processing scheme and doesn't fit into existing field
        // model (i.e. data packed per column rather than per pixel)
        if (f == ChanField::RAW_HEADERS) return;

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
    void operator()(Eigen::Ref<img_t<T>> rh_field, ChanField,
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
                impl::visit_field(ls, ChanField::RAW_HEADERS, zero_field_cols(),
                                  ChanField::RAW_HEADERS, next_headers_m_id,
                                  m_id);
                next_headers_m_id = m_id + 1;
            }

            impl::visit_field(ls, ChanField::RAW_HEADERS,
                              pack_raw_headers_col(), ChanField::RAW_HEADERS,
                              pf, icol, packet_buf);
        }

        // drop invalid
        if (!valid) continue;

        // zero out missing columns if we jumped forward
        if (m_id >= next_valid_m_id) {
            for (const auto& field_type : ls) {
                if (field_type.first == ChanField::RAW_HEADERS) continue;
                if (field_type.first >= ChanField::CUSTOM0 &&
                    field_type.first <= ChanField::CUSTOM9)
                    continue;
                impl::visit_field(ls, field_type.first, zero_field_cols(),
                                  field_type.first, next_valid_m_id, m_id);
            }
            zero_header_cols(ls, next_valid_m_id, m_id);
            next_valid_m_id = m_id + 1;
        }

        // write new header values
        ls.timestamp()[m_id] = ts;
        ls.measurement_id()[m_id] = m_id;
        ls.status()[m_id] = status;

        impl::foreach_field(ls, parse_field_col(), m_id, pf, col_buf);
    }
}

/*
 * Faster version of parse_field_col that works by blocks instead and skips
 * extra checks
 */
template <int BlockDim>
struct parse_field_block {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField f,
                    const sensor::packet_format& pf,
                    const uint8_t* packet_buf) const {
        // user defined fields that we shouldn't change
        if (f >= ChanField::CUSTOM0 && f <= ChanField::CUSTOM9) return;

        pf.block_field<T, BlockDim>(field, f, packet_buf);
    }
};

void ScanBatcher::_parse_by_block(const uint8_t* packet_buf, LidarScan& ls) {
    // zero out missing columns if we jumped forward
    const uint16_t first_m_id =
        pf.col_measurement_id(pf.nth_col(0, packet_buf));
    if (first_m_id >= next_valid_m_id) {
        for (const auto& field_type : ls) {
            if (field_type.first >= ChanField::CUSTOM0 &&
                field_type.first <= ChanField::CUSTOM9)
                continue;
            impl::visit_field(ls, field_type.first, zero_field_cols(),
                              field_type.first, next_valid_m_id, first_m_id);
        }
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
            impl::foreach_field(ls, parse_field_block<16>{}, pf, packet_buf);
            break;
        case 8:
            impl::foreach_field(ls, parse_field_block<8>{}, pf, packet_buf);
            break;
        case 4:
            impl::foreach_field(ls, parse_field_block<4>{}, pf, packet_buf);
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

bool ScanBatcher::operator()(const uint8_t* packet_buf, uint64_t packet_ts,
                             LidarScan& ls) {
    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");
    if (ls.packet_timestamp().rows() != ls.w / pf.columns_per_packet)
        throw std::invalid_argument("unexpected scan columns_per_packet: " +
                                    std::to_string(pf.columns_per_packet));

    // process cached packet and packet ts
    if (cached_packet) {
        cached_packet = false;
        ls.frame_id = -1;
        this->operator()(cache.data(), cache_packet_ts, ls);
    }

    const uint16_t f_id = pf.frame_id(packet_buf);

    const bool raw_headers = impl::raw_headers_enabled(pf, ls);

    if (ls.frame_id == -1) {
        // expecting to start batching a new scan
        next_valid_m_id = 0;
        next_headers_m_id = 0;
        next_valid_packet_id = 0;
        ls.frame_id = f_id;

        const uint8_t f_thermal_shutdown = pf.thermal_shutdown(packet_buf);
        const uint8_t f_shot_limiting = pf.shot_limiting(packet_buf);
        ls.frame_status = frame_status(f_thermal_shutdown, f_shot_limiting);

    } else if (ls.frame_id == static_cast<uint16_t>(f_id + 1)) {
        // drop reordered packets from the previous frame
        return false;
    } else if (ls.frame_id != f_id) {
        // got a packet from a new frame
        for (const auto& field_type : ls) {
            auto end_m_id = next_valid_m_id;
            if (raw_headers && field_type.first == ChanField::RAW_HEADERS) {
                end_m_id = next_headers_m_id;
            }
            if (field_type.first >= ChanField::CUSTOM0 &&
                field_type.first <= ChanField::CUSTOM9)
                continue;
            impl::visit_field(ls, field_type.first, zero_field_cols(),
                              field_type.first, end_m_id, w);
        }

        zero_header_cols(ls, next_valid_m_id, w);

        // zero packet timestamp separately, since it's packet level data
        ls.packet_timestamp()
            .segment(next_valid_packet_id,
                     ls.packet_timestamp().rows() - next_valid_packet_id)
            .setZero();

        // store packet buf and ts data to the cache for later processing
        std::memcpy(cache.data(), packet_buf, cache.size());
        cache_packet_ts = packet_ts;
        cached_packet = true;

        return true;
    }

    // handling packet level data: packet_timestamp
    const uint8_t* col0_buf = pf.nth_col(0, packet_buf);
    const uint16_t packet_id =
        pf.col_measurement_id(col0_buf) / pf.columns_per_packet;
    if (packet_id < ls.packet_timestamp().rows()) {
        if (packet_id >= next_valid_packet_id) {
            // zeroing skipped packets timestamps
            ls.packet_timestamp()
                .segment(next_valid_packet_id, packet_id - next_valid_packet_id)
                .setZero();
            next_valid_packet_id = packet_id + 1;
        }
        ls.packet_timestamp()[packet_id] = packet_ts;
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

    return false;
}

}  // namespace ouster
