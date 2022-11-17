/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <vector>

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

enum frame_status_shifts: uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT = 0,    ///< No shift for thermal shutdown 
    FRAME_STATUS_SHOT_LIMITING_SHIFT = 4        /// shift 4 for shot limiting
};

// clang-format on
using sensor::ChanField;
using sensor::ChanFieldType;
using sensor::UDPProfileLidar;

constexpr int LidarScan::N_FIELDS;

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

struct DefaultFieldsEntry {
    const std::pair<ChanField, ChanFieldType>* fields;
    size_t n_fields;
};

Table<UDPProfileLidar, DefaultFieldsEntry, 32> default_scan_fields{
    {{UDPProfileLidar::PROFILE_LIDAR_LEGACY,
      {legacy_field_slots.data(), legacy_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
      {dual_field_slots.data(), dual_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
      {single_field_slots.data(), single_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
      {lb_field_slots.data(), lb_field_slots.size()}}}};

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

}  // namespace impl

// specify sensor:: namespace for doxygen matching
LidarScan::LidarScan(
    size_t w, size_t h,
    std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>
        field_types)
    : timestamp_{Header<uint64_t>::Zero(w)},
      measurement_id_{Header<uint16_t>::Zero(w)},
      status_{Header<uint32_t>::Zero(w)},
      field_types_{std::move(field_types)},
      w{static_cast<std::ptrdiff_t>(w)},
      h{static_cast<std::ptrdiff_t>(h)},
      headers{w, BlockHeader{ts_t{0}, 0, 0}} {
    // TODO: error on duplicate fields
    for (const auto& ft : field_types_) {
        if (fields_.count(ft.first) > 0)
            throw std::invalid_argument("Duplicated fields found");
        fields_[ft.first] = impl::FieldSlot{ft.second, w, h};
    }
}

LidarScan::LidarScan(size_t w, size_t h, sensor::UDPProfileLidar profile)
    : LidarScan{w, h, impl::lookup_scan_fields(profile)} {}

LidarScan::LidarScan(size_t w, size_t h)
    : LidarScan{w, h, UDPProfileLidar::PROFILE_LIDAR_LEGACY} {}

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

std::vector<LidarScan::ts_t> LidarScan::timestamps() const {
    std::vector<LidarScan::ts_t> res;
    res.reserve(headers.size());
    for (const auto& h : headers) res.push_back(h.timestamp);
    return res;
}

LidarScan::BlockHeader& LidarScan::header(size_t m_id) {
    return headers.at(m_id);
}

const LidarScan::BlockHeader& LidarScan::header(size_t m_id) const {
    return headers.at(m_id);
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

bool operator==(const LidarScan::BlockHeader& a,
                const LidarScan::BlockHeader& b) {
    return a.timestamp == b.timestamp && a.encoder == b.encoder &&
           a.status == b.status;
}

bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.frame_id == b.frame_id && a.w == b.w && a.h == b.h &&
           a.frame_status == b.frame_status && a.fields_ == b.fields_ &&
           a.field_types_ == b.field_types_ &&
           (a.timestamp() == b.timestamp()).all() &&
           (a.measurement_id() == b.measurement_id()).all() &&
           (a.status() == b.status()).all();
}

LidarScanFieldTypes get_field_types(const LidarScan& ls) {
    return {ls.begin(), ls.end()};
}

LidarScanFieldTypes get_field_types(const sensor::sensor_info& info) {
    // Get typical LidarScan to obtain field types
    return impl::lookup_scan_fields(info.format.udp_profile_lidar);
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

    auto ts = ls.timestamp().cast<uint64_t>();
    ss << "  timestamp = (" << ts.minCoeff() << "; " << ts.mean() << "; "
       << ts.maxCoeff() << ")," << std::endl;
    auto mid = ls.measurement_id().cast<uint64_t>();
    ss << "  measurement_id = (" << mid.minCoeff() << "; " << mid.mean() << "; "
       << mid.maxCoeff() << ")," << std::endl;
    auto st = ls.status().cast<uint64_t>();
    ss << "  status = (" << st.minCoeff() << "; " << st.mean() << "; "
       << st.maxCoeff() << ")" << std::endl;

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
    if (azimuth_angles_deg.size() != h || altitude_angles_deg.size() != h)
        throw std::invalid_argument("unexpected scan dimensions");

    double beam_to_lidar_euclidean_distance_mm = beam_to_lidar_transform(0, 3);
    if (beam_to_lidar_transform(2, 3) != 0) {
        beam_to_lidar_euclidean_distance_mm =
            std::sqrt(std::pow(beam_to_lidar_transform(0, 3), 2) +
                      std::pow(beam_to_lidar_transform(2, 3), 2));
    }

    Eigen::ArrayXd encoder(w * h);   // theta_e
    Eigen::ArrayXd azimuth(w * h);   // theta_a
    Eigen::ArrayXd altitude(w * h);  // phi

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

    XYZLut lut;

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
    return (nooffset.array() == 0.0).select(nooffset, nooffset + lut.offset);
}

ScanBatcher::ScanBatcher(size_t w, const sensor::packet_format& pf)
    : w(w),
      h(pf.pixels_per_column),
      next_m_id(0),
      cache(pf.lidar_packet_size),
      pf(pf) {}

ScanBatcher::ScanBatcher(const sensor::sensor_info& info)
    : ScanBatcher(info.format.columns_per_frame, sensor::get_format(info)) {}

namespace {

/*
 * Generic operation to set all columns in the range [start, end) to zero
 */
struct zero_field_cols {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField, std::ptrdiff_t start,
                    std::ptrdiff_t end) {
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

    // zero deprecated header blocks
    for (auto m_id = start; m_id < end; m_id++) ls.header(m_id) = {};
}

/*
 * Generic operation to read a channel field from a packet measurement block
 * into a scan
 */
struct parse_field_col {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField f, uint16_t m_id,
                    const sensor::packet_format& pf, const uint8_t* col_buf) {
        if (f >= ChanField::CUSTOM0 && f <= ChanField::CUSTOM9) return;
        pf.col_field(col_buf, f, field.col(m_id).data(), field.cols());
    }
};

uint64_t frame_status(const uint8_t thermal_shutdown,
                      const uint8_t shot_limiting) {
    uint64_t res = 0;
    res |= (thermal_shutdown & 0x0f)
           << FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT;  // right nibble is thermal
                                                    // shutdown status, apply mask for
                                                    // safety, then shift
    res |= (shot_limiting & 0x0f)
           << FRAME_STATUS_SHOT_LIMITING_SHIFT;  // right nibble is shot limiting, apply mask for
                                                 // safety, then shift
    return res;
}

}  // namespace

bool ScanBatcher::operator()(const uint8_t* packet_buf, LidarScan& ls) {
    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");

    // process cached packet
    if (cached_packet) {
        cached_packet = false;
        ls.frame_id = -1;
        this->operator()(cache.data(), ls);
    }

    const uint16_t f_id = pf.frame_id(packet_buf);

    if (ls.frame_id == -1) {
        // expecting to start batching a new scan
        next_m_id = 0;
        ls.frame_id = f_id;

        const uint8_t f_thermal_shutdown = pf.thermal_shutdown(packet_buf);
        const uint8_t f_shot_limiting = pf.shot_limiting(packet_buf);
        ls.frame_status = frame_status(f_thermal_shutdown, f_shot_limiting);

    } else if (ls.frame_id == static_cast<uint16_t>(f_id + 1)) {
        // drop reordered packets from the previous frame
        return false;
    } else if (ls.frame_id != f_id) {
        // got a packet from a new frame
        impl::foreach_field(ls, zero_field_cols(), next_m_id, w);
        zero_header_cols(ls, next_m_id, w);
        std::memcpy(cache.data(), packet_buf, cache.size());
        cached_packet = true;

        return true;
    }

    // parse measurement blocks
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
        const uint32_t encoder = pf.col_encoder(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (!valid || m_id >= w) continue;

        // zero out missing columns if we jumped forward
        if (m_id >= next_m_id) {
            impl::foreach_field(ls, zero_field_cols(), next_m_id, m_id);
            zero_header_cols(ls, next_m_id, m_id);
            next_m_id = m_id + 1;
        }

        // old header API; will be removed in a future release
        ls.header(m_id) = {ts, encoder, status};

        // write new header values
        ls.timestamp()[m_id] = ts.count();
        ls.measurement_id()[m_id] = m_id;
        ls.status()[m_id] = status;

        impl::foreach_field(ls, parse_field_col(), m_id, pf, col_buf);
    }
    return false;
}  // namespace ouster

}  // namespace ouster
