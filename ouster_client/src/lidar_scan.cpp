#include "ouster/lidar_scan.h"

#include <Eigen/Dense>
#include <cmath>
#include <type_traits>
#include <vector>

#include "ouster/types.h"

namespace ouster {

template <typename T>
struct FieldTag;

template <>
struct FieldTag<uint8_t> {
    static constexpr sensor::ChanFieldType tag = sensor::ChanFieldType::UINT8;
};

template <>
struct FieldTag<uint16_t> {
    static constexpr sensor::ChanFieldType tag = sensor::ChanFieldType::UINT16;
};

template <>
struct FieldTag<uint32_t> {
    static constexpr sensor::ChanFieldType tag = sensor::ChanFieldType::UINT32;
};

template <>
struct FieldTag<uint64_t> {
    static constexpr sensor::ChanFieldType tag = sensor::ChanFieldType::UINT64;
};

struct LidarScan::FieldSlot {
    sensor::ChanFieldType tag;
    union {
        img_t<uint8_t> f8;
        img_t<uint16_t> f16;
        img_t<uint32_t> f32;
        img_t<uint64_t> f64;
    };

    FieldSlot(sensor::ChanFieldType t, size_t w, size_t h) : tag{t} {
        switch (t) {
            case sensor::VOID:
                break;
            case sensor::UINT8:
                new (&f8) img_t<uint8_t>{h, w};
                break;
            case sensor::UINT16:
                new (&f16) img_t<uint16_t>{h, w};
                break;
            case sensor::UINT32:
                new (&f32) img_t<uint32_t>{h, w};
                break;
            case sensor::UINT64:
                new (&f64) img_t<uint64_t>{h, w};
                break;
        }
    }

    FieldSlot() : FieldSlot{sensor::VOID, 0, 0} {};

    ~FieldSlot() { clear(); }

    FieldSlot(const FieldSlot& other) {
        switch (other.tag) {
            case sensor::VOID:
                break;
            case sensor::UINT8:
                new (&f8) img_t<uint8_t>{other.f8};
                break;
            case sensor::UINT16:
                new (&f16) img_t<uint16_t>{other.f16};
                break;
            case sensor::UINT32:
                new (&f32) img_t<uint32_t>{other.f32};
                break;
            case sensor::UINT64:
                new (&f64) img_t<uint64_t>{other.f64};
                break;
        }
        tag = other.tag;
    }

    FieldSlot(FieldSlot&& other) { set_from(other); }

    FieldSlot& operator=(FieldSlot other) {
        clear();
        set_from(other);
        return *this;
    }

    template <typename T>
    Eigen::Ref<img_t<T>> get() {
        if (tag == FieldTag<T>::tag)
            return get_unsafe<T>();
        else
            throw std::invalid_argument("Accessed field at wrong type");
    }

    template <typename T>
    Eigen::Ref<const img_t<T>> get() const {
        if (tag == FieldTag<T>::tag)
            return get_unsafe<T>();
        else
            throw std::invalid_argument("Accessed field at wrong type");
    }

    friend bool operator==(const FieldSlot& l, const FieldSlot& r) {
        if (l.tag != r.tag) return false;
        switch (l.tag) {
            case sensor::VOID:
                return true;
            case sensor::UINT8:
                return (l.f8 == r.f8).all();
            case sensor::UINT16:
                return (l.f16 == r.f16).all();
            case sensor::UINT32:
                return (l.f32 == r.f32).all();
            case sensor::UINT64:
                return (l.f64 == r.f64).all();
            default:
                assert(false);
        }
        // unreachable, appease older gcc
        return false;
    }

   private:
    void set_from(FieldSlot& other) {
        switch (other.tag) {
            case sensor::VOID:
                break;
            case sensor::UINT8:
                new (&f8) img_t<uint8_t>{std::move(other.f8)};
                break;
            case sensor::UINT16:
                new (&f16) img_t<uint16_t>{std::move(other.f16)};
                break;
            case sensor::UINT32:
                new (&f32) img_t<uint32_t>{std::move(other.f32)};
                break;
            case sensor::UINT64:
                new (&f64) img_t<uint64_t>{std::move(other.f64)};
                break;
        }
        tag = other.tag;
        other.clear();
    }

    void clear() {
        switch (tag) {
            case sensor::VOID:
                break;
            case sensor::UINT8:
                f8.~img_t<uint8_t>();
                break;
            case sensor::UINT16:
                f16.~img_t<uint16_t>();
                break;
            case sensor::UINT32:
                f32.~img_t<uint32_t>();
                break;
            case sensor::UINT64:
                f64.~img_t<uint64_t>();
                break;
        }
        tag = sensor::VOID;
    }

    template <typename T>
    Eigen::Ref<img_t<T>> get_unsafe();

    template <typename T>
    Eigen::Ref<const img_t<T>> get_unsafe() const;
};

template <>
Eigen::Ref<img_t<uint8_t>> LidarScan::FieldSlot::get_unsafe() {
    return f8;
}

template <>
Eigen::Ref<img_t<uint16_t>> LidarScan::FieldSlot::get_unsafe() {
    return f16;
}

template <>
Eigen::Ref<img_t<uint32_t>> LidarScan::FieldSlot::get_unsafe() {
    return f32;
}

template <>
Eigen::Ref<img_t<uint64_t>> LidarScan::FieldSlot::get_unsafe() {
    return f64;
}

template <>
Eigen::Ref<const img_t<uint8_t>> LidarScan::FieldSlot::get_unsafe() const {
    return f8;
}

template <>
Eigen::Ref<const img_t<uint16_t>> LidarScan::FieldSlot::get_unsafe() const {
    return f16;
}

template <>
Eigen::Ref<const img_t<uint32_t>> LidarScan::FieldSlot::get_unsafe() const {
    return f32;
}

template <>
Eigen::Ref<const img_t<uint64_t>> LidarScan::FieldSlot::get_unsafe() const {
    return f64;
}

constexpr int LidarScan::N_FIELDS;

LidarScan::LidarScan() = default;
LidarScan::LidarScan(const LidarScan&) = default;
LidarScan::LidarScan(LidarScan&&) = default;
LidarScan& LidarScan::operator=(const LidarScan&) = default;
LidarScan& LidarScan::operator=(LidarScan&&) = default;
LidarScan::~LidarScan() = default;

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

Table<sensor::ChanField, sensor::ChanFieldType, 4> legacy_field_slots{
    {{sensor::RANGE, sensor::UINT32},
     {sensor::SIGNAL, sensor::UINT32},
     {sensor::NEAR_IR, sensor::UINT32},
     {sensor::REFLECTIVITY, sensor::UINT32}}};

// TODO: scan batching doesn't currently zero out missing fields kinda relying
// on headers being zeroed here, but that doesn't work when reusing scans
LidarScan::LidarScan(size_t w, size_t h)
    : timestamp_{Header<uint64_t>::Zero(w)},
      measurement_id_{Header<uint16_t>::Zero(w)},
      status_{Header<uint32_t>::Zero(w)},
      field_types_{legacy_field_slots.begin(), legacy_field_slots.end()},
      w{static_cast<std::ptrdiff_t>(w)},
      h{static_cast<std::ptrdiff_t>(h)},
      headers{w, BlockHeader{ts_t{0}, 0, 0}} {
    for (const auto& ft : legacy_field_slots) {
        fields_[ft.first] = FieldSlot{ft.second, w, h};
    }
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
Eigen::Ref<img_t<T>> LidarScan::field(sensor::ChanField f) {
    return fields_.at(f).get<T>();
}

template <typename T,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
Eigen::Ref<const img_t<T>> LidarScan::field(sensor::ChanField f) const {
    return fields_.at(f).get<T>();
}

// explicitly instantiate for each supported field type
template Eigen::Ref<img_t<uint8_t>> LidarScan::field(sensor::ChanField f);
template Eigen::Ref<img_t<uint16_t>> LidarScan::field(sensor::ChanField f);
template Eigen::Ref<img_t<uint32_t>> LidarScan::field(sensor::ChanField f);
template Eigen::Ref<img_t<uint64_t>> LidarScan::field(sensor::ChanField f);
template Eigen::Ref<const img_t<uint8_t>> LidarScan::field(
    sensor::ChanField f) const;
template Eigen::Ref<const img_t<uint16_t>> LidarScan::field(
    sensor::ChanField f) const;
template Eigen::Ref<const img_t<uint32_t>> LidarScan::field(
    sensor::ChanField f) const;
template Eigen::Ref<const img_t<uint64_t>> LidarScan::field(
    sensor::ChanField f) const;

sensor::ChanFieldType LidarScan::field_type(sensor::ChanField f) const {
    return fields_.at(f).tag;
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

Eigen::Ref<LidarScan::Header<uint32_t>> LidarScan::status() {
    return status_;
}
Eigen::Ref<const LidarScan::Header<uint32_t>> LidarScan::status() const {
    return status_;
}

bool operator==(const LidarScan::BlockHeader& a,
                const LidarScan::BlockHeader& b) {
    return a.timestamp == b.timestamp && a.encoder == b.encoder &&
           a.status == b.status;
}

bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.frame_id && b.frame_id && a.w == b.w && a.h == b.h &&
           a.fields_ == b.fields_ && a.field_types_ == b.field_types_ &&
           a.headers == b.headers && (a.timestamp() == b.timestamp()).all() &&
           (a.measurement_id() == b.measurement_id()).all() &&
           (a.status() == b.status()).all();
}

XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg) {
    if (w <= 0 || h <= 0)
        throw std::invalid_argument("lut dimensions must be greater than zero");
    if (azimuth_angles_deg.size() != h || altitude_angles_deg.size() != h)
        throw std::invalid_argument("unexpected scan dimensions");

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
    lut.offset.col(0) = encoder.cos() - lut.direction.col(0);
    lut.offset.col(1) = encoder.sin() - lut.direction.col(1);
    lut.offset.col(2) = -lut.direction.col(2);
    lut.offset *= lidar_origin_to_beam_origin_mm;

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
    return cartesian(scan.field(sensor::RANGE), lut);
}

LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const XYZLut& lut) {
    if (range.cols() * range.rows() != lut.direction.rows())
        throw std::invalid_argument("unexpected image dimensions");

    auto reshaped = Eigen::Map<const Eigen::Array<LidarScan::raw_t, -1, 1>>(
        range.data(), range.cols() * range.rows());
    auto nooffset = lut.direction.colwise() * reshaped.cast<double>();
    return (nooffset.array() == 0.0).select(nooffset, nooffset + lut.offset);
}

ScanBatcher::ScanBatcher(size_t w, const sensor::packet_format& pf)
    : w(w), h(pf.pixels_per_column), next_m_id(0), ls_write(w, h), pf(pf) {}

/*
 * Set all field columns in the range [start, end) to zero
 */
static void zero_scan_cols(LidarScan& ls, std::ptrdiff_t start,
                           std::ptrdiff_t end) {
    for (const auto& ft : ls) {
        ls.field(ft.first).block(0, start, ls.h, end - start).setZero();
    }
}

bool ScanBatcher::operator()(const uint8_t* packet_buf, LidarScan& ls) {
    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");

    bool swapped = false;
    const uint16_t f_id = pf.frame_id(packet_buf);

    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
        const uint32_t encoder = pf.col_encoder(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status == 0xffffffff);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (!valid || m_id >= w || f_id + 1 == ls_write.frame_id) continue;

        if (ls_write.frame_id != f_id) {
            // if not initializing with first packet
            if (ls_write.frame_id != -1) {
                // zero out remaining missing columns
                zero_scan_cols(ls_write, next_m_id, w);

                // finish the scan and notify callback
                std::swap(ls, ls_write);
                swapped = true;
            }

            // start new frame
            next_m_id = 0;
            ls_write.frame_id = f_id;
        }

        // zero out missing columns if we jumped forward
        if (m_id >= next_m_id) {
            zero_scan_cols(ls_write, next_m_id, m_id);
            next_m_id = m_id + 1;
        }

        // old header API; will be removed in a future release
        ls_write.header(m_id) = {ts, encoder, status};

        // write new header values
        ls_write.timestamp()[m_id] = ts.count();
        ls_write.measurement_id()[m_id] = m_id;
        ls_write.status()[m_id] = status;

        for (const auto& ft : ls_write) {
            pf.col_field(col_buf, ft.first,
                         ls_write.field(ft.first).col(m_id).data(), w);
        }
    }
    return swapped;
}

}  // namespace ouster
