#include "ouster/lidar_scan.h"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace ouster {

constexpr int LidarScan::N_FIELDS;

LidarScan::LidarScan() = default;

// TODO: scan batching doesn't currently zero out missing fields kinda relying
// on headers being zeroed here, but that doesn't work when reusing scans
LidarScan::LidarScan(size_t w, size_t h)
    : w{static_cast<std::ptrdiff_t>(w)},
      h{static_cast<std::ptrdiff_t>(h)},
      data{w * h, N_FIELDS},
      headers{w, BlockHeader{ts_t{0}, 0, 0}},
      timestamp_{header_t<uint64_t>::Zero(w)},
      measurement_id_{header_t<uint16_t>::Zero(w)},
      status_{header_t<uint32_t>::Zero(w)} {}

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

Eigen::Map<LidarScan::data_t, Eigen::Unaligned, LidarScan::DynStride>
LidarScan::block(size_t m_id) {
    return Eigen::Map<data_t, Eigen::Unaligned, DynStride>(
        data.row(m_id).data(), h, N_FIELDS, {w * h, w});
}

Eigen::Map<const LidarScan::data_t, Eigen::Unaligned, LidarScan::DynStride>
LidarScan::block(size_t m_id) const {
    return Eigen::Map<const data_t, Eigen::Unaligned, DynStride>(
        data.row(m_id).data(), h, N_FIELDS, {w * h, w});
}

Eigen::Map<img_t<LidarScan::raw_t>> LidarScan::field(LidarScan::Field f) {
    return Eigen::Map<img_t<raw_t>>(data.col(f).data(), h, w);
}

Eigen::Map<const img_t<LidarScan::raw_t>> LidarScan::field(
    LidarScan::Field f) const {
    return Eigen::Map<const img_t<raw_t>>(data.col(f).data(), h, w);
}

Eigen::Ref<LidarScan::header_t<uint64_t>> LidarScan::timestamp() {
    return timestamp_;
}
Eigen::Ref<const LidarScan::header_t<uint64_t>> LidarScan::timestamp() const {
    return timestamp_;
}

Eigen::Ref<LidarScan::header_t<uint16_t>> LidarScan::measurement_id() {
    return measurement_id_;
}
Eigen::Ref<const LidarScan::header_t<uint16_t>> LidarScan::measurement_id()
    const {
    return measurement_id_;
}

Eigen::Ref<LidarScan::header_t<uint32_t>> LidarScan::status() {
    return status_;
}
Eigen::Ref<const LidarScan::header_t<uint32_t>> LidarScan::status() const {
    return status_;
}

bool operator==(const LidarScan::BlockHeader& a,
                const LidarScan::BlockHeader& b) {
    return a.timestamp == b.timestamp && a.encoder == b.encoder &&
           a.status == b.status;
}

bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.w == b.w && a.h == b.h && (a.data == b.data).all() &&
           a.headers == b.headers && a.frame_id && b.frame_id &&
           (a.timestamp() == b.timestamp()).all() &&
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
    return cartesian(scan.field(LidarScan::RANGE), lut);
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

bool ScanBatcher::operator()(const uint8_t* packet_buf, LidarScan& ls) {
    using row_view_t =
        Eigen::Map<Eigen::Array<LidarScan::raw_t, Eigen::Dynamic,
                                Eigen::Dynamic, Eigen::RowMajor>>;

    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");

    bool swapped = false;

    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const uint16_t f_id = pf.col_frame_id(col_buf);
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
                auto rows = h * LidarScan::N_FIELDS;
                row_view_t{ls_write.data.data(), rows, w}
                    .block(0, next_m_id, rows, w - next_m_id)
                    .setZero();

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
            auto rows = h * LidarScan::N_FIELDS;
            row_view_t{ls_write.data.data(), rows, w}
                .block(0, next_m_id, rows, m_id - next_m_id)
                .setZero();
            next_m_id = m_id + 1;
        }

        // old header API; will be removed in a future release
        ls_write.header(m_id) = {ts, encoder, status};

        // write new header values
        ls_write.timestamp()[m_id] = ts.count();
        ls_write.measurement_id()[m_id] = m_id;
        ls_write.status()[m_id] = status;

        for (uint8_t ipx = 0; ipx < h; ipx++) {
            const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

            ls_write.block(m_id).row(ipx)
                << static_cast<LidarScan::raw_t>(pf.px_range(px_buf)),
                static_cast<LidarScan::raw_t>(pf.px_signal(px_buf)),
                static_cast<LidarScan::raw_t>(pf.px_ambient(px_buf)),
                static_cast<LidarScan::raw_t>(pf.px_reflectivity(px_buf));
        }
    }
    return swapped;
}

}  // namespace ouster
