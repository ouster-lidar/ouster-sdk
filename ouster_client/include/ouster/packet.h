/**
 * @file
 * @brief Utilities to parse lidar and imu packets
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iterator>

#include "ouster/impl/packet_impl.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {

constexpr packet_format packet__1_13_0 = impl::packet__1_14_0<64>();

constexpr packet_format packet__1_14_0__16 = impl::packet__1_14_0<16>();

constexpr packet_format packet__1_14_0__32 = impl::packet__1_14_0<32>();

constexpr packet_format packet__1_14_0__64 = impl::packet__1_14_0<64>();

constexpr packet_format packet__1_14_0__128 = impl::packet__1_14_0<128>();

/**
 * Make a function that batches a single scan (revolution) of data to a
 * LidarScan. The callback f() is invoked with the timestamp of the first column
 * in the scan before adding data from a new scan.
 *
 * @param w number of columns in the lidar scan. One of 512, 1024, or 2048
 * @param f callback when batching a scan is done, with one input argument
 * chrono::nanoseconds scan_ts for timestamp of the start of the scan
 * @return a function taking a lidar packet buffer and LidarScan to which data
 * is added for every point in the scan.
 */
template <typename F>
std::function<void(const uint8_t*, LidarScan& ls)> batch_to_scan(
    int w, const packet_format& pf, F&& f) {
    int h = pf.pixels_per_column;
    int next_m_id{w};
    int32_t cur_f_id{-1};

    constexpr std::chrono::nanoseconds invalid_ts(-1LL);
    std::chrono::nanoseconds scan_ts(invalid_ts);

    // for operations on each row of LidarScan.data
    using row_view_t =
        Eigen::Map<Eigen::Array<LidarScan::raw_t, Eigen::Dynamic,
                                Eigen::Dynamic, Eigen::RowMajor>>;

    return [=](const uint8_t* packet_buf, LidarScan& ls) mutable {
        for (int icol = 0; icol < pf.columns_per_packet; icol++) {
            const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
            const uint16_t m_id = pf.col_measurement_id(col_buf);
            const uint16_t f_id = pf.col_frame_id(col_buf);
            const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
            const bool valid = pf.col_valid(col_buf) == 0xffffffff;

            // drop invalid / out-of-bounds data in case of misconfiguration
            if (!valid || m_id >= w || f_id + 1 == cur_f_id) continue;

            if (f_id != cur_f_id) {
                // if not initializing with first packet
                if (scan_ts != invalid_ts) {
                    // zero out remaining missing columns
                    auto rows = ls.h * 4;
                    row_view_t{ls.data.data(), rows, ls.w}
                        .block(0, next_m_id, rows, w - next_m_id)
                        .setZero();
                    f(scan_ts);
                }

                // start new frame
                scan_ts = ts;
                next_m_id = 0;
                cur_f_id = f_id;
            }

            // zero out missing columns if we jumped forward
            if (m_id >= next_m_id) {
                auto rows = ls.h * 4;
                row_view_t{ls.data.data(), rows, ls.w}
                    .block(0, next_m_id, rows, m_id - next_m_id)
                    .setZero();
                next_m_id = m_id + 1;
            }

            ls.ts[m_id] = ts;
            for (uint8_t ipx = 0; ipx < h; ipx++) {
                // index of the first point in current packet
                const std::ptrdiff_t idx = ls.ind(ipx, m_id);

                const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

                // i, ts, reflectivity, ring, noise, range (mm)
                ls.data.row(idx)
                    << static_cast<LidarScan::raw_t>(pf.px_range(px_buf)),
                    static_cast<LidarScan::raw_t>(pf.px_signal_photons(px_buf)),
                    static_cast<LidarScan::raw_t>(pf.px_noise_photons(px_buf)),
                    static_cast<LidarScan::raw_t>(pf.px_reflectivity(px_buf));
            }
        }
    };
}

}  // namespace sensor
}  // namespace ouster
