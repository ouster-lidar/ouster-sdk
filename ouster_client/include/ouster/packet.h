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
 * random-access iterator. The callback f() is invoked with the timestamp of the
 * first column in the scan before adding data from a new scan. Timestamps for
 * each column are ns relative to the scan timestamp. XYZ coordinates in meters
 * are computed using the provided lookup table.
 *
 * The value type is assumed to be constructed from 9 values: x, y, z,
 * (padding), intensity, ts, reflectivity, noise, range (in mm) and
 * default-constructible. It should be compatible with ouster_ros::Point
 *
 * @param w number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param h number of rows in the lidar scan
 * @param empty value to insert for missing data
 * @param c function outputs a type to which iterator_type can be assigned,
 *          with the following input arguments:
 *          std::ptrdiff_t u for row (i.e. pixel id)
 *          std::ptrdiff_t v for column
 *          std::chrono::nanoseconds ts for absolute timestamp of measurement
 *          std::chrono::nanoseconds scan_ts for timestamp for start of scan
 *          uint32_t range in millimetres,
 *          uint16_t intensity (raw lidar return strength),
 *          uint16_t noise (raw ambient intensity),
 *          uint16_t reflectivity.
 * @param f callback when batching a scan is done, with one input argument
 *          std::chrono::nanoseconds scan_ts for timestamp of the start of s can
 * @return a function taking a lidar packet buffer and random-access iterator to
 * which data is added for every point in the scan.
 */
template <typename iterator_type, typename F, typename C>
std::function<void(const uint8_t*, iterator_type it)> batch_to_iter(
    int w, const packet_format& pf,
    const typename std::iterator_traits<iterator_type>::value_type& empty,
    C&& c, F&& f) {
    int h = pf.pixels_per_column;
    int next_m_id{w};
    int32_t cur_f_id{-1};

    constexpr std::chrono::nanoseconds invalid_ts(-1LL);
    std::chrono::nanoseconds scan_ts(invalid_ts);

    return [=](const uint8_t* packet_buf, iterator_type it) mutable {
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
                    std::fill(it + (h * next_m_id), it + (w * h), empty);
                    f(scan_ts);
                }

                // start new frame
                scan_ts = ts;
                next_m_id = 0;
                cur_f_id = f_id;
            }

            // zero out missing columns if we jumped forward
            if (m_id >= next_m_id) {
                std::fill(it + (h * next_m_id), it + (h * m_id), empty);
                next_m_id = m_id + 1;
            }

            // index of the first point in current packet
            const std::ptrdiff_t idx = h * m_id;

            for (uint8_t ipx = 0; ipx < h; ipx++) {
                const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

                // i, ts, reflectivity, ring, noise, range (mm)
                it[idx + ipx] =
                    c(ipx, m_id, ts, scan_ts, pf.px_range(px_buf),
                      pf.px_signal_photons(px_buf), pf.px_noise_photons(px_buf),
                      pf.px_reflectivity(px_buf));
            }
        }
    };
}

}  // namespace sensor
}  // namespace ouster
