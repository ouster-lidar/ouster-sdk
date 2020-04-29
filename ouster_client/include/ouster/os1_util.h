/**
 * @file
 * @brief Utilities to interpret data returned from the sensor
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <vector>

#include "ouster/os1_packet.h"

namespace ouster {
namespace OS1 {

/**
 * Design values for altitude and azimuth offset angles. Can be used if
 * calibrated values are not available.
 */
extern const std::vector<double> beam_altitude_angles;
extern const std::vector<double> beam_azimuth_angles;

/**
 * Unit of range from OS1 packet, in metres.
 */
constexpr double range_unit = 0.001;  // m

/**
 * Design values for imu and lidar to sensor-frame transforms. See the OS-1
 * manual for details.
 */
extern const std::vector<double> imu_to_sensor_transform;
extern const std::vector<double> lidar_to_sensor_transform;

/**
 * Generate a table of pixel offsets based on the scan width (512, 1024, or 2048
 * columns). These can be used to create a de-staggered range image where each
 * column of pixels has the same azimuth angle from raw sensor output.
 * The offset is the starting column of each row in the de-staggered lidar scan.
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @return vector of H pixel offsets
 */
std::vector<int> get_px_offset(int W, const std::string& prod_line);

/**
 * Make a function that batches a single scan (revolution) of data to a
 * random-access iterator. The callback f() is invoked with the timestamp of the
 * first column in the scan before adding data from a new scan. Timestamps for
 * each column are ns relative to the scan timestamp. XYZ coordinates in meters
 * are computed using the provided lookup table.
 *
 * The value type is assumed to be constructed from 9 values: x, y, z,
 * (padding), intensity, ts, reflectivity, noise, range (in mm) and
 * default-constructible. It should be compatible with PointOS1 in the
 * ouster_ros package.
 *
 * @param w number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param h number of rows in the lidar scan. 64 for the OS1 family of sensors.
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
    int w, int h, const typename iterator_type::value_type& empty, C&& c,
    F&& f) {
    int next_m_id{w};
    int32_t cur_f_id{-1};

    constexpr std::chrono::nanoseconds invalid_ts(-1LL);
    std::chrono::nanoseconds scan_ts(invalid_ts);

    return [=](const uint8_t* packet_buf, iterator_type it) mutable {
        for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
            const uint8_t* col_buf = OS1::nth_col(icol, packet_buf);
            const uint16_t m_id = OS1::col_measurement_id(col_buf);
            const uint16_t f_id = OS1::col_frame_id(col_buf);
            const std::chrono::nanoseconds ts(OS1::col_timestamp(col_buf));
            const bool valid = OS1::col_valid(col_buf) == 0xffffffff;

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
                const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);

                // i, ts, reflectivity, ring, noise, range (mm)
                it[idx + ipx] = c(ipx, m_id, ts, scan_ts, OS1::px_range(px_buf),
                                  OS1::px_signal_photons(px_buf),
                                  OS1::px_noise_photons(px_buf),
                                  OS1::px_reflectivity(px_buf));
            }
        }
    };
}
}  // namespace OS1
}  // namespace ouster
