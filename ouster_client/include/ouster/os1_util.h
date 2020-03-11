/**
 * @file
 * @brief Utilities to interpret data returned from the sensor
 */

#pragma once

#include <algorithm>
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
 * Design values for imu and lidar to sensor-frame transforms. See the OS-1
 * manual for details.
 */
extern const std::vector<double> imu_to_sensor_transform;
extern const std::vector<double> lidar_to_sensor_transform;

/**
 * Generate a matrix of unit vectors pointing radially outwards, useful for
 * efficiently computing cartesian coordinates from ranges.  The result is a n x
 * 3 array of doubles stored in row-major order where each row is the unit
 * vector corresponding to the nth point in a lidar scan, with 0 <= n < H*W. The
 * index into the lidar scan of a point can be obtained by H * j + i (where i is
 * the index to nth_px, and j is the measurement_id of the column, when reading
 * from a packet).
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param H number of rows in the lidar scan. 64 for the OS1 family of sensors.
 * @param beam_azimuth_angles azimuth offsets in degrees for each of H beams
 * @param beam_altitude_angles altitude in degrees for each of H beams
 * @return xyz direction unit vectors for each point in the lidar scan
 */
std::vector<double> make_xyz_lut(
    int W, int H, const std::vector<double>& beam_azimuth_angles,
    const std::vector<double>& beam_altitude_angles);

/**
 * Generate a table of pixel offsets based on the scan width (512, 1024, or 2048
 * columns). These can be used to create a de-staggered range image where each
 * column of pixels has the same azimuth angle from raw sensor output.
 * The offset is the starting column of each row in the de-staggered lidar scan.
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @return vector of H pixel offsets
 */
std::vector<int> get_px_offset(int W);

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
 * @param xyz_lut a lookup table generated from make_xyz_lut, above
 * @param W number of columns in the lidar scan. One of 512, 1024, or 2048.
 * @param H number of rows in the lidar scan. 64 for the OS1 family of sensors.
 * @param empty value to insert for mossing data
 * @param c function to construct a value from x, y, z (m), i, ts, reflectivity,
 * ring, noise, range (mm). Needed to use with Eigen datatypes.
 * @param f callback invoked when batching a scan is done.
 * @return a function taking a lidar packet buffer and random-access iterator to
 * which data is added for every point in the scan.
 */
template <typename iterator_type, typename F, typename C>
std::function<void(const uint8_t*, iterator_type it)> batch_to_iter(
    const std::vector<double>& xyz_lut, int W, int H,
    const typename std::iterator_traits<iterator_type>::value_type& empty,
    C&& c, F&& f) {
    int next_m_id{W};
    int32_t cur_f_id{-1};

    int64_t scan_ts{-1L};

    return [=](const uint8_t* packet_buf, iterator_type it) mutable {
        for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
            const uint8_t* col_buf = OS1::nth_col(icol, packet_buf);
            const uint16_t m_id = OS1::col_measurement_id(col_buf);
            const uint16_t f_id = OS1::col_frame_id(col_buf);
            const uint64_t ts = OS1::col_timestamp(col_buf);
            const bool valid = OS1::col_valid(col_buf) == 0xffffffff;

            // drop invalid / out-of-bounds data in case of misconfiguration
            if (!valid || m_id >= W || f_id + 1 == cur_f_id) continue;

            if (f_id != cur_f_id) {
                // if not initializing with first packet
                if (scan_ts != -1) {
                    // zero out remaining missing columns
                    std::fill(it + (H * next_m_id), it + (H * W), empty);
                    f(scan_ts);
                }

                // start new frame
                scan_ts = ts;
                next_m_id = 0;
                cur_f_id = f_id;
            }

            // zero out missing columns if we jumped forward
            if (m_id >= next_m_id) {
                std::fill(it + (H * next_m_id), it + (H * m_id), empty);
                next_m_id = m_id + 1;
            }

            // index of the first point in current packet
            const int idx = H * m_id;

            for (uint8_t ipx = 0; ipx < H; ipx++) {
                const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
                uint32_t r = OS1::px_range(px_buf);
                int ind = 3 * (idx + ipx);

                // x, y, z(m), i, ts, reflectivity, ring, noise, range (mm)
                it[idx + ipx] = c(r * 0.001f * xyz_lut[ind + 0],
                                  r * 0.001f * xyz_lut[ind + 1],
                                  r * 0.001f * xyz_lut[ind + 2],
                                  OS1::px_signal_photons(px_buf), ts - scan_ts,
                                  OS1::px_reflectivity(px_buf), ipx,
                                  OS1::px_noise_photons(px_buf), r);
            }
        }
    };
}
}
}
