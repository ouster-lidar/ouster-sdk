/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <png.h>

#include <memory>
#include <vector>

#include "os_sensor/common_generated.h"
#include "os_sensor/lidar_scan_stream_generated.h"
#include "ouster/lidar_scan.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

// Encoded single PNG buffer
using ScanChannelData = std::vector<uint8_t>;

// Encoded PNG buffers
using ScanData = std::vector<ScanChannelData>;

bool png_osf_write_init(png_structpp png_ptrp, png_infopp png_info_ptrp);
void png_osf_write_start(png_structp png_ptr, png_infop png_info_ptr,
                         ScanChannelData& res_buf, uint32_t width,
                         uint32_t height, int sample_depth, int color_type,
                         int compression_amount);
/**
 * libpng only versions for Encode/Decode LidarScan to PNG buffers
 */

// ========== Decode Functions ===================================

/**
 * Decode a single field to lidar_scan
 *
 * @param[out] lidar_scan The output object that will be filled as a result of
 *                        decoding.
 * @param[in] scan_data PNG buffers to decode.
 * @param[in] scan_idx Index in `scan_data` of the beginning of field buffers.
 * @param[in] field_type The field of `lidar_scan` to fill in with the decoded
 *                       result.
 * @param[in] px_offset Pixel shift per row used to reconstruct staggered range
 *                      image form.
 * @return false (0) if operation is successful true (1) if error occured
 */
OUSTER_API_FUNCTION
bool fieldDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                 size_t scan_idx, const ouster::FieldType& field_type,
                 const std::vector<int>& px_offset);

/**
 * Decode multiple fields to lidar_scan
 *
 * @param[out] lidar_scan The output object that will be filled as a result of
 *                        decoding.
 * @param[in] scan_data PNG buffers to decode, sequentially in the order of
 *                  field_types
 * @param[in] scan_idxs a vector of indices in `scan_data` of the beginning of
 *                   field buffers that will be decoded. `field_types.size()`
 *                   should be equal to `scan_idxs.size()` i.e. we need to
 *                   provide the index for every field type in
 *                   field_types where it's encoded data located
 * @param[in] field_types a vector of filed_types of lidar scan to decode
 * @param[in] px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
OUSTER_API_FUNCTION
bool fieldDecodeMulti(LidarScan& lidar_scan, const ScanData& scan_data,
                      const std::vector<size_t>& scan_idxs,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset);
/**
 * @defgroup OSFPngDecode8 Decoding Functionality.
 * Decode single PNG buffer (channel_buf) of 8 bit Gray encoding into
 * img.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] img The output img that will be filled as a result of
 *                 decoding.
 * @param[in] channel_buf Single PNG buffer to decode.
 * @return false (0) if operation is successful true (1) if error occured
 */

/** @copydoc OSFPngDecode8 */
template <typename T>
OUSTER_API_FUNCTION bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                                         const ScanChannelData& channel_buf);

/**
 * @copydoc OSFPngDecode8
 * @param[in] px_offset pixel shift per row used to reconstruct staggered range
 *                      image form
 */
template <typename T>
OUSTER_API_FUNCTION bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                                         const ScanChannelData& channel_buf,
                                         const std::vector<int>& px_offset);

/**
 * @defgroup OSFPngDecode16 Decoding Functionality.
 * Decode single PNG buffer (channel_buf) of 16 bit Gray encoding into
 * img.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] img The output img that will be filled as a result of
 *                 decoding.
 * @param[in] channel_buf Single PNG buffer to decode.
 * @return false (0) if operation is successful true (1) if error occured
 */

/**
 * @copydoc OSFPngDecode16
 * @param[in] px_offset pixel shift per row used to reconstruct staggered range
 *                      image form
 */
template <typename T>
OUSTER_API_FUNCTION bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf,
                                          const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode16 */
template <typename T>
OUSTER_API_FUNCTION bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf);

/**
 * @defgroup OSFPngDecode24 Decoding Functionality.
 * Decode single PNG buffer (channel_buf) of 24 bit RGB (8 bit) encoding into
 * img object.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] img The output img that will be filled as a result of
 *                 decoding.
 * @param[in] channel_buf Single PNG buffer to decode.
 * @return false (0) if operation is successful true (1) if error occured
 */

/**
 * @copydoc OSFPngDecode24
 * @param[in] px_offset Pixel shift per row used to reconstruct staggered range
 *                      image form.
 */
template <typename T>
OUSTER_API_FUNCTION bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf,
                                          const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode24 */
template <typename T>
OUSTER_API_FUNCTION bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf);

/**
 * @defgroup OSFPngDecode32 Decoding Functionality.
 * Decode single PNG buffer (channel_buf) of 32 bit RGBA (8 bit) encoding into
 * img object.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] img The output img that will be filled as a result of
 *                 decoding.
 * @param[in] channel_buf Single PNG buffer to decode.
 * @return false (0) if operation is successful true (1) if error occured
 */

/**
 * @copydoc OSFPngDecode32
 * @param[in] px_offset Pixel shift per row used to reconstruct staggered range
 *                      image form.
 */
template <typename T>
OUSTER_API_FUNCTION bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf,
                                          const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode32 */
template <typename T>
OUSTER_API_FUNCTION bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf);

/**
 * @defgroup OSFPngDecode64 Decoding Functionality.
 * Decode single PNG buffer (channel_buf) of 64 bit RGBA (16 bit) encoding into
 * img object.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] img The output img that will be filled as a result of
 *                 decoding.
 * @param[in] channel_buf Single PNG buffer to decode.
 * @return false (0) if operation is successful true (1) if error occured
 */

/**
 * @copydoc OSFPngDecode64
 * @param[in] px_offset Pixel shift per row used to reconstruct staggered range
 *                      image form.
 */
template <typename T>
OUSTER_API_FUNCTION bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf,
                                          const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode64 */
template <typename T>
OUSTER_API_FUNCTION bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                                          const ScanChannelData& channel_buf);

/**
 * Decode Field from a data buffer.
 * May use png compression, depending on field dimensionality.
 *
 * @param[inout] field field to store result in
 * @param[in] buffer buffer to decode
 */
OUSTER_API_FUNCTION
void decodeField(ouster::Field& field, const ScanChannelData& buffer);

}  // namespace osf
}  // namespace ouster
