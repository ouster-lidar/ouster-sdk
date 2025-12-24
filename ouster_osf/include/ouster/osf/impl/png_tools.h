/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <png.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "os_sensor/common_generated.h"
#include "os_sensor/lidar_scan_stream_generated.h"
#include "ouster/lidar_scan.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace impl {

// Non-owning reference to encoded single PNG buffer
struct EncodedScanChannelData {
    const uint8_t* data_internal;
    size_t size_internal;

    inline const uint8_t* data() const { return data_internal; }
    inline size_t size() const { return size_internal; }
};

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
OUSTER_API_FUNCTION bool decode_8bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf);

/**
 * @copydoc OSFPngDecode8
 * @param[in] px_offset pixel shift per row used to reconstruct staggered range
 *                      image form
 */
template <typename T>
OUSTER_API_FUNCTION bool decode_8bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf,
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
OUSTER_API_FUNCTION bool decode_16bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf,
    const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode16 */
template <typename T>
OUSTER_API_FUNCTION bool decode_16bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf);

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
OUSTER_API_FUNCTION bool decode_24bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf,
    const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode24 */
template <typename T>
OUSTER_API_FUNCTION bool decode_24bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf);

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
OUSTER_API_FUNCTION bool decode_32bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf,
    const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode32 */
template <typename T>
OUSTER_API_FUNCTION bool decode_32bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf);

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
OUSTER_API_FUNCTION bool decode_64bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf,
    const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode64 */
template <typename T>
OUSTER_API_FUNCTION bool decode_64bit_image(
    Eigen::Ref<ouster::sdk::core::img_t<T>> img,
    const EncodedScanChannelData& channel_buf);

/**
 * Decode Field from a data buffer.
 * May use png compression, depending on field dimensionality.
 *
 * @param[inout] field field to store result in
 * @param[in] buffer buffer to decode
 */
OUSTER_API_FUNCTION
void decode_field(ouster::sdk::core::Field& field,
                  const EncodedScanChannelData& buffer,
                  const std::vector<int>& px_offset = {});

}  // namespace impl
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
