/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <memory>
#include <vector>

#include "os_sensor/lidar_scan_stream_generated.h"
#include "ouster/lidar_scan.h"

namespace ouster {
namespace osf {

// Encoded single PNG buffer
using ScanChannelData = std::vector<uint8_t>;

// Encoded PNG buffers
using ScanData = std::vector<ScanChannelData>;

// FieldTypes container
using LidarScanFieldTypes =
    std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>;

/**
 * libpng only versions for Encode/Decode LidarScan to PNG buffers
 */

// ========== Decode Functions ===================================

/**
 * Decode the PNG buffers into LidarScan object. This is a dispatch function to
 * the specific decoding functions.
 *
 * @param[out] lidar_scan The output object that will be filled as a result of
 *                        decoding.
 * @param[in] scan_data PNG buffers to decode.
 * @param[in] px_offset Pixel shift per row used to reconstruct staggered range
 *                      image form.
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                const std::vector<int>& px_offset);

#ifdef OUSTER_OSF_NO_THREADING
/// Decoding eUDP LidarScan
// TODO[pb]: Make decoding of just some fields from scan data?? Not now ...
bool scanDecodeFieldsSingleThread(LidarScan& lidar_scan,
                                  const ScanData& scan_data,
                                  const std::vector<int>& px_offset);
#else
/// Decoding eUDP LidarScan, multithreaded version
bool scanDecodeFields(LidarScan& lidar_scan, const ScanData& scan_data,
                      const std::vector<int>& px_offset);
#endif

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
bool fieldDecode(
    LidarScan& lidar_scan, const ScanData& scan_data, size_t scan_idx,
    const std::pair<sensor::ChanField, sensor::ChanFieldType> field_type,
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
bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                     const ScanChannelData& channel_buf);

/**
 * @copydoc OSFPngDecode8
 * @param[in] px_offset pixel shift per row used to reconstruct staggered range
 *                      image form
 */
template <typename T>
bool decode8bitImage(Eigen::Ref<img_t<T>> img,
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
bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode16 */
template <typename T>
bool decode16bitImage(Eigen::Ref<img_t<T>> img,
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
bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode24 */
template <typename T>
bool decode24bitImage(Eigen::Ref<img_t<T>> img,
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
bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode32 */
template <typename T>
bool decode32bitImage(Eigen::Ref<img_t<T>> img,
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
bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngDecode64 */
template <typename T>
bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf);

// ========== Encode Functions ===================================

/**
 * Encode LidarScan to PNG buffers storing all field_types present in an object.
 *
 * @param[in] lidar_scan The LidarScan object to encode.
 * @param[in] px_offset Pixel shift per row used to
 *                      destaggered LidarScan data.
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncode(const LidarScan& lidar_scan,
                    const std::vector<int>& px_offset);

#ifdef OUSTER_OSF_NO_THREADING
/**
 * Encode the lidar scan fields to PNGs channel buffers (ScanData).
 * Single-threaded implementation.
 *
 * @param[in] lidar_scan A lidar scan object to encode.
 * @param[in] px_offset Pixel shift per row used to construct de-staggered range
 *                      image form.
 * @return Encoded PNGs in ScanData in order of field_types.
 */
ScanData scanEncodeFieldsSingleThread(const LidarScan& lidar_scan,
                                      const std::vector<int>& px_offset,
                                      const LidarScanFieldTypes& field_types);
#else
/**
 * Encode the lidar scan fields to PNGs channel buffers (ScanData).
 * Multi-threaded implementation.
 *
 * @param[in] lidar_scan A lidar scan object to encode.
 * @param[in] px_offset Pixel shift per row used to construct de-staggered range
 *                      image form.
 * @param[in] field_types The field types to use for encoding.
 * @return Encoded PNGs in ScanData in order of field_types.
 */
ScanData scanEncodeFields(const LidarScan& lidar_scan,
                          const std::vector<int>& px_offset,
                          const LidarScanFieldTypes& field_types);
#endif
/**
 * Encode a single lidar scan field to PNGs channel buffer and place it to a
 * specified `scan_data[scan_idx]` place
 *
 * @param[in] lidar_scan a lidar scan object to encode
 * @param[in] field_type a filed_type of lidar scan to encode
 * @param[in] px_offset  pixel shift per row used to construct de-staggered
 * range image form
 * @param[out] scan_data channel buffers storage for the encoded lidar_scan
 * @param[in] scan_idx index in `scan_data` of the beginning of field buffers
 * where the result of encoding will be inserted
 * @return false (0) if operation is successful true (1) if error occured
 */
bool fieldEncode(
    const LidarScan& lidar_scan,
    const std::pair<sensor::ChanField, sensor::ChanFieldType> field_type,
    const std::vector<int>& px_offset, ScanData& scan_data, size_t scan_idx);

/**
 * Encode multiple lidar scan fields to PNGs channel buffers and insert them to
 * a specified places `scan_idxs` in `scan_data`.
 *
 * @param[in] lidar_scan a lidar scan object to encode
 * @param[in] field_types a vector of filed_types of
 *                        lidar scan to encode
 * @param[in] px_offset pixel shift per row used to construct de-staggered range
 *                      image form
 * @param[out] scan_data channel buffers storage for the encoded lidar_scan
 * @param[in] scan_idxs a vector of indices in `scan_data` of the beginning of
 *                      field buffers where the result of encoding will be
 *                      inserted. `field_types.size()` should be equal to
 *                      `scan_idxs.size()`
 */
void fieldEncodeMulti(const LidarScan& lidar_scan,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset, ScanData& scan_data,
                      const std::vector<size_t>& scan_idxs);

/**
 * @defgroup OSFPngEncode8 Encoding Functionality.
 * Encode img object into a 8 bit, Gray, PNG buffer.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] res_buf The output buffer with a single encoded PNG.
 * @param[in] img The image object to encode.
 * @return false (0) if operation is successful, true (1) if error occured
 */

/**
 * @copydoc OSFPngEncode8
 * @param[in] px_offset Pixel shift per row used to destagger img data.
 */
template <typename T>
bool encode8bitImage(ScanChannelData& res_buf,
                     const Eigen::Ref<const img_t<T>>& img,
                     const std::vector<int>& px_offset);

/** @copydoc OSFPngEncode8 */
template <typename T>
bool encode8bitImage(ScanChannelData& res_buf,
                     const Eigen::Ref<const img_t<T>>& img);

/**
 * Encode img object into a 16 bit, Gray, PNG buffer.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] res_buf The output buffer with a single encoded PNG.
 * @param[in] img The image object to encode.
 * @param[in] px_offset Pixel shift per row used to destagger img data.
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode16bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

/**
 * Encode 2D image of a typical lidar scan field channel into a 16 bit, Gray,
 * PNG buffer.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] res_buf The output buffer with a single encoded PNG.
 * @param[in] img The image object to encode.
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode16bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

/**
 * @defgroup OSFPngEncode32 Encoding Functionality.
 * Encode 2D image of a typical lidar scan field channel into a 32 bit, RGBA,
 * PNG buffer.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] res_buf The output buffer with a single encoded PNG.
 * @param[in] img 2D image or a single LidarScan field data.
 * @return false (0) if operation is successful, true (1) if error occured
 */

/**
 * @copydoc OSFPngEncode32
 * @param[in] px_offset Pixel shift per row used to destagger img data.
 */
template <typename T>
bool encode32bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngEncode32 */
template <typename T>
bool encode32bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

/**
 * @defgroup OSFPngEncode24 Encoding Functionality.
 * Encode 2D image of a typical lidar scan field channel into a 24 bit, RGB,
 * PNG buffer.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] res_buf The output buffer with a single encoded PNG.
 * @param[in] img 2D image or a single LidarScan field data.
 * @return false (0) if operation is successful, true (1) if error occured
 */

/**
 * @copydoc OSFPngEncode24
 * @param[in] px_offset Pixel shift per row used to destagger img data.
 */
template <typename T>
bool encode24bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngEncode24 */
template <typename T>
bool encode24bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

/**
 * @defgroup OSFPngEncode64 Encoding Functionality.
 * Encode 2D image of a typical lidar scan field channel into a 64 bit, RGBA,
 * PNG buffer.
 *
 * @tparam T The type to use for the output array.
 *
 * @param[out] res_buf The output buffer with a single encoded PNG.
 * @param[in] img 2D image or a single LidarScan field data.
 * @return false (0) if operation is successful, true (1) if error occured
 */

/**
 * @copydoc OSFPngEncode64
 * @param[in] px_offset Pixel shift per row used to destagger img data.
 */
template <typename T>
bool encode64bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

/** @copydoc OSFPngEncode64 */
template <typename T>
bool encode64bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

}  // namespace osf
}  // namespace ouster
